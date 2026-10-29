"""
Offline test for hold_bench_test.py. NO SERIAL PORT: Manager.BEAR is replaced
before bear_interface is imported, and every BearInterface the script asks for
wraps FakeBear. Time is simulated (time.monotonic / time.sleep patched; each
bus transaction costs BUS_S), so a 90 s run takes a few seconds.

The fake arm (bare rig unless a payload is set), + = up:
    J * theta_dd = KT*iq + spring(theta) - rig_gravity(theta) - payload(theta) - friction
  * spring: TAU_SPRING_TABLE (held flat outside it, SIM ONLY); rig gravity
    RIG_MGL*sin(vest); payload m*g*r*sin(vest + offset).
  * friction: Karnopp -- stuck while |net| <= 0.85 N m (static), 0.80 N m
    Coulomb + 0.05 N m s/rad viscous while sliding; a velocity sign change
    inside a substep sticks. (Real friction varies 0.82-0.94 with angle.)
  * Mode 0 current loop: iq follows goal_iq with a 1 ms lag.
  * Stops at 0 and 116.5 deg (no bounce).
  * Watchdog: enabled in Mode 0 with a timeout and NO bus traffic for longer
    than it -> error 0x88, torque_enable reads 3, current 0. Reads count as
    traffic (bench T3). torque_enable = 0 clears it.
  * Noise like the bench: position SD 0.004 deg, velocity SD 0.026 rad/s.
  * Winding temperature: first-order, steady rise 3 C/A^2, tau 120 s.

The --sweep file is made from the SAME physics (down and up legs at
0.10 rad/s), so "the creep average matches the sweep" is a fair check of the
script, not of the fake.

Run from the repo root:
  ~/projects/bear_controller/venv/bin/python3 useful_tools/test_hold_bench_offline.py
"""
import sys
sys.dont_write_bytecode = True

import builtins
import contextlib
import csv
import io
import math
import os
import random
import shutil
import tempfile
import time
from unittest import mock

import numpy as np
from pybear import Manager


def _no_real_bear(*a, **k):
    raise AssertionError("test tried to construct a real Manager.BEAR")


Manager.BEAR = _no_real_bear      # before bear_interface is imported

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# This repo's main_controller first (not whatever the venv's editable install points to).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main_controller import bear_interface as bi                     # noqa: E402
from main_controller.controller import (TAU_SPRING_TABLE, RIG_MGL,    # noqa: E402
                                        KT, PHI_DEG)
import hold_bench_test as hb                                          # noqa: E402
import sweep_tables as ST                                             # noqa: E402

BUS_S = 0.0005
SUB_S = 0.0002
TE_LAG_S = 0.001
FS, FC, BV = 0.85, 0.80, 0.05
TOP = math.radians(116.5)
POS_NOISE = math.radians(0.004)
VEL_NOISE = 0.026
IQ_TAU = 0.001
G = 9.81


def spring_nm(act, scale=1.0):
    vest = math.degrees(act) + PHI_DEG
    lo, hi = TAU_SPRING_TABLE[0], TAU_SPRING_TABLE[-1]
    if vest <= lo[0]:
        return scale * lo[1]
    if vest >= hi[0]:
        return scale * hi[1]
    for (d0, t0), (d1, t1) in zip(TAU_SPRING_TABLE, TAU_SPRING_TABLE[1:]):
        if d0 <= vest <= d1:
            return scale * (t0 + (vest - d0) / (d1 - d0) * (t1 - t0))


def rig_grav(act):
    return RIG_MGL * math.sin(math.radians(math.degrees(act) + PHI_DEG))


def y_true(act_deg, scale=1.0):
    a = math.radians(act_deg)
    return spring_nm(a, scale) - rig_grav(a)


def pay_true(act_deg, kg, r, off):
    return kg * G * r * math.sin(math.radians(act_deg + PHI_DEG + off))


class Clock:
    def __init__(self):
        self.t = 1000.0
        self.sim = None

    def monotonic(self):
        return self.t

    def sleep(self, d):
        self.advance(max(0.0, d))

    def advance(self, d):
        end = self.t + d
        while self.t < end - 1e-12:
            h = min(SUB_S, end - self.t)
            self.sim.step(h)
            self.t += h


class FakeBear:
    def __init__(self, clock, J=0.015, start_deg=116.5, payload=(0.0, 0.0, 0.0),
                 spring_scale=1.0):
        self.clock = clock
        clock.sim = self
        self.J = J
        self.payload = payload
        self.spring_scale = spring_scale
        self.pos = math.radians(start_deg)
        self.vel = 0.0
        self.iq = 0.0
        self.te = 0
        self.te_visible = 0
        self.te_change_at = None
        self.err = 0x80
        self.temp_w, self.temp_p = 30.0, 32.0
        self.last_comm = clock.t
        self.rng = random.Random(7)
        self.regs = {'mode': 0, 'goal_iq': 0.0, 'watchdog_timeout': 0,
                     'limit_i_max': 5.5, 'p_gain_iq': 0.02, 'i_gain_iq': 0.02}
        self.writes = []           # (t, name, value, via)
        self.ignored_goal = 0      # goal_iq writes above limit_i_max (ignored by firmware)
        self.events = []           # (t, kind)
        self.max_fric_override = None
        # injection hooks
        self.on_bulk_rw = None     # fn(fake) before each bulk_read_write
        self.iq_override = None    # fn(fake) -> float or None
        self.vel_override = None
        self.err_override = None
        self.temp_override = None  # fn(fake) -> float or None
        self.fail_bulk = None      # fn(fake) -> True: return a failed bulk call
        self.drop_write = None     # fn(name, value) -> True: lose the write
        self.ext_torque = None     # fn(fake) -> N m added to the arm (+ = up), e.g. a runaway

    # -- physics -------------------------------------------------------------
    def step(self, h):
        if self.te_change_at is not None and self.clock.t >= self.te_change_at:
            self.te_visible, self.te_change_at = self.te, None
        wd = self.regs['watchdog_timeout']
        if (self.te == 1 and self.regs['mode'] == 0 and wd > 0
                and self.clock.t - self.last_comm > wd * 1e-6):
            self.err |= 0x08
            self.te = self.te_visible = 3
        driving = self.te == 1 and self.regs['mode'] == 0
        target = self.regs['goal_iq'] if driving else 0.0
        self.iq += (target - self.iq) * min(1.0, h / IQ_TAU)
        kg, r, off = self.payload
        net = (KT * self.iq + spring_nm(self.pos, self.spring_scale) - rig_grav(self.pos)
               - (kg * G * r * math.sin(self.pos + math.radians(PHI_DEG + off)) if kg else 0.0)
               + (self.ext_torque(self) if self.ext_torque else 0.0))
        fs = self.max_fric_override if self.max_fric_override else FS
        fc = self.max_fric_override if self.max_fric_override else FC
        if self.vel == 0.0:
            if abs(net) > fs:
                self.vel = (net - math.copysign(fc, net)) / self.J * h
        else:
            new = self.vel + (net - math.copysign(fc, self.vel) - BV * self.vel) / self.J * h
            self.vel = 0.0 if new * self.vel < 0 else new
        self.pos += self.vel * h
        if self.pos >= TOP:
            self.pos, self.vel = TOP, min(self.vel, 0.0)
        if self.pos <= 0.0:
            self.pos, self.vel = 0.0, max(self.vel, 0.0)
        # winding temperature: steady rise 3 C/A^2, tau 120 s
        self.temp_w += ((30.0 + 3.0 * self.iq ** 2) - self.temp_w) * h / 120.0

    # -- registers -----------------------------------------------------------
    def _read(self, name):
        if name == 'torque_enable':
            return self.te_visible
        if name == 'present_position':
            return self.pos + self.rng.gauss(0.0, POS_NOISE)
        if name == 'present_velocity':
            o = self.vel_override(self) if self.vel_override else None
            return (self.vel + self.rng.gauss(0.0, VEL_NOISE)) if o is None else o
        if name == 'present_iq':
            o = self.iq_override(self) if self.iq_override else None
            return self.iq if o is None else o
        if name == 'winding_temperature':
            o = self.temp_override(self) if self.temp_override else None
            return self.temp_w if o is None else o
        if name == 'powerstage_temperature':
            return self.temp_p
        return self.regs.get(name)

    def _err(self):
        o = self.err_override(self) if self.err_override else None
        return self.err if o is None else o

    def _write(self, name, value, via):
        self.writes.append((self.clock.t, name, value, via))
        if self.drop_write and self.drop_write(name, value):
            return
        if name == 'torque_enable':
            if value == 0:
                self.err = 0x80
            self.te = value
            self.te_change_at = self.clock.t + TE_LAG_S
        elif name == 'goal_iq' and self.regs['mode'] not in (0, 3):
            return             # SDK 1.2.0: goal_iq writable only in modes 0 and 3
        elif name == 'goal_iq' and abs(value) > self.regs['limit_i_max'] + 1e-9:
            self.ignored_goal += 1
            return             # SDK 1.2.0: a goal above limit_i_max is NOT executed
        else:
            self.regs[name] = value

    def _comm(self):
        self.clock.advance(BUS_S)
        self.last_comm = self.clock.t

    def __getattr__(self, attr):
        if attr.startswith('get_'):
            name = attr[4:]

            def getter(*ids):
                self._comm()
                self.events.append((self.clock.t, 'get_' + name))
                return [([self._read(name)], self._err()) for _ in ids]
            return getter
        if attr.startswith('set_'):
            name = attr[4:]

            def setter(*pairs):
                self._comm()
                self.events.append((self.clock.t, 'set_' + name))
                for _, v in pairs:
                    self._write(name, v, 'single')
            return setter
        raise AttributeError(attr)

    def bulk_read(self, ids, regs):
        self._comm()
        self.events.append((self.clock.t, 'bulk_read'))
        return [([self._read(r) for r in regs], self._err()) for _ in ids]

    def bulk_read_write(self, ids, regs, wnames, wdata):
        if self.on_bulk_rw:
            self.on_bulk_rw(self)
        if self.fail_bulk and self.fail_bulk(self):
            self.clock.advance(3 * 0.002)          # three timed-out tries
            return [([None] * len(regs), None) for _ in ids]
        self._comm()
        self.events.append((self.clock.t, 'bulk_rw'))
        for n, v in zip(wnames, wdata[0]):
            self._write(n, v, 'bulk')
        return [([self._read(r) for r in regs], self._err()) for _ in ids]

    def close_port(self):
        pass


# ---------------------------------------------------------------------------
def make_sweep(path, spring_scale=1.0, lo=30.0, hi=100.0, mass=0.0):
    """A bare observer_excitation-format sweep from the fake's own physics."""
    v, dt = 0.10, 0.002
    rng = random.Random(3)
    f_slide = FC + BV * v
    with open(path, 'w', newline='') as f:
        f.write(f"# observer_excitation.py mode=sweep arm_mass_kg={mass} arm_com_m=0 "
                f"velocity={v} soft_start_s=0.3 (FAKE, test_hold_bench_offline.py)\n")
        w = csv.writer(f)
        w.writerow(['t', 'loop_dt', 'goal_pos', 'pos', 'vel', 'present_iq', 'phase'])
        t = 0.0
        for phase, a0, a1 in (('down', hi, lo), ('up', lo, hi)):
            n = int(abs(math.radians(a1 - a0)) / v / dt)
            for k in range(n):
                g = math.radians(a0) + math.copysign(v * dt * k, a1 - a0)
                y = spring_nm(g, spring_scale) - rig_grav(g)
                iq = (-y + (-f_slide if phase == 'down' else f_slide)) / KT
                w.writerow([f"{t:.4f}", dt, f"{g:.6f}", f"{g + rng.gauss(0, POS_NOISE):.6f}",
                            0.0, f"{iq + rng.gauss(0, 0.02):.5f}", phase])
                t += dt


RUNS = []


class _RecRun(hb.Run):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        RUNS.append(self)


hb.Run = _RecRun


def invoke(fake, argv, clock, typed='GO', rt=True):
    made = []

    def factory():
        made.append(1)
        iface = bi.BearInterface.__new__(bi.BearInterface)
        iface.bear = fake
        iface.id = 1
        iface.last_disable_attempts = iface.last_disable_ms = None
        return iface

    logdir = tempfile.mkdtemp(prefix='hold_bench_test_')
    out = io.StringIO()
    n0 = len(fake.writes)
    RUNS.clear()
    sched = os.SCHED_FIFO if rt else os.SCHED_OTHER
    with mock.patch.object(hb, 'BearInterface', factory), \
            mock.patch.object(hb, 'LOG_SUBDIR', logdir), \
            mock.patch.object(os, 'sched_getscheduler', return_value=sched), \
            mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', clock.sleep), \
            mock.patch.object(builtins, 'input', lambda prompt='': typed), \
            contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        try:
            rc = hb.main(argv)
        except SystemExit as e:
            rc = f"SystemExit({e.code!s:.300})"
    files = sorted(os.listdir(logdir))
    res = {'rc': rc, 'out': out.getvalue(), 'ports': len(made),
           'writes': fake.writes[n0:], 'csv': None, 'run': RUNS[-1] if RUNS else None}
    if files:
        with open(os.path.join(logdir, files[0])) as f:
            lines = f.read().splitlines()
        res['csv'] = {'comments': [l for l in lines if l.startswith('#')],
                      'rows': list(csv.DictReader([l for l in lines if not l.startswith('#')])),
                      'name': files[0], 'path_text': '\n'.join(lines)}
    shutil.rmtree(logdir)
    return res


RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(f"    [{'PASS' if cond else 'FAIL'}] {name}{(' -- ' + detail) if detail else ''}")


def end_line(res):
    if not res['csv']:
        return None
    return next((l for l in res['csv']['comments'] if l.startswith('# END')), None)


def write_summary(writes):
    """Run-length summary of the writes, for the listing Sam reviews."""
    out = []
    for t, n, v, via in writes:
        key = (n, via) if (n == 'goal_iq' and via == 'bulk') else (n, v, via)
        if out and out[-1][0] == key:
            out[-1][1] += 1
        else:
            out.append([key, 1])
    return [f"{k[0]} = {'<command>' if len(k) == 2 else k[1]} [{k[-1]}]"
            + (f" x{c}" if c > 1 else '') for k, c in out]


def rows_np(res):
    rows = res['csv']['rows']
    f = lambda c: np.array([float(r[c]) for r in rows])
    return dict(t=f('t'), pos=f('pos'), iq=f('iq'), iq_sent=f('iq_sent'), u_cmd=f('u_cmd'),
                sp=f('sp'), phase=np.array([r['phase'] for r in rows]),
                aidx=np.array([int(r['angle_idx']) for r in rows]), loop_dt=f('loop_dt'))


def creep_avg(d, k, hz=None):
    hz = hb.CREEP_HZ if hz is None else hz
    m = (d['aidx'] == k) & (d['phase'] == 'creep')
    t = d['t'][m]
    keep = m & (d['t'] >= t.min() + 1.0 / hz)
    return KT * d['iq'][keep].mean(), np.degrees(np.ptp(d['pos'][keep])) / 2


def still_stats(d, k):
    m = (d['aidx'] == k) & (d['phase'] == 'still')
    t = d['t'][m]
    last = m & (d['t'] >= t.max() - 5.0)
    return KT * d['iq'][last].mean(), np.degrees(np.ptp(d['pos'][m]))


def common_after_enable(res, label, cap=4.8, raised=False):
    """Every armed run must end disabled with the watchdog reset, and write
    nothing outside the declared set."""
    allowed = {'watchdog_timeout', 'goal_iq', 'mode', 'torque_enable'}
    if raised:
        allowed.add('limit_i_max')
    names = {w[1] for w in res['writes']}
    check(f"{label}: only declared registers written", names <= allowed, str(sorted(names)))
    w = res['writes']
    te0 = [x for x in w if x[1] == 'torque_enable' and x[2] == 0]
    check(f"{label}: ends with torque_enable = 0 written", bool(te0))
    last_wd = [x for x in w if x[1] == 'watchdog_timeout']
    check(f"{label}: watchdog reset to 0 at the end", bool(last_wd) and last_wd[-1][2] == 0)
    check(f"{label}: CSV saved with an END line", end_line(res) is not None, end_line(res) or '')
    if res['csv'] and res['csv']['rows']:
        d = rows_np(res)
        keep = d['phase'] != 'idle'
        d = {k: (v[keep] if isinstance(v, np.ndarray) and len(v) == len(keep) else v) for k, v in d.items()}
        check(f"{label}: |iq command| <= {cap:g} A always", np.abs(d['iq_sent']).max() <= cap + 1e-9,
              f"max {np.abs(d['iq_sent']).max():.3f}")
        # judged on the logged time stamps (the thermal hold logs every 10th cycle)
        du = np.abs(np.diff(d['u_cmd']))
        lim = hb.RATE_NM_S * np.maximum(np.diff(d['t']), hb.PERIOD_S) * 1.05 + 1e-6
        check(f"{label}: torque never changes faster than 4 N m/s", np.all(du <= lim),
              f"max rate {np.max(du / np.maximum(np.diff(d['t']), 1e-9)):.2f} N m/s")


def bare_args(sweep, *extra):
    return ['--sweep', sweep, *extra]


def main():
    tmp = tempfile.mkdtemp(prefix='hold_fake_sweeps_')
    sweep = os.path.join(tmp, 'sweep_bare.csv')
    make_sweep(sweep)
    sweep_narrow = os.path.join(tmp, 'sweep_narrow.csv')
    make_sweep(sweep_narrow, lo=50, hi=90)
    sweep_strong = os.path.join(tmp, 'sweep_strong.csv')
    make_sweep(sweep_strong, spring_scale=2.6)
    sweep_wrench = os.path.join(tmp, 'sweep_wrench.csv')
    make_sweep(sweep_wrench, mass=0.711)
    sweep_mid = os.path.join(tmp, 'sweep_mid.csv')          # x1.6: 40 deg needs ~6.6 A
    make_sweep(sweep_mid, spring_scale=1.6)

    tab = ST.sweep_tables(sweep)
    print("0. Fake sweep parses; its y matches the fake's physics")
    errs = [abs(r[2] - y_true(r[0] - PHI_DEG)) for r in tab]
    check("sweep_tables y within 0.03 N m of the fake's true y", max(errs) < 0.03,
          f"max {max(errs):.3f} over {len(tab)} bins")

    print("\n1. Refusals before the port is opened (no writes possible)")
    for argv, why in [
            (bare_args(sweep, '--angles', '30'), 'angle below 35'),
            (bare_args(sweep, '--angles', '80,96'), 'angle above 95'),
            (bare_args(sweep, '--creep-deg', '3'), 'creep 3 deg'),
            (bare_args(sweep, '--thermal', '--thermal-s', '1900'), 'thermal 1900 s'),
            (bare_args(sweep, '--i-max', '5.0'), '--i-max below 5.5'),
            (bare_args(sweep, '--i-max', '10.5'), '--i-max above the 10 A ceiling'),
            (bare_args(sweep, '--i-max', 'nan'), '--i-max NaN'),
            (bare_args(sweep, '--idle-s', '-1'), '--idle-s negative'),
            (bare_args(sweep, '--payload-kg', '0.711'), 'payload kg without r'),
            (bare_args(sweep, '--kp', '500'), 'kp out of range'),
            (bare_args(sweep, '--release-margin', '0.05'), 'release margin below 0.1'),
            (bare_args(sweep, '--creep-ff', '1.2'), 'friction feedforward over 1.0 (over-compensation)'),
            (bare_args(sweep_wrench), 'non-bare sweep'),
            (bare_args(sweep_narrow, '--angles', '80,40'), "angle outside the sweep's range"),
            (bare_args(sweep_strong, '--angles', '40'), 'predicted > 4.8 A'),
            (bare_args(sweep, '--angles', '80,60,40,80,60,40'), 'planned run > 120 s'),
            (bare_args(sweep, '--angles', '88', '--payload-kg', '2', '--payload-r-m', '0.4'),
             'payload that would fall when released (2 kg at 0.4 m)'),
            (bare_args(sweep, '--payload-kg', '0.711', '--payload-r-m', '0.40'),
             'payload that would fall when released (0.711 kg at 0.40 m)'),
            (['--angles', '80'], 'no --sweep')]:
        clock = Clock()
        fake = FakeBear(clock)
        res = invoke(fake, argv, clock)
        check(f"refused: {why}", str(res['rc']).startswith('SystemExit') and res['ports'] == 0
              and not res['writes'], f"{str(res['rc'])[:90]}")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, bare_args(sweep), clock, rt=False)
    check("refused: not at RT priority (no port opened)",
          'real-time' in str(res['rc']) and res['ports'] == 0)

    print("\n2. Refusals before enabling (port open, reads only, ZERO writes)")
    for setup, why in [
            (lambda f: setattr(f, 'te_visible', 1), 'actuator already enabled'),
            (lambda f: setattr(f, 'err', 0x90), 'fault latched'),
            (lambda f: setattr(f, 'pos', math.radians(90)), 'arm not on the top stop'),
            (lambda f: setattr(f, 'temp_w', 66.0), 'too hot'),
            (lambda f: f.regs.__setitem__('limit_i_max', 6.0), 'limit_i_max 6 A')]:
        clock = Clock()
        fake = FakeBear(clock)
        setup(fake)
        res = invoke(fake, bare_args(sweep), clock)
        check(f"refused: {why}", 'REFUSED' in str(res['rc']) and not res['writes'],
              f"writes {len(res['writes'])}")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, bare_args(sweep), clock, typed='go')
    check("not started without GO (ZERO writes, no CSV)",
          res['rc'] == 1 and not res['writes'] and res['csv'] is None)

    # Oct 28: after a Position-Mode sweep the actuator is left in mode 2 with a
    # stale goal_iq (read -5.5 A); the old enable() wrote goal_iq before mode 0,
    # the write was ignored, and enable refused.
    clock = Clock()
    fake = FakeBear(clock)
    fake.regs['mode'], fake.regs['goal_iq'] = 2, -5.5
    left = invoke(fake, bare_args(sweep, '--angles', '80'), clock)
    first_bulk = next((v for _, n, v, via in left['writes'] if n == 'goal_iq' and via == 'bulk'), None)
    check("actuator left in Position Mode by a sweep (mode 2, goal_iq -5.5): enable "
          "switches mode first, zeroes goal_iq, run completes, first command 0",
          left['rc'] == 0 and 'END completed' in (end_line(left) or '') and first_bulk == 0.0,
          f"{(end_line(left) or str(left['rc']))[:90]}; first bulk goal_iq {first_bulk}")

    print("\n3. Bare run, J 0.015 (the full default plan)")
    clock = Clock()
    fake = FakeBear(clock, J=0.015)
    bare = invoke(fake, bare_args(sweep), clock)
    check("completed", bare['rc'] == 0 and 'END completed' in (end_line(bare) or ''),
          end_line(bare) or bare['out'][-300:])
    common_after_enable(bare, 'bare')
    print("    writes, in order:")
    for line in write_summary(bare['writes']):
        print(f"      {line}")
    seq = [(n, v) for _, n, v, via in bare['writes'] if not (n == 'goal_iq' and via == 'bulk')]
    check("write order: watchdog 10000, mode 0, goal_iq 0, torque_enable 1, ..., "
          "goal_iq 0, torque_enable 0, watchdog 0",
          seq == [('watchdog_timeout', 10000), ('mode', 0), ('goal_iq', 0.0),
                  ('torque_enable', 1), ('goal_iq', 0.0), ('torque_enable', 0),
                  ('watchdog_timeout', 0)], str(seq))
    ev = fake.events
    first = next(i for i, e in enumerate(ev) if e[1] == 'bulk_rw')
    last = max(i for i, e in enumerate(ev) if e[1] == 'bulk_rw')
    inside = {e[1] for e in ev[first:last + 1]}
    check("enabled loop uses ONLY bulk_read_write (one packet per cycle)", inside == {'bulk_rw'},
          str(sorted(inside)))
    d = rows_np(bare)
    check("loop period ~2 ms", abs(np.median(d['loop_dt']) - 0.002) < 0.0003,
          f"median {np.median(d['loop_dt']) * 1000:.2f} ms")
    for k, deg in enumerate(hb.DEFAULT_ANGLES):
        u_c, amp = creep_avg(d, k)
        u_s, pp = still_stats(d, k)
        truth = -y_true(deg)
        check(f"{deg:g} deg: creep average within 0.05 N m of the fake's true -y",
              abs(u_c - truth) < 0.05, f"{u_c:+.3f} vs {truth:+.3f}")
        check(f"{deg:g} deg: arm really creeps (measured amp >= 0.8 deg)", amp >= 0.8,
              f"{amp:.2f} deg")
        check(f"{deg:g} deg: still reading inside the friction band", abs(u_s - truth) <= FS + 0.05,
              f"{u_s:+.3f} (truth {truth:+.3f}, band +-{FS})")
        check(f"{deg:g} deg: no hunting while still (p-p < 0.5 deg)", pp < 0.5, f"{pp:.3f} deg")
    bare_creep = [creep_avg(d, k)[0] for k in range(3)]
    print("    script's own quick lines:")
    for l in bare['csv']['comments']:
        if l.startswith('# quick'):
            print(f"      {l}")

    print("\n4. Gains across arm inertias (J 0.006 and 0.03)")
    for J in (0.006, 0.03):
        clock = Clock()
        fake = FakeBear(clock, J=J)
        res = invoke(fake, bare_args(sweep), clock)
        ok = res['rc'] == 0
        detail = end_line(res) or ''
        if ok:
            dd = rows_np(res)
            errs = [abs(creep_avg(dd, k)[0] + y_true(deg)) for k, deg in enumerate(hb.DEFAULT_ANGLES)]
            pps = [still_stats(dd, k)[1] for k in range(3)]
            amps = [creep_avg(dd, k)[1] for k in range(3)]
            ok = max(errs) < 0.05 and max(pps) < 0.5 and min(amps) >= 0.8
            detail = (f"creep err max {max(errs):.3f} N m, still p-p max {max(pps):.3f} deg, "
                      f"creep amp min {min(amps):.2f} deg")
        check(f"J {J}: completes, creep within 0.05, no hunting, full creep", ok, detail)

    print("\n5. Payload run (0.711 kg at 0.1651 m, offset 0): delta-u = the weight's torque")
    pay = (0.711, 0.1651, 0.0)
    clock = Clock()
    fake = FakeBear(clock, J=0.02, payload=pay)
    res = invoke(fake, bare_args(sweep, '--payload-kg', '0.711', '--payload-r-m', '0.1651'), clock)
    check("payload run completed", res['rc'] == 0, end_line(res) or res['out'][-300:])
    if res['rc'] == 0:
        dp = rows_np(res)
        for k, deg in enumerate(hb.DEFAULT_ANGLES):
            du = creep_avg(dp, k)[0] - bare_creep[k]
            tp = pay_true(deg, *pay)
            check(f"{deg:g} deg: delta-u within 0.07 N m of m*g*r*sin", abs(du - tp) < 0.07,
                  f"{du:+.3f} vs {tp:+.3f}")
        check("payload in the CSV header", any('payload_kg=0.711' in l for l in res['csv']['comments']))

    print("\n6. Aborts (each must disable, reset the watchdog and save the log)")
    T_ARM = 1000.0 + 25.0      # well into the run (into the 80 deg hold)

    def after(t):
        return lambda f: f.clock.t > t

    cases = [
        ('|iq| >= 5.0 A for 3 cycles', lambda f: setattr(f, 'iq_override',
                                                            lambda x: 5.2 if after(T_ARM)(x) else None), '|iq|'),
        ('real runaway: +4 N m pushes the arm up', lambda f: setattr(f, 'ext_torque',
                                                 lambda x: 4.0 if after(T_ARM)(x) else 0.0), 'speed'),
        ('error byte 0x90', lambda f: setattr(f, 'err_override',
                                              lambda x: 0x90 if after(T_ARM)(x) else None), 'error byte'),
        ('exchange fails', lambda f: setattr(f, 'fail_bulk', after(T_ARM)), 'exchange failed'),
        ('temperature 76 C', lambda f: setattr(f, 'temp_override',
                                               lambda x: 76.0 if after(T_ARM)(x) else None), 'temperature'),
        ('arm jams (target runs away > 5 deg)',
         lambda f: setattr(f, 'on_bulk_rw', lambda x: setattr(x, 'max_fric_override', 50.0)
                           if after(T_ARM)(x) else None), 'from the target'),
        ('15 ms stall (watchdog / loop gap)',
         lambda f: setattr(f, 'on_bulk_rw', lambda x: (x.clock.advance(0.015), setattr(x, 'on_bulk_rw', None))
                           if after(T_ARM)(x) else None), ''),
    ]
    for name, setup, expect in cases:
        clock = Clock()
        fake = FakeBear(clock)
        setup(fake)
        res = invoke(fake, bare_args(sweep), clock)
        el = end_line(res) or ''
        check(f"abort: {name}", 'ABORT' in el and expect in el and res['rc'] == 1, el[:110])
        common_after_enable(res, f"  [{name}]")

    clock = Clock()
    fake = FakeBear(clock)

    def ctrl_c(x):
        if after(T_ARM)(x):
            raise KeyboardInterrupt
    fake.on_bulk_rw = ctrl_c
    res = invoke(fake, bare_args(sweep), clock)
    check("Ctrl-C: interrupted, disabled, log saved", 'interrupted' in (end_line(res) or ''),
          end_line(res) or '')
    common_after_enable(res, "  [Ctrl-C]")

    print("\n6b. Velocity-register glitches must NOT abort (position decides)")
    clock = Clock()
    fake = FakeBear(clock)
    fired = []

    def one_glitch(x):
        if after(T_ARM)(x) and not fired:
            fired.append(1)
            return 7.0                       # one wrong sample, like Aug's +7 rad/s
        return None
    fake.vel_override = one_glitch
    res = invoke(fake, bare_args(sweep), clock)
    check("one +7 rad/s register sample: run completes", 'END completed' in (end_line(res) or ''),
          end_line(res) or '')
    clock = Clock()
    fake = FakeBear(clock)
    fake.vel_override = lambda x: 2.5 if after(T_ARM)(x) else None   # register stuck
    res = invoke(fake, bare_args(sweep), clock)
    ok = 'END completed' in (end_line(res) or '')
    if ok:
        dd = rows_np(res)
        errs = [abs(creep_avg(dd, k)[0] + y_true(deg)) for k, deg in enumerate(hb.DEFAULT_ANGLES)]
        ok = max(errs) < 0.05
        detail = (f"creep err max {max(errs):.3f} N m; "
                  + next((l[2:60] for l in res['csv']['comments'] if 'disagreed' in l), 'no note'))
    else:
        detail = end_line(res) or ''
    check("register stuck at 2.5 rad/s: completes, readings still right, disagreement noted", ok,
          detail)

    print("\n6c. Disable NOT confirmed (torque_enable = 0 lost): watchdog must stay ARMED")
    clock = Clock()
    fake = FakeBear(clock)
    fake.drop_write = lambda n, v: n == 'torque_enable' and v == 0
    fake.err_override = lambda x: 0x90 if after(T_ARM)(x) else None   # force an abort
    res = invoke(fake, bare_args(sweep), clock)
    wd_writes = [w for w in res['writes'] if w[1] == 'watchdog_timeout']
    check("no watchdog_timeout = 0 write after an unconfirmed disable",
          [w[2] for w in wd_writes] == [10000], str([w[2] for w in wd_writes]))
    check("footer says DISABLE NOT CONFIRMED / cut power",
          any('DISABLE NOT CONFIRMED' in l for l in res['csv']['comments']))
    fake.err_override = None
    clock.advance(0.020)                      # silence after the script exits
    check("...and the armed watchdog then trips: error bit 3 set, current ~0",
          bool(fake.err & 0x08) and abs(fake.iq) < 0.1, f"err {hex(fake.err)}, iq {fake.iq:.3f} A")

    clock = Clock()
    fake = FakeBear(clock)
    fake.drop_write = lambda n, v: n == 'watchdog_timeout' and v == 10000
    res = invoke(fake, bare_args(sweep), clock)
    el = end_line(res) or ''
    check("watchdog write lost -> refused at its read-back, never enabled",
          'REFUSED/FAILED' in el and not any(w[1] == 'torque_enable' and w[2] == 1 for w in res['writes']),
          el[:110])

    print("\n7. Thermal mode")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '30'), clock)
    check("thermal 30 s completed", res['rc'] == 0, end_line(res) or res['out'][-300:])
    if res['rc'] == 0:
        dt_ = rows_np(res)
        m = dt_['phase'] == 'thermal'
        hz = m.sum() / (dt_['t'][m].max() - dt_['t'][m].min())
        check("thermal hold logged at ~50 Hz", 40 < hz < 60, f"{hz:.1f} Hz")
        common_after_enable(res, "  [thermal]")
    clock = Clock()
    fake = FakeBear(clock)
    fake.temp_override = lambda x: 66.0 if x.clock.t > 1000.0 + 20.0 else None
    res = invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '600'), clock)
    el = end_line(res) or ''
    check("thermal hold ends early and NORMALLY at 65 C", 'END completed' in el and
          any('thermal hold ended' in l for l in res['csv']['comments']), el)
    if res['csv']:
        dd = rows_np(res)
        fade = dd['phase'] == 'fade'
        check("...and the fade starts near the top, never mid-range (>= 95 deg)",
              fade.any() and np.degrees(dd['pos'][fade]).min() >= 95.0,
              f"fade from {np.degrees(dd['pos'][fade]).min():.1f} deg" if fade.any() else 'no fade')

    print("\n8. Stress: extreme inertias x the weight hung at -25 / +25 deg")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, bare_args(sweep, '--payload-kg', '0.711', '--payload-r-m', '0.1651',
                                 '--payload-angle-deg', '-25'), clock)
    check("weight at -25 deg refused at the default 0.2 N m release margin (fake: +0.15)",
          'SystemExit' in str(res['rc']) and res['ports'] == 0 and not res['writes'],
          str(res['rc'])[:60])
    for J in (0.006, 0.045):
        for off in (-25.0, 25.0):
            pay = (0.711, 0.1651, off)
            clock = Clock()
            fake = FakeBear(clock, J=J, payload=pay)
            res = invoke(fake, bare_args(sweep, '--payload-kg', '0.711', '--payload-r-m', '0.1651',
                                         '--payload-angle-deg', str(off),
                                         '--release-margin', '0.1'), clock)
            ok, detail = res['rc'] == 0, end_line(res) or ''
            if ok:
                dd = rows_np(res)
                errs = [abs(creep_avg(dd, k)[0] - (-y_true(deg) + pay_true(deg, *pay)))
                        for k, deg in enumerate(hb.DEFAULT_ANGLES)]
                amps = [creep_avg(dd, k)[1] for k in range(3)]
                ok = max(errs) < 0.05 and min(amps) >= 0.7
                early = any('final ramp ended' in l for l in res['csv']['comments'])
                detail = (f"creep err max {max(errs):.3f} N m, amp min {min(amps):.2f} deg"
                          f"{', final ramp ended early (as designed)' if early else ''}")
            check(f"J {J}, weight at {off:+.0f} deg: completes, reads it, creeps", ok, detail)

    print("\n10. Raised current limit (--i-max), strong-spring fake")
    # spring x1.6: the 40 deg hold needs ~6.6 A -> refused by default, runs at 8 A
    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    res = invoke(fake, bare_args(sweep_mid), clock)
    check("x1.6 spring at the default limit: refused before the port opens (needs > 4.8 A)",
          'SystemExit' in str(res['rc']) and res['ports'] == 0 and not res['writes'], str(res['rc'])[:80])
    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    el = end_line(res) or ''
    check("--i-max 8 on the strong spring: completes", res['rc'] == 0, el or res['out'][-300:])
    seq = [(n, v) for _, n, v, via in res['writes'] if not (n == 'goal_iq' and via == 'bulk')]
    check("write order: watchdog 10000, limit_i_max 8, mode 0, goal_iq 0, torque_enable 1, ..., "
          "goal_iq 0, torque_enable 0, watchdog 0, limit_i_max 5.5",
          seq == [('watchdog_timeout', 10000), ('limit_i_max', 8.0), ('mode', 0), ('goal_iq', 0.0),
                  ('torque_enable', 1), ('goal_iq', 0.0), ('torque_enable', 0),
                  ('watchdog_timeout', 0), ('limit_i_max', 5.5)], str(seq))
    check("limit restored: actuator reads 5.5 A after the run", fake.regs['limit_i_max'] == 5.5)
    check("no command was ever above the limit in force (firmware ignored none)",
          fake.ignored_goal == 0, f"{fake.ignored_goal} ignored")
    if res['rc'] == 0:
        dd = rows_np(res)
        peak = np.abs(dd['iq']).max()
        check("the run really used more than the old 4.8 A cap", peak > 4.8, f"peak |iq| {peak:.2f} A")
        errs = [abs(creep_avg(dd, k)[0] + y_true(deg, 1.6)) for k, deg in enumerate(hb.DEFAULT_ANGLES)]
        check("creep readings still right at the higher torque (within 0.05 N m)", max(errs) < 0.05,
              f"max err {max(errs):.3f}")
        common_after_enable(res, "  [i-max 8]", cap=7.3, raised=True)
        check("header records the raised limit", any('i_max=8' in l for l in res['csv']['comments']))

    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    fake.iq_override = lambda x: -7.6 if after(T_ARM)(x) else None
    res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    el = end_line(res) or ''
    check("abort scales with the limit: |iq| >= 7.5 A x3 aborts", '7.5 A' in el, el)
    check("...and the limit is still restored after the confirmed disable",
          fake.regs['limit_i_max'] == 5.5 and bool(res['writes'])
          and res['writes'][-1][1:3] == ('limit_i_max', 5.5))
    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    fake.iq_override = lambda x: -5.2 if after(T_ARM)(x) and x.clock.t < T_ARM + 0.05 else None
    res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    check("...and 5.2 A (over the OLD abort) does not abort at --i-max 8", res['rc'] == 0,
          end_line(res) or '')

    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    fake.drop_write = lambda n, v: n == 'torque_enable' and v == 0
    fake.err_override = lambda x: 0x90 if after(T_ARM)(x) else None
    res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    lw = [w[2] for w in res['writes'] if w[1] == 'limit_i_max']
    check("unconfirmed disable: limit NOT restored (no writes after it), footer says so",
          lw == [8.0] and any('left at 8' in l for l in res['csv']['comments']), str(lw))

    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    fake.drop_write = lambda n, v: n == 'limit_i_max' and v == 8.0
    res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    el = end_line(res) or ''
    check("limit write lost -> refused at its read-back, never enabled",
          'REFUSED/FAILED' in el and not any(w[1] == 'torque_enable' and w[2] == 1 for w in res['writes']),
          el[:100])

    # the old script's way (no limit write) would have every command above 5.5 A ignored:
    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    with mock.patch.object(bi.BearInterface, 'set_current_limit',
                           lambda self, amps: setattr(self, 'max_iq', amps) or amps):
        res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    check("(fake check) skipping the firmware limit write -> commands above 5.5 A ignored, run fails",
          fake.ignored_goal > 0 and res['rc'] != 0, f"{fake.ignored_goal} ignored; "
          f"{(end_line(res) or '')[:60]}")

    # set_current_limit() directly: refuses while enabled, out of range, NaN; nothing written
    for setup, amps, exc, why in (
            (lambda f: setattr(f, 'te_visible', 1), 8.0, RuntimeError, 'while enabled'),
            (None, 10.5, ValueError, 'above the 10 A ceiling'),
            (None, float('nan'), ValueError, 'NaN'),
            (None, 0.0, ValueError, 'zero')):
        clock = Clock()
        fake = FakeBear(clock)
        if setup:
            setup(fake)
        iface = bi.BearInterface.__new__(bi.BearInterface)
        iface.bear, iface.id = fake, 1
        try:
            iface.set_current_limit(amps)
            raised = None
        except Exception as e:                     # noqa: BLE001
            raised = e
        check(f"set_current_limit refuses {why}: {exc.__name__}, limit_i_max not written",
              isinstance(raised, exc) and not any(w[1] == 'limit_i_max' for w in fake.writes),
              repr(raised)[:70])
    clock = Clock()
    fake = FakeBear(clock)
    iface = bi.BearInterface.__new__(bi.BearInterface)
    iface.bear, iface.id = fake, 1
    rb = iface.set_current_limit(8.0)
    check("set_current_limit(8) while disabled: written, read back, clamp follows",
          rb == 8.0 and fake.regs['limit_i_max'] == 8.0 and iface.max_iq == 8.0)

    print("\n11. Idle log (--idle-s): reads only, then the normal run")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '30', '--idle-s', '60'), clock)
    check("thermal with 60 s idle log completes", res['rc'] == 0, end_line(res) or res['out'][-300:])
    if res['rc'] == 0:
        dd = rows_np(res)
        idle = dd['phase'] == 'idle'
        span = dd['t'][idle].max() - dd['t'][idle].min()
        hz = idle.sum() / max(span, 1e-9)
        check("idle rows: ~60 s at ~5 Hz", 58 < span < 61 and 4 < hz < 6, f"{span:.1f} s, {hz:.1f} Hz")
        first_write = min(w[0] for w in res['writes'])
        t_idle_end = 1000.0 + dd['t'][idle].max()
        check("no write of any kind during the idle log", first_write >= t_idle_end - 1e-6,
              f"first write {first_write - 1000:.2f} s, idle ended {t_idle_end - 1000:.2f} s")
        check("idle summary in the header", any(l.startswith('# idle 60 s') for l in res['csv']['comments']))
        common_after_enable(res, "  [idle+thermal]")
    clock = Clock()
    fake = FakeBear(clock)
    calls = []

    def bump(x):                               # someone moves the arm during the idle log
        calls.append(1)
        if len(calls) == 20:
            x.pos = math.radians(100.0)
    orig = fake.bulk_read

    def br(ids, regs):
        bump(fake)
        return orig(ids, regs)
    fake.bulk_read = br
    res = invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '30', '--idle-s', '60'), clock)
    el = end_line(res) or ''
    check("arm leaves the stop during idle -> stops with ZERO writes", 'left the top stop' in el
          and not res['writes'], el[:80])
    clock = Clock()
    fake = FakeBear(clock)
    n_reads = []

    def br2(ids, regs, _o=fake.bulk_read):
        n_reads.append(1)
        if len(n_reads) == 30:
            raise KeyboardInterrupt
        return _o(ids, regs)
    fake.bulk_read = br2
    res = invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '30', '--idle-s', '60'), clock)
    check("Ctrl-C during idle -> interrupted, ZERO writes, log saved",
          'interrupted' in (end_line(res) or '') and not res['writes'], end_line(res) or '')
    ok_parse = True
    try:
        hb.parse_args(['--sweep', sweep, '--thermal', '--thermal-s', '1800'])
    except SystemExit:
        ok_parse = False
    check("--thermal-s 1800 accepted (was 600 max)", ok_parse)

    print("\n9. hold_law is pure and bounded")
    u, I = hb.hold_law(0.01, 0.0, 0.0, 0.002, 40, 60, 1.5)
    check("hold_law: P + I - D arithmetic", abs(u - (0.4 + 60 * 0.01 * 0.002)) < 1e-12)
    u, I = hb.hold_law(10.0, 0.0, 3.2, 1.0, 40, 60, 1.5)
    check("hold_law: integral clamped to the command cap", abs(I - hb.U_LIMIT_NM) < 1e-12)

    shutil.rmtree(tmp)
    n_ok = sum(ok for _, ok in RESULTS)
    print(f"\n{n_ok}/{len(RESULTS)} checks passed")
    if n_ok != len(RESULTS):
        print("FAILED:")
        for n, ok in RESULTS:
            if not ok:
                print(f"  {n}")
        sys.exit(1)


if __name__ == '__main__':
    main()