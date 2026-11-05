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
import struct
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

# ---- Level 2 rig (STATE 8 #31), measured Oct 29 ----------------------------------
# Average of the two Level 2 bare sweeps (observer_sweep_bare_20261029_142652 and
# _145313, raw-data b8884e7), per 5-deg vest bin, actuator deg:
L2_ACT = (63.1, 68.1, 73.0, 78.0, 83.0, 88.0, 93.0, 97.5)
L2_Y = (3.576, 3.292, 3.088, 2.892, 2.817, 2.785, 2.742, 2.744)    # spring - rig gravity, N m
L2_F = (1.036, 1.372, 1.311, 1.148, 1.168, 1.151, 1.172, 1.112)    # sliding friction at 0.1 rad/s
# Above the sweep (it stops at ~98 deg) y FALLS steeply toward the stop. Estimated from
# the first down-ramp of H1 L2 where the arm slid steadily (u + y + f = 0, f from the
# nearest bin): 2.57 at 101 deg, 0.81 at 109 deg [inferred, 40-46 samples each; the
# same method gave 2.83 at 95 vs the sweep's 2.74]. At the stop: 0 assumed.
L2_ACT_ALL = L2_ACT + (101.0, 109.0, 116.5)
L2_Y_ALL = L2_Y + (2.57, 0.81, 0.0)
# Static friction ABOVE sliding, from hold_bench_20261029_143245 (H1 L2) with the same y:
# the arm held a net 1.82 N m without moving at 70 deg (sweep f there 1.35 -> gap ~0.47);
# at 80 and 90 deg the stuck maximum (1.09 / 1.15) matched the sweep's f (1.16 / 1.16)
# -> gap ~0. One run, three angles [inferred]; the band edges are a guess.
L2_GAP_NM, L2_GAP_FULL_DEG, L2_GAP_ZERO_DEG = 0.47, 72.0, 80.0
# During the up-ramp breakaway the arm plateaued at ~1.8-1.9 rad/s with a net 1.4-1.6 N m
# still pushing -> sliding friction ~1.5 N m there vs 1.35 at 0.1 rad/s -> viscous ~0.09.
L2_BV = 0.09
# Winding heating measured Oct 29 (H3 at 4.8 A, settled start): k 0.30 C/A^2, tau 205 s.
# (The default fake's 3 C/A^2 was set for 2-4 A holds; at Level 2's 6-7 A it crosses
# TEMP_WARN and the script's thermal derate -- correctly -- withholds the torque.)
L2_HEAT = (0.30, 205.0)


def l2_y(act):
    return float(np.interp(math.degrees(act), L2_ACT_ALL, L2_Y_ALL))   # held flat below 63


def l2_fc(act):
    return float(np.interp(math.degrees(act), L2_ACT, L2_F))


def l2_fs(act, gap=L2_GAP_NM):
    """Static (break-free) friction: sliding + gap below 72 deg, tapering to 0 at 80 deg."""
    deg = math.degrees(act)
    w = min(1.0, max(0.0, (L2_GAP_ZERO_DEG - deg) / (L2_GAP_ZERO_DEG - L2_GAP_FULL_DEG)))
    return l2_fc(act) + gap * w


def l2_fake(clock, J=0.015, start_deg=116.5, gap_up=L2_GAP_NM, gap_down=L2_GAP_NM):
    """FakeBear with the Level 2 rig's measured y, sliding friction and heating, plus an
    adjustable static excess (N m, in the 72-80 deg band) to break free UP and DOWN. The
    real arm at 70 deg sometimes slipped near the sliding value and sometimes held ~0.47
    above it, in both directions, so the tests use 0 / up only / both."""
    return FakeBear(clock, J=J, start_deg=start_deg,
                    spring_fn=lambda a: l2_y(a) + rig_grav(a),
                    fric=dict(fc=l2_fc, fs_up=lambda a: l2_fs(a, gap_up),
                              fs_down=lambda a: l2_fs(a, gap_down), bv=L2_BV),
                    heat=L2_HEAT)


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
                 spring_scale=1.0, spring_fn=None, fric=None, heat=(3.0, 120.0)):
        self.clock = clock
        clock.sim = self
        self.J = J
        self.payload = payload
        self.spring_scale = spring_scale
        # optional: spring torque as fn(act rad) (replaces table x scale), and friction
        # as dict(fc=fn(act) sliding, fs_up / fs_down=fn(act) static to break free upward /
        # downward, bv=viscous) (replaces FS/FC/BV)
        self.spring_fn = spring_fn
        self.fric = fric
        self.breakaways = []       # (t, act deg, |net| N m) each time the stuck arm starts to slide
        self.heat_k, self.heat_tau = heat   # winding: steady rise k C/A^2, time constant tau s
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
        spring = (self.spring_fn(self.pos) if self.spring_fn
                  else spring_nm(self.pos, self.spring_scale))
        net = (KT * self.iq + spring - rig_grav(self.pos)
               - (kg * G * r * math.sin(self.pos + math.radians(PHI_DEG + off)) if kg else 0.0)
               + (self.ext_torque(self) if self.ext_torque else 0.0))
        if self.fric:
            fc, bv = self.fric['fc'](self.pos), self.fric['bv']
            fs_up, fs_dn = self.fric['fs_up'](self.pos), self.fric['fs_down'](self.pos)
        else:
            fc, bv, fs_up, fs_dn = FC, BV, FS, FS
        if self.max_fric_override:
            fs_up = fs_dn = fc = self.max_fric_override
        if self.vel == 0.0:
            if net > fs_up or net < -fs_dn:
                self.vel = (net - math.copysign(fc, net)) / self.J * h
                self.breakaways.append((self.clock.t, math.degrees(self.pos), net))
        else:
            new = self.vel + (net - math.copysign(fc, self.vel) - bv * self.vel) / self.J * h
            self.vel = 0.0 if new * self.vel < 0 else new
        self.pos += self.vel * h
        if self.pos >= TOP:
            self.pos, self.vel = TOP, min(self.vel, 0.0)
        if self.pos <= 0.0:
            self.pos, self.vel = 0.0, max(self.vel, 0.0)
        # winding temperature: steady rise heat_k C/A^2 (3 by default), tau heat_tau (120 s)
        self.temp_w += ((30.0 + self.heat_k * self.iq ** 2) - self.temp_w) * h / self.heat_tau

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
def make_sweep(path, spring_scale=1.0, lo=30.0, hi=100.0, mass=0.0, y_fn=None, f_fn=None, bv=BV):
    """A bare observer_excitation-format sweep from the fake's own physics.
    y_fn / f_fn (fn(act rad) -> N m) replace the table spring and FC (Level 2 fake)."""
    v, dt = 0.10, 0.002
    rng = random.Random(3)
    f_const = FC + BV * v
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
                y = y_fn(g) if y_fn else spring_nm(g, spring_scale) - rig_grav(g)
                f_slide = (f_fn(g) + bv * v) if f_fn else f_const
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
    # _git stubbed: subprocess waits with time.sleep (patched = simulated) while timing
    # out on the real clock, so a slow git under CPU load would jump simulated time.
    with mock.patch.object(hb, 'BearInterface', factory), \
            mock.patch.object(hb, '_git', lambda *argv: 'test'), \
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
        except KeyboardInterrupt:
            rc = 'KeyboardInterrupt (escaped main)'
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
          rb == 8.0 and fake.regs['limit_i_max'] == 8.0 and getattr(iface, 'max_iq', None) == 8.0)
    clock = Clock()
    fake = FakeBear(clock)
    fake.drop_write = lambda n, v: n == 'limit_i_max'
    fake.regs['limit_i_max'] = 4.0              # firmware actually below both old and new
    iface = bi.BearInterface.__new__(bi.BearInterface)
    iface.bear, iface.id = fake, 1
    try:
        iface.set_current_limit(8.0)
    except RuntimeError:
        pass
    check("bad read-back (4.0) below old (5.5) and new (8): clamp follows the read-back",
          getattr(iface, 'max_iq', None) == 4.0, f"max_iq {getattr(iface, 'max_iq', None)}")

    clock = Clock()
    fake = FakeBear(clock, spring_scale=1.6)
    calls = []
    real_disable = bi.BearInterface.disable

    def ki_disable(self):
        calls.append(1)
        if len(calls) == 1:
            raise KeyboardInterrupt
        return real_disable(self)
    fake.err_override = lambda x: 0x90 if after(T_ARM)(x) else None
    with mock.patch.object(bi.BearInterface, 'disable', ki_disable):
        res = invoke(fake, bare_args(sweep_mid, '--i-max', '8'), clock)
    lw = [w[2] for w in res['writes'] if w[1] == 'limit_i_max']
    check("Ctrl-C inside disable() with a raised limit: no restore write, footer says it was left",
          lw == [8.0] and res['csv'] is not None
          and any('left at 8' in l for l in res['csv']['comments']), f"{lw}; rc {res['rc']}")

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

    # STATE 8 #33: a failed idle read is skipped and counted, not an abort
    def idle_fails(fail_at, raise_at=(), alter=None):
        """Bulk reads number 1.. during the run; fail (or raise) at these numbers.
        alter = {number: fn(values, err) -> (values, err)} changes a read that answers."""
        clock = Clock()
        fake = FakeBear(clock)
        n = []

        def br(ids, regs, _o=fake.bulk_read):
            n.append(1)
            if len(n) in raise_at:
                fake._comm()
                raise struct.error('unpack requires a buffer of 4 bytes')
            if fail_at(len(n)):
                clock.advance(3 * 0.002)
                return [([None] * len(regs), None) for _ in ids]
            if alter and len(n) in alter:
                return [alter[len(n)](*r) for r in _o(ids, regs)]
            return _o(ids, regs)
        fake.bulk_read = br
        return invoke(fake, bare_args(sweep, '--thermal', '--thermal-s', '30', '--idle-s', '60'),
                      clock), n

    def fail_line(res):
        return next((l for l in (res['csv']['comments'] if res['csv'] else [])
                     if l.startswith('# idle failed reads')), '')
    # preflight does a few bulk reads first; idle reads start after them
    res, n = idle_fails(lambda k: k == 40)
    dd = rows_np(res) if res['csv'] else None
    n_idle = int((dd['phase'] == 'idle').sum()) if dd is not None else 0
    check("one failed idle read: skipped, run completes, idle rows one short (299 of 300)",
          res['rc'] == 0 and n_idle == 299, f"{end_line(res)}; idle rows {n_idle}")
    check("...the header names it with its time", fail_line(res).startswith('# idle failed reads: 1 at t = '),
          fail_line(res))
    first_write = min(w[0] for w in res['writes']) if res['writes'] else None
    check("...still no write of any kind during the idle log, then the normal run",
          first_write is not None and first_write >= 1000.0 + dd['t'][dd['phase'] == 'idle'].max() - 1e-6)
    if res['rc'] == 0:
        common_after_enable(res, "  [idle, 1 failed read]")
    res, n = idle_fails(lambda k: False, raise_at=(50,))
    check("PyBEAR raising struct.error on an idle read: counted as a failed read, run completes",
          res['rc'] == 0 and fail_line(res).startswith('# idle failed reads: 1 '), fail_line(res))
    res, n = idle_fails(lambda k: 40 <= k < 44)
    check("4 failed in a row: completes", res['rc'] == 0 and 'reads: 4 ' in fail_line(res), fail_line(res))
    res, n = idle_fails(lambda k: 40 <= k < 45)
    el = end_line(res) or ''
    check("5 failed in a row: stops, ZERO writes, says so", '5 failed reads in a row' in el
          and not res['writes'], el[:100])
    res, n = idle_fails(lambda k: k > 30 and k % 4 == 0)
    el = end_line(res) or ''
    check("25 scattered failures: stops, ZERO writes", '25 in all' in el and not res['writes'], el[:100])
    res_clean, clean = idle_fails(lambda k: False)
    n_last = len(clean)                      # bulk_read is only used by preflight + idle: the last idle read
    check("a clean idle log adds 'idle failed reads: 0' to the header",
          fail_line(res_clean) == '# idle failed reads: 0', fail_line(res_clean))
    res, n = idle_fails(lambda k: k >= n_last - 1)
    el = end_line(res) or ''
    check("log ends on failures and the actuator still doesn't answer: no arming, ZERO writes",
          'not answering at arming' in el and not res['writes'] and len(n) == n_last + 1,
          f"{el[:100]}; reads {len(n)} (clean {n_last})")
    res, n = idle_fails(lambda k: k == n_last)
    check("log ends on ONE failure, the fresh read answers: arms and completes",
          res['rc'] == 0 and len(n) == n_last + 1 and 'reads: 1 ' in fail_line(res),
          f"{end_line(res)}; {fail_line(res)}")
    # ...and the fresh read gets the same checks as every idle read (review Nov 4)
    for why, fn, word in [
            ('arm off the stop', lambda v, e: ([math.radians(100.0)] + v[1:], e), 'left the top stop'),
            (f'temperature >= TEMP_WARN ({hb.TEMP_WARN} C)',
             lambda v, e: (v[:3] + [hb.TEMP_WARN + 0.5, v[4]], e), 'temperature'),
            ('error byte 0x90', lambda v, e: (v, 0x90), 'error byte 0x90')]:
        res, n = idle_fails(lambda k: k == n_last, alter={n_last + 1: fn})
        el = end_line(res) or ''
        check(f"log ends on a failure, the fresh read answers with {why}: ABORT before arming, "
              f"ZERO writes", 'fresh read before arming' in el and word in el and not res['writes']
              and len(n) == n_last + 1 and 'reads: 1 ' in fail_line(res),
              f"{el[:110]}; writes {len(res['writes'])}")
    ok_parse = True
    try:
        hb.parse_args(['--sweep', sweep, '--thermal', '--thermal-s', '1800'])
    except SystemExit:
        ok_parse = False
    check("--thermal-s 1800 accepted (was 600 max)", ok_parse)

    print("\n12. Level 2 rig (STATE 8 #31): measured y / friction / heating, adjustable static excess")
    sweep_l2 = os.path.join(tmp, 'sweep_l2.csv')
    make_sweep(sweep_l2, lo=60, hi=100, y_fn=l2_y, f_fn=l2_fc, bv=L2_BV)
    tab_l2 = ST.sweep_tables(sweep_l2)
    errs = [abs(r[2] - l2_y(math.radians(r[0] - PHI_DEG))) for r in tab_l2]
    # 0.05, not 0.03: the L2 curve is piecewise linear with kinks; 5-deg bin means across a
    # kink differ from the curve at the bin's mean angle by ~0.03
    check("L2 fake sweep: sweep_tables y within 0.05 N m of the fake's y", max(errs) < 0.05,
          f"max {max(errs):.3f}")
    L2_ARGS = ('--angles', '90,80,70', '--i-max', '8.5')          # the Oct 29 H1 L2 command

    def l2_run(*extra, up=0.0, down=0.0, J=0.015, angles=L2_ARGS):
        clock = Clock()
        fake = l2_fake(clock, J=J, gap_up=up, gap_down=down)
        return invoke(fake, bare_args(sweep_l2, *angles, *extra), clock), fake

    def l2_creep(res, k, deg):
        d = rows_np(res)
        m = (d['aidx'] == k) & (d['phase'] == 'creep')
        if m.sum() < 600 or d['t'][m].max() - d['t'][m].min() < 6:
            return None, None
        u, amp = creep_avg(d, k)
        return u + l2_y(math.radians(deg)), amp

    def quick(res, deg):
        return next((l for l in res['csv']['comments'] if l.startswith(f'# quick: {deg} deg')), '')

    res, fake = l2_run()
    check("L2, no static excess: Oct 29 command completes", res['rc'] == 0, end_line(res) or '')
    for k, deg in enumerate((90, 80, 70)):
        e, amp = l2_creep(res, k, deg)
        check(f"L2 no excess, {deg} deg: creep within 0.05 N m, slid (amp >= 0.8, not STUCK)",
              e is not None and abs(e) < 0.05 and amp >= 0.8 and 'STUCK' not in quick(res, deg)
              and 'last cycle p-p' in quick(res, deg), f"err {e}, amp {amp}; {quick(res, deg)[-40:]}")
    common_after_enable(res, "  [L2 no excess]", cap=7.8, raised=True)

    res, fake = l2_run(up=0.25)
    e70, amp70 = l2_creep(res, 2, 70)
    e80, _ = l2_creep(res, 1, 80)
    check("L2, 0.25 N m upward static excess (72-80 deg band): completes; 80 deg still reads "
          "within 0.05, 70 deg is biased low by > 0.1 (Oct 29: -0.42) -- friction beats the creep",
          res['rc'] == 0 and e80 is not None and abs(e80) < 0.05 and e70 is not None and e70 < -0.1,
          f"80: {e80}; 70: {e70} (amp {amp70})")

    res, fake = l2_run(up=0.47)
    el = end_line(res) or ''
    big = [b for b in fake.breakaways if b[2] > 1.75]
    check("L2, 0.47 N m upward excess (the Oct 29 breakaway: net 1.81 N m): the arm breaks free "
          "upward at > 1.75 N m net near 70 deg and the run ABORTS (target / speed)",
          big and 65 < big[0][1] < 75 and ('from the target' in el or 'speed' in el),
          f"{[(round(b[1], 1), round(b[2], 2)) for b in big[:2]]}; {el[:60]}")
    common_after_enable(res, "  [L2 breakaway abort]", cap=7.8, raised=True)
    lw = [w[2] for w in res['writes'] if w[1] == 'limit_i_max']
    check("  [L2 breakaway abort]: limit_i_max raised to 8.5, then restored to 5.5 after the "
          "confirmed disable", lw == [8.5, 5.5], str(lw))

    print("\n13. --creep-scale-f (step 3): creep amplitude follows the sweep's friction")
    for argv, why in [(bare_args(sweep, '--creep-scale-f', '0.3'), '--creep-scale-f below 0.5'),
                      (bare_args(sweep, '--creep-scale-f', '2.5'), '--creep-scale-f above 2'),
                      (bare_args(sweep, '--creep-scale-f', 'nan'), '--creep-scale-f NaN')]:
        clock = Clock()
        res = invoke(FakeBear(clock), argv, clock)
        check(f"refused: {why}", str(res['rc']).startswith('SystemExit') and res['ports'] == 0
              and not res['writes'], str(res['rc'])[:80])
    res, fake = l2_run('--creep-scale-f', '1.0')
    hdr = next((l for l in res['csv']['comments'] if 'creep amplitude per angle' in l), '')
    f70 = ST.interp_inside(tab_l2, 70 + PHI_DEG, 1)
    f90 = ST.interp_inside(tab_l2, 90 + PHI_DEG, 1)
    check("header lists each angle's amplitude = 1.0 x f/1.0 (70 deg bigger than 90 deg)",
          f"70 deg {f70:.2f}" in hdr and f"90 deg {f90:.2f}" in hdr and f70 > f90 + 0.1,
          hdr[:150])
    e90, a90 = l2_creep(res, 0, 90)
    e70, a70 = l2_creep(res, 2, 70)
    check("no excess: completes; the arm creeps more at 70 than at 90 deg; both within 0.05 N m",
          res['rc'] == 0 and a70 > a90 + 0.1 and abs(e70) < 0.05 and abs(e90) < 0.05,
          f"amp 90 {a90:.2f} / 70 {a70:.2f}; err {e90:+.3f} / {e70:+.3f}")
    res_d, _ = l2_run(up=0.25, down=0.25, angles=('--angles', '70', '--i-max', '8.5'))
    res_s, _ = l2_run('--creep-scale-f', '1.0', up=0.25, down=0.25,
                      angles=('--angles', '70', '--i-max', '8.5'))
    e_s, a_s = l2_creep(res_s, 0, 70)
    check("0.25 N m excess both ways at 70 deg: the default 1 deg creep aborts, the scaled one "
          "(1.35 deg) completes and reads within 0.1 N m",
          res_d['rc'] != 0 and res_s['rc'] == 0 and e_s is not None and abs(e_s) < 0.1,
          f"default: {(end_line(res_d) or '')[:45]}; scaled: {(end_line(res_s) or '')[:20]} err {e_s}")
    common_after_enable(res_s, "  [scaled creep]", cap=7.8, raised=True)
    res, _ = l2_run('--creep-scale-f', '1.0', up=0.47, angles=('--angles', '70', '--i-max', '8.5'))
    check("0.47 N m upward excess: the scaled creep still aborts (the overshoot is set by "
          "gap^2 / (J x rate limit), not the creep) -- and ends safely",
          res['rc'] != 0 and any(w in (end_line(res) or '') for w in ('from the target', 'speed')),
          end_line(res) or '')
    common_after_enable(res, "  [scaled creep, 0.47 excess]", cap=7.8, raised=True)
    a_hi = hb.parse_args(bare_args(sweep_l2, '--angles', '90,70', '--i-max', '8.5',
                                   '--creep-scale-f', '0.5', '--creep-deg', '1.0'))
    a_lo = hb.parse_args(bare_args(sweep_l2, '--angles', '90,70', '--i-max', '8.5',
                                   '--creep-scale-f', '2.0', '--creep-deg', '0.6'))
    check("scaled amplitudes are kept inside 0.5-2 deg (F0 0.5 -> 2.0 cap; F0 2 x 0.6 deg -> 0.5 floor)",
          a_hi.creep_amps == [2.0, 2.0] and a_lo.creep_amps == [0.5, 0.5],
          f"{a_hi.creep_amps} / {a_lo.creep_amps}")
    a_th = hb.parse_args(bare_args(sweep, '--thermal', '--creep-scale-f', '1.0'))
    check("--thermal ignores --creep-scale-f (no creep)", a_th.creep_amps is None)
    clock = Clock()
    res = invoke(FakeBear(clock), bare_args(sweep), clock)
    check("default (no --creep-scale-f): header says --creep-deg at every angle; quick lines "
          "carry 'last cycle p-p'", any('creep amplitude: --creep-deg at every angle' in l
                                          for l in res['csv']['comments'])
          and all('last cycle p-p' in quick(res, int(d)) for d in hb.DEFAULT_ANGLES))

    # STUCK marker: friction far above anything the hold can push, only while the creep runs
    clock = Clock()
    fake = FakeBear(clock)

    def stick_in_creep(x):
        run = RUNS[-1] if RUNS else None
        x.max_fric_override = 6.0 if (run is not None and getattr(run, '_phase_now', '') == 'creep') else None
    orig_log = hb.Run.log

    def log_spy(self, row):
        self._phase_now = hb.PHASES[int(row[2])]
        return orig_log(self, row)
    hb.Run.log = log_spy
    fake.on_bulk_rw = stick_in_creep
    try:
        res = invoke(fake, bare_args(sweep, '--angles', '80'), clock)
    finally:
        hb.Run.log = orig_log
    q = quick(res, 80)
    check("an arm that cannot slide during the creep: quick line says STUCK",
          'STUCK' in q, q[-70:])

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