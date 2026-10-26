"""
Offline test for observer_excitation.py --j-sine and fit_j_sine.py.
NO SERIAL PORT: Manager.BEAR is replaced with a function that raises before
bear_interface is imported, and BearInterface is replaced by a factory around
SineFake. Time is simulated (time.monotonic / time.sleep patched; every bus
transaction costs BUS_S, so position and current are read ~2 ms apart, as on
the bench). The real script file is run in-process (runpy), unmodified.

The fake actuator (Position Mode, mode 2, torque_enable 1):
  * setpoint follows goal_position through a 2nd-order filter, clamped to the
    WRITTEN limit_acc_max and limit_velocity_max (so a 2.5 Hz sine is
    flattened unless the limits are raised);
  * arm velocity tracks setpoint velocity + POS_KP*(setpoint - pos) with a
    VEL_TAU lag; motor torque = J*acc - y(pos) + friction + viscous, with
    y = TAU_SPRING_TABLE - RIG_MGL*sin(vest) (sim only: last value held above
    the table), friction F0 + F1*(pos - 65 deg) times a smoothed sign;
  * disabled: spring-driven, friction-limited (park releases into the stop).
Sensor noise on position / velocity / current.

Checks:
  1. argument guards refuse before any port is opened;
  2. a default sweep is unchanged (limits 1.0 / 5.0) and still parses with
     fit_j_release.sweep_tables; the new t_pos/t_vel/t_iq columns are there;
  3. --j-sine runs write limits 2.0 / 30.0, log ordered read timestamps, keep
     |iq| under the aborts, and the five pinned frequencies recover J
     (0.008, 0.015, 0.025) within 5 %, verdict PASS;
  4. J below the floor -> FAIL; a missing frequency -> INCOMPLETE;
     an aborted run -> INCOMPLETE; the cycle-start clock is biased.

Run from the repo root:
  ~/projects/bear_controller/venv/bin/python3 useful_tools/test_j_sine_offline.py
"""
import sys
sys.dont_write_bytecode = True

import builtins
import contextlib
import io
import math
import os
import random
import runpy
import shutil
import tempfile
import time
from unittest import mock

from pybear import Manager


def _no_real_bear(*a, **k):
    raise AssertionError("test tried to construct a real Manager.BEAR")


Manager.BEAR = _no_real_bear      # before bear_interface is imported

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from main_controller import bear_interface as bi                     # noqa: E402
from main_controller import config as cfg                            # noqa: E402
from main_controller.controller import (TAU_SPRING_TABLE, RIG_MGL,    # noqa: E402
                                        KT, PHI_DEG)
import fit_j_sine as FS                                               # noqa: E402
import fit_j_release as FR                                            # noqa: E402

REAL_BI = bi.BearInterface   # captured before any patching

SCRIPT = os.path.join(HERE, 'observer_excitation.py')
BUS_S = 0.001           # s per bus transaction (the ~1 ms reply grid)
SUB_S = 0.0002          # physics substep
TE_LAG_S = 0.001
SP_WN = 60.0            # rad/s, setpoint follower natural frequency
POS_KP = 12.0           # 1/s
VEL_TAU = 0.02          # s
F0, F1 = 0.85, 0.6      # N m, N m/rad: Coulomb friction and its angle slope
B_VISC = 0.05           # N m s/rad
POS_NOISE = math.radians(0.004)
VEL_NOISE = 0.026
IQ_NOISE = 0.02
TOP_STOP = math.radians(116.5)
CENTER = math.radians(65.0)


def spring_nm(act):
    vest = math.degrees(act) + PHI_DEG
    lo, hi = TAU_SPRING_TABLE[0], TAU_SPRING_TABLE[-1]
    if vest <= lo[0]:
        return lo[1]
    if vest >= hi[0]:
        return hi[1]
    for (d0, t0), (d1, t1) in zip(TAU_SPRING_TABLE, TAU_SPRING_TABLE[1:]):
        if d0 <= vest <= d1:
            return t0 + (vest - d0) / (d1 - d0) * (t1 - t0)


def y_nm(act):
    return spring_nm(act) - RIG_MGL * math.sin(math.radians(math.degrees(act) + PHI_DEG))


class Clock:
    def __init__(self):
        self.t = 5000.0
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


class SineFake:
    def __init__(self, clock, J, start_deg=116.5, seed=15, abort_iq_at=None):
        # start at rest on the top stop, as the bare arm is before every run
        self.clock, self.J = clock, J
        clock.sim = self
        self.pos, self.vel, self.iq = math.radians(start_deg), 0.0, 0.0
        self.sp, self.sp_v = 0.0, 0.0
        self.te = self.te_visible = 0
        self.te_change_at = None
        self.err = 0x80
        self.rng = random.Random(seed)
        self.regs = {'mode': 0, 'goal_position': 0.0, 'goal_iq': 0.0,
                     'limit_position_min': -0.0872665, 'limit_position_max': 2.18166,
                     'limit_velocity_max': 1.0, 'limit_acc_max': 5.0}
        self.writes = []
        self.max_iq = 0.0
        self.abort_iq_at = abort_iq_at       # sim time after enable to fake a current spike
        self.t_enable = None

    def step(self, h):
        r = self.regs
        a = SP_WN ** 2 * (r['goal_position'] - self.sp) - 2 * SP_WN * self.sp_v
        amax, vmax = r['limit_acc_max'], r['limit_velocity_max']
        a = max(-amax, min(amax, a))
        self.sp_v = max(-vmax, min(vmax, self.sp_v + a * h))
        self.sp += self.sp_v * h
        if self.te_change_at is not None and self.clock.t >= self.te_change_at:
            self.te_visible, self.te_change_at = self.te, None
        if self.te == 1 and r['mode'] == 2:
            v_cmd = max(-vmax, min(vmax, self.sp_v + POS_KP * (self.sp - self.pos)))
            acc = (v_cmd - self.vel) / VEL_TAU
            self.vel += acc * h
            self.pos += self.vel * h
            fr = (F0 + F1 * (self.pos - CENTER)) * math.tanh(self.vel / 0.02)
            tau = self.J * acc - y_nm(self.pos) + fr + B_VISC * self.vel
            self.iq = tau / KT
        else:
            self.iq = 0.0
            tau = y_nm(self.pos)
            if self.vel == 0.0:
                if abs(tau) > F0:
                    self.vel = (tau - math.copysign(F0, tau)) / self.J * h
            else:
                new = self.vel + (tau - math.copysign(F0, self.vel)) / self.J * h
                self.vel = 0.0 if new * self.vel < 0 else new
            self.pos += self.vel * h
        if self.pos >= TOP_STOP:
            self.pos, self.vel = TOP_STOP, min(self.vel, 0.0)

    def _read(self, name):
        if name == 'torque_enable':
            return self.te_visible
        if name == 'present_position':
            return self.pos + self.rng.gauss(0.0, POS_NOISE)
        if name == 'present_velocity':
            return self.vel + self.rng.gauss(0.0, VEL_NOISE)
        if name == 'present_iq':
            v = self.iq + self.rng.gauss(0.0, IQ_NOISE)
            if (self.abort_iq_at is not None and self.t_enable is not None
                    and self.clock.t - self.t_enable > self.abort_iq_at):
                v = -5.4
            self.max_iq = max(self.max_iq, abs(self.iq))
            return v
        if name == 'goal_position':
            return self.sp
        if name in ('winding_temperature', 'powerstage_temperature'):
            return 30.0
        if name == 'input_voltage':
            return 19.5
        return self.regs.get(name)

    def _write(self, name, value):
        self.writes.append((self.clock.t, name, value))
        if name == 'goal_position':
            if self.regs['mode'] in (2, 3) and \
                    self.regs['limit_position_min'] <= value <= self.regs['limit_position_max']:
                self.regs[name] = value
            return
        if name == 'torque_enable':
            if value == 1 and self.t_enable is None:
                self.t_enable = self.clock.t
            if value == 0:
                self.err = 0x80
            self.te = value
            self.te_change_at = self.clock.t + TE_LAG_S
            return
        self.regs[name] = value

    def __getattr__(self, attr):
        if attr.startswith('get_'):
            name = attr[4:]

            def getter(*ids):
                self.clock.advance(BUS_S)
                return [([self._read(name)], self.err) for _ in ids]
            return getter
        if attr.startswith('set_'):
            name = attr[4:]

            def setter(*pairs):
                self.clock.advance(BUS_S)
                for _, v in pairs:
                    self._write(name, v)
            return setter
        raise AttributeError(attr)

    def close_port(self):
        pass


def invoke(argv, fake=None, clock=None):
    """Run the real observer_excitation.py in-process. Returns dict."""
    made = []

    def factory():
        made.append(1)
        if fake is None:
            raise AssertionError("script opened the port")
        iface = REAL_BI.__new__(REAL_BI)
        iface.bear = fake
        iface.id = 1
        iface.last_disable_attempts = iface.last_disable_ms = None
        return iface

    logdir = tempfile.mkdtemp(prefix='jsine_')
    out = io.StringIO()
    patches = [mock.patch.object(bi, 'BearInterface', factory),
               mock.patch.object(cfg, 'LOG_DIR', logdir),
               mock.patch.object(builtins, 'input', lambda p='': print(p) or ''),
               mock.patch.object(sys, 'argv', ['observer_excitation.py'] + argv)]
    if clock is not None:
        patches += [mock.patch.object(time, 'monotonic', clock.monotonic),
                    mock.patch.object(time, 'sleep', clock.sleep)]
    rc = 0
    with contextlib.ExitStack() as st:
        for p in patches:
            st.enter_context(p)
        st.enter_context(contextlib.redirect_stdout(out))
        st.enter_context(contextlib.redirect_stderr(out))
        try:
            runpy.run_path(SCRIPT, run_name='__main__')
        except SystemExit as e:
            rc = e.code
        except Exception as e:                  # e.g. the port factory refused
            rc = f"exception: {e!r}"
    csvs = []
    for root, _, files in os.walk(logdir):
        csvs += [os.path.join(root, f) for f in files if f.endswith('.csv')]
    keep = None
    if csvs:
        keep = os.path.join(tempfile.gettempdir(), 'jsine_keep_' + os.path.basename(csvs[0]))
        n = 0
        while os.path.exists(keep):
            n += 1
            keep = keep.replace('.csv', f'_{n}.csv')
        shutil.copy(csvs[0], keep)
    shutil.rmtree(logdir)
    return dict(rc=rc, out=out.getvalue(), ports=len(made), csv=keep)


FAILS = []


def check(name, cond, detail=''):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ''))
    if not cond:
        FAILS.append(name)


BASE = ['--mode', 'sine', '--j-sine', '--arm-mass-kg', '0', '--arm-com-m', '0',
        '--center-deg', '65', '--amp-deg', '5', '--velocity', '0.10', '--soft-start-s', '0.3']
PLAN = [(0.5, 6), (1.0, 8), (1.5, 10), (2.0, 12), (2.5, 14)]


def jsine_runs(J, freqs=PLAN, abort_at=None):
    paths = []
    for f, cyc in freqs:
        clock = Clock()
        fake = SineFake(clock, J, abort_iq_at=abort_at if f == 2.5 else None)
        res = invoke(BASE + ['--freq-hz', f'{f:g}', '--cycles', str(cyc)], fake, clock)
        res['fake'] = fake
        paths.append(res)
    return paths


def main():
    print("1. Argument guards (no port may be opened)")
    bad = {
        'j-sine with sweep': ['--mode', 'sweep', '--j-sine', '--arm-mass-kg', '0', '--arm-com-m', '0'],
        'j-sine with a load': BASE[:4] + ['0.5'] + BASE[5:] + ['--freq-hz', '1'],
        'amp 8 deg': BASE[:10] + ['8'] + BASE[11:] + ['--freq-hz', '1'],
        'centre 50 deg': BASE[:8] + ['50'] + BASE[9:] + ['--freq-hz', '1'],
        'freq 3.5 Hz': BASE + ['--freq-hz', '3.5'],
        'amp 6 at 3 Hz (1.97 rad/s)': BASE[:10] + ['6'] + BASE[11:] + ['--freq-hz', '3'],
        'no --j-sine at 2.5 Hz (0.8 cap kept)': [a for a in BASE if a != '--j-sine'] + ['--freq-hz', '2.5'],
        'approach velocity 1.0 (0.8 cap kept)': BASE[:12] + ['1.0'] + BASE[13:] + ['--freq-hz', '1'],
    }
    for name, argv in bad.items():
        r = invoke(argv)
        check(f"refused: {name}", r['rc'] not in (0, None) and r['ports'] == 0,
              f"rc {r['rc']}, ports {r['ports']}")

    print("2. Default sweep unchanged")
    clock = Clock()
    fake = SineFake(clock, 0.015)
    r = invoke(['--mode', 'sweep', '--arm-mass-kg', '0', '--arm-com-m', '0',
                '--velocity', '0.10', '--soft-start-s', '0.3'], fake, clock)
    lim = {n: v for _, n, v in fake.writes if n in ('limit_velocity_max', 'limit_acc_max')}
    check("sweep completes", r['rc'] == 0 and r['csv'] is not None, f"rc {r['rc']}")
    check("sweep limits 1.0 / 5.0", lim == {'limit_velocity_max': 1.0, 'limit_acc_max': 5.0}, str(lim))
    if r['csv']:
        try:
            tab = FR.sweep_tables(r['csv'])
            check("sweep still parses (fit_j_release.sweep_tables)", len(tab) > 5, f"{len(tab)} bins")
        except SystemExit as e:
            check("sweep still parses (fit_j_release.sweep_tables)", False, str(e))
        hdr = open(r['csv']).read()
        check("new columns t_pos,t_vel,t_iq in header", ',t_pos,t_vel,t_iq' in hdr)
        os.remove(r['csv'])

    for J in (0.015, 0.008, 0.025):
        print(f"3. J sine runs, true J {J}")
        runs = jsine_runs(J)
        ok = all(x['rc'] == 0 and x['csv'] for x in runs)
        check("all five runs complete", ok, ', '.join(str(x['rc']) for x in runs))
        if not ok:
            print(runs[-1]['out'][-2000:])
            continue
        lim = {n: v for _, n, v in runs[-1]['fake'].writes if n in ('limit_velocity_max', 'limit_acc_max')}
        check("j-sine limits 2.0 / 30.0", lim == {'limit_velocity_max': 2.0, 'limit_acc_max': 30.0}, str(lim))
        check("max |iq| under the 5.0 A abort", max(x['fake'].max_iq for x in runs) < 5.0,
              f"{max(x['fake'].max_iq for x in runs):.2f} A")
        rows = FS.read_run(runs[-1]['csv'])
        dt_pi = rows['t_iq'] - rows['t_pos']
        check("t < t_pos < t_vel < t_iq, pos->iq ~2 ms",
              bool((rows['t_pos'] > rows['t']).all() and (dt_pi > 0).all())
              and 0.0015 < float(dt_pi.mean()) < 0.0025, f"mean {dt_pi.mean() * 1000:.2f} ms")
        text, lab, Jf = FS.run([x['csv'] for x in runs])
        check(f"J recovered within 5 % ({Jf:.5f})", abs(Jf / J - 1) < 0.05)
        exp = 'PASS' if FS.J_RANGE[0] <= J <= FS.J_RANGE[1] else 'FAIL'
        check(f"verdict {exp}", lab == exp, lab)
        if J == 0.015:
            print('\n'.join('     ' + l for l in text.splitlines()))
            per_c = [FS.analyse_run(FS.read_run(x['csv']), clock='cycle') for x in runs]
            import numpy as np
            Jc, *_ = FS.line_fit(np.array([p['w'] ** 2 for p in per_c]), np.array([p['K_in'] for p in per_c]))
            check("cycle-start clock is biased (> 5 %) -> timestamps matter", abs(Jc / J - 1) > 0.05,
                  f"{Jc:.5f}")
            text4, lab4, _ = FS.run([x['csv'] for x in runs[:-1]])
            check("missing 2.5 Hz -> INCOMPLETE", lab4 == 'INCOMPLETE', lab4)
        for x in runs:
            os.remove(x['csv'])

    print("4. J below the physical floor, and an aborted run")
    runs = jsine_runs(0.004)
    text, lab, Jf = FS.run([x['csv'] for x in runs if x['csv']])
    check(f"true J 0.004 -> FAIL (fit {Jf:.5f})", lab == 'FAIL', lab)
    for x in runs:
        if x['csv']:
            os.remove(x['csv'])
    runs = jsine_runs(0.015, abort_at=12.0)
    check("2.5 Hz run aborted on the current spike", runs[-1]['rc'] == 1, f"rc {runs[-1]['rc']}")
    paths = [x['csv'] for x in runs if x['csv']]
    text, lab, _ = FS.run(paths)
    check("aborted run -> INCOMPLETE", lab == 'INCOMPLETE', lab)
    for p in paths:
        os.remove(p)

    print()
    if FAILS:
        print(f"{len(FAILS)} CHECK(S) FAILED: " + '; '.join(FAILS))
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())