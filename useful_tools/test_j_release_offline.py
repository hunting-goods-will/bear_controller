"""
Offline test for j_release_test.py and the confirmed enable_position_mode().
NO SERIAL PORT: Manager.BEAR is replaced with a function that raises before
anything else is imported, and every BearInterface the script asks for is
built around FakeBear. Time is simulated (time.monotonic / time.sleep are
patched; each bus transaction costs BUS_S), so a full run takes seconds.

The fake actuator:
  * Position Mode (mode 2, torque_enable 1): tracks goal_position at up to
    limit_velocity_max; present_iq = (gravity - spring +- friction) / KT.
  * Disabled: the drive is the TAU_SPRING_TABLE spring minus RIG_MGL*sin(vest)
    gravity minus 0.85 Nm friction (also the static breakaway), J = 0.015.
    SIM ONLY: the table ends at actuator 104.6 deg; above that the last value
    is held so the arm can reach the stop. Top stop 116.5 deg, no bounce.
  * goal_position writes are IGNORED outside [limit_position_min,
    limit_position_max] or while mode is not 2/3 (SDK 1.2.0).
  * torque_enable read-back lags the write by TE_LAG_S; torque_enable = 0
    clears a latched fault.

Run from the repo root:
  ~/projects/bear_controller/venv/bin/python3 useful_tools/test_j_release_offline.py
"""
import sys
sys.dont_write_bytecode = True

import builtins
import contextlib
import csv
import io
import math
import os
import shutil
import tempfile
import time
from unittest import mock

from pybear import Manager


def _no_real_bear(*a, **k):
    raise AssertionError("test tried to construct a real Manager.BEAR")


Manager.BEAR = _no_real_bear      # before bear_interface is imported

from main_controller import bear_interface as bi                     # noqa: E402
from main_controller.config import MIN_ANGLE, MAX_ANGLE               # noqa: E402
from main_controller.controller import (TAU_SPRING_TABLE, RIG_MGL,    # noqa: E402
                                        KT, PHI_DEG)
import j_release_test as jr                                           # noqa: E402

BUS_S = 0.0005          # s per bus transaction
SUB_S = 0.0002          # physics substep
TE_LAG_S = 0.001        # torque_enable read-back lag
J = 0.015
FRICTION = 0.85
TOP_STOP = math.radians(116.5)
BOTTOM_STOP = 0.0
POS_KP = 12.0           # 1/s, Position Mode velocity command per rad of error
ACC_MAX = 5.0           # rad/s^2, firmware setpoint profile (limit_acc_max).
                        # Oct 14 probe: the goal_position read-back is a profiled
                        # setpoint that ramps to the written goal even while
                        # disabled; Position Mode tracks that setpoint.
VEL_TAU = 0.03          # s, velocity response


def spring_nm(act):
    vest = math.degrees(act) + PHI_DEG
    lo, hi = TAU_SPRING_TABLE[0], TAU_SPRING_TABLE[-1]
    if vest <= lo[0]:
        return lo[1]
    if vest >= hi[0]:
        return hi[1]           # SIM ONLY: hold the last table value
    for (d0, t0), (d1, t1) in zip(TAU_SPRING_TABLE, TAU_SPRING_TABLE[1:]):
        if d0 <= vest <= d1:
            return t0 + (vest - d0) / (d1 - d0) * (t1 - t0)


def gravity_nm(act):
    return RIG_MGL * math.sin(math.radians(math.degrees(act) + PHI_DEG))


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
    def __init__(self, clock, start_deg=116.5):
        self.clock = clock
        clock.sim = self
        self.pos = math.radians(start_deg)
        self.top = max(TOP_STOP, self.pos)   # a start above the stop rests there
        self.vel = 0.0
        self.sp, self.sp_v = 0.0, 0.0   # profiled setpoint (register starts at 0)
        self.iq = 0.0
        self.te = 0
        self.te_visible = 0
        self.te_change_at = None
        self.err = 0x80
        self.regs = {'mode': 0, 'goal_position': 0.0, 'goal_iq': 0.0,
                     'limit_position_min': -0.0872665,
                     'limit_position_max': 2.18166,
                     'limit_velocity_max': 1.0}
        self.writes = []          # (t, name, value, via, accepted)
        self.events = []          # (t, kind)  kind: 'bulk_read' | 'bulk_rw'
        self.enables = 0          # accepted torque_enable = 1 writes
        self.stuck_next_free = False
        self.stuck = False
        # Injection hooks
        self.drop_write = None    # fn(name, value) -> True to lose the write
        self.on_bulk_rw = None    # fn() called before each bulk_read_write
        self.iq_override = None   # fn() -> float or None
        self.err_override = None  # fn() -> int or None (reads only)

    # -- physics -------------------------------------------------------------
    def _update_sp(self, h):
        """Move the setpoint toward goal_position: accelerate at ACC_MAX up to
        limit_velocity_max, brake in time to stop on the goal."""
        g, v = self.regs['goal_position'], self.sp_v
        e = g - self.sp
        if abs(e) < 1e-9 and abs(v) < 1e-6:
            self.sp, self.sp_v = g, 0.0
            return
        d = 1.0 if e > 0 else -1.0
        if v * d > 0 and v * v / (2 * ACC_MAX) >= abs(e):
            v -= d * ACC_MAX * h
        else:
            v += d * ACC_MAX * h
        vmax = self.regs['limit_velocity_max']
        v = max(-vmax, min(vmax, v))
        new = self.sp + v * h
        if (g - new) * d <= 0:
            new, v = g, 0.0
        self.sp, self.sp_v = new, v

    def step(self, h):
        self._update_sp(h)
        if self.te_change_at is not None and self.clock.t >= self.te_change_at:
            self.te_visible, self.te_change_at = self.te, None
        if self.te == 1 and self.regs['mode'] == 2:
            vmax = self.regs['limit_velocity_max']
            v_cmd = max(-vmax, min(vmax, POS_KP * (self.sp - self.pos)))
            self.vel += (v_cmd - self.vel) * min(1.0, h / VEL_TAU)
            sgn = (self.vel > 0) - (self.vel < 0)
            self.iq = (gravity_nm(self.pos) - spring_nm(self.pos) + FRICTION * sgn) / KT
        else:
            self.iq = 0.0
            fric = 10.0 if self.stuck else FRICTION
            tau = spring_nm(self.pos) - gravity_nm(self.pos)
            if self.vel == 0.0:
                if abs(tau) > fric:
                    self.vel = (tau - math.copysign(fric, tau)) / J * h
            else:
                new = self.vel + (tau - math.copysign(fric, self.vel)) / J * h
                self.vel = 0.0 if new * self.vel < 0 else new
        self.pos += self.vel * h
        if self.pos >= self.top:
            self.pos, self.vel = self.top, min(self.vel, 0.0)
        if self.pos <= BOTTOM_STOP:
            self.pos, self.vel = BOTTOM_STOP, max(self.vel, 0.0)

    # -- registers -----------------------------------------------------------
    def _read(self, name):
        if name == 'torque_enable':
            return self.te_visible
        if name == 'present_position':
            return self.pos
        if name == 'goal_position':
            return self.sp
        if name == 'present_velocity':
            return self.vel
        if name == 'present_iq':
            o = self.iq_override() if self.iq_override else None
            return self.iq if o is None else o
        if name == 'input_voltage':
            return 24.0
        if name == 'winding_temperature':
            return 30.0
        if name == 'powerstage_temperature':
            return 32.0
        return self.regs.get(name)

    def _err(self):
        o = self.err_override() if self.err_override else None
        return self.err if o is None else o

    def _write(self, name, value, via):
        t = self.clock.t
        if self.drop_write and self.drop_write(name, value):
            self.writes.append((t, name, value, via, 'LOST'))
            return
        if name == 'goal_position':
            ok = (self.regs['mode'] in (2, 3) and self.regs['limit_position_min']
                  <= value <= self.regs['limit_position_max'])
            self.writes.append((t, name, value, via, 'ok' if ok else 'IGNORED'))
            if ok:
                self.regs[name] = value
            return
        self.writes.append((t, name, value, via, 'ok'))
        if name == 'torque_enable':
            if value == 1:
                self.enables += 1
                self.stuck = False
                if self.stuck_next_free:
                    self.stuck_next_free = False
                    self._stuck_pending = True
            else:
                self.err = 0x80                    # clears a latched fault
                if getattr(self, '_stuck_pending', False):
                    self.stuck, self._stuck_pending = True, False
            self.te = value
            self.te_change_at = self.clock.t + TE_LAG_S
        else:
            self.regs[name] = value

    def __getattr__(self, attr):
        if attr.startswith('get_'):
            name = attr[4:]

            def getter(*ids):
                self.clock.advance(BUS_S)
                return [([self._read(name)], self._err()) for _ in ids]
            return getter
        if attr.startswith('set_'):
            name = attr[4:]

            def setter(*pairs):
                self.clock.advance(BUS_S)
                for _, v in pairs:
                    self._write(name, v, 'single')
            return setter
        raise AttributeError(attr)

    def bulk_read(self, ids, regs):
        self.clock.advance(BUS_S)
        self.events.append((self.clock.t, 'bulk_read'))
        return [([self._read(r) for r in regs], self._err()) for _ in ids]

    def bulk_read_write(self, ids, regs, wnames, wdata):
        if self.on_bulk_rw:
            self.on_bulk_rw()
        self.clock.advance(BUS_S)
        self.events.append((self.clock.t, 'bulk_rw'))
        for n, v in zip(wnames, wdata[0]):
            self._write(n, v, 'bulk')
        return [([self._read(r) for r in regs], self._err()) for _ in ids]

    def close_port(self):
        pass


# ---------------------------------------------------------------------------
RUNS = []      # Run instances created by the script (for t0)


class _RecordingRun(jr.Run):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        RUNS.append(self)


jr.Run = _RecordingRun


def invoke(fake, argv, clock):
    """One script invocation against `fake`. Returns dict with rc/out/csv."""
    made = []

    def factory():
        made.append(1)
        iface = bi.BearInterface.__new__(bi.BearInterface)
        iface.bear = fake
        iface.id = 1
        iface.last_disable_attempts = iface.last_disable_ms = None
        return iface

    logdir = tempfile.mkdtemp(prefix='j_release_test_')
    out = io.StringIO()
    n_writes0 = len(fake.writes)
    RUNS.clear()
    with mock.patch.object(jr, 'BearInterface', factory), \
            mock.patch.object(jr, 'LOG_SUBDIR', logdir), \
            mock.patch.object(os, 'sched_getscheduler', return_value=os.SCHED_FIFO), \
            mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', clock.sleep), \
            mock.patch.object(builtins, 'input', lambda prompt='': print(prompt) or ''), \
            contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        try:
            rc = jr.main(argv)
        except SystemExit as e:
            rc = f"SystemExit({e.code!s:.200})"
    files = sorted(os.listdir(logdir))
    res = {'rc': rc, 'out': out.getvalue(), 'ports': len(made),
           'writes': fake.writes[n_writes0:], 'csv': None, 'run': RUNS[-1] if RUNS else None}
    if files:
        with open(os.path.join(logdir, files[0])) as f:
            lines = f.read().splitlines()
        hdr = [l for l in lines if l.startswith('#')]
        body = [l for l in lines if not l.startswith('#')]
        rows = list(csv.DictReader(body))
        res['csv'] = {'comments': hdr, 'rows': rows, 'name': files[0]}
    shutil.rmtree(logdir)
    return res


def free_phase_writes(res, fake):
    """Writes whose time falls inside any free phase (first..last free row)."""
    run, rows = res['run'], res['csv']['rows']
    spans = {}
    for r in rows:
        if r['phase'] == 'free':
            t = float(r['t']) + run.t0
            a, b = spans.get(r['release_idx'], (t, t))
            spans[r['release_idx']] = (min(a, t), max(b, t))
    return [w for w in res['writes'] for a, b in spans.values() if a - 1e-9 <= w[0] <= b + 1e-9]


def end_line(res):
    return next((l for l in res['csv']['comments'] if l.startswith('# END')), None) \
        if res['csv'] else None


def compress(writes):
    """Run-length list of (name, value-or-range, via, status)."""
    out = []
    for t, n, v, via, st in writes:
        key = (n, via, st) if n == 'goal_position' and via == 'bulk' else (n, v, via, st)
        if out and out[-1][0] == key:
            out[-1][1] += 1
            out[-1][3] = v
        else:
            out.append([key, 1, v, v])
    lines = []
    for key, k, v0, v1 in out:
        n, via, st = key[0], key[-2], key[-1]
        if n == 'goal_position' and via == 'bulk':
            val = f"{math.degrees(v0):.2f}->{math.degrees(v1):.2f} deg"
        elif n == 'goal_position':
            val = f"{v0:.5f} rad ({math.degrees(v0):.2f} deg)"
        else:
            val = f"{v0:g}"
        lines.append(f"{n} = {val} [{via}{'' if st == 'ok' else ', ' + st}]"
                     f"{f' x{k}' if k > 1 else ''}")
    return lines


RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(f"    [{'PASS' if cond else 'FAIL'}] {name}{(' -- ' + detail) if detail else ''}")


def common_checks(res, fake, expect_enable):
    rows = res['csv']['rows'] if res['csv'] else []
    check("no write during any free phase", res['csv'] is None or not free_phase_writes(res, fake))
    goals = [w[2] for w in res['writes'] if w[1] == 'goal_position']
    check("every goal_position write within [MIN_ANGLE, MAX_ANGLE]",
          all(MIN_ANGLE <= g <= MAX_ANGLE for g in goals))
    nbulk = sum(1 for r in rows if r['phase'] != 'release')
    rw = sum(1 for w in res['writes'] if w[3] == 'bulk')
    check("one bulk packet per logged cycle; bulk writes only on enabled cycles",
          rw == sum(1 for r in rows if r['phase'] in ('approach', 'hold')),
          f"{nbulk} cycles, {rw} bulk goal writes")
    if expect_enable:
        last_te = [w for w in res['writes'] if w[1] == 'torque_enable'][-1]
        check("last torque_enable write is 0 and accepted",
              last_te[2] == 0 and last_te[4] == 'ok')
    te1 = [w for w in res['writes'] if w[1] == 'torque_enable' and w[2] == 1]
    return te1


def show(res, n_footer=20):
    for l in res['csv']['comments'] if res['csv'] else []:
        if l.startswith('# release') or l.startswith('# END'):
            print('     ', l[2:])


def scenario(title):
    print(f"\n=== {title} ===")


def main():
    # 1. Normal 15 releases, as three invocations (1, 2, 2), same fake.
    scenario("1. normal run: 15 releases as --repeats 1, 2, 2")
    clock = Clock()
    fake = FakeBear(clock)
    first = None
    for reps in (1, 2, 2):
        res = invoke(fake, ['--repeats', str(reps)], clock)
        first = first or res
        print(f"  --repeats {reps}: rc {res['rc']}, {res['csv']['name']}")
        show(res)
        check("complete", end_line(res).startswith('# END: complete'))
        common_checks(res, fake, True)
        foot = [l for l in res['csv']['comments'] if l.startswith('# release')]
        check(f"{3 * reps} release lines, none stuck, all disable attempts 1",
              len(foot) == 3 * reps and all('stuck no' in l and 'attempts 1 ' in l
                                            for l in foot))
        dts = [float(r['loop_dt']) for r in res['csv']['rows'] if r['loop_dt']]
        check("loop dt", max(dts) < 0.0025, f"median {sorted(dts)[len(dts)//2]*1000:.2f} ms, "
              f"max {max(dts)*1000:.2f} ms")
    WRITE_LIST = compress(first['writes'])

    # 2. Lost goal_position write inside the confirmed enable.
    scenario("2. lost goal_position write in the confirmed enable (release 1)")
    clock = Clock()
    fake = FakeBear(clock)
    lost = {'n': 0}

    def drop_goal(name, value):
        if name == 'goal_position' and lost['n'] == 0:
            lost['n'] = 1
            return True
        return False
    fake.drop_write = drop_goal
    res = invoke(fake, ['--repeats', '1'], clock)
    show(res)
    te1 = common_checks(res, fake, True)
    check("abort names the goal_position read-back",
          'goal_position reads' in end_line(res), end_line(res))
    check("torque_enable = 1 never written", not te1)
    check("motor never enabled in the fake", fake.enables == 0)
    print("    writes:", '; '.join(compress(res['writes'])[-6:]))

    # 3. torque_enable = 0 that takes 2 attempts (release 2).
    scenario("3. torque_enable = 0 needs 2 attempts at release 2")
    clock = Clock()
    fake = FakeBear(clock)
    state = {'done': False}

    def drop_te0(name, value):
        if (name == 'torque_enable' and value == 0 and fake.enables == 2
                and not state['done']):
            state['done'] = True
            return True
        return False
    fake.drop_write = drop_te0
    res = invoke(fake, ['--repeats', '1'], clock)
    show(res)
    common_checks(res, fake, True)
    foot = [l for l in res['csv']['comments'] if l.startswith('# release')]
    check("complete", end_line(res).startswith('# END: complete'))
    check("release 2 reports attempts 2", 'attempts 2 ' in foot[1], foot[1][2:90])
    check("releases 1 and 3 report attempts 1",
          'attempts 1 ' in foot[0] and 'attempts 1 ' in foot[2])

    # 4. Stuck release (release 2, 75 deg).
    scenario("4. stuck release at release 2 (friction 10 Nm in that free phase)")
    clock = Clock()
    fake = FakeBear(clock)
    orig_write = fake._write

    def write_hook(name, value, via):
        if name == 'torque_enable' and value == 1 and fake.enables == 1:
            fake.stuck_next_free = True      # the 2nd enable's release sticks
        orig_write(name, value, via)
    fake._write = write_hook
    res = invoke(fake, ['--repeats', '1'], clock)
    show(res)
    common_checks(res, fake, True)
    foot = [l for l in res['csv']['comments'] if l.startswith('# release')]
    check("complete", end_line(res).startswith('# END: complete'))
    check("release 2 stuck YES, no rest", 'stuck YES' in foot[1] and 'none' in foot[1])
    check("releases 1 and 3 not stuck", 'stuck no' in foot[0] and 'stuck no' in foot[2])
    r3 = [r for r in res['csv']['rows'] if r['release_idx'] == '3' and r['phase'] == 'approach']
    check("release 3 approach starts from ~75 deg",
          abs(math.degrees(float(r3[0]['pos'])) - 75) < 1.0,
          f"{math.degrees(float(r3[0]['pos'])):.2f} deg")

    # 5. Over-current mid-hold, sustained 5.1 A (release 2).
    for label, amps, expect in (("5. over-current mid-hold, sustained 5.1 A",
                                 5.1, 'for 3 consecutive'),
                                ("5b. over-current mid-hold, single 5.4 A sample",
                                 5.4, 'immediate trip')):
        scenario(label + " (release 2)")
        clock = Clock()
        fake = FakeBear(clock)
        cnt = {'n': 0, 'fired': False}

        def iq_hook(amps=amps, cnt=cnt, fake=fake):
            if (fake.enables == 2 and fake.te == 1
                    and abs(fake.regs['goal_position'] - math.radians(75)) < 1e-12):
                cnt['n'] += 1
                if cnt['n'] > 100 and (amps > 5.3 and not cnt['fired'] or amps < 5.3):
                    cnt['fired'] = True
                    return amps
            return None
        fake.iq_override = iq_hook
        res = invoke(fake, ['--repeats', '1'], clock)
        show(res)
        te1 = common_checks(res, fake, True)
        check(f"abort: {expect}", expect in end_line(res), end_line(res))
        last = [r for r in res['csv']['rows'] if r['phase'] != 'release'][-1]
        check("last logged row is the hold sample that tripped",
              last['phase'] == 'hold' and abs(float(last['iq'])) >= 5.0,
              f"phase {last['phase']}, iq {last['iq']}")
        check("no enable after the abort", len(te1) == 2)
        check("arm released (fake disabled)", fake.te == 0)

    # 6. Ctrl-C mid-approach (release 2).
    scenario("6. Ctrl-C mid-approach (release 2, 300th cycle)")
    clock = Clock()
    fake = FakeBear(clock)
    k = {'n': 0}

    def ctrl_c():
        if fake.enables == 2:
            k['n'] += 1
            if k['n'] == 300:
                raise KeyboardInterrupt
    fake.on_bulk_rw = ctrl_c
    res = invoke(fake, ['--repeats', '1'], clock)
    show(res)
    te1 = common_checks(res, fake, True)
    check("END is Ctrl-C", end_line(res) == '# END: abort -- Ctrl-C')
    last = [r for r in res['csv']['rows'] if r['phase'] != 'release'][-1]
    check("last row is release 2 approach",
          last['phase'] == 'approach' and last['release_idx'] == '2',
          f"{last['phase']} r{last['release_idx']} at {math.degrees(float(last['pos'])):.1f} deg")
    after = [w for w in res['writes'] if w[0] > te1[-1][0]]
    check("after Ctrl-C: goal_iq 0 then torque_enable 0, nothing else",
          [(w[1], w[2]) for w in after if w[3] == 'single'] == [('goal_iq', 0.0), ('torque_enable', 0)])

    # 7. Plan over 120 s.
    scenario("7. plan over 120 s (--repeats 5)")
    clock = Clock()
    fake = FakeBear(clock)
    res = invoke(fake, ['--repeats', '5'], clock)
    msg = [l for l in res['out'].splitlines() if 'run cap' in l]
    print("     ", msg[0] if msg else res['out'][-300:])
    check("refused (SystemExit)", str(res['rc']).startswith('SystemExit'))
    check("port never opened, no CSV", res['ports'] == 0 and res['csv'] is None)
    check("--repeats 4 (108 s) would be accepted",
          jr.planned_time([90, 75, 60], 4, 0.3) <= jr.RUN_CAP_S,
          f"{jr.planned_time([90, 75, 60], 4, 0.3):.1f} s; repeats 5 = "
          f"{jr.planned_time([90, 75, 60], 5, 0.3):.1f} s")

    # 8. Start at 118.5 deg.
    scenario("8. start at 118.5 deg")
    clock = Clock()
    fake = FakeBear(clock, start_deg=118.5)
    res = invoke(fake, ['--repeats', '1'], clock)
    msg = [l for l in res['out'].splitlines() if 'REFUSED' in l] or [str(res['rc'])]
    print("     ", msg[0][:200])
    check("refused at preflight", 'REFUSED: arm at 118.50 deg' in str(res['rc']))
    check("no register write at all", res['writes'] == [])
    check("no CSV", res['csv'] is None)

    # 9. Error bits in a free phase (release 2): finish that drop, stop.
    scenario("9. error bit 2 (0x84) during the release 2 free phase")
    clock = Clock()
    fake = FakeBear(clock)

    def err_hook():
        if fake.enables == 2 and fake.te == 0 and fake.pos > math.radians(100):
            return 0x84
        return None
    fake.err_override = err_hook
    res = invoke(fake, ['--repeats', '1'], clock)
    show(res)
    te1 = common_checks(res, fake, True)
    check("stopped after the drop", 'stopped after the drop' in end_line(res), end_line(res))
    check("no re-enable after release 2", len(te1) == 2)
    foot = [l for l in res['csv']['comments'] if l.startswith('# release')]
    check("release 2 footer present and names the error", len(foot) == 2 and '0x84' in foot[1])

    print("\n=== Register writes, invocation 1 of scenario 1 (--repeats 1), in order ===")
    for l in WRITE_LIST:
        print("   ", l)

    bad = [n for n, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(bad)}/{len(RESULTS)} checks passed")
    for n in bad:
        print("  FAILED:", n)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())