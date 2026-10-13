"""
Watchdog bench test T1-T5 (Block 3). THIS SCRIPT ENABLES TORQUE (Mode 0).

Setup: bare rig, arm resting on the TOP STOP (the spring holds it there),
actuator powered, nothing else running. Hands clear. Ctrl-C stops everything.

Pass criteria are pre-registered in PLAN.md (Block 3) and STATE.md section 5.
They are coded below exactly as written there; do not change them after a run.

  T1  write goal_iq = 0 every loop for 10 s       -> no watchdog error
  T2  stop writing (total silence)                -> error within timeout + 1 loop,
                                                     with bit 3 set
  T3  keep reading but stop writing               -> RECORD whether it trips
  T4  command -0.5 A, then stop writing           -> after the trip, |iq| < 0.1 A
                                                     (or damping only)
  T5  disable()                                   -> clears the error; re-arms cleanly
      (procedure set Oct 13 after run 1: disable(), then poll every 5 ms for up
       to 500 ms and record time-to-clear; run 1 read once at ~1 ms and FAILED)

Every loop uses ONE exchange() (one bulk packet) at 500 Hz, the pre-registered
loop transaction. Rows are kept in memory and written to CSV only at the end,
so file writes never stall the 2 ms loop.

Run from the repo root, at RT priority, pinned like the comm-budget run:
  cd ~/projects/bear_controller
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\
      ~/projects/bear_controller/useful_tools/watchdog_bench_test.py

Output: logs/phase2/watchdog/watchdog_test_<timestamp>.csv (raw rows) and a
printed PASS/FAIL/RECORD summary (also in the CSV header).
"""
import csv
import math
import os
import sys
import time

sys.dont_write_bytecode = True   # under sudo, leave no root-owned __pycache__

from main_controller.bear_interface import BearInterface
from main_controller.config import (LOG_DIR, WATCHDOG_TIMEOUT_MS,
                                    WATCHDOG_MIN_MS, WATCHDOG_MAX_MS)

# ---- Test constants (from the pre-registered plan) ---------------------------
PERIOD_S = 0.002                       # 500 Hz
TIMEOUT_S = WATCHDOG_TIMEOUT_MS / 1000.0
T2_SILENCE_S = TIMEOUT_S + PERIOD_S    # "within timeout + 1 loop" = 12 ms
T1_DURATION_S = 10.0
T3_READ_ONLY_S = 0.100                 # reads only for 10x the timeout
T4_IQ = -0.5                           # A; spring (~1 Nm) overpowers -0.34 Nm
T4_HOLD_S = 0.5
POST_TRIP_READ_S = 0.050               # reads after a trip, to check |iq|
CLEAR_WINDOW_S = 0.500                 # T5: how long disable() gets to clear a trip
CLEAR_POLL_S = 0.005                   # T5: poll every 5 ms, record time-to-clear
SETTLE_S = 0.5                         # feed goal_iq = 0 before each test
IQ_PASS_A = 0.1                        # T4 pass threshold

# ---- Software aborts (any of these -> stop and disable) ----------------------
MIN_POS_RAD = math.radians(100.0)      # arm must stay near the top stop
MAX_ABS_VEL = 1.0                      # rad/s; the arm should be still
MAX_ABS_IQ = 0.8                       # A; nothing here commands more than 0.5
MAX_TOTAL_S = 60.0

LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'watchdog')


class Abort(Exception):
    pass


def check_safe(state, t0_script):
    """Raise Abort if the arm left the stop, moved, or drew unexpected current."""
    if state['position'] < MIN_POS_RAD:
        raise Abort(f"position {math.degrees(state['position']):.1f} deg left the top stop")
    if abs(state['velocity']) > MAX_ABS_VEL:
        raise Abort(f"velocity {state['velocity']:.2f} rad/s; arm should be still")
    if abs(state['iq']) > MAX_ABS_IQ:
        raise Abort(f"iq {state['iq']:.2f} A over {MAX_ABS_IQ} A")
    if time.monotonic() - t0_script > MAX_TOTAL_S:
        raise Abort("script ran past its time cap")


class Bench:
    def __init__(self, iface):
        self.iface = iface
        self.rows = []
        self.t0 = time.monotonic()
        self.results = []

    # -- logging --------------------------------------------------------------
    def log(self, test, action, state, err, dt=None, note=''):
        self.rows.append([
            test, f"{time.monotonic() - self.t0:.6f}",
            '' if dt is None else f"{dt * 1000:.3f}", action,
            '' if state is None else f"{state['position']:.5f}",
            '' if state is None else f"{state['velocity']:.4f}",
            '' if state is None else f"{state['iq']:.4f}",
            '' if state is None or 'iq_sent' not in state else f"{state['iq_sent']:.3f}",
            '' if err is None else hex(err), note])

    def result(self, test, verdict, detail):
        line = f"{test}: {verdict} -- {detail}"
        print(line)
        self.results.append(line)

    # -- building blocks ------------------------------------------------------
    def arm_and_enable(self, test):
        wd = self.iface.configure_watchdog(WATCHDOG_TIMEOUT_MS)
        self.iface.enable()
        self.log(test, 'armed+enabled', None, 0x80, note=f"watchdog {wd} us")

    def feed(self, test, iq, duration):
        """Fixed-rate loop: one exchange(iq) every 2 ms. Returns dt list."""
        dts = []
        next_t = time.monotonic()
        last = next_t
        end = next_t + duration
        while True:
            now = time.monotonic()
            if now >= end:
                break
            state, err = self.iface.exchange(iq)
            t = time.monotonic()
            dt = t - last
            last = t
            dts.append(dt)
            if state is None:
                self.log(test, 'exchange FAILED', None, None, dt)
                raise Abort(f"{test}: exchange failed (all retries used)")
            self.log(test, 'exchange', state, err, dt)
            check_safe(state, self.t0)
            if self.iface.has_error(err) or self.iface.watchdog_tripped(err):
                raise Abort(f"{test}: error byte {hex(err)} while feeding")
            next_t += PERIOD_S
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()   # fell behind; don't try to catch up
        return dts

    def read_status(self, test, note):
        """One read (no write). Returns (state, err)."""
        values, err = self.iface.bear.bulk_read(
            [self.iface.id], self.iface.LOOP_READ_REGS)[0]
        if err is None or any(v is None for v in values):
            self.log(test, 'read FAILED', None, None, note=note)
            return None, None
        pos, vel, iq, temp = values
        state = {'position': pos, 'velocity': vel, 'iq': iq, 'temp': temp}
        self.log(test, 'read', state, err, note=note)
        return state, err

    def read_torque_enable(self, test):
        (te,), err = self.iface.bear.get_torque_enable(self.iface.id)[0]
        self.log(test, 'read torque_enable', None, err, note=f"torque_enable={te}")
        return te, err

    def _poll_clear(self, test, t_start, label):
        """Poll torque_enable + error byte every 5 ms for up to 500 ms.
        Returns ms-to-clear, or None if still latched."""
        while True:
            te, err = self.read_torque_enable(test)
            elapsed = time.monotonic() - t_start
            if te == 0 and err == 0x80:
                return elapsed * 1000
            if elapsed > CLEAR_WINDOW_S:
                self.log(test, f'{label}: still latched', None, err,
                         note=f"torque_enable={te}")
                return None
            time.sleep(CLEAR_POLL_S)

    def disable_and_check(self, test):
        """T5 check: disable() must clear the error within 500 ms.
        Records time-to-clear. If disable() doesn't clear it, tries one lone
        torque_enable = 0 (recorded, but T5 still FAILS). Returns True only
        if disable() itself cleared it."""
        name = 'T5' if test == 'T5' else f"T5 (after {test})"
        t = time.monotonic()
        self.iface.disable()
        ms = self._poll_clear(test, t, 'after disable()')
        if ms is not None:
            self.result(name, 'PASS', f"disable() cleared it; torque_enable 0 and "
                                      f"error 0x80 after {ms:.1f} ms")
            return True
        t = time.monotonic()
        self.iface.bear.set_torque_enable((self.iface.id, 0))
        ms = self._poll_clear(test, t, 'after lone torque_enable=0')
        self.result(name, 'FAIL',
                    "disable() did not clear it within "
                    f"{CLEAR_WINDOW_S*1000:.0f} ms; lone torque_enable = 0 "
                    + (f"cleared it after {ms:.1f} ms" if ms is not None
                       else "did not clear it either -- power-cycle needed"))
        return False

    # -- the tests ------------------------------------------------------------
    def t1_t2(self):
        # T1: feed goal_iq = 0 every loop for 10 s; no watchdog error allowed.
        self.arm_and_enable('T1')
        dts = self.feed('T1', 0.0, T1_DURATION_S)
        late = sum(1 for d in dts if d > TIMEOUT_S)
        dmax = max(dts) * 1000
        self.result('T1', 'PASS',
                    f"{len(dts)} loops, no watchdog error; max dt {dmax:.2f} ms; "
                    f"{late} loops with dt > {WATCHDOG_TIMEOUT_MS} ms")

        # T2: total silence for timeout + 1 loop, then ONE read.
        t_silent = time.monotonic()
        self.log('T2', 'silence start', None, None)
        time.sleep(T2_SILENCE_S)
        state, err = self.read_status('T2', note=f"after {T2_SILENCE_S*1000:.0f} ms silence")
        waited = (time.monotonic() - t_silent) * 1000
        te, _ = self.read_torque_enable('T2')
        tripped = self.iface.watchdog_tripped(err)
        self.result('T2', 'PASS' if tripped else 'FAIL',
                    f"bit 3 {'SET' if tripped else 'NOT set'} after {waited:.1f} ms "
                    f"silence (limit {T2_SILENCE_S*1000:.0f} ms + read time); error byte "
                    f"{'none' if err is None else hex(err)}; torque_enable={te} (recorded)")
        return self.disable_and_check('T2')

    def t3(self):
        # T3: keep READING (no writes) for 100 ms; record whether it trips.
        self.arm_and_enable('T3')
        self.feed('T3', 0.0, SETTLE_S)
        t_last_write = time.monotonic()
        trip_ms = None
        next_t = t_last_write
        while time.monotonic() - t_last_write < T3_READ_ONLY_S:
            state, err = self.read_status('T3', note='read only')
            if state is not None:
                check_safe(state, self.t0)
            if self.iface.watchdog_tripped(err) and trip_ms is None:
                trip_ms = (time.monotonic() - t_last_write) * 1000
                break
            next_t += PERIOD_S
            s = next_t - time.monotonic()
            if s > 0:
                time.sleep(s)
        if trip_ms is None:
            self.result('T3', 'RECORD',
                        f"no trip in {T3_READ_ONLY_S*1000:.0f} ms of reads only "
                        f"-> reads DO keep the watchdog alive")
        else:
            self.result('T3', 'RECORD',
                        f"tripped {trip_ms:.1f} ms after the last write despite reads "
                        f"-> only WRITES keep the watchdog alive")
        return self.disable_and_check('T3')

    def t4(self):
        # T4: hold -0.5 A, then stop writing; after the trip |iq| must be < 0.1 A.
        self.arm_and_enable('T4')
        self.feed('T4', 0.0, SETTLE_S)
        self.feed('T4', T4_IQ, T4_HOLD_S)
        self.log('T4', 'silence start', None, None)
        time.sleep(T2_SILENCE_S)
        t_end = time.monotonic() + POST_TRIP_READ_S
        tripped, iqs, vels = False, [], []
        while time.monotonic() < t_end:
            state, err = self.read_status('T4', note='after silence')
            if state is None:
                continue
            check_safe(state, self.t0)
            if self.iface.watchdog_tripped(err):
                tripped = True
                iqs.append(abs(state['iq']))
                vels.append(abs(state['velocity']))
            time.sleep(PERIOD_S)
        if not tripped:
            self.result('T4', 'FAIL', "watchdog did not trip after the silence")
        else:
            worst = max(iqs)
            ok = worst < IQ_PASS_A
            note = '' if ok else (f" -- check 'damping only': max |vel| "
                                  f"{max(vels):.3f} rad/s during those reads")
            self.result('T4', 'PASS' if ok else 'FAIL (review)',
                        f"after trip: max |iq| {worst:.3f} A over {len(iqs)} reads "
                        f"(need < {IQ_PASS_A} A){note}")
        return self.disable_and_check('T4')

    def t5_rearm(self):
        # T5: after the trips above, arm and enable again and feed cleanly.
        self.arm_and_enable('T5')
        self.feed('T5', 0.0, SETTLE_S)
        self.result('T5 re-arm', 'PASS', f"re-armed and fed {SETTLE_S}s with no error")
        return self.disable_and_check('T5')

    # -- output -----------------------------------------------------------------
    def save(self, preflight):
        made = []
        p = os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        path = os.path.join(LOG_SUBDIR,
                            f"watchdog_test_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            for line in preflight + self.results:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(['test', 't_s', 'dt_ms', 'action', 'pos_rad', 'vel_rad_s',
                        'iq_A', 'iq_sent_A', 'err', 'note'])
            w.writerows(self.rows)
        uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
        if os.geteuid() == 0 and uid and gid:
            for q in made + [LOG_SUBDIR, path]:
                try:
                    os.chown(q, int(uid), int(gid))
                except OSError as e:
                    print(f"WARNING: chown {q} failed: {e}")
        print(f"\nLog: {path}")


def preflight(iface):
    """Read-only checks before anything is enabled. Returns header lines."""
    lines = []
    (te,), err = iface.bear.get_torque_enable(iface.id)[0]
    if te != 0 or err != 0x80:
        raise SystemExit(f"REFUSED: torque_enable={te}, error byte {err}. "
                         f"Need a disabled actuator with no fault.")
    (lim,), _ = iface.bear.get_limit_i_max(iface.id)[0]
    if lim is None or lim > 5.5:
        raise SystemExit(f"REFUSED: limit_i_max reads {lim}; expected <= 5.5 A.")
    values, err = iface.bear.bulk_read([iface.id], iface.LOOP_READ_REGS)[0]
    if err is None:
        raise SystemExit("REFUSED: could not read state.")
    pos, vel, iq, temp = values
    if pos < MIN_POS_RAD:
        raise SystemExit(f"REFUSED: arm at {math.degrees(pos):.1f} deg. Rest it on "
                         f"the top stop (~116 deg) first.")
    lines += [f"started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
              f"watchdog {WATCHDOG_TIMEOUT_MS} ms (range {WATCHDOG_MIN_MS}-{WATCHDOG_MAX_MS})",
              f"loop {1/PERIOD_S:.0f} Hz; sched {os.sched_getscheduler(0)} "
              f"prio {os.sched_getparam(0).sched_priority} affinity "
              f"{sorted(os.sched_getaffinity(0))}",
              f"start: pos {math.degrees(pos):.2f} deg, vel {vel:.3f}, iq {iq:.3f}, "
              f"winding {temp:.1f} C, limit_i_max {lim}"]
    for l in lines:
        print(l)
    return lines


def main():
    # Pre-registered conditions: RT priority, pinned (same as Block 1).
    if os.sched_getscheduler(0) != os.SCHED_FIFO:
        raise SystemExit(
            "REFUSED: not running at real-time priority. From the repo root run:\n"
            "  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\\n"
            "      ~/projects/bear_controller/useful_tools/watchdog_bench_test.py")
    iface = BearInterface()
    bench = Bench(iface)
    header = []
    try:
        header = preflight(iface)
        if input("\nArm on the top stop, hands clear. Type GO to start: ").strip() != 'GO':
            raise SystemExit("Not started.")
        bench.t0 = time.monotonic()
        for test in (bench.t1_t2, bench.t3, bench.t4, bench.t5_rearm):
            if not test():
                raise Abort("a T5 check failed; remaining tests skipped "
                            "(can't re-arm from an uncleared trip)")
    except (Abort, RuntimeError) as e:
        print(f"\nABORT: {e}")
        bench.results.append(f"ABORT: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        bench.results.append("Interrupted by Ctrl-C")
    finally:
        try:
            iface.disable()
            # Disarm, so later scripts see the flash value (0 = off) again and
            # enable() keeps refusing until they arm the watchdog themselves.
            iface.bear.set_watchdog_timeout((iface.id, 0))
            (wd,), _ = iface.bear.get_watchdog_timeout(iface.id)[0]
            print(f"Disabled; watchdog reset to {wd} us.")
            bench.results.append(f"end: disabled, watchdog reset to {wd} us")
        finally:
            if bench.rows or header:
                bench.save(header)
            iface.bear.close_port()
    print("\n" + "\n".join(bench.results))


if __name__ == '__main__':
    main()