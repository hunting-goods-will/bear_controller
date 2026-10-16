"""
J release test (Block 4). THIS SCRIPT ENABLES TORQUE (Position Mode).

Setup: bare rig, arm resting on the TOP STOP (~116.5 deg; the spring holds it
there), foam pad on the stop, actuator powered, nothing else running. Hands
clear. Ctrl-C stops everything.

Each release: Position Mode drives the arm DOWN to a target height, holds it
until settled, then disable() lets go and the spring snaps the arm up into the
top stop. The free phase is logged with READS ONLY, for fitting the device
inertia J offline.

  1. enable_position_mode() at the present position (confirmed enable).
  2. approach: ramp goal_position to the target at --approach-velocity, soft
     start/stop SOFT_START_S (observer_excitation.py's run_leg ramp).
  3. hold: until the position has moved less than SETTLE_SPAN_DEG over the
     last SETTLE_S, after at least MIN_HOLD_S. Not settled within MAX_HOLD_S ->
     abort. (Position, not the velocity register: at a Position-Mode hold the
     register jitters with SD ~0.026 rad/s while the position is still to
     ~0.004 deg -- Sep 30 dwell. The Oct 14 shakedown never "settled" on the
     old |vel| < 0.02 rule.)
  4. release: iface.disable() (confirmed by read-back, up to 3 attempts).
     Not confirmed -> abort.
  5. free: reads only (no writes of any kind) until pos >= FREE_REST_DEG and
     the position has moved less than SETTLE_SPAN_DEG over SETTLE_S, or
     FREE_MAX_S after the release command.
     Moved < STUCK_DEG in the first STUCK_WINDOW_S -> marked "stuck", and the
     run continues from wherever the arm is.

Releases run in --heights order, repeated --repeats times, interleaved
(90,75,60,90,75,60,...). Heights must be within 30-100 deg.

RUN IT AS THREE INVOCATIONS. The planned-time check charges every release the
worst case (approach from MAX_ANGLE, the full MAX_HOLD_S hold, the full
FREE_MAX_S free phase) and refuses anything over RUN_CAP_S = 120 s. That is
27.0 s per 90,75,60 set at 0.3 rad/s, so --repeats 5 (135 s) is refused. The
15 releases are run as --repeats 1, then 2, then 2, same order each time.

ABORTS (any -> iface.disable() immediately, NO park; every planned release
already snaps the arm from >= 60 deg into the top stop, so an abort release is
the same motion). Checked every enabled cycle: error byte != 0x80;
|iq| > IQ_TRIP_A once or > IQ_ABORT_A for IQ_ABORT_COUNT consecutive cycles;
|goal - pos| > TRACKING_ABORT_DEG; max(winding, powerstage) >= TEMP_MAX;
failed bulk call; loop dt > MAX_LOOP_DT_S; > RUN_CAP_S since the first
enable; Ctrl-C. Free phase: error bits 2-6 or a failed read -> finish logging
that drop, then stop (no re-enable).

RUN
---
Standing rule: run test_connection.py first. Then, from the repo root:
  python main_controller/test_connection.py
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 useful_tools/j_release_test.py --repeats 1
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 useful_tools/j_release_test.py --repeats 2
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 useful_tools/j_release_test.py --repeats 2

Offline test (no serial port, fake actuator):
  ~/projects/bear_controller/venv/bin/python3 useful_tools/test_j_release_offline.py

Output: logs/phase2/j_release/j_release_<YYYYmmdd_HHMMSS>.csv, written after
the final disable (also on abort), chowned to SUDO_UID:SUDO_GID. '#' header,
one row per bus cycle, '#' footer with one line per release and an END line.
"""
import sys
sys.dont_write_bytecode = True   # under sudo, leave no root-owned __pycache__

import argparse
import csv
import math
import os
import subprocess
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import (LOG_DIR, MIN_ANGLE, MAX_ANGLE, TEMP_WARN,
                                    TEMP_MAX, SAFETY_CHECKS_CONFIRMED)

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'j_release')

# ---- Release plan ------------------------------------------------------------
DEFAULT_HEIGHTS = (90.0, 75.0, 60.0)
DEFAULT_REPEATS = 5     # refused by the plan check; see the docstring
HEIGHT_MIN_DEG, HEIGHT_MAX_DEG = 30.0, 100.0
DEFAULT_APPROACH_V = 0.3     # rad/s
MAX_APPROACH_V = 0.5         # rad/s

# ---- Approach ramp (observer_excitation.py's run_leg) ------------------------
SOFT_START_S = 0.5           # s, ramp time at each end of the approach
RAMP_FLOOR = 0.15            # minimum ramp; without it the step decays to zero
ARRIVE_EPS = math.radians(0.5)

# ---- Hold / free phase rules -------------------------------------------------
SETTLE_SPAN_DEG = 0.1        # "still" = position moved less than this...
SETTLE_S = 0.3               # ...over the last SETTLE_S (mean |v| < 0.006 rad/s)
MIN_HOLD_S = 0.5
MAX_HOLD_S = 3.0             # not settled by then -> abort
HOLD_IQ_WINDOW_S = 0.2       # footer: mean iq over the last 0.2 s of hold
FREE_REST_DEG = 110.0        # free phase ends at rest at/above this...
FREE_MAX_S = 2.5             # ...or this long after the release command
STUCK_DEG = 1.0              # moved less than this...
STUCK_WINDOW_S = 1.0         # ...in this long after release -> "stuck"

# ---- Start position (preflight) ----------------------------------------------
REST_MIN_DEG = 110.0
REST_MAX_DEG = math.degrees(MAX_ANGLE)   # 118. Firmware ignores a goal above
                                         # limit_position_max, so the confirmed
                                         # enable could not hold the arm there.

# ---- Aborts (observer_excitation.py values) ----------------------------------
RUN_CAP_S = 120.0            # s, planned (worst case) and since first enable
IQ_ABORT_A = 5.0             # A, sustained...
IQ_ABORT_COUNT = 3           # ...for this many consecutive cycles
IQ_TRIP_A = 5.3              # A, any single cycle
TRACKING_ABORT_DEG = 15.0
MAX_LOOP_DT_S = 0.050

# ---- Loop --------------------------------------------------------------------
PERIOD_S = 0.002             # 500 Hz
MIN_SLEEP_S = 0.0005         # always sleep at least this, even on overrun
READ_REGS = ['present_position', 'present_velocity', 'present_iq',
             'input_voltage', 'winding_temperature', 'powerstage_temperature']

# ---- Settings: observer_excitation.py's block --------------------------------
IQ_P, IQ_I, IQ_D = 0.02, 0.02, 0.0
VEL_P, VEL_I, VEL_D = 4.5, 0.001, 0.0
POS_P, POS_I, POS_D = 5.0, 0.0, 0.2
LIMIT_I_MAX = 5.5
LIMIT_VELOCITY_MAX = 1.0
LIMIT_ACC_MAX = 5.0
# Firmware position limit for THIS script only (Oct 15). With 118 deg (config
# MAX_ANGLE) the arm's impact on the top stop overshot to 120.18 deg and tripped
# the Joint Limit error (0x90) in the free phase, after the fit window. 125 deg
# (2.1817 rad, the value already in flash) clears the overshoot. Goals are still
# clamped in software to MIN_ANGLE..MAX_ANGLE, and the start must rest <= 118.
FW_LIMIT_POSITION_MAX = math.radians(125.0)
# Written in this order while disabled, then every one read back and compared
# before any enable. No save_config.
SETTINGS = [
    ('p_gain_iq', IQ_P), ('i_gain_iq', IQ_I), ('d_gain_iq', IQ_D),
    ('p_gain_id', IQ_P), ('i_gain_id', IQ_I), ('d_gain_id', IQ_D),
    ('p_gain_velocity', VEL_P), ('i_gain_velocity', VEL_I),
    ('d_gain_velocity', VEL_D), ('p_gain_position', POS_P),
    ('i_gain_position', POS_I), ('d_gain_position', POS_D),
    ('limit_i_max', LIMIT_I_MAX),
    ('limit_velocity_max', LIMIT_VELOCITY_MAX),
    ('limit_acc_max', LIMIT_ACC_MAX),
    ('limit_position_min', MIN_ANGLE),
    ('limit_position_max', FW_LIMIT_POSITION_MAX),
]
READBACK_RTOL, READBACK_ATOL = 1e-4, 1e-6   # float32 registers

COLUMNS = ['t', 'loop_dt', 'release_idx', 'target_deg', 'phase',
           'torque_enabled', 'goal_pos', 'pos', 'vel', 'iq', 'input_voltage',
           'w_temp', 'p_temp', 'err']


class Abort(Exception):
    """A runtime safety check tripped -> disable immediately, no park."""


class DropStop(Exception):
    """Error bits or a failed read in a free phase: the drop was logged to the
    end; stop without re-enabling."""


def _git(*argv):
    # -c safe.directory: under sudo, git refuses a repo owned by another user.
    try:
        r = subprocess.run(['git', '-c', f'safe.directory={REPO_DIR}',
                            '-C', REPO_DIR, *argv],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else 'unavailable'
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


def leg_time(dist_rad, velocity):
    """Conservative approach duration: cruise plus both soft ramps."""
    return dist_rad / velocity + 2.0 * SOFT_START_S


def planned_time(heights_deg, repeats, velocity):
    """Worst case: every approach starts at MAX_ANGLE, every hold runs the
    full MAX_HOLD_S, every free phase the full FREE_MAX_S."""
    per_set = sum(leg_time(MAX_ANGLE - math.radians(h), velocity)
                  + MAX_HOLD_S + FREE_MAX_S for h in heights_deg)
    return per_set * repeats


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--heights', default=','.join(f'{h:g}' for h in DEFAULT_HEIGHTS),
                    help='release heights, actuator deg, in order (default 90,75,60)')
    ap.add_argument('--repeats', type=int, default=DEFAULT_REPEATS,
                    help=f'times through --heights (default {DEFAULT_REPEATS}; '
                         f'see the docstring)')
    ap.add_argument('--approach-velocity', type=float, default=DEFAULT_APPROACH_V,
                    help=f'rad/s (default {DEFAULT_APPROACH_V}, max {MAX_APPROACH_V})')
    a = ap.parse_args(argv)
    try:
        a.heights = [float(h) for h in a.heights.split(',') if h.strip()]
    except ValueError:
        ap.error('--heights must be comma-separated numbers')
    if not a.heights:
        ap.error('--heights is empty')
    for h in a.heights:
        # NaN fails this comparison too, so it is refused here.
        if not HEIGHT_MIN_DEG <= h <= HEIGHT_MAX_DEG:
            ap.error(f'height {h} deg is outside {HEIGHT_MIN_DEG:.0f}-'
                     f'{HEIGHT_MAX_DEG:.0f} deg')
    if a.repeats < 1:
        ap.error('--repeats must be >= 1')
    if not 0.0 < a.approach_velocity <= MAX_APPROACH_V:
        ap.error(f'--approach-velocity must be in (0, {MAX_APPROACH_V}] rad/s')
    a.planned_s = planned_time(a.heights, a.repeats, a.approach_velocity)
    if a.planned_s > RUN_CAP_S:
        ap.error(f'planned run {a.planned_s:.1f} s (worst case: approach from '
                 f'{REST_MAX_DEG:.0f} deg, {MAX_HOLD_S:.1f} s hold, '
                 f'{FREE_MAX_S:.1f} s free per release) exceeds the '
                 f'{RUN_CAP_S:.0f} s run cap. Split the run into several '
                 f'invocations with a smaller --repeats.')
    return a


class Run:
    def __init__(self, iface, args):
        self.iface = iface
        self.args = args
        self.t0 = time.monotonic()        # 't' column = s since script start
        self.rows = []
        self.header = []
        self.footer = []                  # per-release lines
        self.summary = []                 # same, for printing at the end
        self.release_idx = 0
        self.target_deg = None
        self.torque_enabled = 0           # as the script believes
        self.enable_attempted = False
        self.t_first_enable = None
        self.t_prev = None                # loop_dt reference
        self.next = None                  # loop deadline
        self.iq_over = 0
        self.temp_warned = False
        self.last = None                  # last good state dict

    # -- bus cycle ---------------------------------------------------------------
    def reset_timing(self):
        """After enable/disable (several single transactions): restart the
        loop clock, so loop_dt and its abort only measure cycle-to-cycle gaps
        inside a phase."""
        self.t_prev = None
        self.next = time.monotonic()

    def cycle(self, phase, goal):
        """ONE bulk packet: write goal_position and read state (goal given),
        or read only (goal None). Logs the row. Returns (state|None, err,
        now, loop_dt)."""
        bear, i = self.iface.bear, self.iface.id
        if goal is not None:
            goal = max(MIN_ANGLE, min(MAX_ANGLE, goal))
            values, err = bear.bulk_read_write([i], READ_REGS, ['goal_position'],
                                               [[goal]])[0]
        else:
            values, err = bear.bulk_read([i], READ_REGS)[0]
        now = time.monotonic()
        loop_dt = None if self.t_prev is None else now - self.t_prev
        self.t_prev = now
        ok = err is not None and values is not None and all(v is not None for v in values)
        state = None
        if ok:
            pos, vel, iq, vin, tw, tp = values
            state = {'pos': pos, 'vel': vel, 'iq': iq, 'vin': vin, 'tw': tw, 'tp': tp}
            self.last = state
        self.rows.append([
            f"{now - self.t0:.6f}", '' if loop_dt is None else f"{loop_dt:.6f}",
            self.release_idx, f"{self.target_deg:g}", phase, self.torque_enabled,
            '' if goal is None else f"{goal:.6f}",
            *(['', '', '', '', '', ''] if state is None else [
                f"{state['pos']:.6f}", f"{state['vel']:.5f}", f"{state['iq']:.4f}",
                f"{state['vin']:.2f}", f"{state['tw']:.1f}", f"{state['tp']:.1f}"]),
            '' if err is None else hex(err)])
        return state, err, now, loop_dt

    def marker(self, phase, t):
        """A row with no bus sample (the release command / its return)."""
        self.rows.append([f"{t - self.t0:.6f}", '', self.release_idx,
                          f"{self.target_deg:g}", phase, self.torque_enabled,
                          '', '', '', '', '', '', '', ''])

    def sleep_to_deadline(self):
        # Fixed-rate loop that always sleeps; on overrun, resync rather than
        # try to catch up with back-to-back cycles (observer_excitation tick()).
        self.next += PERIOD_S
        delay = self.next - time.monotonic()
        if delay < MIN_SLEEP_S:
            delay = MIN_SLEEP_S
            self.next = time.monotonic() + MIN_SLEEP_S
        time.sleep(delay)

    def enabled_cycle(self, phase, goal):
        """One enabled cycle: send goal, read, log, check, sleep. Logged
        before checking, so the sample that trips an abort is in the CSV."""
        goal = max(MIN_ANGLE, min(MAX_ANGLE, goal))
        s, err, now, loop_dt = self.cycle(phase, goal)
        if s is None:
            raise Abort(f"bulk_read_write failed (all retries used) in {phase}")
        if err != 0x80:
            raise Abort(f"error byte {hex(err)} in {phase}")
        if abs(s['iq']) > IQ_TRIP_A:
            raise Abort(f"|iq| {abs(s['iq']):.3f} A > {IQ_TRIP_A} A at "
                        f"{math.degrees(s['pos']):.1f} deg (immediate trip)")
        self.iq_over = self.iq_over + 1 if abs(s['iq']) > IQ_ABORT_A else 0
        if self.iq_over >= IQ_ABORT_COUNT:
            raise Abort(f"|iq| > {IQ_ABORT_A} A for {self.iq_over} consecutive "
                        f"cycles (last {abs(s['iq']):.3f} A at "
                        f"{math.degrees(s['pos']):.1f} deg)")
        if abs(goal - s['pos']) > math.radians(TRACKING_ABORT_DEG):
            raise Abort(f"tracking: goal {math.degrees(goal):.1f} vs actual "
                        f"{math.degrees(s['pos']):.1f} deg")
        eff = max(s['tw'], s['tp'])
        if eff >= TEMP_MAX:
            raise Abort(f"effective temp {eff:.1f} C >= TEMP_MAX {TEMP_MAX} C")
        if eff >= TEMP_WARN:
            self.temp_warned = True     # printed after the release, not here
        if loop_dt is not None and loop_dt > MAX_LOOP_DT_S:
            raise Abort(f"loop dt {loop_dt * 1000:.1f} ms > "
                        f"{MAX_LOOP_DT_S * 1000:.0f} ms in {phase}")
        if now - self.t_first_enable > RUN_CAP_S:
            raise Abort(f"{RUN_CAP_S:.0f} s since the first enable")
        self.sleep_to_deadline()
        return s, now

    # -- one release ---------------------------------------------------------------
    def enable_here(self):
        if self.t_first_enable is not None and \
                time.monotonic() - self.t_first_enable > RUN_CAP_S:
            raise Abort(f"{RUN_CAP_S:.0f} s since the first enable")
        pos = self.last['pos']
        self.enable_attempted = True
        if self.t_first_enable is None:
            self.t_first_enable = time.monotonic()
        try:
            self.iface.enable_position_mode(pos)
        except RuntimeError as e:
            raise Abort(str(e))
        self.torque_enabled = 1
        self.iq_over = 0
        self.reset_timing()
        return pos

    def approach(self, start, target):
        """observer_excitation.py's run_leg ramp, at --approach-velocity.
        Direction from start/target: after a stuck release it can go up."""
        v = self.args.approach_velocity
        decel = v * SOFT_START_S
        direction = 1.0 if target > start else -1.0
        goal = start
        t0 = time.monotonic()
        t_prev = t0
        while True:
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now
            remaining = abs(target - goal)
            ramp = min(1.0, (now - t0) / SOFT_START_S, remaining / decel)
            ramp = max(ramp, RAMP_FLOOR)
            goal += direction * v * ramp * dt
            goal = max(MIN_ANGLE, min(goal, MAX_ANGLE))
            arrived = (goal - target) * direction >= 0 or remaining < ARRIVE_EPS
            if arrived:
                goal = target
            self.enabled_cycle('approach', goal)
            if arrived:
                return

    @staticmethod
    def span_over(samples, now, window):
        """Position span (rad) over the last `window` s of (t, pos, ...) samples,
        or None if the samples don't cover the whole window yet."""
        lo = hi = None
        covered = False
        for smp in reversed(samples):
            if smp[0] < now - window:
                covered = True
                break
            p = smp[1]
            lo = p if lo is None or p < lo else lo
            hi = p if hi is None or p > hi else hi
        return (hi - lo) if covered and lo is not None else None

    def hold(self, target):
        """Hold the target until settled. Returns (hold position, mean iq over
        the last HOLD_IQ_WINDOW_S)."""
        t0 = time.monotonic()
        samples = []                     # (t, pos, iq)
        span = None
        while True:
            s, now = self.enabled_cycle('hold', target)
            samples.append((now, s['pos'], s['iq']))
            span = self.span_over(samples, now, SETTLE_S)
            if (now - t0 >= MIN_HOLD_S and span is not None
                    and span < math.radians(SETTLE_SPAN_DEG)):
                break
            if now - t0 > MAX_HOLD_S:
                moved = 'n/a' if span is None else f"{math.degrees(span):.3f} deg"
                raise Abort(f"hold at {math.degrees(target):.1f} deg not settled "
                            f"within {MAX_HOLD_S:.1f} s (moved {moved} in the last "
                            f"{SETTLE_S} s)")
        t_end = samples[-1][0]
        last_iq = [iq for t, _, iq in samples if t >= t_end - HOLD_IQ_WINDOW_S]
        return samples[-1][1], sum(last_iq) / len(last_iq)

    def release(self):
        """disable(), timed. Abort if not confirmed."""
        t_cmd = time.monotonic()
        self.marker('release', t_cmd)
        ok = self.iface.disable()
        t_ret = time.monotonic()
        if ok:
            self.torque_enabled = 0
        self.marker('release', t_ret)
        self.reset_timing()
        info = {'t_cmd': t_cmd, 'call_ms': (t_ret - t_cmd) * 1000, 'ok': ok,
                'attempts': self.iface.last_disable_attempts,
                'confirm_ms': self.iface.last_disable_ms}
        if not ok:
            raise Abort("release: disable() NOT CONFIRMED")
        return info

    def free(self, t_cmd, pos_release):
        """Reads only. Returns (peak |vel|, time to rest or None, stuck,
        drop_error or None)."""
        peak, rest_t, moved = 0.0, None, 0.0
        stuck = None
        drop_error = None
        hist = []                        # (t, pos) of good samples
        while True:
            s, err, now, _ = self.cycle('free', None)
            elapsed = now - t_cmd
            if s is None:
                drop_error = drop_error or "failed read in free phase"
            elif err is not None and err & self.iface.ERROR_BITS:
                drop_error = drop_error or f"error byte {hex(err)} in free phase"
            if s is not None:
                peak = max(peak, abs(s['vel']))
                if elapsed <= STUCK_WINDOW_S:
                    moved = max(moved, abs(s['pos'] - pos_release))
                hist.append((now, s['pos']))
                if s['pos'] >= math.radians(FREE_REST_DEG):
                    span = self.span_over(hist, now, SETTLE_S)
                    if span is not None and span < math.radians(SETTLE_SPAN_DEG):
                        rest_t = (now - SETTLE_S) - t_cmd
            if stuck is None and elapsed > STUCK_WINDOW_S:
                stuck = moved < math.radians(STUCK_DEG)
            if rest_t is not None or elapsed >= FREE_MAX_S:
                break
            self.sleep_to_deadline()
        if stuck is None:                # came to rest inside the window
            stuck = moved < math.radians(STUCK_DEG)
        return peak, rest_t, stuck, drop_error

    def one_release(self, idx, target_deg):
        self.release_idx, self.target_deg = idx, target_deg
        self.temp_warned = False
        target = math.radians(target_deg)
        start = self.enable_here()
        self.approach(start, target)
        hold_pos, hold_iq = self.hold(target)
        info = self.release()
        peak, rest_t, stuck, drop = self.free(info['t_cmd'], hold_pos)
        line = (f"release {idx:2d}: target {target_deg:g} deg, hold "
                f"{math.degrees(hold_pos):.2f} deg, mean iq last "
                f"{HOLD_IQ_WINDOW_S:.1f}s {hold_iq:+.3f} A, t_release_cmd "
                f"{info['t_cmd'] - self.t0:.4f} s, disable() {info['call_ms']:.1f} ms "
                f"attempts {info['attempts']} (confirmed at "
                f"{info['confirm_ms']:.1f} ms), peak |vel| {peak:.3f} rad/s, "
                f"time to rest "
                f"{'none (2.5 s timeout)' if rest_t is None else f'{rest_t:.3f} s'}"
                f", stuck {'YES' if stuck else 'no'}"
                f"{', TEMP_WARN reached' if self.temp_warned else ''}"
                f"{', ' + drop if drop else ''}")
        self.footer.append(line)
        print(line)                       # the one line per release
        if drop:
            raise DropStop(drop)

    # -- output --------------------------------------------------------------------
    def save(self, end_note):
        made = []
        p = os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        path = os.path.join(LOG_SUBDIR,
                            f"j_release_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            for line in self.header:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(COLUMNS)
            w.writerows(self.rows)
            for line in self.footer:
                f.write(f"# {line}\n")
            f.write(f"# END: {end_note}\n")
        uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
        if os.geteuid() == 0 and uid and gid:
            for q in made + [path]:
                try:
                    os.chown(q, int(uid), int(gid))
                except OSError as e:
                    print(f"WARNING: chown {q} failed: {e}")
        return path


def read_state(iface):
    values, err = iface.bear.bulk_read([iface.id], READ_REGS)[0]
    if err is None or any(v is None for v in values):
        raise SystemExit("REFUSED: could not read state.")
    pos, vel, iq, vin, tw, tp = values
    return {'pos': pos, 'vel': vel, 'iq': iq, 'vin': vin, 'tw': tw, 'tp': tp}, err


def check_rest(s):
    deg = math.degrees(s['pos'])
    if not REST_MIN_DEG <= deg <= REST_MAX_DEG:
        raise SystemExit(
            f"REFUSED: arm at {deg:.2f} deg; it must rest on the top stop at "
            f"{REST_MIN_DEG:.0f}-{REST_MAX_DEG:.0f} deg. Above {REST_MAX_DEG:.0f} "
            f"deg (MAX_ANGLE) the firmware ignores the hold goal. Check the "
            f"homing and re-home if needed (STATE 2.2) -- do NOT raise MAX_ANGLE.")


def preflight(iface, run):
    """Read-only checks, then the settings (written while disabled, read back).
    Raises SystemExit before any write if a read-only check fails."""
    (te,), err = iface.bear.get_torque_enable(iface.id)[0]
    if te != 0 or err != 0x80:
        raise SystemExit(f"REFUSED: torque_enable={te}, error byte {err}. "
                         f"Need a disabled actuator with no fault.")
    s, err = read_state(iface)
    if err != 0x80:
        raise SystemExit(f"REFUSED: error byte {hex(err)}.")
    check_rest(s)
    eff = max(s['tw'], s['tp'])
    if eff >= TEMP_WARN:
        raise SystemExit(f"REFUSED: effective temp {eff:.1f} C >= TEMP_WARN "
                         f"({TEMP_WARN} C). Let it cool.")

    for name, val in SETTINGS:
        getattr(iface.bear, 'set_' + name)((iface.id, val))
    readback = []
    for name, val in SETTINGS:
        (rb,), _ = getattr(iface.bear, 'get_' + name)(iface.id)[0]
        readback.append((name, val, rb))
        if rb is None or abs(rb - val) > READBACK_ATOL + READBACK_RTOL * abs(val):
            raise SystemExit(f"REFUSED: read-back mismatch on {name}: set {val}, "
                             f"read {rb}. Nothing was enabled.")

    a = run.args
    dirty = 'yes' if _git('status', '--porcelain') not in ('', 'unavailable') else 'no'
    run.header = [
        f"j_release_test.py  started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}  "
        f"git {_git('rev-parse', '--short', 'HEAD')} dirty: {dirty}",
        f"argv: {' '.join(sys.argv)}",
        f"heights_deg={','.join(f'{h:g}' for h in a.heights)} repeats={a.repeats} "
        f"order={','.join(f'{h:g}' for h in a.heights * a.repeats)} "
        f"approach_velocity={a.approach_velocity} rad/s soft_start_s={SOFT_START_S}",
        f"planned_s={a.planned_s:.1f} (worst case: approach from {REST_MAX_DEG:.0f} "
        f"deg, {MAX_HOLD_S} s hold, {FREE_MAX_S} s free per release) "
        f"run_cap_s={RUN_CAP_S}",
        f"settle: position moved < {SETTLE_SPAN_DEG} deg over the last {SETTLE_S} s, "
        f"min hold {MIN_HOLD_S} s, abort if not settled in {MAX_HOLD_S} s",
        f"free: reads only until pos >= {FREE_REST_DEG} deg and moved < "
        f"{SETTLE_SPAN_DEG} deg over {SETTLE_S} s, or {FREE_MAX_S} s after the release "
        f"command; stuck = moved < {STUCK_DEG} deg in the first {STUCK_WINDOW_S} s",
        f"aborts: err != 0x80; |iq| > {IQ_TRIP_A} A once or > {IQ_ABORT_A} A x"
        f"{IQ_ABORT_COUNT}; |goal-pos| > {TRACKING_ABORT_DEG} deg; temp >= "
        f"{TEMP_MAX} C (warn {TEMP_WARN}); failed bulk call; loop dt > "
        f"{MAX_LOOP_DT_S * 1000:.0f} ms; > {RUN_CAP_S:.0f} s since first enable; "
        f"Ctrl-C. Abort -> disable(), no park",
        f"loop {1 / PERIOD_S:.0f} Hz, one bulk packet per cycle; sched "
        f"{os.sched_getscheduler(0)} prio {os.sched_getparam(0).sched_priority} "
        f"affinity {sorted(os.sched_getaffinity(0))}",
        f"start: pos {math.degrees(s['pos']):.2f} deg, vel {s['vel']:.4f}, iq "
        f"{s['iq']:.3f}, input {s['vin']:.2f} V, winding {s['tw']:.1f} C, "
        f"powerstage {s['tp']:.1f} C",
        "settings (register=set/read): " + " ".join(
            f"{n}={v:.6g}/{rb:.6g}" for n, v, rb in readback),
        "units: t s since script start (monotonic); loop_dt s (blank on the first "
        "cycle after an enable/disable); goal_pos, pos rad; vel rad/s; iq A; "
        "input_voltage V; temps C; torque_enabled as the script believes. "
        "'release' rows are markers with no bus sample: t just before disable() "
        "and t when it returned.",
    ]
    for line in run.header:
        print(line)
    return s


def main(argv=None):
    args = parse_args(argv)
    # Pre-registered conditions: RT priority, pinned (same as Block 1).
    if os.sched_getscheduler(0) != os.SCHED_FIFO:
        raise SystemExit(
            "REFUSED: not running at real-time priority. From the repo root run:\n"
            "  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\\n"
            "      ~/projects/bear_controller/useful_tools/j_release_test.py")
    if not SAFETY_CHECKS_CONFIRMED:
        raise SystemExit("REFUSED: config.SAFETY_CHECKS_CONFIRMED is False. Trace "
                         "the ESTOP wiring, check the bench fixture, confirm real "
                         "tunables, then have a human set it.")
    print(f"Planned (worst case) {args.planned_s:.1f} s of {RUN_CAP_S:.0f} s cap; "
          f"{len(args.heights) * args.repeats} releases.")

    iface = BearInterface()
    run = Run(iface, args)
    end_note = 'unhandled exception'      # overwritten on every handled path
    path = None
    try:
        preflight(iface, run)
        n = len(args.heights) * args.repeats
        input(f"\nHANDS CLEAR: the arm snaps up into the top stop {n} times. "
              f"Foam pad on the stop? Press Enter to start: ")
        # Re-read: the arm may have moved at the prompt.
        s, err = read_state(iface)
        if err != 0x80:
            raise SystemExit(f"REFUSED: error byte {hex(err)} at start.")
        check_rest(s)
        run.last = s
        run.target_deg = args.heights[0]
        order = args.heights * args.repeats
        for idx, h in enumerate(order, start=1):
            run.one_release(idx, h)
        end_note = f"complete -- {len(order)} releases"
    except Abort as e:
        end_note = f"abort -- {e}"
    except DropStop as e:
        end_note = f"stopped after the drop -- {e}"
    except KeyboardInterrupt:
        end_note = "abort -- Ctrl-C"
    except SystemExit as e:
        end_note = f"refused -- {e}"
        raise
    finally:
        # Disable FIRST, before any file write. Only if an enable was ever
        # attempted: a preflight refusal must not write anything.
        if run.enable_attempted:
            if not end_note.startswith('complete'):
                print(f"\n{end_note.upper()}\nDisabling now (no park): the arm "
                      f"may snap up into the top stop.")
            iface.disable()
        if run.release_idx and len(run.footer) < run.release_idx:
            run.footer.append(f"release {run.release_idx:2d}: target "
                              f"{run.target_deg:g} deg -- NOT COMPLETED ({end_note})")
        if run.header:
            path = run.save(end_note)
        try:
            iface.bear.close_port()
        except Exception:
            pass
    print("\n" + "\n".join(run.footer + [f"END: {end_note}"]))
    if path:
        print(f"Log: {path}")
    return 0 if end_note.startswith('complete') else 1


if __name__ == '__main__':
    sys.exit(main())