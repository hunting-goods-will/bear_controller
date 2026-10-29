"""
Hold bench test (PLAN v4 3.3/3.4, H1-H3). THIS SCRIPT ENABLES TORQUE (Mode 0).

Setup: bare rig (or the 0.711 kg weight for H2), arm resting on the TOP STOP,
actuator powered, nothing else running. Run the bare H0 sweep first and pass
it with --sweep. HANDS CLEAR, weights only. Ctrl-C stops everything.

WHAT IT DOES
------------
A software position hold in torque mode, the measuring half of the hold-phase
design. Each 2 ms cycle: error e = target - angle;

    u = KP*e + integral(KI*e) - KD*velocity  (+ creep friction feedforward)   (N m, + = up)

then capped at 4.8 A (default; --i-max raises it), rate-limited to 4 N m/s, joint-limit and thermal
derates, and sent as goal_iq = u/KT in ONE bulk packet (exchange()).
hold_law() is the only control-law code here and is deliberately small: the
real hold (controller v2) is Sam's to write; this is a bench instrument.

Default run (about 90 s):
  ramp the target from the top stop down to 80 deg at 0.1 rad/s
  80 deg: settle 3 s -> still 10 s -> creep +-1 deg at 0.25 Hz x 3 cycles
          (cycle 1 eases in from zero and is not averaged)
  60 deg: the same;  40 deg: the same
  ramp the target back up to 118 deg (1.5 deg ABOVE the top stop: the arm
  rides onto the stop under control), fade the torque to 0, disable. If the
  arm stops following on that last ramp (> 2 deg below the target: the stop,
  or a payload needing upward torque the 90-105 deg taper refuses) AND is at
  or above 95 deg, the ramp ends there and the fade starts; a payload may then sink a little until
  friction holds it (velocity abort still armed).
During the creep a friction feedforward (0.8 x the sweep's friction at that
angle, + while the target moves up, - while it moves down) flips friction at
each reversal; without it the fake arm crept only +-0.2-0.4 deg (stick-slip)
and read up to 0.12 N m off. It averages to zero over whole cycles.
The bare rig rests on the stop because the spring beats gravity, so every
hold here needs DOWNWARD (negative) torque; the worn version is upward-only.

  --payload-kg/--payload-r-m/--payload-angle-deg   H2 (header + the current check)
  --thermal                                        H3: one still hold at
      --thermal-deg (60) for --thermal-s (<= 1800) s; ends early, normally, at
      TEMP_WARN (65 C). Rows logged at 50 Hz during that hold.
  --idle-s N      before arming (after GO), log temperatures and state for N s
                  with NO writes (5 Hz rows, phase 'idle'): the thermal fit's
                  starting state and ambient. Use with --thermal from cold.
  --i-max A       per-run firmware current limit, 5.5 (default, nothing written)
                  to IQ_CEILING (10 A). Written while disabled and read back
                  (BearInterface.set_current_limit), restored to 5.5 after a
                  confirmed disable. Command cap = A - 0.7, abort = A - 0.5 for 3
                  cycles, prediction limit = A - 0.7 (defaults 4.8 / 5.0 / 4.8).

REFUSES BEFORE OPENING THE PORT: bad arguments; a payload that would FALL when
released (spring - rig gravity - payload gravity < +0.2 N m anywhere from 25 to
118 deg; outside the sweep's range the nearest bin is held); hold angles outside 35-95 deg
or outside the sweep's measured range; the sweep not a bare
observer_excitation.py sweep; any hold predicted (from the sweep, with and
without the payload) to need more than the command cap (4.8 A by default)
including friction; planned run > 120 s (non-thermal); not at RT priority.
REFUSES BEFORE ENABLING: actuator enabled or faulted; limit_i_max > 5.5;
arm not resting at >= 110 deg; max(winding, powerstage) >= 65 C;
SAFETY_CHECKS_CONFIRMED not True; anything but GO typed at the prompt.

ABORTS (-> disable() at once; the spring then snaps the arm up into the top
stop -- from 40 deg that is 76 deg of travel, more than the Oct 15 releases):
failed exchange; any error bit or a watchdog trip; |iq| >= 5.0 A (default;
--i-max - 0.5) for 3 cycles;
speed > 2 rad/s for 3 cycles in a row, judged from the POSITION (one velocity-
register glitch doesn't abort; the D term uses the position-derived speed when
the register disagrees by > 1 rad/s);
|target - angle| > 5 deg (not during the fade); angle < 25 deg; loop gap
> 10 ms; max temperature >= 75 C; time cap; Ctrl-C.

EVERY WRITE (nothing else is written; no gains, no save_config):
  (--idle-s: none -- reads only until the idle log ends)
  1. watchdog_timeout = 10000 us, read back          (configure_watchdog)
  1b. only with --i-max above 5.5: limit_i_max = A, read back (set_current_limit)
  2. mode = 0, goal_iq = 0, torque_enable = 1, each read back   (enable; mode
     first since Oct 28 -- goal_iq can't be written in mode 2)
  3. goal_iq every cycle, inside the one bulk packet  (exchange)
  4. on exit: goal_iq = 0, torque_enable = 0, confirmed, <= 3 tries (disable);
     then watchdog_timeout = 0, read back, and (if 1b) limit_i_max = 5.5, read
     back -- ONLY if the disable was confirmed. Unconfirmed: the watchdog is
     left armed (it trips in ~10 ms), the limit is left as it is (RAM only; a
     power cycle restores 5.5), and the script says to cut power.
Read only, into the header: limit_i_max, iq current-loop gains.

RUN (from the repo root, after test_connection.py and the H0 sweep):
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\
      useful_tools/hold_bench_test.py --sweep <H0 sweep csv>
  ... --payload-kg 0.711 --payload-r-m <r> --payload-angle-deg <hang angle>
  ... --thermal --idle-s 300 [--i-max 7]
Offline (fake actuator, no port): useful_tools/test_hold_bench_offline.py

Output: logs/phase2/hold_bench/hold_bench_<YYYYmmdd_HHMMSS>.csv (or
hold_thermal_...), written after the final disable (also on abort), chowned
to SUDO_UID:SUDO_GID. '#' header, one row per cycle, '#' footer.
"""
import sys
sys.dont_write_bytecode = True   # under sudo, leave no root-owned __pycache__

import argparse
import csv
import math
import os
import subprocess
import time

import numpy as np

from main_controller.bear_interface import BearInterface
from main_controller.config import (LOG_DIR, TEMP_WARN, TEMP_MAX, IQ_CEILING,
                                    WATCHDOG_TIMEOUT_MS, SAFETY_CHECKS_CONFIRMED)
from main_controller.controller import KT, PHI_DEG
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # sweep_tables.py sits here
import sweep_tables as ST  # noqa: E402

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'hold_bench')

# ---- Hold law (bench instrument; see hold_law()) ------------------------------
KP_DEFAULT, KI_DEFAULT, KD_DEFAULT = 40.0, 60.0, 1.5   # N m/rad, N m/(rad s), N m s/rad
KP_RANGE, KI_RANGE, KD_RANGE = (5.0, 100.0), (0.0, 200.0), (0.0, 5.0)
RATE_NM_S = 4.0                    # torque rate limit (Phase 1 value; flange history)
I_MAX_DEFAULT = 5.5                # firmware limit_i_max as flashed (config.MAX_IQ)
CAP_BELOW_A, ABORT_BELOW_A = 0.7, 0.5   # cap = limit - 0.7, abort = limit - 0.5
IQ_CMD_MAX_A = I_MAX_DEFAULT - CAP_BELOW_A   # 4.8 A: default command cap, below the 5.0 A abort
U_LIMIT_NM = IQ_CMD_MAX_A * KT     # 3.216 N m (default)

# ---- Plan ---------------------------------------------------------------------
DEFAULT_ANGLES = (80.0, 60.0, 40.0)   # actuator deg
HOLD_MIN_DEG, HOLD_MAX_DEG = 35.0, 95.0
RAMP_V = 0.10                      # rad/s, target speed between angles
RAMP_TR = 0.5                      # s, speed ramp at each end of a move
SETTLE_S = 3.0
STILL_S = 10.0
CREEP_DEG, CREEP_HZ, CREEP_CYCLES = 1.0, 0.25, 3   # 0.5 Hz x 4 was PLAN's; the fake
                                   # crept fuller and more accurately at 0.25 Hz
CREEP_DEG_RANGE = (0.5, 2.0)
CREEP_FF = 0.8                     # x sweep friction; see creep_ff(). Under 1 on
                                   # purpose: x1.2 limit-cycled and aborted on the
                                   # fake (the Phase 1 k_f < 1 lesson); x0.6-0.8 read
                                   # within 0.02 N m with 0.74-0.98 deg of creep.
CREEP_FF_RANGE = (0.0, 1.0)
END_DEG = 118.0                    # final target, ~1.5 deg ABOVE the top stop
                                   # (116.5; rests 115-116.5). The arm rides up at
                                   # 0.1 rad/s onto the stop under control, the
                                   # hold then asks for UPWARD torque, which
                                   # joint_limit_scale zeroes above 105 deg, so the
                                   # fade releases nothing. (Fading at 110 deg let
                                   # the spring fling the fake arm into the stop at
                                   # ~2 rad/s.) Error at the stop <= 3 deg < abort.
THERMAL_DEG, THERMAL_S = 60.0, 600.0
THERMAL_S_RANGE = (10.0, 1800.0)
IDLE_S_RANGE = (0.0, 1800.0)
IDLE_PERIOD_S = 0.2                # 5 Hz rows during the idle log
THERMAL_LOG_EVERY = 10             # 50 Hz rows during the thermal hold

# ---- Checks -------------------------------------------------------------------
REST_MIN_DEG = 110.0               # start on the top stop
PREDICT_MAX_A = IQ_CMD_MAX_A       # default; per run = the command cap
PREDICT_MARGIN_A = 0.3
RUN_CAP_S = 120.0
THERMAL_EXTRA_S = 60.0             # cap = thermal-s + this
IQ_ABORT_A, IQ_ABORT_COUNT = I_MAX_DEFAULT - ABORT_BELOW_A, 3   # 5.0 A default
V_ABORT = 2.0                      # rad/s, judged on the POSITION-derived speed...
V_ABORT_COUNT = 3                  # ...for this many cycles in a row. A single velocity-
                                   # register glitch (Aug: +7 and -9.9 rad/s wrong-sign
                                   # samples) must not abort, because every abort lets
                                   # the spring snap the arm into the stop (from 40 deg:
                                   # 76 deg of travel). Claude Code review, Oct 27.
VEL_SANITY = 1.0                   # rad/s: register vs position-derived speed; if they
                                   # disagree by more, the hold's D term uses the latter
NET_MARGIN = 0.2                   # N m, default for --release-margin (0.1-0.5): with a payload, the spring must beat rig + payload
                                   # gravity by this much everywhere the arm can go, so a
                                   # released arm rises to the stop and never falls
POS_ERR_ABORT_DEG = 5.0
END_FOLLOW_DEG = 2.0               # final ramp only: arm > 2 deg below the target
END_FOLLOW_MIN_DEG = 95.0          # AND at/above 95 deg = it has stopped following
                                   # (top stop, or the 90-105 deg taper refusing
                                   # upward torque a payload needs) -> fade from
                                   # there, normally. The angle condition matters:
                                   # without it a slow breakaway at the START of
                                   # the ramp (target 2 deg ahead of a stuck arm at
                                   # 60 deg) faded the torque mid-range and the
                                   # spring flung the fake arm up at 2 rad/s.
POS_MIN_ABORT_DEG = 25.0
MAX_LOOP_DT_S = 0.010              # = watchdog timeout
PERIOD_S = 0.002                   # 500 Hz
G = 9.81

PHASES = ('ramp', 'settle', 'still', 'creep', 'fade', 'thermal', 'idle')
COLUMNS = ['t', 'loop_dt', 'phase', 'angle_idx', 'target_deg', 'sp', 'pos', 'vel',
           'iq', 'iq_sent', 'u_des', 'u_cmd', 'integ', 'w_temp', 'p_temp', 'err']


class Abort(Exception):
    """A runtime check tripped -> disable at once."""


def hold_law(e, v, integ, dt, kp, ki, kd, u_limit=U_LIMIT_NM):
    """PID on position, derivative on the MEASURED velocity (no kick when the
    target moves). Returns (u in N m, new integral). Pure: no bus, no state."""
    integ = integ + ki * e * dt
    integ = max(-u_limit, min(u_limit, integ))
    return kp * e + integ - kd * v, integ


def creep_ff(tau, a, f_sweep):
    """Friction feedforward during the creep: +f while the target moves up,
    -f while it moves down (the target is x + A sin(w tau), so its direction
    is the sign of cos(w tau)). Without it the hold must build ~2 f of error
    torque at every reversal before the arm slides (a +-1.2 deg dead band at
    kp 40), and the fake arm crept only +-0.2-0.4 deg. It averages to zero over
    whole cycles, so it does not bias the reading. Scaled by --creep-ff."""
    c = math.cos(2 * math.pi * a.creep_hz * tau)
    return creep_env(tau, a) * a.creep_ff * f_sweep * (1.0 if c > 0 else -1.0)


def creep_env(tau, a):
    """0 -> 1 smoothstep over the first creep cycle (that cycle is never
    averaged). Why: during the still phase a stuck arm lets the integral walk
    the torque to an EDGE of the friction band; a creep that starts at full
    size with the feedforward pushing the same way flung the fake arm 3 deg
    and set up a +-4 deg oscillation (payload run). Easing in avoids it."""
    x = min(1.0, max(0.0, tau * a.creep_hz))
    return x * x * (3.0 - 2.0 * x)


# ---- target profiles ----------------------------------------------------------
def move_profile(a0, a1, v=RAMP_V, tr=RAMP_TR):
    """Trapezoidal-speed move a0 -> a1 (rad). Returns (duration, fn(tau))."""
    d = abs(a1 - a0)
    if d < 1e-9:
        return 0.0, (lambda tau: a1)
    s = 1.0 if a1 > a0 else -1.0
    tr_e = min(tr, math.sqrt(d * tr / v))      # short move: triangle
    vpk = v * tr_e / tr
    T = d / vpk + tr_e

    def fn(tau):
        tau = min(max(tau, 0.0), T)
        if tau < tr_e:
            x = vpk * tau * tau / (2 * tr_e)
        elif tau < T - tr_e:
            x = vpk * (tr_e / 2 + tau - tr_e)
        else:
            x = d - vpk * (T - tau) ** 2 / (2 * tr_e)
        return a0 + s * x
    return T, fn


def build_plan(start_rad, a):
    """List of segments: (phase, angle_idx, target_deg, duration, fn(tau))."""
    segs, here = [], start_rad
    angles = [a.thermal_deg] if a.thermal else a.angles
    for k, deg in enumerate(angles):
        tgt = math.radians(deg)
        T, fn = move_profile(here, tgt)
        segs.append(('ramp', k, deg, T, fn))
        segs.append(('settle', k, deg, SETTLE_S, (lambda tau, x=tgt: x)))
        if a.thermal:
            segs.append(('thermal', k, deg, a.thermal_s, (lambda tau, x=tgt: x)))
        else:
            segs.append(('still', k, deg, STILL_S, (lambda tau, x=tgt: x)))
            amp, w = math.radians(a.creep_deg), 2 * math.pi * a.creep_hz
            segs.append(('creep', k, deg, a.creep_cycles / a.creep_hz,
                         (lambda tau, x=tgt, amp=amp, w=w:
                          x + creep_env(tau, a) * amp * math.sin(w * tau))))
        here = tgt
    T, fn = move_profile(here, math.radians(END_DEG))
    segs.append(('ramp', -1, END_DEG, T, fn))
    segs.append(('fade', -1, END_DEG, getattr(a, 'u_limit', U_LIMIT_NM) / RATE_NM_S + 0.2,
                 (lambda tau, x=math.radians(END_DEG): x)))
    return segs


# ---- prediction from the H0 sweep ---------------------------------------------
def payload_torque(a, act_deg):
    """Downward torque of the added weight (N m) at an actuator angle."""
    if a.payload_kg <= 0:
        return 0.0
    vest = act_deg + PHI_DEG + a.payload_angle_deg
    return a.payload_kg * G * a.payload_r_m * math.sin(math.radians(vest))


def predictions(table, a):
    """Per hold angle: y, f, predicted u (bare, and with payload), peak |iq|.
    Refuses (ValueError) outside the sweep's range or above the run's command cap."""
    pmax = getattr(a, 'predict_max', PREDICT_MAX_A)
    out = []
    for deg in ([a.thermal_deg] if a.thermal else a.angles):
        vest = deg + PHI_DEG
        y = ST.interp_inside(table, vest, 2)
        f = ST.interp_inside(table, vest, 1)
        if math.isnan(y) or math.isnan(f):
            raise ValueError(f"hold {deg:g} deg (vest {vest:.1f}) is outside the sweep's "
                             f"measured range (vest {table[0, 0]:.1f}-{table[-1, 0]:.1f})")
        u_bare = -y
        u_pay = -y + payload_torque(a, deg)
        peak = (max(abs(u_bare), abs(u_pay)) + f) / KT + PREDICT_MARGIN_A
        if peak > pmax:
            raise ValueError(f"hold {deg:g} deg predicted to need ~{peak:.2f} A "
                             f"(|u| {max(abs(u_bare), abs(u_pay)):.2f} + friction {f:.2f} N m, "
                             f"+{PREDICT_MARGIN_A} A margin) > {pmax:g} A"
                             + ("" if pmax > IQ_CMD_MAX_A else " (--i-max raises it)"))
        out.append(dict(deg=deg, vest=vest, y=y, f=f, u_bare=u_bare, u_pay=u_pay, peak=peak))
    return out


def release_check(table, a):
    """Every abort and the normal end release the arm (disable). Bare, the spring wins
    and the arm rises to the top stop. With a payload it must still win EVERYWHERE the
    arm can be: y(theta) - payload torque >= NET_MARGIN from 25 deg (the low-angle abort)
    to 118 deg. Inside the sweep's range y is measured; outside it, the nearest bin's y
    is held (an assumption, flagged in the header). Returns (min net, at act deg).
    Claude Code review, Oct 27: 2 kg at 0.4 m was accepted and would fall 1.6-5.6 N m
    harder than friction holds."""
    lo, hi = table[0, 0] - PHI_DEG, table[-1, 0] - PHI_DEG
    worst = (float('inf'), None)
    for act in np.arange(POS_MIN_ABORT_DEG, END_DEG + 0.5, 0.5):
        vest = min(max(act + PHI_DEG, table[0, 0]), table[-1, 0])
        y = float(np.interp(vest, table[:, 0], table[:, 2]))
        net = y - payload_torque(a, act)
        if net < worst[0]:
            worst = (net, float(act))
    return worst, (lo, hi)


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sweep', required=True, help='bare H0 sweep from THIS session')
    ap.add_argument('--angles', default=','.join(f'{x:g}' for x in DEFAULT_ANGLES))
    ap.add_argument('--creep-deg', type=float, default=CREEP_DEG)
    ap.add_argument('--creep-hz', type=float, default=CREEP_HZ)
    ap.add_argument('--creep-cycles', type=int, default=CREEP_CYCLES)
    ap.add_argument('--payload-kg', type=float, default=0.0)
    ap.add_argument('--payload-r-m', type=float, default=0.0)
    ap.add_argument('--payload-angle-deg', type=float, default=0.0,
                    help="weight's line offset from the vest reference (header + check)")
    ap.add_argument('--creep-ff', type=float, default=CREEP_FF,
                    help='friction feedforward during the creep, x sweep friction (0 = off)')
    ap.add_argument('--release-margin', type=float, default=NET_MARGIN,
                    help='N m the spring must beat rig + payload gravity by, everywhere (0.1-0.5)')
    ap.add_argument('--thermal', action='store_true')
    ap.add_argument('--thermal-deg', type=float, default=THERMAL_DEG)
    ap.add_argument('--thermal-s', type=float, default=THERMAL_S)
    ap.add_argument('--idle-s', type=float, default=0.0,
                    help='log state for this long before arming, no writes (cold-start baseline)')
    ap.add_argument('--i-max', type=float, default=I_MAX_DEFAULT,
                    help=f'per-run firmware current limit, {I_MAX_DEFAULT}-{IQ_CEILING} A '
                         f'(cap = A - {CAP_BELOW_A}, abort = A - {ABORT_BELOW_A})')
    ap.add_argument('--kp', type=float, default=KP_DEFAULT)
    ap.add_argument('--ki', type=float, default=KI_DEFAULT)
    ap.add_argument('--kd', type=float, default=KD_DEFAULT)
    a = ap.parse_args(argv)
    try:
        a.angles = [float(x) for x in a.angles.split(',') if x.strip()]
    except ValueError:
        ap.error('--angles must be comma-separated numbers')
    if not a.angles:
        ap.error('--angles is empty')
    for deg in a.angles + [a.thermal_deg]:
        if not HOLD_MIN_DEG <= deg <= HOLD_MAX_DEG:      # NaN fails too
            ap.error(f'hold angle {deg} outside {HOLD_MIN_DEG:g}-{HOLD_MAX_DEG:g} deg')
    if not CREEP_DEG_RANGE[0] <= a.creep_deg <= CREEP_DEG_RANGE[1]:
        ap.error(f'--creep-deg must be {CREEP_DEG_RANGE[0]}-{CREEP_DEG_RANGE[1]}')
    if not 0.1 <= a.creep_hz <= 1.0:
        ap.error('--creep-hz must be 0.1-1.0')
    if not CREEP_FF_RANGE[0] <= a.creep_ff <= CREEP_FF_RANGE[1]:
        ap.error(f'--creep-ff must be {CREEP_FF_RANGE[0]}-{CREEP_FF_RANGE[1]}')
    if not 2 <= a.creep_cycles <= 10:
        ap.error('--creep-cycles must be 2-10 (the first cycle eases in and is not averaged)')
    if not 0.0 <= a.payload_kg <= 2.0 or not 0.0 <= a.payload_r_m <= 0.4:
        ap.error('--payload-kg must be 0-2 and --payload-r-m 0-0.4')
    if (a.payload_kg > 0) != (a.payload_r_m > 0):
        ap.error('give both --payload-kg and --payload-r-m, or neither')
    if not -90.0 <= a.payload_angle_deg <= 90.0:
        ap.error('--payload-angle-deg must be -90..90')
    if not THERMAL_S_RANGE[0] <= a.thermal_s <= THERMAL_S_RANGE[1]:
        ap.error(f'--thermal-s must be {THERMAL_S_RANGE[0]:g}-{THERMAL_S_RANGE[1]:g}')
    if not IDLE_S_RANGE[0] <= a.idle_s <= IDLE_S_RANGE[1]:
        ap.error(f'--idle-s must be {IDLE_S_RANGE[0]:g}-{IDLE_S_RANGE[1]:g}')
    if not I_MAX_DEFAULT <= a.i_max <= IQ_CEILING:          # NaN fails too
        ap.error(f'--i-max must be {I_MAX_DEFAULT}-{IQ_CEILING} A (config.IQ_CEILING)')
    a.raised = a.i_max > I_MAX_DEFAULT
    a.iq_cmd_max = a.i_max - CAP_BELOW_A
    a.iq_abort = a.i_max - ABORT_BELOW_A
    a.u_limit = a.iq_cmd_max * KT
    a.predict_max = a.iq_cmd_max
    for name, rng in (('kp', KP_RANGE), ('ki', KI_RANGE), ('kd', KD_RANGE)):
        if not rng[0] <= getattr(a, name) <= rng[1]:
            ap.error(f'--{name} must be {rng[0]:g}-{rng[1]:g}')
    a.table = ST.sweep_tables(a.sweep)           # SystemExit if not a bare sweep
    try:
        a.pred = predictions(a.table, a)
    except ValueError as e:
        ap.error(str(e))
    if not 0.1 <= a.release_margin <= 0.5:
        ap.error('--release-margin must be 0.1-0.5 N m')
    a.release = None
    if a.payload_kg > 0:
        (net, at), rng = release_check(a.table, a)
        a.release = (net, at, rng)
        if net < a.release_margin:
            ap.error(f"payload would not be safe to release: spring - (rig + payload) gravity "
                     f"is {net:+.2f} N m at {at:.1f} deg (need >= +{a.release_margin}). An abort or "
                     f"the end of the run would let it fall. Lighter weight or shorter r.")
    worst_start = math.radians(120.0)
    a.planned_s = sum(s[3] for s in build_plan(worst_start, a))
    a.cap_s = (a.thermal_s + THERMAL_EXTRA_S) if a.thermal else RUN_CAP_S
    if a.planned_s > a.cap_s:
        ap.error(f'planned run {a.planned_s:.1f} s exceeds the {a.cap_s:.0f} s cap')
    return a


def _git(*argv):
    try:
        r = subprocess.run(['git', '-c', f'safe.directory={REPO_DIR}', '-C', REPO_DIR, *argv],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else 'unavailable'
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


class Run:
    def __init__(self, iface, a):
        self.iface, self.a = iface, a
        self.t0 = time.monotonic()
        n = int(a.cap_s / PERIOD_S) + 2000
        if a.thermal:
            n = int((a.cap_s - a.thermal_s) / PERIOD_S + a.thermal_s / PERIOD_S
                    / THERMAL_LOG_EVERY) + 2000
        n += int(a.idle_s / IDLE_PERIOD_S) + 10
        self.buf = np.full((n, len(COLUMNS)), np.nan)
        self.n = 0
        self.header, self.footer = [], []
        self.end_reason = 'not started'
        self.t_enable = None
        self.armed = False                 # True once configure_watchdog is called
        self.limit_written = False         # True once a raised limit_i_max write was sent

    def log(self, row):
        if self.n < len(self.buf):
            self.buf[self.n] = row
            self.n += 1

    # -- preflight (reads only) ----------------------------------------------
    def preflight(self):
        bear, i = self.iface.bear, self.iface.id
        if SAFETY_CHECKS_CONFIRMED is not True:
            raise SystemExit("REFUSED: SAFETY_CHECKS_CONFIRMED is not True in config.py")
        (te,), err = bear.get_torque_enable(i)[0]
        if te != 0 or err != 0x80:
            raise SystemExit(f"REFUSED: torque_enable={te}, error byte "
                             f"{err if err is None else hex(err)}; need 0 and 0x80.")
        (lim,), _ = bear.get_limit_i_max(i)[0]
        if lim is None or lim > 5.5:
            raise SystemExit(f"REFUSED: limit_i_max reads {lim}; expected <= 5.5 A.")
        (kpi,), _ = bear.get_p_gain_iq(i)[0]
        (kii,), _ = bear.get_i_gain_iq(i)[0]
        values, err = bear.bulk_read([i], self.iface.LOOP_READ_REGS)[0]
        if err is None or any(v is None for v in values):
            raise SystemExit("REFUSED: could not read state.")
        pos, vel, iq, tw, tp = values
        if math.degrees(pos) < REST_MIN_DEG:
            raise SystemExit(f"REFUSED: arm at {math.degrees(pos):.1f} deg; rest it on the "
                             f"top stop (>= {REST_MIN_DEG:g} deg) first.")
        if max(tw, tp) >= TEMP_WARN:
            raise SystemExit(f"REFUSED: temperature {max(tw, tp):.1f} C >= {TEMP_WARN} C.")
        a = self.a
        self.start_pos = pos
        self.plan = build_plan(pos, a)
        self.header += [
            f"hold_bench_test.py  git {_git('rev-parse', '--short', 'HEAD')} "
            f"dirty={'yes' if _git('status', '--porcelain', '--', 'useful_tools', 'main_controller') else 'no'}",
            f"started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
            f"mode={'thermal' if a.thermal else 'holds'} angles={a.angles if not a.thermal else [a.thermal_deg]} "
            f"creep_deg={a.creep_deg} creep_hz={a.creep_hz} creep_cycles={a.creep_cycles} creep_ff={a.creep_ff} "
            f"thermal_s={a.thermal_s if a.thermal else 0}",
            f"payload_kg={a.payload_kg} payload_r_m={a.payload_r_m} "
            f"payload_angle_deg={a.payload_angle_deg}",
            f"gains kp={a.kp} ki={a.ki} kd={a.kd}  rate={RATE_NM_S} N m/s  "
            f"cmd cap {a.iq_cmd_max:g} A  KT={KT}",
            f"i_max={a.i_max:g} ({'raised for this run; restored to ' + str(I_MAX_DEFAULT) + ' after a confirmed disable' if a.raised else 'default, not written'}) "
            f"abort {a.iq_abort:g} A x{IQ_ABORT_COUNT}  idle_s={a.idle_s:g}",
            f"sweep={os.path.abspath(a.sweep)}",
            f"readback: limit_i_max={lim} p_gain_iq={kpi} i_gain_iq={kii}",
            f"start: pos {math.degrees(pos):.2f} deg vel {vel:.3f} iq {iq:.3f} "
            f"winding {tw:.1f} C powerstage {tp:.1f} C",
            f"sched {os.sched_getscheduler(0)} planned_s={a.planned_s:.1f} cap_s={a.cap_s:.0f}",
        ]
        if a.release:
            net, at, (lo, hi) = a.release
            self.header.append(f"release check: min spring - (rig + payload) = {net:+.3f} N m at "
                               f"{at:.1f} deg (>= {a.release_margin}); sweep measured act {lo:.1f}-{hi:.1f} "
                               f"deg, nearest bin held outside")
        for p in a.pred:
            self.header.append(
                f"predict {p['deg']:g} deg (vest {p['vest']:.1f}): y={p['y']:.3f} f={p['f']:.3f} "
                f"u_bare={p['u_bare']:.3f} u_payload={p['u_pay']:.3f} N m peak~{p['peak']:.2f} A")
        for line in self.header:
            print(line)

    # -- idle log (reads only, before arming) -------------------------------------
    def idle(self):
        """Log state at 5 Hz for --idle-s seconds with NO writes, so a thermal
        fit knows the starting state (cold, or still cooling from a sweep).
        Stops (no writes) on an error byte, heat, or the arm leaving the stop."""
        a, bear, i = self.a, self.iface.bear, self.iface.id
        if a.idle_s <= 0:
            return
        print(f"Idle log {a.idle_s:.0f} s (no writes; Ctrl-C stops with nothing written)...")
        t_start = time.monotonic()
        nxt = t_start
        first = last = None
        while time.monotonic() - t_start < a.idle_s:
            values, err = bear.bulk_read([i], self.iface.LOOP_READ_REGS)[0]
            now = time.monotonic()
            if err is None or any(v is None for v in values):
                raise Abort("idle log: read failed")
            pos, vel, iq, tw, tp = values
            self.log([now - self.t0, IDLE_PERIOD_S, PHASES.index('idle'), -2, float('nan'),
                      float('nan'), pos, vel, iq, float('nan'), float('nan'), float('nan'),
                      float('nan'), tw, tp, err])
            first = first or (now, tw, tp)
            last = (now, tw, tp)
            if err != 0x80:
                raise Abort(f"idle log: error byte {hex(err)}")
            if max(tw, tp) >= TEMP_WARN:
                raise Abort(f"idle log: temperature {max(tw, tp):.1f} C >= {TEMP_WARN} C")
            if math.degrees(pos) < REST_MIN_DEG:
                raise Abort(f"idle log: arm left the top stop ({math.degrees(pos):.1f} deg)")
            nxt += IDLE_PERIOD_S
            s = nxt - time.monotonic()
            if s > 0:
                time.sleep(s)
        if first and last:
            mins = max((last[0] - first[0]) / 60.0, 1e-9)
            self.header.append(f"idle {a.idle_s:.0f} s: winding {first[1]:.2f} -> {last[1]:.2f} C, "
                               f"powerstage {first[2]:.2f} -> {last[2]:.2f} C "
                               f"({(last[1] - first[1]) / mins:+.2f} / {(last[2] - first[2]) / mins:+.2f} C/min)")
            print(self.header[-1])

    # -- the loop ----------------------------------------------------------------
    def run(self):
        a, iface = self.a, self.iface
        self.armed = True                  # from here on, exit must disable + disarm
        wd = iface.configure_watchdog(WATCHDOG_TIMEOUT_MS)
        if a.raised:
            self.limit_written = True      # from here on, a confirmed exit restores it
            rb = iface.set_current_limit(a.i_max)
            self.header.append(f"limit_i_max raised to {rb} A for this run")
        iface.enable()
        self.header.append(f"watchdog {wd} us; enabled")
        self.t_enable = time.monotonic()

        state, err = iface.exchange(0.0)
        if state is None:
            raise Abort("first exchange failed")
        integ, u_cmd = 0.0, 0.0
        over = 0
        over_v = 0                       # cycles in a row with position-derived |v| > V_ABORT
        v_pos = 0.0                      # position-derived speed of the last cycle
        self.vel_disagree = 0            # cycles the D term used v_pos instead of the register
        k = 0
        seg_i, seg_t0 = 0, time.monotonic()
        t_prev = time.monotonic()
        nxt = t_prev
        cycle = 0
        while True:
            now = time.monotonic()
            while seg_i < len(self.plan) and now - seg_t0 >= self.plan[seg_i][3]:
                seg_t0 += self.plan[seg_i][3]
                seg_i += 1
            if seg_i >= len(self.plan):
                self.end_reason = 'completed'
                break
            phase, aidx, tdeg, _, fn = self.plan[seg_i]
            sp = fn(now - seg_t0)
            dt = now - t_prev
            dtc = min(max(dt, PERIOD_S), MAX_LOOP_DT_S)
            pos, vel = state['position'], state['velocity']
            if abs(vel - v_pos) > VEL_SANITY:          # register glitch: trust the position
                vel = v_pos
                self.vel_disagree += 1
            e = sp - pos
            if phase == 'ramp' and aidx == -1 and e > math.radians(END_FOLLOW_DEG) \
                    and math.degrees(pos) >= END_FOLLOW_MIN_DEG \
                    and not getattr(self, '_end_early', False):
                self._end_early = True
                self.footer.append(f"final ramp ended at {math.degrees(pos):.1f} deg (arm stopped "
                                   f"following: top stop or the upward-torque taper)")
                seg_t0 = now - self.plan[seg_i][3]       # next cycle: fade

            if phase == 'fade':
                u_des = 0.0
            else:
                u_des, integ_new = hold_law(e, vel, integ, dtc, a.kp, a.ki, a.kd, a.u_limit)
                if phase == 'creep' and a.creep_ff > 0:
                    u_des += creep_ff(now - seg_t0, a, a.pred[aidx]['f'])
            u_des = max(-a.u_limit, min(a.u_limit, u_des))
            step = RATE_NM_S * dtc
            u_new = max(u_cmd - step, min(u_cmd + step, u_des))
            if phase != 'fade':
                # conditional integration: don't wind up while the rate limit
                # or the cap holds the command back in the error's direction
                limited = abs(u_new - u_des) > 1e-12
                if not (limited and (u_des - u_new) * e > 0):
                    integ = integ_new
            u_cmd = u_new
            iq_cmd = u_cmd / KT
            iq_cmd = iface.joint_limit_scale(iq_cmd, state)
            iq_cmd = iface.thermal_scale(iq_cmd, state)
            iq_cmd = max(-a.iq_cmd_max, min(a.iq_cmd_max, iq_cmd))

            state, err = iface.exchange(iq_cmd)
            t_now = time.monotonic()
            loop_dt = t_now - t_prev
            t_prev = t_now
            if state is None:
                raise Abort("exchange failed (all retries used)")
            v_pos = (state['position'] - pos) / max(loop_dt, 1e-4)
            cycle += 1
            if phase != 'thermal' or cycle % THERMAL_LOG_EVERY == 0:
                self.log([t_now - self.t0, loop_dt, PHASES.index(phase), aidx, tdeg, sp,
                          state['position'], state['velocity'], state['iq'], state['iq_sent'],
                          u_des, u_cmd, integ, state['temp'], state['temp_powerstage'], err])
            # ---- checks -----------------------------------------------------
            if iface.watchdog_tripped(err):
                raise Abort(f"watchdog tripped (error {hex(err)})")
            if iface.has_error(err):
                raise Abort(f"error byte {hex(err)}")
            over = over + 1 if abs(state['iq']) >= a.iq_abort else 0
            if over >= IQ_ABORT_COUNT:
                raise Abort(f"|iq| >= {a.iq_abort:g} A for {IQ_ABORT_COUNT} cycles "
                            f"({state['iq']:.2f} A)")
            over_v = over_v + 1 if abs(v_pos) > V_ABORT else 0
            if over_v >= V_ABORT_COUNT:
                raise Abort(f"speed {v_pos:.2f} rad/s (from position) > {V_ABORT} for "
                            f"{V_ABORT_COUNT} cycles")
            if phase != 'fade' and abs(sp - state['position']) > math.radians(POS_ERR_ABORT_DEG):
                raise Abort(f"{math.degrees(sp - state['position']):+.1f} deg from the target")
            if math.degrees(state['position']) < POS_MIN_ABORT_DEG:
                raise Abort(f"angle {math.degrees(state['position']):.1f} deg < "
                            f"{POS_MIN_ABORT_DEG} deg")
            if loop_dt > MAX_LOOP_DT_S:
                raise Abort(f"loop gap {loop_dt * 1000:.1f} ms > {MAX_LOOP_DT_S * 1000:.0f} ms")
            tmax = max(state['temp'], state['temp_powerstage'])
            if tmax >= TEMP_MAX:
                raise Abort(f"temperature {tmax:.1f} C >= {TEMP_MAX} C")
            if t_now - self.t_enable > a.cap_s:
                raise Abort(f"time cap {a.cap_s:.0f} s")
            if phase == 'thermal' and tmax >= TEMP_WARN:
                self.footer.append(f"thermal hold ended at {TEMP_WARN} C after "
                                   f"{now - seg_t0:.1f} s")
                seg_t0 = now - self.plan[seg_i][3]       # skip to the next segment
            # ---- pace -------------------------------------------------------
            nxt += PERIOD_S
            s = nxt - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                nxt = time.monotonic()

    # -- quick numbers (raw; the analyzer compares them with the sweep) --------
    def quick(self):
        if self.n == 0:
            return []
        d = self.buf[:self.n]
        col = {c: i for i, c in enumerate(COLUMNS)}
        out = []
        for k, p in enumerate(self.a.pred):
            m_st = (d[:, col['angle_idx']] == k) & (d[:, col['phase']] == PHASES.index('still'))
            m_cr = (d[:, col['angle_idx']] == k) & (d[:, col['phase']] == PHASES.index('creep'))
            m_th = (d[:, col['angle_idx']] == k) & (d[:, col['phase']] == PHASES.index('thermal'))
            parts = [f"{p['deg']:g} deg:"]
            if m_st.any():
                tt = d[m_st, col['t']]
                last = m_st & (d[:, col['t']] >= tt.max() - 5.0)
                parts.append(f"still u {KT * np.mean(d[last, col['iq']]):+.3f} N m "
                             f"(last 5 s), angle p-p {np.degrees(np.ptp(d[m_st, col['pos']])):.2f} deg")
            if m_cr.any():
                tt = d[m_cr, col['t']]
                per = 1.0 / self.a.creep_hz
                keep = m_cr & (d[:, col['t']] >= tt.min() + per)    # skip cycle 1
                ncyc = int(round((tt.max() - tt.min()) * self.a.creep_hz)) - 1
                if ncyc >= 1 and keep.any():
                    parts.append(f"creep u {KT * np.mean(d[keep, col['iq']]):+.3f} N m "
                                 f"(cycles 2-{ncyc + 1}), measured amp "
                                 f"{np.degrees(np.ptp(d[keep, col['pos']])) / 2:.2f} deg")
            if m_th.any():
                parts.append(f"thermal {d[m_th, col['t']].max() - d[m_th, col['t']].min():.0f} s, "
                             f"u {KT * np.mean(d[m_th, col['iq']]):+.3f} N m, winding "
                             f"{np.nanmin(d[m_th, col['w_temp']]):.1f}->"
                             f"{np.nanmax(d[m_th, col['w_temp']]):.1f} C")
            out.append('  '.join(parts))
        return out

    def save(self):
        made, p = [], os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        stem = 'hold_thermal' if self.a.thermal else 'hold_bench'
        path = os.path.join(LOG_SUBDIR, f"{stem}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            for line in self.header:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(COLUMNS)
            for r in self.buf[:self.n]:
                row = list(r)
                row[2] = PHASES[int(row[2])]
                row[3] = int(row[3])
                row[15] = hex(int(row[15]))
                w.writerow([f"{x:.9g}" if isinstance(x, float) else x for x in row])
            for line in self.footer:
                f.write(f"# {line}\n")
        uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
        if os.geteuid() == 0 and uid and gid:
            for q in made + [LOG_SUBDIR, path]:
                try:
                    os.chown(q, int(uid), int(gid))
                except OSError as e:
                    print(f"WARNING: chown {q} failed: {e}")
        print(f"\nLog: {path}")
        return path


def main(argv=None):
    a = parse_args(argv)
    if os.sched_getscheduler(0) != os.SCHED_FIFO:
        raise SystemExit(
            "REFUSED: not at real-time priority. From the repo root run:\n"
            "  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 "
            "useful_tools/hold_bench_test.py --sweep <csv> ...")
    iface = BearInterface()
    run = Run(iface, a)
    started = False
    try:
        run.preflight()
        if input("\nArm on the top stop, HANDS CLEAR. Type GO to start: ").strip() != 'GO':
            run.end_reason = 'not started (no GO)'
            print("Not started.")
            return 1
        started = True
        run.idle()
        run.run()
    except Abort as e:
        run.end_reason = f"ABORT: {e}"
        print(f"\nABORT: {e}")
    except RuntimeError as e:                  # configure_watchdog / enable refused
        run.end_reason = f"REFUSED/FAILED: {e}"
        print(f"\n{run.end_reason}")
    except KeyboardInterrupt:
        run.end_reason = 'interrupted (Ctrl-C)'
        print("\nInterrupted.")
    finally:
        try:
            if run.armed:      # a refusal before arming writes nothing at all
                ok = iface.disable()
                if ok:
                    iface.bear.set_watchdog_timeout((iface.id, 0))
                    (wd,), _ = iface.bear.get_watchdog_timeout(iface.id)[0]
                    run.footer.append(f"disable confirmed attempts={iface.last_disable_attempts} "
                                      f"ms={iface.last_disable_ms}; watchdog reset to {wd} us")
                    if run.limit_written:
                        try:
                            rb = iface.set_current_limit(I_MAX_DEFAULT)
                            run.footer.append(f"limit_i_max restored to {rb} A")
                        except Exception as e:             # noqa: BLE001
                            msg = (f"limit_i_max NOT restored ({e}); power-cycle the actuator "
                                   f"(flash holds {I_MAX_DEFAULT} A) before any other script")
                            print(f"\n{msg}")
                            run.footer.append(msg)
                else:
                    # Disable NOT confirmed: leave the watchdog ARMED. With no more traffic it
                    # trips within ~10 ms and the drive goes to damping (bench T2/T4). Resetting
                    # it here would remove the only backstop if torque_enable = 0 was lost
                    # (the Oct 13 failure mode). Claude Code review, Oct 27.
                    msg = ("DISABLE NOT CONFIRMED -- watchdog LEFT ARMED (trips in ~10 ms with no "
                           "traffic). CUT ACTUATOR POWER and check it before anything else."
                           + (f" limit_i_max left at {run.a.i_max:g} A (RAM; power cycle restores "
                              f"{I_MAX_DEFAULT})." if run.limit_written else ""))
                    print(f"\n{msg}")
                    run.footer.append(msg)
        finally:
            if getattr(run, 'vel_disagree', 0):
                run.footer.append(f"velocity register disagreed with the position by > "
                                  f"{VEL_SANITY} rad/s on {run.vel_disagree} cycles (D term used "
                                  f"the position-derived speed)")
            q = run.quick()
            run.footer += [f"quick: {line}" for line in q]
            run.footer.append(f"END {run.end_reason}")
            if started or run.n:
                run.save()
            iface.bear.close_port()
    print("\n" + "\n".join(run.footer))
    return 0 if run.end_reason == 'completed' else 1


if __name__ == '__main__':
    sys.exit(main())