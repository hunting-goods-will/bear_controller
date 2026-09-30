"""
Observer excitation harness. The ACTUATOR drives the arm in Position Mode; the
script logs a fixed-rate record of goal, position, velocity and current for
fitting a state observer offline.

USAGE
-----
    python3 useful_tools/observer_excitation.py --mode sweep \\
        --arm-mass-kg 0 --arm-com-m 0.2739
    python3 useful_tools/observer_excitation.py --mode sine \\
        --freq-hz 0.3 --arm-mass-kg 1.5025 --arm-com-m 0.2739

Real-time priority (sudo chrt -f 80) is deliberately deferred: the position
loop runs in the actuator firmware, so host jitter only delays goal updates
and samples -- it cannot destabilise the loop -- and every row logs loop_dt
so the jitter is visible in the data.

--arm-mass-kg / --arm-com-m are REQUIRED, no defaults, and parsed before
BearInterface() opens the port -- same rule as validate_model.py. They are
written into every CSV row.

MODES
-----
sweep  validate_model.py's constant-velocity legs: approach to SWEEP_TOP_DEG,
       down to SWEEP_BOTTOM_DEG, dwell, up to SWEEP_TOP_DEG, at --velocity.
sine   goal = center + A * env(t) * sin(2*pi*f*t), sent every loop, for
       --cycles cycles. env() is a smoothstep that ramps the amplitude IN over
       the first cycle and OUT over the last, so the goal starts and ends at
       center with zero velocity (--cycles >= 3 gives at least one full-
       amplitude cycle). Before the sine, the arm is brought to center with a
       constant-velocity approach leg at --velocity.

Both modes park before disabling, toward the stop the arm goes to when
released: HIGH (PARK_HIGH_DEG) on a bare rig, where the spring wins; LOW
(PARK_LOW_DEG, as validate_model.py) with a load, where the load wins. The
target and the reason are printed before the Enter prompt. The park
is monitored: it gives up and disables immediately (the arm may move) if
|present_iq| > PARK_IQ_ABORT_A for PARK_IQ_COUNT consecutive samples, if the
arm has not moved PARK_PROGRESS_DEG toward PARK_DEG within PARK_STALL_S, or at
PARK_DEADLINE_S. Park samples are logged with phase='park'.

--soft-start-s sets the ramp time at each end of a constant-velocity leg. In
sweep mode the constant-velocity span of each leg is printed before the port
opens and the script refuses if it is < MIN_CONST_SPAN_DEG.

ENVELOPE (checked BEFORE enabling; the script refuses to start if violated)
--------
  * the excitation trajectory stays inside actuator ENVELOPE_MIN_DEG ..
    ENVELOPE_MAX_DEG (20..100 deg). The only motion outside it is the
    approach from the rest position (the load rests on the top stop at
    115-119 deg and cannot be repositioned by hand -- same handling as
    validate_model.py) down to the start of the trajectory.
  * peak goal velocity <= MAX_GOAL_VELOCITY (A*2*pi*f for sine; --velocity
    for sweep and for the approach leg).
  * planned run time, including the approach, <= RUN_CAP_S.

RUNTIME ABORTS (all park, then disable)
--------------
  |present_iq| > IQ_ABORT_A for IQ_ABORT_COUNT consecutive samples, or
  > IQ_TRIP_A on any single sample; |goal - pos| > TRACKING_ABORT; error byte
  != 128; effective temperature >= TEMP_MAX; elapsed since enable > RUN_CAP_S;
  Ctrl-C.

NO TORQUE IS COMMANDED DIRECTLY. Position Mode does the driving.
"""
import argparse
import csv
import math
import os
import sys
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import (
    LOG_DIR, MIN_ANGLE, MAX_ANGLE, TEMP_WARN, TEMP_MAX, SAFETY_CHECKS_CONFIRMED)

# --- Envelope ----------------------------------------------------------------
ENVELOPE_MIN_DEG = 20.0      # the excitation trajectory must stay inside this
ENVELOPE_MAX_DEG = 100.0
MAX_GOAL_VELOCITY = 0.8      # rad/s, peak goal velocity anywhere in the run
RUN_CAP_S = 120.0            # s, enable-to-end (park not included)
# Current aborts. Set from MEASURED bare-rig current, not a friction-free
# estimate: the first run (observer_sweep_bare_20260930_144400) tripped the
# old 4.0 A threshold at 4.095 A, act 59.6 deg, on a normal down leg.
# August bare sweeps model_validation_20260820_234335 / _235718, steady-state
# down leg, max |iq| per 10 deg bin: 20 deg 4.76/4.82, 30 deg 4.65/4.67,
# 40 deg 4.51/4.53, 60 deg 4.20/4.43 A (down legs overall 3.2-4.8 A; up legs
# <= 3.1 A). Wrench up leg predicted ~4.5 A with friction. Firmware
# limit_i_max stays 5.5 A (LIMIT_I_MAX below).
IQ_ABORT_A = 5.0             # A, |present_iq| sustained abort threshold...
IQ_ABORT_COUNT = 3           # ...for this many consecutive samples
IQ_TRIP_A = 5.3              # A, |present_iq| immediate abort (one sample)
MIN_SOFT_START_S = 0.3       # s, floor on --soft-start-s
MIN_CONST_SPAN_DEG = 20.0    # deg, sweep legs must have this much at full speed

# --- Arguments: parsed and checked BEFORE BearInterface() opens the port ----
_ap = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
_ap.add_argument('--mode', choices=('sweep', 'sine'), required=True)
_ap.add_argument('--arm-mass-kg', type=float, required=True,
                 help='mass of the load actually fitted, kg (0 for bare rig)')
_ap.add_argument('--arm-com-m', type=float, required=True,
                 help='distance from pivot to the load CoM, m')
_ap.add_argument('--velocity', type=float, default=0.10,
                 help='sweep leg / approach velocity, rad/s (default 0.10)')
_ap.add_argument('--soft-start-s', type=float, default=1.5,
                 help=f'leg ramp time at each end, s (default 1.5, '
                      f'min {MIN_SOFT_START_S})')
_ap.add_argument('--freq-hz', type=float, default=None,
                 help='sine frequency, Hz (required for --mode sine)')
_ap.add_argument('--amp-deg', type=float, default=20.0,
                 help='sine amplitude, deg (default 20)')
_ap.add_argument('--center-deg', type=float, default=60.0,
                 help='sine center, actuator deg (default 60)')
_ap.add_argument('--cycles', type=int, default=5,
                 help='sine cycles incl. ramp-in and ramp-out (default 5, min 3)')
_args = _ap.parse_args()

if not SAFETY_CHECKS_CONFIRMED:
    raise SystemExit("REFUSED: config.SAFETY_CHECKS_CONFIRMED is False. Trace "
                     "the ESTOP wiring, check the bench fixture, confirm real "
                     "tunables, then have a human set it.")
# NaN compares False against everything, so it would slip through every
# envelope check below. Reject non-finite values first.
for _name in ('arm_mass_kg', 'arm_com_m', 'velocity', 'soft_start_s',
              'freq_hz', 'amp_deg', 'center_deg'):
    _v = getattr(_args, _name)
    if _v is not None and not math.isfinite(_v):
        _ap.error(f"--{_name.replace('_', '-')} must be finite")
if _args.arm_mass_kg < 0 or _args.arm_com_m < 0:
    _ap.error("--arm-mass-kg and --arm-com-m must be >= 0")
ARM_MASS_KG = _args.arm_mass_kg
ARM_COM_M = _args.arm_com_m
MODE = _args.mode
VELOCITY = _args.velocity
if not 0.0 < VELOCITY <= MAX_GOAL_VELOCITY:
    _ap.error(f"--velocity must be in (0, {MAX_GOAL_VELOCITY}] rad/s")
SOFT_START = _args.soft_start_s   # s to ramp velocity up at the start of a leg
if not SOFT_START >= MIN_SOFT_START_S:
    _ap.error(f"--soft-start-s must be >= {MIN_SOFT_START_S}")

# --- Sweep (validate_model.py's legs, clipped to the envelope) --------------
SWEEP_TOP_DEG = 100.0
SWEEP_BOTTOM_DEG = 30.0      # was 20: bare down-leg |iq| peaks ~4.8 A at 20
                             # deg vs ~4.65 A at 30 (see IQ_ABORT_A)
DECEL_RAD = VELOCITY * SOFT_START   # distance over which to decelerate
RAMP_FLOOR = 0.15            # minimum ramp; without it the step decays to zero
ARRIVE_EPS = math.radians(0.5)
DWELL_AT_END = 2.0           # s hold between legs (logged and monitored)

# --- Gains: same block as validate_model.py ---------------------------------
IQ_P, IQ_I, IQ_D = 0.02, 0.02, 0.0
VEL_P, VEL_I, VEL_D = 4.5, 0.001, 0.0
POS_P, POS_I, POS_D = 5.0, 0.0, 0.2
LIMIT_I_MAX = 5.5
LIMIT_VELOCITY_MAX = 1.0
LIMIT_ACC_MAX = 5.0

TRACKING_ABORT = math.radians(15.0)   # goal vs actual divergence

# Written in this order, then every one read back and compared before enabling.
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
    ('limit_position_max', MAX_ANGLE),
]

# Registers are float32; anything further off than this did not take.
READBACK_RTOL, READBACK_ATOL = 1e-4, 1e-6

# Start above MAX_ANGLE with a load: same handling as validate_model.py.
HARD_STOP_DEG = 119.5

# Park before disabling, TOWARD THE STOP THE ARM GOES TO WHEN RELEASED, so the
# release is a short move into the stop it would reach anyway:
#  * bare rig (arm_mass_kg == 0): the spring beats rig gravity everywhere, so a
#    released arm springs UP into the top stop (115-119 deg). Park HIGH. The
#    first run parked a bare arm low at 24 deg (drawing up to -4.8 A to pull it
#    down against the spring) and it sprang the full range into the top stop.
#  * loaded: the load beats the spring and a released arm drops into the
#    bottom stop. Park LOW, as validate_model.py does.
PARK_HIGH_DEG = 110.0        # below MAX_ANGLE (118)
PARK_LOW_DEG = 24.0
if ARM_MASS_KG == 0:
    PARK_DEG, PARK_DIR = PARK_HIGH_DEG, +1.0
    PARK_WHY = ("bare rig (arm_mass_kg == 0): the spring beats rig gravity "
                "everywhere, so a released arm springs UP into the top stop")
else:
    PARK_DEG, PARK_DIR = PARK_LOW_DEG, -1.0
    PARK_WHY = (f"loaded (arm_mass_kg = {ARM_MASS_KG}): the load beats the "
                f"spring, so a released arm drops DOWN into the bottom stop")
PARK_VELOCITY = 0.15
PARK_DEADLINE_S = 20.0       # s, give up and release (validate_model: 60)
PARK_IQ_ABORT_A = 5.0        # A, release if |present_iq| exceeds this...
PARK_IQ_COUNT = 3            # ...for this many consecutive samples
PARK_PROGRESS_DEG = 1.0      # deg toward PARK_DEG that must happen...
PARK_STALL_S = 2.0           # ...within this long, or release

# --- Loop timing --------------------------------------------------------------
LOOP_PERIOD_S = 0.005        # 200 Hz target; actual spacing is logged as loop_dt
MIN_SLEEP_S = 0.0005         # always sleep at least this, even on overrun


def sine_envelope(t, freq, cycles):
    """Smoothstep amplitude ramp: 0 -> 1 over the first cycle, 1 -> 0 over the
    last. Keeps |d goal / dt| <= A*2*pi*f throughout (checked numerically
    below), with zero velocity at both ends."""
    x = max(0.0, min(1.0, t * freq, cycles - t * freq))
    return x * x * (3.0 - 2.0 * x)


def sine_goal(t, center, amp, freq, cycles):
    return center + amp * sine_envelope(t, freq, cycles) * math.sin(
        2.0 * math.pi * freq * t)


def leg_time(dist_rad, velocity):
    """Conservative leg duration: cruise plus both soft ramps."""
    return dist_rad / velocity + 2.0 * SOFT_START


# --- Envelope checks (pure, before any hardware) -----------------------------
ENV_MIN = math.radians(ENVELOPE_MIN_DEG)
ENV_MAX = math.radians(ENVELOPE_MAX_DEG)
if not (MIN_ANGLE < ENV_MIN < ENV_MAX < MAX_ANGLE):
    raise SystemExit("REFUSED: envelope is not inside config MIN/MAX_ANGLE.")
if not MIN_ANGLE < math.radians(PARK_DEG) < MAX_ANGLE:
    raise SystemExit(f"REFUSED: park target {PARK_DEG} deg is not strictly "
                     f"inside MIN_ANGLE..MAX_ANGLE.")

if MODE == 'sine':
    if _args.freq_hz is None:
        _ap.error("--freq-hz is required for --mode sine")
    FREQ_HZ = _args.freq_hz
    AMP_DEG = _args.amp_deg
    CENTER_DEG = _args.center_deg
    CYCLES = _args.cycles
    if FREQ_HZ <= 0 or AMP_DEG <= 0:
        _ap.error("--freq-hz and --amp-deg must be > 0")
    if CYCLES < 3:
        _ap.error("--cycles must be >= 3 (ramp-in, >=1 full, ramp-out)")
    if (CENTER_DEG - AMP_DEG < ENVELOPE_MIN_DEG
            or CENTER_DEG + AMP_DEG > ENVELOPE_MAX_DEG):
        _ap.error(f"sine spans {CENTER_DEG - AMP_DEG:.1f}..{CENTER_DEG + AMP_DEG:.1f}"
                  f" deg, outside envelope {ENVELOPE_MIN_DEG:.0f}.."
                  f"{ENVELOPE_MAX_DEG:.0f} deg")
    peak_v = math.radians(AMP_DEG) * 2.0 * math.pi * FREQ_HZ
    if peak_v > MAX_GOAL_VELOCITY:
        _ap.error(f"peak goal velocity A*2*pi*f = {peak_v:.3f} rad/s exceeds "
                  f"{MAX_GOAL_VELOCITY} rad/s")
    EXCITE_S = CYCLES / FREQ_HZ
    # Backstop: sample the exact goal function that will be sent and check it.
    _dt = min(1e-3, 1.0 / (FREQ_HZ * 2000.0))
    _prev, _lo, _hi, _vmax = None, float('inf'), float('-inf'), 0.0
    for _i in range(int(EXCITE_S / _dt) + 1):
        _g = sine_goal(_i * _dt, math.radians(CENTER_DEG), math.radians(AMP_DEG),
                       FREQ_HZ, CYCLES)
        _lo, _hi = min(_lo, _g), max(_hi, _g)
        if _prev is not None:
            _vmax = max(_vmax, abs(_g - _prev) / _dt)
        _prev = _g
    if _lo < ENV_MIN or _hi > ENV_MAX or _vmax > MAX_GOAL_VELOCITY:
        raise SystemExit(f"REFUSED: sampled sine goal spans "
                         f"{math.degrees(_lo):.2f}..{math.degrees(_hi):.2f} deg, "
                         f"peak {_vmax:.3f} rad/s")
    START_DEG = CENTER_DEG
    CONST_SPAN_DEG = None    # sine has no constant-velocity legs to check
else:
    if _args.freq_hz is not None:
        _ap.error("--freq-hz only applies to --mode sine")
    FREQ_HZ = AMP_DEG = CENTER_DEG = CYCLES = None
    if not (ENVELOPE_MIN_DEG <= SWEEP_BOTTOM_DEG < SWEEP_TOP_DEG
            <= ENVELOPE_MAX_DEG):
        raise SystemExit("REFUSED: sweep endpoints outside the envelope.")
    # Deceleration occupies the last VELOCITY*SOFT_START of each leg (DECEL_RAD);
    # the accel ramp covers about half that. Charge a full VELOCITY*SOFT_START to
    # both ends, so this under-reports the span rather than over-reporting it.
    _leg_rad = math.radians(SWEEP_TOP_DEG - SWEEP_BOTTOM_DEG)
    CONST_SPAN_DEG = math.degrees(_leg_rad - 2.0 * VELOCITY * SOFT_START)
    print(f"Constant-velocity span per sweep leg: {CONST_SPAN_DEG:.1f} deg of "
          f"{SWEEP_TOP_DEG - SWEEP_BOTTOM_DEG:.0f} deg "
          f"(--velocity {VELOCITY}, --soft-start-s {SOFT_START}; "
          f"min {MIN_CONST_SPAN_DEG:.0f})")
    if CONST_SPAN_DEG < MIN_CONST_SPAN_DEG:
        _ap.error(f"constant-velocity span {CONST_SPAN_DEG:.1f} deg < "
                  f"{MIN_CONST_SPAN_DEG:.0f} deg; lower --velocity or "
                  f"--soft-start-s")
    peak_v = VELOCITY
    EXCITE_S = (2.0 * leg_time(math.radians(SWEEP_TOP_DEG - SWEEP_BOTTOM_DEG),
                               VELOCITY) + DWELL_AT_END)
    START_DEG = SWEEP_TOP_DEG

if EXCITE_S > RUN_CAP_S:
    _ap.error(f"planned excitation {EXCITE_S:.1f}s exceeds the {RUN_CAP_S:.0f}s "
              f"run cap")

LOAD_LABEL = 'bare' if ARM_MASS_KG == 0 else 'wrench'
log_path = os.path.join(
    LOG_DIR, 'phase2', 'observer_csvs',
    f"observer_{MODE}_{LOAD_LABEL}_{time.strftime('%Y%m%d_%H%M%S')}.csv")


class Abort(RuntimeError):
    """A runtime safety check tripped. Caught at top level -> park, disable."""


iface = BearInterface()
S = {}   # loop state: t_enable, t_prev, next, n, temps, warned


def read():
    pos, err = iface.bear.get_present_position(iface.id)[0]
    vel, _ = iface.bear.get_present_velocity(iface.id)[0]
    iq, _ = iface.bear.get_present_iq(iface.id)[0]
    return pos[0], vel[0], iq[0], err


def thermal_ok(w, p):
    eff = max(w, p)
    if eff >= TEMP_MAX:
        raise Abort(f"Effective temp {eff:.1f}C >= TEMP_MAX")
    return eff


def log_row(writer, now, goal, pos, vel, iq, phase):
    """One CSV row. Also used by the park loop, so t and loop_dt run on
    continuously from the excitation into the park."""
    loop_dt = now - S['t_prev']
    S['t_prev'] = now
    temps = S['temps']
    writer.writerow([
        f"{now - S['t_enable']:.6f}", f"{loop_dt:.6f}", f"{goal:.6f}",
        f"{pos:.6f}", f"{vel:.6f}", f"{iq:.5f}",
        f"{temps['w']:.1f}", f"{temps['p']:.1f}", MODE, phase,
        f"{FREQ_HZ:.5f}" if FREQ_HZ is not None else '',
        f"{AMP_DEG:.3f}" if AMP_DEG is not None else '',
        f"{CENTER_DEG:.3f}" if CENTER_DEG is not None else '',
        f"{VELOCITY:.4f}", f"{SOFT_START:.3f}",
        f"{ARM_MASS_KG:.4f}", f"{ARM_COM_M:.4f}",
    ])
    return loop_dt


def tick(writer, goal, phase, now):
    """One control cycle: send goal, read, log, check, sleep to the deadline.
    Logged before checking, so the sample that trips an abort is in the CSV."""
    iface.bear.set_goal_position((iface.id, goal))
    pos, vel, iq, err = read()
    S['n'] += 1
    loop_dt = log_row(writer, now, goal, pos, vel, iq, phase)

    if err != 128:
        raise Abort(f"Error byte {err}")
    if abs(iq) > IQ_TRIP_A:
        raise Abort(f"|present_iq| {abs(iq):.3f} A > {IQ_TRIP_A} A at "
                    f"{math.degrees(pos):.1f} deg (immediate trip)")
    S['iq_over'] = S['iq_over'] + 1 if abs(iq) > IQ_ABORT_A else 0
    if S['iq_over'] >= IQ_ABORT_COUNT:
        raise Abort(f"|present_iq| > {IQ_ABORT_A} A for {S['iq_over']} "
                    f"consecutive samples (last {abs(iq):.3f} A at "
                    f"{math.degrees(pos):.1f} deg)")
    if abs(goal - pos) > TRACKING_ABORT:
        raise Abort(f"Position loop diverged: goal {math.degrees(goal):.1f} vs "
                    f"actual {math.degrees(pos):.1f} deg")
    if now - S['t_enable'] > RUN_CAP_S:
        raise Abort(f"Run cap {RUN_CAP_S:.0f}s exceeded")

    temps = S['temps']
    if now - temps['t'] > 1.0:
        w, _ = iface.bear.get_winding_temperature(iface.id)[0]
        p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
        temps.update(w=w[0], p=p[0], t=now)
        eff = thermal_ok(temps['w'], temps['p'])
        if eff >= TEMP_WARN and not S['warned']:
            print(f"  WARNING: effective temp {eff:.1f}C >= TEMP_WARN "
                  f"({TEMP_WARN}C). Aborts at {TEMP_MAX}C.")
            S['warned'] = True

    if S['n'] % 200 == 0:
        print(f"  {phase:8s} goal {math.degrees(goal):6.2f}  act "
              f"{math.degrees(pos):6.2f}  vel {vel:+6.3f}  iq {iq:+6.3f}  "
              f"dt {loop_dt * 1000:5.2f}ms  {temps['w']:.0f}/{temps['p']:.0f}C")

    # Fixed-rate loop that always sleeps. On overrun, resync rather than try
    # to catch up with back-to-back cycles.
    S['next'] += LOOP_PERIOD_S
    delay = S['next'] - time.monotonic()
    if delay < MIN_SLEEP_S:
        delay = MIN_SLEEP_S
        S['next'] = time.monotonic() + MIN_SLEEP_S
    time.sleep(delay)


def run_leg(writer, phase, start, target):
    """validate_model.py's constant-velocity leg, at --velocity."""
    direction = 1.0 if target > start else -1.0
    goal = start
    t0 = time.monotonic()
    t_prev = t0
    print(f"\n--- {phase.upper()}: {math.degrees(start):.1f} -> "
          f"{math.degrees(target):.1f} deg at {VELOCITY:.2f} rad/s ---")

    while True:
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        elapsed = now - t0

        remaining = abs(target - goal)
        ramp = min(1.0, elapsed / SOFT_START, remaining / DECEL_RAD)
        ramp = max(ramp, RAMP_FLOOR)
        goal += direction * VELOCITY * ramp * dt
        goal = max(MIN_ANGLE, min(goal, MAX_ANGLE))
        arrived = (goal - target) * direction >= 0 or remaining < ARRIVE_EPS
        if arrived:
            goal = target
        tick(writer, goal, phase, now)
        if arrived:
            return goal


def hold(writer, phase, goal, duration):
    """Hold a fixed goal, still logging and checking every cycle."""
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        tick(writer, goal, phase, now)
        if now - t0 >= duration:
            return


def run_sine(writer):
    center = math.radians(CENTER_DEG)
    amp = math.radians(AMP_DEG)
    duration = CYCLES / FREQ_HZ
    print(f"\n--- SINE: {CENTER_DEG:.1f} +/- {AMP_DEG:.1f} deg at {FREQ_HZ:.3f} Hz, "
          f"{CYCLES} cycles ({duration:.1f}s), peak {peak_v:.3f} rad/s ---")
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        t = now - t0
        if t >= duration:
            tick(writer, center, 'sine', now)
            return center
        goal = sine_goal(t, center, amp, FREQ_HZ, CYCLES)
        goal = max(ENV_MIN, min(goal, ENV_MAX))
        tick(writer, goal, 'sine', now)


def park_and_disable(f, writer):
    """Move under control toward the stop the arm goes to when released
    (PARK_DIR: up for a bare rig, down with a load), THEN disable, so the
    release is a short move into that stop. Runs on every exit path once
    enable was attempted. If the arm is already at or past PARK_DEG in
    PARK_DIR it releases directly -- driving it back would only lengthen the
    move into the stop. The park is monitored and disables IMMEDIATELY --
    the arm may move into its stop -- if:
      * |present_iq| > PARK_IQ_ABORT_A for PARK_IQ_COUNT consecutive samples,
      * the arm has not moved PARK_PROGRESS_DEG toward PARK_DEG in PARK_STALL_S,
      * PARK_DEADLINE_S passes,
      * a read/write fails (a comm failure may be why we are exiting), or
      * Ctrl-C arrives mid-park.
    Samples are logged with phase='park' when a log is open."""
    release = None
    parked = False
    try:
        pos, vel, iq, _ = read()
        target = math.radians(PARK_DEG)
        # Signed distance still to travel toward the target, in PARK_DIR.
        if (target - pos) * PARK_DIR <= math.radians(1.0):
            print(f"Already at/past the park target ({PARK_DEG:.0f} deg, "
                  f"{'up' if PARK_DIR > 0 else 'down'}). Releasing.")
            parked = True
        else:
            print(f"Parking {'UP' if PARK_DIR > 0 else 'DOWN'} from "
                  f"{math.degrees(pos):.1f} to {PARK_DEG:.0f} deg before "
                  f"release...")
            goal = pos
            now = time.monotonic()
            t_prev = now
            deadline = now + PARK_DEADLINE_S
            ref_pos, ref_t = pos, now     # progress-watchdog reference
            iq_over = 0
            while (target - goal) * PARK_DIR > 0:
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                goal += PARK_DIR * PARK_VELOCITY * dt
                if (goal - target) * PARK_DIR > 0:     # never step past target
                    goal = target
                iface.bear.set_goal_position((iface.id, goal))
                pos, vel, iq, _ = read()
                if writer is not None:
                    log_row(writer, now, goal, pos, vel, iq, 'park')

                iq_over = iq_over + 1 if abs(iq) > PARK_IQ_ABORT_A else 0
                if iq_over >= PARK_IQ_COUNT:
                    release = (f"|present_iq| > {PARK_IQ_ABORT_A} A for "
                               f"{iq_over} consecutive samples during park")
                    break
                if (pos - ref_pos) * PARK_DIR > math.radians(PARK_PROGRESS_DEG):
                    ref_pos, ref_t = pos, now
                elif now - ref_t > PARK_STALL_S:
                    release = (f"no {PARK_PROGRESS_DEG:.0f} deg of progress "
                               f"toward {PARK_DEG:.0f} deg in {PARK_STALL_S:.0f}s "
                               f"(stuck at {math.degrees(pos):.1f} deg)")
                    break
                if now > deadline:
                    release = f"park deadline {PARK_DEADLINE_S:.0f}s passed"
                    break
                time.sleep(0.005)
            else:
                print("Parked.")
                parked = True
    except Exception as e:
        release = f"park failed ({e})"
    finally:
        if not parked and release is None:
            release = "park interrupted"
        if release is not None:
            print(f"\nRELEASING EARLY: {release}.\n"
                  f"{'THE ARM MAY SPRING UP INTO THE TOP STOP' if PARK_DIR > 0 else 'THE LOAD MAY DROP INTO THE BOTTOM STOP'}"
                  f" -- support the arm.")
        # Disable before touching the log: a failed write must not block it.
        iface.disable()
        if f is not None:
            try:
                f.write(f"# PARK: {'released early -- ' + release if release else 'parked'}\n")
            except Exception:
                pass


enable_attempted = False
f = writer = None
end_note = 'unhandled exception'   # overwritten on every handled path
try:
    pos, vel, iq, err = read()
    w, _ = iface.bear.get_winding_temperature(iface.id)[0]
    p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
    S['temps'] = {'w': w[0], 'p': p[0], 't': time.monotonic()}
    S['warned'] = False

    print("=" * 74)
    print(f"  OBSERVER EXCITATION -- mode {MODE}, Position Mode drives")
    print(f"  load {ARM_MASS_KG:.4f} kg at {ARM_COM_M * 1000:.1f} mm ({LOAD_LABEL})")
    print("=" * 74)
    print(f"pos {math.degrees(pos):.2f} deg  err {err}  "
          f"w {S['temps']['w']:.1f}C  p {S['temps']['p']:.1f}C")

    if err != 128:
        raise Abort(f"Error byte {err} before enabling")
    if max(S['temps']['w'], S['temps']['p']) >= TEMP_WARN:
        raise Abort(f"Already at/above TEMP_WARN ({TEMP_WARN}C) before "
                    f"enabling. Let it cool.")
    if pos <= MIN_ANGLE:
        raise SystemExit("Start position below MIN_ANGLE.")
    if pos >= math.radians(HARD_STOP_DEG):
        raise SystemExit(
            f"Start position {math.degrees(pos):.2f} deg is above the expected "
            f"hard stop ({HARD_STOP_DEG} deg). Check the homing offset.")
    if pos >= MAX_ANGLE:
        print(f"\nNote: resting at {math.degrees(pos):.2f} deg, above MAX_ANGLE "
              f"({math.degrees(MAX_ANGLE):.1f}). Position Mode will grab here and "
              f"the first goal is clamped into range. This is expected -- the "
              f"load cannot be repositioned by hand.")

    approach_s = leg_time(abs(pos - math.radians(START_DEG)), VELOCITY)
    planned_s = approach_s + EXCITE_S
    if planned_s > RUN_CAP_S:
        raise Abort(f"Planned run {planned_s:.1f}s (approach {approach_s:.1f}s) "
                    f"exceeds the {RUN_CAP_S:.0f}s run cap")

    for name, val in SETTINGS:
        getattr(iface.bear, 'set_' + name)((iface.id, val))

    readback = []
    print(f"\n{'register':>20}{'set':>12}{'readback':>14}")
    for name, val in SETTINGS:
        rb, _ = getattr(iface.bear, 'get_' + name)(iface.id)[0]
        rb = rb[0]
        readback.append((name, val, rb))
        ok = rb is not None and abs(rb - val) <= READBACK_ATOL + READBACK_RTOL * abs(val)
        print(f"{name:>20}{val:12.6f}{rb if rb is None else f'{rb:14.6f}'}"
              f"{'' if ok else '   <-- MISMATCH'}")
        if not ok:
            raise Abort(f"Readback mismatch on {name}: set {val}, read {rb}")

    print(f"\nPlanned run {planned_s:.0f}s (approach {approach_s:.0f}s + "
          f"{MODE} {EXCITE_S:.0f}s), cap {RUN_CAP_S:.0f}s. Peak goal velocity "
          f"{peak_v:.3f} rad/s.\nKeep a hand near the arm but do NOT support it.")
    print(f"\nPARK: {'HIGH' if PARK_DIR > 0 else 'LOW'} at {PARK_DEG:.0f} deg, "
          f"then disable.\n  Why: {PARK_WHY}.")
    input("Press Enter to enable and begin: ")

    # Re-read: the arm may have moved while waiting at the prompt, and
    # enabling at a stale position would lurch to it.
    pos, _, _, err = read()
    if err != 128:
        raise Abort(f"Error byte {err} before enabling")
    if pos <= MIN_ANGLE or pos >= math.radians(HARD_STOP_DEG):
        raise SystemExit(f"Position {math.degrees(pos):.2f} deg out of range "
                         f"at enable.")

    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    # Not a `with` block: the file stays open through the park (finally
    # below), so park samples land in the same log.
    f = open(log_path, 'w', newline='')
    f.write(f"# observer_excitation.py  started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
    f.write(f"# argv: {' '.join(sys.argv)}\n")
    f.write(f"# mode={MODE} arm_mass_kg={ARM_MASS_KG} arm_com_m={ARM_COM_M} "
            f"velocity={VELOCITY} soft_start_s={SOFT_START} freq_hz={FREQ_HZ} "
            f"amp_deg={AMP_DEG} center_deg={CENTER_DEG} cycles={CYCLES}\n")
    f.write(f"# units: t s since the enable command (time.monotonic); loop_dt s; "
            f"goal_pos, pos rad; vel rad/s (present_velocity register); "
            f"present_iq A; w_temp, p_temp C (sampled ~1 Hz, not refreshed "
            f"during park)\n")
    f.write(f"# envelope_deg={ENVELOPE_MIN_DEG}..{ENVELOPE_MAX_DEG} "
            f"max_goal_velocity={MAX_GOAL_VELOCITY} run_cap_s={RUN_CAP_S} "
            f"iq_abort_a={IQ_ABORT_A}x{IQ_ABORT_COUNT} iq_trip_a={IQ_TRIP_A} "
            f"tracking_abort_deg={math.degrees(TRACKING_ABORT):.1f} "
            f"loop_period_s={LOOP_PERIOD_S} const_span_deg="
            f"{'' if CONST_SPAN_DEG is None else f'{CONST_SPAN_DEG:.1f}'}\n")
    f.write(f"# park: deg={PARK_DEG} dir={'up' if PARK_DIR > 0 else 'down'} "
            f"velocity={PARK_VELOCITY} "
            f"deadline_s={PARK_DEADLINE_S} iq_abort_a={PARK_IQ_ABORT_A}x"
            f"{PARK_IQ_COUNT} progress={PARK_PROGRESS_DEG}deg/{PARK_STALL_S}s\n")
    f.write(f"# start_pos_deg={math.degrees(pos):.3f} "
            f"start_w_temp={S['temps']['w']:.1f} "
            f"start_p_temp={S['temps']['p']:.1f} "
            f"planned_s={planned_s:.1f}\n")
    f.write("# readback (register=set/read): " + " ".join(
        f"{n}={v:.6g}/{rb:.6g}" for n, v, rb in readback) + "\n")
    writer = csv.writer(f)
    writer.writerow(['t', 'loop_dt', 'goal_pos', 'pos', 'vel', 'present_iq',
                     'w_temp', 'p_temp', 'mode', 'phase', 'freq_hz',
                     'amp_deg', 'center_deg', 'velocity', 'soft_start_s',
                     'arm_mass_kg', 'arm_com_m'])

    # Loop state is initialised BEFORE the enable command, so the park can
    # always log even if enabling itself fails part-way.
    now = time.monotonic()
    S.update(t_enable=now, t_prev=now, next=now, n=0, iq_over=0)
    enable_attempted = True
    iface.enable_position_mode(pos)

    start = math.radians(START_DEG)
    cur = run_leg(writer, 'approach', pos, start)
    if MODE == 'sweep':
        cur = run_leg(writer, 'down', cur, math.radians(SWEEP_BOTTOM_DEG))
        print(f"\nHolding {DWELL_AT_END:.0f}s before the up leg...")
        hold(writer, 'dwell', cur, DWELL_AT_END)
        run_leg(writer, 'up', cur, math.radians(SWEEP_TOP_DEG))
    else:
        run_sine(writer)
    end_note = 'complete'
    print("\nComplete.")

except Abort as e:
    end_note = f"abort -- {e}"
    print(f"\nABORT: {e}")
    exit_code = 1
except KeyboardInterrupt:
    end_note = "abort -- Ctrl-C"
    print("\nInterrupted.")
    exit_code = 130
else:
    exit_code = 0
finally:
    if f is not None:
        try:
            f.write(f"# END: {end_note}\n")
        except Exception:
            pass
    if enable_attempted:
        print("Parking before release.")
        park_and_disable(f, writer)
    else:
        # Never enabled: nothing to park. Still leave it zeroed and disabled.
        iface.disable()
    if f is not None:
        f.close()
        print(f"Raw data: {log_path}")

sys.exit(exit_code)
