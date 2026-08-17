"""
Bidirectional Position-Mode sweep — separates tau_spring(theta) from friction.

WHY THIS EXISTS
---------------
position_hold_characterization.py swept one direction only. A single-direction
sweep fuses spring torque and friction inseparably: every point settles on the
same edge of the gearbox stiction band, so iq_hold = f(spring) + f(friction)
with the friction sign fixed by the approach direction.

The controller assists on the way UP. Friction there has the OPPOSITE sign from
the down-sweep dataset, so using that data directly biases the assist low by
roughly 2x the friction torque.

This script visits the SAME absolute angles twice — once descending, once
ascending — so that at each grid angle:

    tau_spring(theta) = tau_gravity(theta + PHI) - (tau_act_up + tau_act_down)/2
    tau_friction(theta) = (tau_act_up - tau_act_down) / 2

PREDICTION TO CHECK BEFORE POST-PROCESSING
------------------------------------------
Static friction resists the LAST direction of motion.
  Approached from above: friction acts up   -> actuator pushes harder down
  Approached from below: friction acts down -> actuator pushes less hard down
So UP-leg iq_hold should be consistently LESS NEGATIVE than DOWN-leg.
If it comes back MORE negative, STOP. Either the sign reasoning is wrong or the
approach directions aren't what the labels say. Do not post-process.

REQUIREMENTS
------------
config.MAX_ANGLE must be >= GRID_TOP + MAX_ANGLE_MARGIN (i.e. set it to 118 for
this run). Asserted at startup rather than left to memory.
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import (
    MIN_ANGLE, MAX_ANGLE, LOG_DIR, SAFETY_CHECKS_CONFIRMED, TEMP_WARN, TEMP_MAX
)

# --- Gains: unchanged from the run that produced 12/12 clean targets ---
IQ_ID_P, IQ_ID_I, IQ_ID_D = 0.02, 0.02, 0.0
VEL_P, VEL_I, VEL_D = 4.5, 0.001, 0.0
POS_P, POS_I, POS_D = 5.0, 0.0, 0.2

LIMIT_I_MAX = 5.5
LIMIT_VELOCITY_MAX = 1.0
LIMIT_ACC_MAX = 5.0

# --- CHANGE 1: absolute grid, not start-relative -----------------------------
# The old script built targets as start_position + direction*step*i, which is
# why the last run landed on 106.93/96.85/86.90 -- arbitrary offsets from
# wherever the arm happened to be sitting. Differencing two legs requires the
# SAME nominal angles on both, otherwise you interpolate, and interpolation
# error lands directly on the friction term you're trying to extract.
GRID_TOP_DEG = 110.0
GRID_BOTTOM_DEG = 10.0
GRID_STEP_DEG = 10.0
MAX_ANGLE_MARGIN_DEG = 5.0

# --- CHANGE 2: tighter settle tolerance --------------------------------------
# 3 deg was fine reading one curve. Differencing two curves turns a 3 deg
# mismatch between legs into dtau/dtheta error (~0.03-0.09 Nm at the slopes in
# the existing data). Small next to a suspected ~1 Nm friction term, but 1 deg
# costs almost nothing.
POSITION_SETTLE_TOLERANCE = math.radians(1.0)

RAMP_STEP_SIZE = 0.03
RAMP_STEP_SLEEP = 0.05
SETTLE_VELOCITY_THRESHOLD = 0.05
SETTLE_DWELL = 1.5
PER_TARGET_TIMEOUT = 30.0
POLL_SLEEP = 0.1

# Only rewrite limit_i_max when it actually changes materially -- saves an
# RS485 round-trip per poll. Irrelevant here at ~10Hz; matters at 500Hz.
LIMIT_WRITE_DEADBAND = 0.05


if not SAFETY_CHECKS_CONFIRMED:
    raise SystemExit(
        "\nBLOCKED: config.SAFETY_CHECKS_CONFIRMED is False.\n"
        "Same risk category as every other live script here."
    )

# Safety-by-construction: refuse to start if the config can't hold the grid.
_required_max = math.radians(GRID_TOP_DEG + MAX_ANGLE_MARGIN_DEG)
if MAX_ANGLE < _required_max:
    raise SystemExit(
        f"\nBLOCKED: MAX_ANGLE is {math.degrees(MAX_ANGLE):.1f} deg but this "
        f"sweep needs at least {GRID_TOP_DEG + MAX_ANGLE_MARGIN_DEG:.1f} deg "
        f"to reach GRID_TOP={GRID_TOP_DEG} deg with margin.\n"
        f"Set MAX_ANGLE = math.radians(118) in config.py for this run."
    )
if MIN_ANGLE >= math.radians(GRID_BOTTOM_DEG):
    raise SystemExit(
        f"\nBLOCKED: MIN_ANGLE ({math.degrees(MIN_ANGLE):.1f} deg) is at or "
        f"above GRID_BOTTOM ({GRID_BOTTOM_DEG} deg)."
    )

os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(
    LOG_DIR, f"bidirectional_sweep_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)

iface = BearInterface()
_last_written_limit = None


def read_state():
    pos, err = iface.bear.get_present_position(iface.id)[0]
    vel, _ = iface.bear.get_present_velocity(iface.id)[0]
    iq, _ = iface.bear.get_present_iq(iface.id)[0]
    id_current, _ = iface.bear.get_present_id(iface.id)[0]
    voltage, _ = iface.bear.get_input_voltage(iface.id)[0]
    w_temp, _ = iface.bear.get_winding_temperature(iface.id)[0]
    p_temp, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
    return {
        'position': pos[0], 'velocity': vel[0], 'iq': iq[0],
        'id': id_current[0], 'voltage': voltage[0],
        'winding_temp': w_temp[0], 'powerstage_temp': p_temp[0],
        'error': err,
    }


def thermal_limit_scale(state):
    """Scales limit_i_max off max(winding, powerstage) -- matches the firmware's
    own fault logic. Winding-only monitoring is a false sense of safety."""
    global _last_written_limit
    w, p = state['winding_temp'], state['powerstage_temp']
    if w is None or p is None:
        return LIMIT_I_MAX
    effective = max(w, p)
    if effective < TEMP_WARN:
        scaled = LIMIT_I_MAX
    elif effective >= TEMP_MAX:
        scaled = 0.0
        print(f"WARNING: effective temp {effective:.1f}C >= TEMP_MAX. limit_i_max -> 0.")
    else:
        scaled = LIMIT_I_MAX * (1.0 - (effective - TEMP_WARN) / (TEMP_MAX - TEMP_WARN))

    if _last_written_limit is None or abs(scaled - _last_written_limit) > LIMIT_WRITE_DEADBAND or scaled == 0.0:
        iface.bear.set_limit_i_max((iface.id, scaled))
        _last_written_limit = scaled
    return scaled


def guard(state):
    """Unconditional per-poll checks. Raises to unwind into the finally block."""
    if state['error'] != 128:
        raise RuntimeError(f"Non-normal error byte: {state['error']}")
    pos = state['position']
    if pos <= MIN_ANGLE or pos >= MAX_ANGLE:
        raise RuntimeError(f"Position {pos:.4f} rad crossed a joint limit.")


def move_to_target(target, from_position, direction_sign):
    """CHANGE 4a: ramp phase is now PURELY a ramp -- it never logs and never
    decides anything was reached. The old script's ramp block could complete a
    dwell mid-ramp, write an at_target=False row, set last_logged_position, and
    then have the settle loop's dedupe check suppress the REAL equilibrium row
    while still breaking out. Result: a bad row and no good one for that target.
    Didn't fire last run (1.5s dwell rarely completes mid-ramp) but it was live.

    Returns (peak_ramp_iq, max_overshoot_rad, last_state).
    """
    distance = target - from_position
    steps = max(1, int(abs(distance) / RAMP_STEP_SIZE))
    peak_ramp_iq = 0.0
    max_overshoot = 0.0
    state = read_state()

    for i in range(1, steps + 1):
        goal = from_position + distance * (i / steps)
        iface.bear.set_goal_position((iface.id, goal))
        time.sleep(RAMP_STEP_SLEEP)

        state = read_state()
        scaled = thermal_limit_scale(state)
        guard(state)

        peak_ramp_iq = max(peak_ramp_iq, abs(state['iq']))
        # CHANGE 3: overshoot tracking. If the up-leg overshoots and settles
        # back DOWN onto the target, that point was approached from above --
        # it's a down-leg measurement wearing an up-leg label, and it silently
        # poisons the difference. Measure it so it can be discarded, not averaged.
        max_overshoot = max(max_overshoot, (state['position'] - target) * direction_sign)

        print(f"  [ramp {i}/{steps}] goal={goal:+.4f} pos={state['position']:+.4f} "
              f"vel={state['velocity']:+.4f} iq={state['iq']:+.4f} "
              f"w={state['winding_temp']:.1f} p={state['powerstage_temp']:.1f} "
              f"lim={scaled:.2f} err={state['error']}")

    iface.bear.set_goal_position((iface.id, target))
    return peak_ramp_iq, max_overshoot, state


def settle_and_measure(target, direction_sign, prior_overshoot):
    """CHANGE 4b: the ONLY place a measurement is taken. Exactly one row per
    target, or none. Returns (avg_iq, state, max_overshoot, reached) or
    (None, state, max_overshoot, False) on timeout."""
    deadline = time.monotonic() + PER_TARGET_TIMEOUT
    dwell_start = None
    samples = []
    max_overshoot = prior_overshoot
    state = read_state()

    while time.monotonic() < deadline:
        state = read_state()
        scaled = thermal_limit_scale(state)
        guard(state)

        max_overshoot = max(max_overshoot, (state['position'] - target) * direction_sign)
        pos_err = abs(state['position'] - target)

        print(f"  [settle] pos={state['position']:+.4f} vel={state['velocity']:+.4f} "
              f"iq={state['iq']:+.4f} err_rad={pos_err:.4f} "
              f"over={math.degrees(max_overshoot):+.2f}deg "
              f"w={state['winding_temp']:.1f} p={state['powerstage_temp']:.1f} lim={scaled:.2f}")

        if abs(state['velocity']) < SETTLE_VELOCITY_THRESHOLD:
            if dwell_start is None:
                dwell_start = time.monotonic()
                samples = []
            samples.append(state['iq'])
            if time.monotonic() - dwell_start >= SETTLE_DWELL:
                avg_iq = sum(samples) / len(samples)
                return avg_iq, state, max_overshoot, pos_err <= POSITION_SETTLE_TOLERANCE
        else:
            dwell_start = None
            samples = []

        time.sleep(POLL_SLEEP)

    return None, state, max_overshoot, False


def run_leg(writer, f, targets, label, direction_sign, start_position):
    current = start_position
    print(f"\n{'='*70}\n  {label.upper()} LEG -- {len(targets)} targets\n{'='*70}")

    for n, target in enumerate(targets, 1):
        print(f"\n--- [{label}] Target {n}/{len(targets)}: "
              f"{target:.4f} rad ({math.degrees(target):.2f} deg) ---")

        peak_iq, overshoot, _ = move_to_target(target, current, direction_sign)
        avg_iq, state, overshoot, reached = settle_and_measure(target, direction_sign, overshoot)

        if avg_iq is None:
            print(f"  TIMEOUT -- no equilibrium at {math.degrees(target):.2f} deg. No row written.")
            current = state['position']
            continue

        writer.writerow([
            time.time(), label,
            f"{target:.4f}", f"{math.degrees(target):.2f}",
            f"{state['position']:.4f}", f"{math.degrees(state['position']):.2f}",
            f"{avg_iq:.4f}", f"{peak_iq:.4f}",
            f"{math.degrees(overshoot):.3f}",
            f"{state['id']:.4f}", f"{state['voltage']:.2f}",
            f"{state['winding_temp']:.1f}", f"{state['powerstage_temp']:.1f}",
            reached,
        ])
        f.flush()
        print(f"  LOGGED  pos={math.degrees(state['position']):.2f}deg  "
              f"avg_iq={avg_iq:+.4f}A  overshoot={math.degrees(overshoot):+.2f}deg  "
              f"reached={reached}")
        current = state['position']

    return current


# --- Build the absolute grid -------------------------------------------------
n_steps = int(round((GRID_TOP_DEG - GRID_BOTTOM_DEG) / GRID_STEP_DEG))
grid_deg = [GRID_BOTTOM_DEG + i * GRID_STEP_DEG for i in range(n_steps + 1)]
grid_rad = [math.radians(d) for d in grid_deg]
down_targets = list(reversed(grid_rad))
up_targets = list(grid_rad)

try:
    state = read_state()
    if state['position'] is None:
        raise SystemExit("Could not read position -- run test_connection.py first.")
    start_position = state['position']
    print(f"Start: {start_position:.4f} rad ({math.degrees(start_position):.2f} deg)  "
          f"err={state['error']}  w={state['winding_temp']:.1f}C  p={state['powerstage_temp']:.1f}C")

    if start_position <= MIN_ANGLE or start_position >= MAX_ANGLE:
        raise SystemExit("Start position outside joint limits -- reposition by hand.")
    if start_position < grid_rad[-1]:
        raise SystemExit(
            f"Start position {math.degrees(start_position):.2f} deg is below "
            f"GRID_TOP {GRID_TOP_DEG} deg. Raise the arm by hand toward the top "
            f"stop before starting so the first leg is genuinely descending."
        )

    for setter, val in [
        (iface.bear.set_p_gain_iq, IQ_ID_P), (iface.bear.set_i_gain_iq, IQ_ID_I),
        (iface.bear.set_d_gain_iq, IQ_ID_D), (iface.bear.set_p_gain_id, IQ_ID_P),
        (iface.bear.set_i_gain_id, IQ_ID_I), (iface.bear.set_d_gain_id, IQ_ID_D),
        (iface.bear.set_p_gain_velocity, VEL_P), (iface.bear.set_i_gain_velocity, VEL_I),
        (iface.bear.set_d_gain_velocity, VEL_D), (iface.bear.set_p_gain_position, POS_P),
        (iface.bear.set_i_gain_position, POS_I), (iface.bear.set_d_gain_position, POS_D),
        (iface.bear.set_limit_i_max, LIMIT_I_MAX),
        (iface.bear.set_limit_velocity_max, LIMIT_VELOCITY_MAX),
        (iface.bear.set_limit_acc_max, LIMIT_ACC_MAX),
        (iface.bear.set_limit_position_min, MIN_ANGLE),
        (iface.bear.set_limit_position_max, MAX_ANGLE),
    ]:
        setter((iface.id, val))
    _last_written_limit = LIMIT_I_MAX

    print(f"\nGrid: {grid_deg[0]:.0f} to {grid_deg[-1]:.0f} deg in {GRID_STEP_DEG:.0f} deg "
          f"steps ({len(grid_deg)} points), visited DOWN then UP = "
          f"{2 * len(grid_deg)} measurements.")
    input("Keep a hand near the arm, ready to Ctrl+C. Press Enter to enable and begin: ")

    iface.enable_position_mode(start_position)

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'sweep_direction', 'target_rad', 'target_deg',
            'angle_rad', 'angle_deg', 'iq_hold_A', 'peak_ramp_iq_A',
            'max_overshoot_deg', 'present_id_A', 'input_voltage_V',
            'winding_temp_C', 'powerstage_temp_C', 'reached_target',
        ])

        pos = run_leg(writer, f, down_targets, 'down', -1.0, start_position)
        print("\n  Down leg complete. Up leg starts from here -- no repositioning.")
        run_leg(writer, f, up_targets, 'up', +1.0, pos)

    print(f"\nSweep complete. Logged to {log_path}")

    input("\nPress Enter to disable: ")

finally:
    iface.disable()
    print("Actuator disabled.")