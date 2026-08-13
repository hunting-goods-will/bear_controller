"""
FINAL Position-Mode spring characterization sweep.
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import MIN_ANGLE, MAX_ANGLE, LOG_DIR, SAFETY_CHECKS_CONFIRMED, TEMP_WARN, TEMP_MAX

IQ_ID_P, IQ_ID_I, IQ_ID_D = 0.02, 0.02, 0.0
VEL_P, VEL_I, VEL_D = 4.5, 0.001, 0.0
POS_P, POS_I, POS_D = 5.0, 0.0, 0.2

LIMIT_I_MAX = 5.5
LIMIT_VELOCITY_MAX = 1.0
LIMIT_ACC_MAX = 5.0

RAMP_STEP_SIZE = 0.03
RAMP_STEP_SLEEP = 0.05

SETTLE_VELOCITY_THRESHOLD = 0.05
POSITION_SETTLE_TOLERANCE = math.radians(3)
SETTLE_DWELL = 1.5
PER_TARGET_TIMEOUT = 30.0

SWEEP_STEP = math.radians(10)

if not SAFETY_CHECKS_CONFIRMED:
    raise SystemExit(
        "\nBLOCKED: config.SAFETY_CHECKS_CONFIRMED is False.\n"
        "Same risk category as every other live script here."
    )

os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(
    LOG_DIR, f"position_sweep_FINAL_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)

iface = BearInterface()


def read_state():
    """Returns dict: position, velocity, iq, id_current, input_voltage,
    winding_temp, powerstage_temp, error."""
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
    w, p = state['winding_temp'], state['powerstage_temp']
    if w is None or p is None:
        return LIMIT_I_MAX
    effective = max(w, p)   # matches the firmware's own fault logic
    if effective < TEMP_WARN:
        scaled = LIMIT_I_MAX
    elif effective >= TEMP_MAX:
        scaled = 0.0
        print(f"WARNING: effective temp {effective:.1f}C at/above TEMP_MAX. limit_i_max -> 0.")
    else:
        scaled = LIMIT_I_MAX * (1.0 - (effective - TEMP_WARN) / (TEMP_MAX - TEMP_WARN))
    iface.bear.set_limit_i_max((iface.id, scaled))
    return scaled


def check_limits_and_abort(position):
    if position <= MIN_ANGLE or position >= MAX_ANGLE:
        iface.disable()
        raise RuntimeError(f"Position {position:.3f} rad crossed the finalized joint limit.")


def log_row(writer, f, target, state, iq_hold_avg, peak_ramp_iq, at_target):
    writer.writerow([
        time.time(), f"{target:.4f}", f"{math.degrees(target):.2f}",
        f"{state['position']:.4f}", f"{math.degrees(state['position']):.2f}",
        f"{iq_hold_avg:.4f}", f"{peak_ramp_iq:.4f}",
        f"{state['id']:.4f}", f"{state['voltage']:.2f}",
        f"{state['winding_temp']:.1f}", f"{state['powerstage_temp']:.1f}",
        at_target,
    ])
    f.flush()


try:
    state = read_state()
    start_position = state['position']
    if start_position is None:
        raise SystemExit("Could not read starting position — check connection first.")
    print(f"Starting position: {start_position:.4f} rad ({math.degrees(start_position):.1f} deg)  "
          f"error: {state['error']}")

    if start_position <= MIN_ANGLE or start_position >= MAX_ANGLE:
        raise SystemExit(
            f"Starting position outside [{MIN_ANGLE:.4f}, {MAX_ANGLE:.4f}] rad — reposition by hand first."
        )

    sweep_end = float(input(
        f"Sweep to which end target, in rad? (current: {start_position:.4f}, "
        f"valid range [{MIN_ANGLE:.4f}, {MAX_ANGLE:.4f}]): "
    ))
    if sweep_end <= MIN_ANGLE or sweep_end >= MAX_ANGLE:
        raise SystemExit(f"Target {sweep_end:.4f} rad is outside the finalized range. Nothing enabled.")

    direction = 1 if sweep_end > start_position else -1
    num_targets = max(1, int(abs(sweep_end - start_position) / SWEEP_STEP))
    targets = [start_position + direction * SWEEP_STEP * i for i in range(1, num_targets + 1)]
    if abs(targets[-1] - sweep_end) > 1e-6:
        targets.append(sweep_end)
    print(f"Sweep plan: {len(targets)} target(s) from {start_position:.4f} to {sweep_end:.4f} rad.")

    iface.bear.set_p_gain_iq((iface.id, IQ_ID_P))
    iface.bear.set_i_gain_iq((iface.id, IQ_ID_I))
    iface.bear.set_d_gain_iq((iface.id, IQ_ID_D))
    iface.bear.set_p_gain_id((iface.id, IQ_ID_P))
    iface.bear.set_i_gain_id((iface.id, IQ_ID_I))
    iface.bear.set_d_gain_id((iface.id, IQ_ID_D))
    iface.bear.set_p_gain_velocity((iface.id, VEL_P))
    iface.bear.set_i_gain_velocity((iface.id, VEL_I))
    iface.bear.set_d_gain_velocity((iface.id, VEL_D))
    iface.bear.set_p_gain_position((iface.id, POS_P))
    iface.bear.set_i_gain_position((iface.id, POS_I))
    iface.bear.set_d_gain_position((iface.id, POS_D))

    iface.bear.set_limit_i_max((iface.id, LIMIT_I_MAX))
    iface.bear.set_limit_velocity_max((iface.id, LIMIT_VELOCITY_MAX))
    iface.bear.set_limit_acc_max((iface.id, LIMIT_ACC_MAX))
    iface.bear.set_limit_position_min((iface.id, MIN_ANGLE))
    iface.bear.set_limit_position_max((iface.id, MAX_ANGLE))

    input(f"About to enable and sweep {len(targets)} target(s) across the FINALIZED range. "
          f"Keep a hand near the arm, ready to Ctrl+C. Press Enter to proceed: ")

    iface.enable_position_mode(start_position)

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'target_rad', 'target_deg', 'angle_rad', 'angle_deg',
                          'iq_hold_A', 'peak_ramp_iq_A', 'present_id_A', 'input_voltage_V',
                          'winding_temp_C', 'powerstage_temp_C', 'reached_target'])

        current_position = start_position
        for target_num, target in enumerate(targets, 1):
            print(f"\n--- Target {target_num}/{len(targets)}: {target:.4f} rad "
                  f"({math.degrees(target):.1f} deg) ---")

            distance = target - current_position
            num_ramp_steps = max(1, int(abs(distance) / RAMP_STEP_SIZE))
            dwell_start = None
            dwell_iq_samples = []
            last_logged_position = None
            reached_target = False
            peak_ramp_iq = 0.0
            target_start_time = time.monotonic()
            state = read_state()

            for i in range(1, num_ramp_steps + 1):
                goal = current_position + distance * (i / num_ramp_steps)
                iface.bear.set_goal_position((iface.id, goal))
                time.sleep(RAMP_STEP_SLEEP)

                state = read_state()
                scaled_limit = thermal_limit_scale(state)
                peak_ramp_iq = max(peak_ramp_iq, abs(state['iq']))
                print(f"  [ramp {i}/{num_ramp_steps}]  goal={goal:+.3f}  pos={state['position']:+.3f}  "
                      f"vel={state['velocity']:+.3f}  iq={state['iq']:+.3f}  "
                      f"w_temp={state['winding_temp']}  p_temp={state['powerstage_temp']}  "
                      f"limit_i_max={scaled_limit:.2f}  err={state['error']}")

                if state['error'] != 128:
                    raise RuntimeError(f"Non-normal error byte: {state['error']}")
                check_limits_and_abort(state['position'])

                position_error = abs(state['position'] - target)
                if abs(state['velocity']) < SETTLE_VELOCITY_THRESHOLD:
                    if dwell_start is None:
                        dwell_start = time.monotonic()
                        dwell_iq_samples = []
                    dwell_iq_samples.append(state['iq'])
                    if time.monotonic() - dwell_start >= SETTLE_DWELL:
                        if last_logged_position is None or abs(state['position'] - last_logged_position) > POSITION_SETTLE_TOLERANCE:
                            avg_iq = sum(dwell_iq_samples) / len(dwell_iq_samples)
                            at_target = position_error <= POSITION_SETTLE_TOLERANCE
                            log_row(writer, f, target, state, avg_iq, peak_ramp_iq, at_target)
                            print(f"  LOGGED equilibrium: pos={state['position']:.4f}  avg_iq={avg_iq:.4f}  at_target={at_target}")
                            last_logged_position = state['position']
                            if at_target:
                                reached_target = True
                else:
                    dwell_start = None
                    dwell_iq_samples = []

            if not reached_target:
                print("  Ramp done, goal now fixed at target. Waiting for settle...")
                while time.monotonic() - target_start_time < PER_TARGET_TIMEOUT:
                    state = read_state()
                    scaled_limit = thermal_limit_scale(state)
                    position_error = abs(state['position'] - target)
                    print(f"  [settle]  pos={state['position']:+.3f}  vel={state['velocity']:+.3f}  "
                          f"iq={state['iq']:+.3f}  pos_err={position_error:.3f}  "
                          f"w_temp={state['winding_temp']}  p_temp={state['powerstage_temp']}  "
                          f"limit_i_max={scaled_limit:.2f}  err={state['error']}")

                    if state['error'] != 128:
                        raise RuntimeError(f"Non-normal error byte: {state['error']}")
                    check_limits_and_abort(state['position'])

                    if abs(state['velocity']) < SETTLE_VELOCITY_THRESHOLD:
                        if dwell_start is None:
                            dwell_start = time.monotonic()
                            dwell_iq_samples = []
                        dwell_iq_samples.append(state['iq'])
                        if time.monotonic() - dwell_start >= SETTLE_DWELL:
                            if last_logged_position is None or abs(state['position'] - last_logged_position) > POSITION_SETTLE_TOLERANCE:
                                avg_iq = sum(dwell_iq_samples) / len(dwell_iq_samples)
                                at_target = position_error <= POSITION_SETTLE_TOLERANCE
                                log_row(writer, f, target, state, avg_iq, peak_ramp_iq, at_target)
                                print(f"  LOGGED equilibrium: pos={state['position']:.4f}  avg_iq={avg_iq:.4f}  at_target={at_target}")
                                last_logged_position = state['position']
                            reached_target = position_error <= POSITION_SETTLE_TOLERANCE
                            break
                    else:
                        dwell_start = None
                        dwell_iq_samples = []

                    time.sleep(0.1)

            current_position = state['position']
            if not reached_target:
                print(f"  Did not confirm arrival at {target:.4f} rad.")

        print(f"\nSweep complete. Logged to {log_path}")

    input("\nSweep done — press Enter to disable: ")

finally:
    iface.disable()
    print("Actuator disabled.")