"""
Position-Mode spring characterization sweep.
[docstring unchanged from before]
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import MAX_ANGLE, LOG_DIR, SAFETY_CHECKS_CONFIRMED, TEMP_WARN, TEMP_MAX

MIN_SEARCH_ANGLE = math.radians(5)

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
    LOG_DIR, f"position_sweep_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)

iface = BearInterface()


def read_state():
    pos, err = iface.bear.get_present_position(iface.id)[0]
    vel, _ = iface.bear.get_present_velocity(iface.id)[0]
    iq, _ = iface.bear.get_present_iq(iface.id)[0]
    return pos[0], vel[0], iq[0], err


def thermal_limit_scale():
    temp, _ = iface.bear.get_winding_temperature(iface.id)[0]
    temperature = temp[0]
    if temperature is None:
        return None, LIMIT_I_MAX
    if temperature < TEMP_WARN:
        scaled = LIMIT_I_MAX
    elif temperature >= TEMP_MAX:
        scaled = 0.0
        print(f"WARNING: winding temp {temperature:.1f}C at/above TEMP_MAX. limit_i_max -> 0.")
    else:
        scale = 1.0 - (temperature - TEMP_WARN) / (TEMP_MAX - TEMP_WARN)
        scaled = LIMIT_I_MAX * scale
    iface.bear.set_limit_i_max((iface.id, scaled))
    return temperature, scaled


def check_limits_and_abort(position):
    if position <= MIN_SEARCH_ANGLE or position >= MAX_ANGLE:
        iface.disable()
        raise RuntimeError(f"Position {position:.3f} rad crossed the software bound.")


try:
    pos, err = iface.bear.get_present_position(iface.id)[0]
    start_position = pos[0]
    if start_position is None:
        raise SystemExit("Could not read starting position — check connection first.")
    print(f"Starting position: {start_position:.4f} rad  (error byte: {err})")

    if start_position <= MIN_SEARCH_ANGLE or start_position >= MAX_ANGLE:
        raise SystemExit(
            f"Starting position {start_position:.4f} rad is outside "
            f"[{MIN_SEARCH_ANGLE:.4f}, {MAX_ANGLE:.4f}] — reposition by hand first."
        )

    sweep_end = float(input(
        f"Sweep to which end target, in rad? (current: {start_position:.4f}, "
        f"valid range [{MIN_SEARCH_ANGLE:.4f}, {MAX_ANGLE:.4f}]): "
    ))
    if sweep_end <= MIN_SEARCH_ANGLE or sweep_end >= MAX_ANGLE:
        raise SystemExit(f"Target {sweep_end:.4f} rad is outside the allowed range. Nothing enabled.")

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
    iface.bear.set_limit_position_min((iface.id, MIN_SEARCH_ANGLE))
    iface.bear.set_limit_position_max((iface.id, MAX_ANGLE))

    input(f"About to enable and sweep {len(targets)} target(s). Keep a hand near "
          f"the arm, ready to Ctrl+C. Press Enter to proceed: ")

    iface.enable_position_mode(start_position)

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'target_rad', 'angle_rad', 'iq_hold_A',
                          'temperature_C', 'reached_target'])

        current_position = start_position
        for target_num, target in enumerate(targets, 1):
            print(f"\n--- Target {target_num}/{len(targets)}: {target:.4f} rad ---")

            distance = target - current_position
            num_ramp_steps = max(1, int(abs(distance) / RAMP_STEP_SIZE))
            dwell_start = None
            last_logged_position = None
            reached_target = False
            target_start_time = time.monotonic()
            position = current_position

            # --- Phase 1: ramp goal_position incrementally toward target ---
            for i in range(1, num_ramp_steps + 1):
                goal = current_position + distance * (i / num_ramp_steps)
                iface.bear.set_goal_position((iface.id, goal))
                time.sleep(RAMP_STEP_SLEEP)

                position, velocity, iq_present, err = read_state()
                temperature, scaled_limit = thermal_limit_scale()
                print(f"  [ramp {i}/{num_ramp_steps}]  goal={goal:+.3f}  pos={position:+.3f}  "
                      f"vel={velocity:+.3f}  iq={iq_present:+.3f}  temp={temperature}  "
                      f"limit_i_max={scaled_limit:.2f}  err={err}")

                if err != 128:
                    raise RuntimeError(f"Non-normal error byte: {err}")
                check_limits_and_abort(position)

                position_error = abs(position - target)
                if abs(velocity) < SETTLE_VELOCITY_THRESHOLD:
                    if dwell_start is None:
                        dwell_start = time.monotonic()
                    elif time.monotonic() - dwell_start >= SETTLE_DWELL:
                        if last_logged_position is None or abs(position - last_logged_position) > POSITION_SETTLE_TOLERANCE:
                            at_target = position_error <= POSITION_SETTLE_TOLERANCE
                            writer.writerow([time.time(), f"{target:.4f}", f"{position:.4f}", f"{iq_present:.4f}",
                                              f"{temperature:.1f}" if temperature is not None else "", at_target])
                            f.flush()
                            print(f"  LOGGED equilibrium: pos={position:.4f}  iq={iq_present:.4f}  at_target={at_target}")
                            last_logged_position = position
                            if at_target:
                                reached_target = True
                else:
                    dwell_start = None

            # --- Phase 2: goal is now fixed at target. Keep monitoring
            # until it actually settles, or times out -- this is the
            # phase that was missing before. Bounded by target_start_time,
            # so total budget spans ramp + settle together. ---
            if not reached_target:
                print("  Ramp done, goal now fixed at target. Waiting for settle...")
                while time.monotonic() - target_start_time < PER_TARGET_TIMEOUT:
                    position, velocity, iq_present, err = read_state()
                    temperature, scaled_limit = thermal_limit_scale()
                    position_error = abs(position - target)
                    print(f"  [settle]  pos={position:+.3f}  vel={velocity:+.3f}  iq={iq_present:+.3f}  "
                          f"pos_err={position_error:.3f}  temp={temperature}  "
                          f"limit_i_max={scaled_limit:.2f}  err={err}")

                    if err != 128:
                        raise RuntimeError(f"Non-normal error byte: {err}")
                    check_limits_and_abort(position)

                    if abs(velocity) < SETTLE_VELOCITY_THRESHOLD:
                        if dwell_start is None:
                            dwell_start = time.monotonic()
                        elif time.monotonic() - dwell_start >= SETTLE_DWELL:
                            if last_logged_position is None or abs(position - last_logged_position) > POSITION_SETTLE_TOLERANCE:
                                at_target = position_error <= POSITION_SETTLE_TOLERANCE
                                writer.writerow([time.time(), f"{target:.4f}", f"{position:.4f}", f"{iq_present:.4f}",
                                                  f"{temperature:.1f}" if temperature is not None else "", at_target])
                                f.flush()
                                print(f"  LOGGED equilibrium: pos={position:.4f}  iq={iq_present:.4f}  at_target={at_target}")
                                last_logged_position = position
                            reached_target = position_error <= POSITION_SETTLE_TOLERANCE
                            break
                    else:
                        dwell_start = None

                    time.sleep(0.1)

            current_position = position
            if not reached_target:
                print(f"  Did not confirm arrival at {target:.4f} rad.")

        print(f"\nSweep complete. Logged to {log_path}")

    input("\nSweep done — press Enter to disable: ")

finally:
    iface.disable()
    print("Actuator disabled.")