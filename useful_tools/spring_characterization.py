"""
Holding-current search across the ROM — NEXT_STEPS_v2.md Group B item 5.

Single continuous monitoring loop per attempt — no keypress-gated phase
boundary. Current is applied once; position/velocity/limits are watched
every tick from that instant through to a verdict. Ease off naturally
whenever ready, no need to signal it — this removes the earlier blind gap
where nothing was monitored between "hold" ending and "release" starting.

A minimum elapsed time (MIN_TEST_TIME) before any "held" verdict is
accepted ensures a real release opportunity happened, not just a still-
firmly-supported moment read as success.

MIN_SEARCH_ANGLE is script-local, deliberately more permissive than
config.MIN_ANGLE's 20 deg operating margin — that constant belongs to the
future autonomous controller and carries a different risk profile.

Post-processing into tau_spring(theta) — needs rig mass/CoM, still
unmeasured — happens separately, after the sweep, not here.
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import MAX_ANGLE, LOG_DIR, SAFETY_CHECKS_CONFIRMED

MIN_SEARCH_ANGLE = math.radians(5)

IQ_START_SEED = -1.0
IQ_STEP_INITIAL = 0.2

DRIFT_TOLERANCE = math.radians(10)   # your confirmed value — backup check,
                                       # runs continuously now, one reference
                                       # point for the whole attempt
VELOCITY_THRESHOLD = 0.05
CONVERGENCE_DWELL = 1.0
MIN_TEST_TIME = 6.0            # s — must elapse before ANY "held" verdict,
                                # ensures a real release chance happened
TOTAL_ATTEMPT_TIMEOUT = 15.0   # s — hard cap on the whole attempt

MAX_ATTEMPTS_PER_REGION = 15
ADVANCE_IQ_BIAS = 0.05
ADVANCE_SETTLE_TIME = 0.5
REPOSITION_TOLERANCE = math.radians(3)

if not SAFETY_CHECKS_CONFIRMED:
    raise SystemExit(
        "\nBLOCKED: config.SAFETY_CHECKS_CONFIRMED is False.\n"
        "This script enables torque and searches for holding current with "
        "a hand on the linkage the whole time — same risk category as "
        "main.py's live controller, same gate required."
    )

os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(
    LOG_DIR, f"spring_characterization_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)


class JointLimitAbort(Exception):
    pass


def _check_limits_and_zero(iface, position):
    if position <= MIN_SEARCH_ANGLE or position >= MAX_ANGLE:
        iface.set_iq(0.0)
        raise JointLimitAbort(f"position {position:.3f} rad crossed joint limit")


def attempt_hold(iface, iq_candidate):
    """Returns (outcome, drift_sign, applied_iq).
    outcome: 'held' | 'runaway' | 'ambiguous_timeout'"""
    applied_iq = iface.set_iq(iq_candidate)
    position_at_apply = iface.get_state()['position']
    print(f"  Current applied ({applied_iq:+.3f}A). Hold firmly, then ease off "
          f"naturally whenever ready — watching continuously, no keypress needed.")

    start_time = time.monotonic()
    dwell_start = None
    prev_vel_mag = None

    while time.monotonic() - start_time < TOTAL_ATTEMPT_TIMEOUT:
        state = iface.get_state()
        position, velocity, measured_iq = state['position'], state['velocity'], state['iq']
        drift = position - position_at_apply
        elapsed = time.monotonic() - start_time
        print(f"    t={elapsed:4.1f}s  pos={position:+.3f}  drift={drift:+.3f}  "
              f"vel={velocity:+.3f}  cmd_iq={applied_iq:+.3f}  measured_iq={measured_iq:+.3f}")

        _check_limits_and_zero(iface, position)   # hard backstop, unconditional, every tick

        if abs(drift) > DRIFT_TOLERANCE:
            iface.set_iq(0.0)
            return 'runaway', (1 if drift > 0 else -1), applied_iq

        vel_mag = abs(velocity)
        if vel_mag < VELOCITY_THRESHOLD:
            if dwell_start is None:
                dwell_start = time.monotonic()
            elif (time.monotonic() - dwell_start >= CONVERGENCE_DWELL
                  and elapsed >= MIN_TEST_TIME):
                return 'held', None, applied_iq
        else:
            dwell_start = None
            if prev_vel_mag is not None and vel_mag >= prev_vel_mag:
                iface.set_iq(0.0)
                return 'runaway', (1 if drift > 0 else -1), applied_iq

        prev_vel_mag = vel_mag
        time.sleep(0.1)

    iface.set_iq(0.0)
    return 'ambiguous_timeout', None, applied_iq


def wait_for_target(iface, target, tolerance):
    while True:
        position = iface.get_state()['position']
        _check_limits_and_zero(iface, position)
        if abs(position - target) <= tolerance:
            return
        print(f"    Reposition to target: pos={position:+.3f}  target={target:+.3f}")
        time.sleep(0.3)


def find_holding_current(iface, iq_guess, target_position):
    iq = iq_guess
    prev_direction = None
    step = IQ_STEP_INITIAL
    bracket_lo = None
    bracket_hi = None

    attempt = 1
    while attempt <= MAX_ATTEMPTS_PER_REGION:
        if attempt > 1:
            print(f"  Reposition to this region's target ({target_position:+.3f} rad).")
            wait_for_target(iface, target_position, REPOSITION_TOLERANCE)

        input(f"  Attempt {attempt}: trying iq={iq:+.3f}A. Press Enter when ready "
              f"(still fully supporting the arm): ")
        outcome, drift_sign, applied_iq = attempt_hold(iface, iq)

        if outcome == 'held':
            return True, applied_iq, attempt

        if outcome == 'ambiguous_timeout':
            print("  Timed out ambiguously — retrying same value.")
            attempt += 1
            continue

        print(f"  Runaway {'up' if drift_sign > 0 else 'down'} — re-grab and support the arm now.")
        input("  Press Enter once you've re-supported it: ")

        if drift_sign > 0:
            bracket_hi = applied_iq
        else:
            bracket_lo = applied_iq

        if bracket_lo is not None and bracket_hi is not None:
            iq = (bracket_lo + bracket_hi) / 2
            print(f"  Bracketed [{bracket_lo:+.3f}, {bracket_hi:+.3f}] — bisecting to {iq:+.3f}A.")
        else:
            if prev_direction is not None:
                step *= 2
            iq = applied_iq - step if drift_sign > 0 else applied_iq + step
            print(f"  Still searching — next try {iq:+.3f}A (step {step:.3f}).")
        prev_direction = drift_sign
        attempt += 1

    return False, iq, MAX_ATTEMPTS_PER_REGION


iface = BearInterface()

try:
    iface.enable()
    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'angle_rad', 'iq_hold_A', 'attempts', 'converged'])

        region = 1
        iq_start = IQ_START_SEED
        while True:
            input(f"\n--- Region {region} ---\n"
                  f"Position the arm by hand, keep it supported, then press Enter "
                  f"when ready to begin searching (Ctrl+C to stop): ")
            target_position = iface.get_state()['position']

            converged, iq_hold, attempts = find_holding_current(iface, iq_start, target_position)
            angle = iface.get_state()['position']

            writer.writerow([time.time(), f"{angle:.4f}", f"{iq_hold:.4f}", attempts, converged])
            f.flush()

            if not converged:
                print(f"  DID NOT CONVERGE within {MAX_ATTEMPTS_PER_REGION} attempts.")
            else:
                print(f"  CONVERGED: angle={angle:.3f} rad  iq_hold={iq_hold:.3f} A  attempts={attempts}")

            proceed = input("Advance to next region? [Enter to continue, 'done' to stop]: ")
            if proceed.strip().lower() == 'done':
                break

            iface.set_iq(iq_hold + ADVANCE_IQ_BIAS)
            time.sleep(ADVANCE_SETTLE_TIME)
            iq_start = iq_hold
            region += 1

except JointLimitAbort as e:
    print(f"\nABORTED: {e}")
finally:
    iface.set_iq(0.0)
    iface.disable()
    print(f"\nLogged to {log_path}")