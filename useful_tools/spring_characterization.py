"""
Holding-current search across the ROM — NEXT_STEPS_v2.md Group B item 5.

Logs (angle, iq_hold, iterations, converged) per hand-positioned region to
a CSV. Current-mode only. Post-processing into tau_spring(theta) — which
DOES need rig mass/CoM — happens separately, after the sweep, not here.

Do not run this for real until Group A item 3 (rig mass/CoM) is measured
and you've confirmed the SAFETY_CHECKS_CONFIRMED-in-enable() question above.
"""
import csv
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import (
    MIN_ANGLE, MAX_ANGLE, LOG_DIR, SAFETY_CHECKS_CONFIRMED
)

# --- Search-specific tuning. Starting guesses, not PI-confirmed values ---
IQ_STEP = 0.10               # A — NEXT_STEPS_v2.md §5: "start ~0.02A"
MAX_SEARCH_ITERATIONS = 30   # NEXT_STEPS_v2.md §5: "capped, e.g. 30"
VELOCITY_THRESHOLD = 1    # rad/s, "near-zero" for convergence. Distinct
                              # from controller.py's VELOCITY_THRESHOLD —
                              # that one detects intent-to-lift; this one
                              # detects drift while holding. Don't conflate.
CONVERGENCE_DWELL = 0.4      # s of sustained near-zero velocity = converged
ADVANCE_IQ_BIAS = 0.05       # A — nudge toward the next target region
ADVANCE_SETTLE_TIME = 0.5    # s to let the nudge settle

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
    """Position crossed MIN_ANGLE/MAX_ANGLE during a search. Hard stop,
    not a soft failure — distinct from ordinary search timeout."""
    pass


def search_holding_current(iface, iq_start=0.0):
    """Search for the current that holds the current angle steady.
    Returns (converged: bool, iq_hold, iterations). Raises JointLimitAbort
    if position crosses the limit mid-search — caller must not swallow it."""
    iq = iq_start
    dwell_start = None

    for iteration in range(1, MAX_SEARCH_ITERATIONS + 1):
        state = iface.get_state()
        position, velocity = state['position'], state['velocity']

        if position <= MIN_ANGLE or position >= MAX_ANGLE:
            iface.set_iq(0.0)  # immediate — don't wait for the outer finally
            raise JointLimitAbort(
                f"position {position:.3f} rad crossed joint limit mid-search"
           )

        if abs(velocity) < VELOCITY_THRESHOLD:
            if dwell_start is None:
                dwell_start = time.monotonic()
            elif time.monotonic() - dwell_start >= CONVERGENCE_DWELL:
                return True, iq, iteration
        else:
            dwell_start = None  # drifted again — reset the dwell clock
            iq += IQ_STEP if velocity < 0 else -IQ_STEP
            iface.set_iq(iq)

        time.sleep(0.02)

    # Timed out without converging. Diagnostic, not a joint-limit emergency
    # — log it and let the operator decide, per NEXT_STEPS_v2.md §5: "don't
    # silently retry forever."
    return False, iq, MAX_SEARCH_ITERATIONS


iface = BearInterface()

try:
    iface.enable()
    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'angle_rad', 'iq_hold_A', 'iterations', 'converged'])

        region = 1
        iq_start = 0.0
        while True:
            input(f"\n--- Region {region} ---\n"
                  f"Position the arm by hand, keep it supported, then press "
                  f"Enter to start the search (Ctrl+C to stop): ")

            converged, iq_hold, iterations = search_holding_current(iface, iq_start=iq_start)
            angle = iface.get_state()['position']

            writer.writerow([time.time(), f"{angle:.4f}", f"{iq_hold:.4f}", iterations, converged])
            f.flush()

            status = "converged" if converged else "DID NOT CONVERGE (timed out)"
            print(f"  angle={angle:.3f} rad  iq_hold={iq_hold:.3f} A  "
                  f"iterations={iterations}  [{status}]")
            if not converged:
                print("  Non-convergence is diagnostic — look into why "
                      "before trusting nearby data points, don't just re-run.")

            print("  Ease your hand off gently — confirm it actually holds.")
            time.sleep(1.5)

            proceed = input("Advance to next region? [Enter to continue, 'done' to stop]: ")
            if proceed.strip().lower() == 'done':
                break

            iface.set_iq(iq_hold + ADVANCE_IQ_BIAS)
            time.sleep(ADVANCE_SETTLE_TIME)
            iq_start = iq_hold  # warm-start the next search from here, not 0
            region += 1

except JointLimitAbort as e:
    print(f"\nABORTED: {e}")
finally:
    iface.set_iq(0.0)
    iface.disable()
    print(f"\nLogged to {log_path}")