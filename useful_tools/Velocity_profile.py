"""
Human velocity profiling — read-only. Determines the assist-trigger threshold.

WHY THIS EXISTS
---------------
controller.py needs a velocity threshold to decide when the upward stroke has
begun. NEXT_STEPS_v3 item 7 treated this as a value to be set by judgment.
It shouldn't be. It's a SEPARATION problem: the threshold must lie above the
velocities produced by incidental motion (fidgeting, walking, reaching low)
and below the velocities produced by the slowest INTENTIONAL lift.

So this script does not measure "a velocity." It collects labeled velocity
distributions for distinct movement conditions, and reports the percentiles
that define the gap between them. If the distributions OVERLAP, that is a real
and important negative result: velocity alone is insufficient as an intent
signal, and the trigger needs a second input (current, or position history).

SAFETY
------
The actuator is DISABLED for the entire run. Nothing is ever commanded. The
encoder reads regardless of enable state, so full data is available with zero
actuation risk. The script verifies present_iq stays near zero and aborts if
it does not.

KNOWN BIAS — READ THIS BEFORE USING THE NUMBERS
-----------------------------------------------
With the actuator disabled the wearer is fighting ~0.67 Nm of measured
friction plus rig inertia, unassisted. Their natural velocity profile WITH
assist will differ — most likely faster. The threshold derived here is a
starting value, not a final one. Expect to re-run this once the controller
is live and iterate. Do not present these numbers as final to the PI.
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import LOG_DIR, MIN_ANGLE, MAX_ANGLE

PHI_DEG = 72.0              # actuator -> vest frame offset
MIN_SAMPLE_RATE_HZ = 60.0   # below this, fast lifts are undersampled
RATE_TEST_SECONDS = 3.0
IQ_DISABLED_TOLERANCE = 0.15  # A; present_iq should sit near zero when disabled

# Trial protocol. (label, seconds, reps, instruction)
# Labels group into two families the threshold must separate:
#   INTENT  = lift_slow, lift_fast, overhead_task
#   NO_INTENT = static_hold_*, incidental
# lower_controlled is logged separately — it is neither, but it is what the
# downward-stroke logic will eventually have to recognise.
TRIALS = [
    ("static_hold_low",   10, 2, "Stand still, arm relaxed at the bottom stop. Do not move."),
    ("static_hold_mid",   10, 2, "Raise the arm to roughly horizontal and HOLD it still."),
    ("static_hold_high",  10, 2, "Raise the arm near the top of its travel and HOLD it still."),
    ("incidental",        20, 2, "Move naturally WITHOUT any deliberate overhead lift: shift "
                                 "weight, walk in place, fidget, reach for something at waist height."),
    ("lift_slow",          8, 3, "From the bottom, raise the arm as SLOWLY as you still would "
                                 "during real work. This defines the lower bound of intent."),
    ("lift_fast",          8, 3, "From the bottom, raise the arm quickly, as if reaching up fast."),
    ("lower_controlled",   8, 3, "From the top, lower the arm under control back to the bottom."),
    ("overhead_task",     20, 2, "Simulate real work: lift, hold overhead, make small adjustments "
                                 "while held, then lower. Repeat for the whole block."),
]


def percentile(sorted_vals, p):
    """Linear-interpolated percentile. Pure Python — numpy is not guaranteed on the Pi."""
    if not sorted_vals:
        return float('nan')
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * (p / 100.0)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


iface = BearInterface()


def read_fast():
    """Hot-loop read: only what changes fast. Temperatures are read per-block."""
    pos, err = iface.bear.get_present_position(iface.id)[0]
    vel, _ = iface.bear.get_present_velocity(iface.id)[0]
    iq, _ = iface.bear.get_present_iq(iface.id)[0]
    return pos[0], vel[0], iq[0], err


def read_temps():
    w, _ = iface.bear.get_winding_temperature(iface.id)[0]
    p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
    return w[0], p[0]


def record_block(writer, f, label, rep, duration, w_temp, p_temp):
    """Records one labelled block. Returns list of velocities for the summary."""
    velocities = []
    t0 = time.monotonic()
    n = 0
    max_iq_seen = 0.0

    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= duration:
            break
        pos, vel, iq, err = read_fast()
        n += 1
        max_iq_seen = max(max_iq_seen, abs(iq))
        velocities.append(vel)

        writer.writerow([
            f"{time.time():.4f}", label, rep, f"{elapsed:.4f}",
            f"{pos:.5f}", f"{math.degrees(pos):.3f}",
            f"{math.degrees(pos) + PHI_DEG:.3f}",
            f"{vel:.5f}", f"{iq:.4f}", f"{w_temp:.1f}", f"{p_temp:.1f}", err,
        ])

        if abs(iq) > IQ_DISABLED_TOLERANCE:
            raise RuntimeError(
                f"present_iq reached {iq:.3f} A with the actuator supposedly "
                f"disabled. Something is commanding torque. STOP and investigate."
            )
        if pos <= MIN_ANGLE or pos >= MAX_ANGLE:
            # Informational only. Nothing is commanded, so this is not a hazard —
            # it just means the encoder left the modelled range.
            print(f"    (note: position {math.degrees(pos):.1f} deg outside "
                  f"configured joint limits)")

    f.flush()
    rate = n / duration
    print(f"    {n} samples, {rate:.1f} Hz, peak |iq| {max_iq_seen:.3f} A")
    return velocities


os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(
    LOG_DIR, f"velocity_profile_{time.strftime('%Y%m%d_%H%M%S')}.csv"
)

collected = {}

try:
    # --- Preflight, NOBODY in the vest yet -----------------------------------
    print("=" * 72)
    print("  PREFLIGHT — the vest should NOT be worn yet")
    print("=" * 72)

    iface.disable()
    print("Actuator explicitly disabled.")
    time.sleep(0.5)

    pos, vel, iq, err = read_fast()
    w_temp, p_temp = read_temps()
    print(f"pos={math.degrees(pos):.2f} deg (vest {math.degrees(pos) + PHI_DEG:.2f} deg)  "
          f"vel={vel:+.4f} rad/s  iq={iq:+.4f} A  err={err}  "
          f"w={w_temp:.1f}C  p={p_temp:.1f}C")

    if abs(iq) > IQ_DISABLED_TOLERANCE:
        raise SystemExit(
            f"ABORT: present_iq is {iq:.3f} A at rest with the actuator disabled. "
            f"Expected near zero. Do not put a person in this rig until resolved."
        )

    # Measure achieved sample rate before committing a human to the session.
    print(f"\nMeasuring achieved sample rate for {RATE_TEST_SECONDS:.0f}s...")
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < RATE_TEST_SECONDS:
        read_fast()
        n += 1
    rate = n / (time.monotonic() - t0)
    print(f"Achieved {rate:.1f} Hz ({1000.0 / rate:.2f} ms per sample).")

    if rate < MIN_SAMPLE_RATE_HZ:
        raise SystemExit(
            f"ABORT: {rate:.1f} Hz is below the {MIN_SAMPLE_RATE_HZ:.0f} Hz floor. "
            f"Fast lifts would be undersampled and the resulting percentiles would "
            f"be artifacts. Fix the read path before running a human session."
        )

    total_s = sum(d * r for _, d, r, _ in TRIALS)
    print(f"\nProtocol: {len(TRIALS)} conditions, "
          f"{sum(r for _, _, r, _ in TRIALS)} blocks, "
          f"{total_s}s of recording (~{total_s / 60.0:.0f} min plus setup).")
    print("Ctrl+C between blocks is safe — every block is flushed to disk on completion.\n")

    input("Have the wearer put the vest on now. Press Enter when they are ready: ")

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp', 'trial_label', 'rep', 'elapsed_s',
            'position_rad', 'position_deg_actuator', 'position_deg_vest',
            'velocity_rad_s', 'iq_A', 'winding_temp_C', 'powerstage_temp_C', 'error',
        ])

        for label, duration, reps, instruction in TRIALS:
            for rep in range(1, reps + 1):
                print("\n" + "-" * 72)
                print(f"  {label}  (rep {rep}/{reps}, {duration}s)")
                print(f"  {instruction}")
                print("-" * 72)
                input("  Press Enter to start recording: ")
                print("  RECORDING...")

                w_temp, p_temp = read_temps()
                vels = record_block(writer, f, label, rep, duration, w_temp, p_temp)
                collected.setdefault(label, []).extend(vels)

    print("\n" + "=" * 72)
    print(f"  Complete. Raw data: {log_path}")
    print("=" * 72)

except KeyboardInterrupt:
    print("\n\nInterrupted. Data collected so far has been written to disk.")

finally:
    iface.disable()
    print("Actuator disabled.")

    # --- Summary: the whole point of the exercise ---------------------------
    if collected:
        print("\n" + "=" * 72)
        print("  VELOCITY DISTRIBUTIONS BY CONDITION (rad/s)")
        print("=" * 72)
        print(f"{'condition':<20}{'n':>7}{'p50':>9}{'p90':>9}{'p95':>9}{'p99':>9}{'max':>9}")
        for label in [t[0] for t in TRIALS]:
            if label not in collected:
                continue
            v = sorted(collected[label])
            print(f"{label:<20}{len(v):>7}"
                  f"{percentile(v, 50):>9.4f}{percentile(v, 90):>9.4f}"
                  f"{percentile(v, 95):>9.4f}{percentile(v, 99):>9.4f}{max(v):>9.4f}")

        # The separation question, stated directly.
        no_intent = sorted(
            v for lab in ('static_hold_low', 'static_hold_mid', 'static_hold_high', 'incidental')
            for v in collected.get(lab, [])
        )
        # Only the rising portion of intentional lifts defines the lower bound.
        intent_rising = sorted(
            v for lab in ('lift_slow', 'lift_fast', 'overhead_task')
            for v in collected.get(lab, []) if v > 0
        )

        if no_intent and intent_rising:
            ceiling = percentile(no_intent, 99)
            floor = percentile(intent_rising, 10)
            print("\n" + "=" * 72)
            print("  SEPARATION")
            print("=" * 72)
            print(f"  Incidental/static ceiling (p99):  {ceiling:.4f} rad/s")
            print(f"  Intentional rising floor (p10):   {floor:.4f} rad/s")
            if floor > ceiling:
                print(f"\n  CLEAN SEPARATION. Gap = {floor - ceiling:.4f} rad/s.")
                print(f"  Candidate v_lo = {ceiling:.4f}, v_hi = {floor:.4f}")
                print(f"  (blend band for controller.py, not a hard threshold)")
            else:
                print(f"\n  OVERLAP of {ceiling - floor:.4f} rad/s.")
                print("  Velocity alone CANNOT separate intent from incidental motion.")
                print("  The trigger needs a second input. This is a real result, not a")
                print("  failed run — report it rather than tuning the threshold to hide it.")
        print(f"\n  Reminder: unassisted baseline. Re-measure once assist is live.\n")