"""
Analyse a live human assist run.

    python3 useful_tools/analyze_human_run.py logs/human_live_YYYYMMDD_HHMMSS.csv

SECTION 2, IN CONTEXT
----------------------
Section 2 checks whether commanded torque tracks tau_residual -- the value
controller.py itself computed for this same run -- restricted to samples
where the blend is fully engaged (blend_w >= 0.99), not saturated at the
torque ceiling, and not mid-ramp on the rate limiter ("settled" samples).

Because tau_requested is a deterministic, near-linear function of that same
row's tau_residual (see main_controller/controller.py), a clean fit here
confirms the control law's own arithmetic is behaving as coded -- it is a
self-consistency check, not a test of whether the spring/gravity MODEL
matches physical reality. For that, see logs/model_validation_csvs/ and
extract_spring.py, which compare the model's prediction against
independently measured actuator current from a scripted, non-human run.

The other sections check the safety mechanisms actually engaged rather than
being present but never exercised.
"""
import csv
import math
import sys

from main_controller.controller import MAX_TAU_RATE


def pct(v, p):
    if not v:
        return float('nan')
    s = sorted(v)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] * (hi - k) + s[hi] * (k - lo)


def fl(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def main(path):
    rows = list(csv.DictReader(open(path, newline='')))
    n = len(rows)
    if n < 50:
        raise SystemExit(f"Only {n} rows.")

    t = [fl(r, 't') for r in rows]
    dt = [fl(r, 'loop_dt') for r in rows]
    act = [fl(r, 'act_deg') for r in rows]
    vest = [fl(r, 'vest_deg') for r in rows]
    vel = [fl(r, 'vel_rad_s') for r in rows]
    w = [fl(r, 'blend_w') for r in rows]
    tau = [fl(r, 'tau_requested') for r in rows]
    iq_app = [fl(r, 'iq_applied') for r in rows]
    iq_meas = [fl(r, 'iq_measured') for r in rows]
    res = [fl(r, 'tau_residual') for r in rows]
    latched = [r.get('latched') == 'True' for r in rows]
    vel_ok = [r.get('vel_ok') != 'False' for r in rows]
    sat = [r.get('saturated') == 'True' for r in rows]
    dur = t[-1] - t[0]

    print(f"\nFile: {path}")
    print(f"{n} samples over {dur:.1f}s at {n/dur:.0f} Hz")
    print(f"Angle range: act {min(act):.1f} to {max(act):.1f}  "
          f"(vest {min(vest):.1f} to {max(vest):.1f})")

    # --- 1. DID ASSIST ENGAGE, AND ONLY WHERE IT SHOULD ---------------------
    on = [i for i in range(n) if iq_app[i] and iq_app[i] > 0.01]
    print("\n" + "=" * 72)
    print("  1. ENGAGEMENT")
    print("=" * 72)
    print(f"  assist commanded on {len(on)} samples ({100*len(on)/n:.1f}%)")
    if on:
        print(f"  peak torque {max(tau):.3f} Nm / {max(iq_app):.3f} A")
        print(f"  engaged over act {min(act[i] for i in on):.1f} "
              f"to {max(act[i] for i in on):.1f} deg")
        bad_dir = [i for i in on if vel[i] is not None and vel[i] < 0]
        print(f"  commanded while moving DOWN: {len(bad_dir)}"
              f"{'  <-- SHOULD BE ZERO' if bad_dir else '  (correct)'}")
        still = [i for i in on if vel[i] is not None and abs(vel[i]) < 0.05]
        print(f"  commanded while nearly still: {len(still)}")
        print(f"  saturated at the clamp: {sum(sat)} samples "
              f"({100*sum(sat)/n:.1f}%)"
              f"{'  <-- curve is being flattened' if sum(sat) > 0.02*n else ''}")

    # --- 2. SELF-CONSISTENCY: TORQUE vs THE CONTROLLER'S OWN RESIDUAL -------
    print("\n" + "=" * 72)
    print("  2. SELF-CONSISTENCY: TORQUE vs THE CONTROLLER'S OWN RESIDUAL")
    print("=" * 72)

    rate_limit = 0.75 * MAX_TAU_RATE
    settled = []
    blank_tau = 0
    for i in on:
        if w[i] is None or w[i] < 0.99:
            continue
        if sat[i]:
            continue
        if tau[i] is None:
            blank_tau += 1
            continue
        if i == 0 or tau[i - 1] is None or dt[i] is None or dt[i] <= 0:
            blank_tau += 1
            continue
        if abs(tau[i] - tau[i - 1]) / dt[i] > rate_limit:
            continue
        settled.append(i)
    if blank_tau:
        print(f"  skipped (blank tau_requested, or no dt to check rate): {blank_tau}")
    print(f"  settled samples (blend_w>=0.99, not saturated, |dtau/dt|<"
          f"{rate_limit:.2f} Nm/s): {len(settled)} of {len(on)} engaged")

    bins = {}
    for i in settled:
        if res[i] is None:
            continue
        b = round(act[i] / 10.0) * 10.0
        bins.setdefault(b, []).append((tau[i], res[i], iq_meas[i]))
    if len(bins) < 2:
        print("  Too few settled bins to judge shape.")
    else:
        print(f"{'act':>6}{'vest':>7}{'n':>7}{'residual':>11}{'tau_cmd':>10}"
              f"{'iq_meas':>10}")
        bin_tc = []
        for b in sorted(bins):
            v = bins[b]
            r = sum(x[1] for x in v)/len(v)
            tc = sum(x[0] for x in v)/len(v)
            im = sum(x[2] for x in v if x[2] is not None)/max(1, len(v))
            bin_tc.append(tc)
            print(f"{b:6.0f}{b+72:7.0f}{len(v):7}{r:11.3f}{tc:10.3f}{im:10.3f}")
        spread = max(bin_tc) - min(bin_tc)
        print(f"\n  commanded torque range across bins: {spread:.3f} Nm "
              f"({min(bin_tc):.3f} to {max(bin_tc):.3f})")

        fit = [(res[i], tau[i]) for i in settled
               if res[i] is not None and res[i] > 0]
        if len(fit) < 2:
            print("  Too few settled samples with tau_residual > 0 to fit.")
        else:
            xs = [p[0] for p in fit]
            ys = [p[1] for p in fit]
            k = len(xs); mx = sum(xs)/k; my = sum(ys)/k
            sxx = sum((a-mx)**2 for a in xs)
            sxy = sum((a-mx)*(b-my) for a, b in zip(xs, ys))
            syy = sum((b-my)**2 for b in ys)
            slope = sxy / sxx if sxx else 0.0
            intercept = my - slope * mx
            r2 = (sxy * sxy) / (sxx * syy) if sxx and syy else 0.0
            print(f"\n  fit: tau_requested = {slope:.3f} * tau_residual + "
                  f"{intercept:.3f}   (n={k}, R^2={r2:.3f})")
            print("  slope is the effective alpha actually used this run; "
                  "intercept is roughly k_f * TAU_FRICTION * blend_w.")
            print("  This confirms the control law's own arithmetic is behaving")
            print("  as coded -- it is not evidence the spring/gravity model")
            print("  matches reality. See model_validation logs + extract_spring.py")
            print("  for that comparison.")

    # --- 3. RATE LIMITER ----------------------------------------------------
    rates = [abs(tau[i]-tau[i-1])/dt[i] for i in range(1, n)
             if dt[i] and dt[i] > 0]
    print("\n" + "=" * 72)
    print("  3. RATE LIMITER  (the fix for the flange failure)")
    print("=" * 72)
    print(f"  dtau/dt  p99 {pct(rates,99):.2f}   max {max(rates):.2f} Nm/s "
          f"(limit 4.0)")
    over = sum(1 for x in rates if x > 4.2)
    print(f"  samples over 4.2 Nm/s: {over}"
          f"{'  <-- LIMITER LEAKING' if over else '  (holding)'}")

    # --- 4. VELOCITY SIGNAL -------------------------------------------------
    rej = sum(1 for x in vel_ok if not x)
    print("\n" + "=" * 72)
    print("  4. VELOCITY SIGNAL")
    print("=" * 72)
    print(f"  rejected samples: {rej} ({100*rej/n:.2f}%)")
    print(f"  peak |velocity|: {max(abs(v) for v in vel if v is not None):.2f} rad/s")
    if rej > 0.005*n:
        print("  -> register unreliable at speed; blend band needs re-examining")

    # --- 5. LATCH -----------------------------------------------------------
    trans = [i for i in range(1, n) if latched[i] != latched[i-1]]
    segs, start = [], 0
    for i in trans:
        if latched[start]:
            segs.append(t[i]-t[start])
        start = i
    print("\n" + "=" * 72)
    print("  5. LATCH  (the fix for stutter)")
    print("=" * 72)
    print(f"  latch transitions: {len(trans)} ({len(trans)/dur:.2f}/s)")
    if segs:
        print(f"  engaged segment length  p50 {pct(segs,50)*1000:.0f} ms   "
              f"min {min(segs)*1000:.0f} ms")
        print(f"  segments under 150 ms (min-on-time): "
              f"{sum(1 for s in segs if s < 0.150)}"
              f"{'  <-- latch not holding' if any(s < 0.145 for s in segs) else '  (correct)'}")

    print("\n" + "=" * 72)
    print("  THERMAL")
    print("=" * 72)
    wt = [fl(r, 'w_temp') for r in rows]
    pt = [fl(r, 'p_temp') for r in rows]
    print(f"  winding {min(wt):.0f} to {max(wt):.0f} C     "
          f"powerstage {min(pt):.0f} to {max(pt):.0f} C\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])