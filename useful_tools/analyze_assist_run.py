"""
Diagnose an assist run CSV. Paste the OUTPUT, not the data.

    python3 useful_tools/analyze_assist_run.py logs/human_live_YYYYMMDD_HHMMSS.csv
    python3 useful_tools/analyze_assist_run.py logs/human_monitor_YYYYMMDD_HHMMSS.csv

FOUR QUESTIONS
--------------
1. LOOP TIMING. Is the loop keeping up, and did reads stall? A comm stall in
   live mode leaves the last commanded torque applied for the duration.

2. VELOCITY SIGNAL QUALITY. The firmware velocity register is what gates all
   assist. If it is unreliable, both the assist trigger AND the earlier human
   velocity profiling inherit the problem. Compared here against a numerical
   derivative of the position column -- an independent estimate from a
   different register.

3. BLEND CHATTER. How often does the command switch on and off, and how long
   are the segments? Rapid toggling is felt as stutter.

4. COMMAND TRACKING (live only). Does iq_measured follow iq_applied? If not,
   either the actuator is not in torque mode, or it is saturating.

Model validation is deliberately restricted to near-constant-velocity windows:
controller.py is quasi-static and has no J*alpha term, so during acceleration
the prediction is expected to be wrong and comparing there proves nothing.

A fifth question used to live here: whether commanded torque shows a
positive-feedback (negative-damping) signature right after assist onset.
It's removed. Onset was detected purely from blend_w crossing zero, which is
a function of velocity alone and fires identically whether or not any torque
is applied -- so the "assist onsets analysed" / before-after acceleration
numbers were computed and printed on zero-torque MONITOR runs too, where
they cannot mean anything. The script only gated the final verdict string on
max(iq_applied), not the onset detection itself, which is the artifact. A
valid version of this test needs a live run and a monitor run at matched
conditions (same movement, same timing) compared against each other -- not
derivable from a single log, so it isn't attempted here.
"""
import csv
import math
import sys

DT_WARN = 0.050          # s, matches the loop guard in run_assist.py
DERIV_HALFWIN = 5        # samples either side for the smoothed derivative
STEADY_ACC_MAX = 0.5     # rad/s^2, threshold for "near-constant velocity"


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] * (hi - k) + s[hi] * (k - lo)


def f(row, key):
    v = row.get(key, '')
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main(path):
    rows = list(csv.DictReader(open(path, newline='')))
    if len(rows) < 20:
        raise SystemExit(f"Only {len(rows)} rows -- too short to analyse.")

    t = [f(r, 't') for r in rows]
    dt = [f(r, 'loop_dt') for r in rows]
    pos = [f(r, 'pos_rad') for r in rows]
    vel = [f(r, 'vel_rad_s') for r in rows]
    w = [f(r, 'blend_w') for r in rows]
    tau = [f(r, 'tau_requested') for r in rows]
    iq_app = [f(r, 'iq_applied') for r in rows]
    iq_meas = [f(r, 'iq_measured') for r in rows]
    act = [f(r, 'act_deg') for r in rows]
    n = len(rows)
    dur = t[-1] - t[0]

    print(f"\nFile: {path}")
    print(f"Rows {n}   duration {dur:.1f}s   mean rate {n / dur:.1f} Hz")
    print(f"Commanded torque at any point: "
          f"{'YES (live)' if max(iq_app) > 0.001 else 'no (monitor)'}")

    # -- 1. LOOP TIMING ------------------------------------------------------
    d = [x for x in dt[1:] if x is not None]
    slow = [x for x in d if x > DT_WARN]
    print("\n" + "=" * 74)
    print("  1. LOOP TIMING")
    print("=" * 74)
    print(f"  dt ms   p50 {pct(d, 50) * 1000:6.2f}   p90 {pct(d, 90) * 1000:6.2f}   "
          f"p99 {pct(d, 99) * 1000:6.2f}   max {max(d) * 1000:6.2f}")
    print(f"  iterations over {DT_WARN * 1000:.0f} ms: {len(slow)} "
          f"({100.0 * len(slow) / len(d):.2f}%)")
    if slow:
        print(f"  -> in live mode each leaves torque applied for that long")

    # -- 2. VELOCITY SIGNAL QUALITY ------------------------------------------
    # Smoothed centred derivative of position: an independent estimate.
    dpos = [None] * n
    for i in range(DERIV_HALFWIN, n - DERIV_HALFWIN):
        dtw = t[i + DERIV_HALFWIN] - t[i - DERIV_HALFWIN]
        if dtw > 0:
            dpos[i] = (pos[i + DERIV_HALFWIN] - pos[i - DERIV_HALFWIN]) / dtw

    pairs = [(vel[i], dpos[i]) for i in range(n)
             if dpos[i] is not None and vel[i] is not None]
    err = [abs(a - b) for a, b in pairs]
    moving = [(a, b) for a, b in pairs if abs(b) > 0.10]
    # Dropout: position clearly moving, reported velocity near zero.
    drop = [(a, b) for a, b in pairs if abs(b) > 0.20 and abs(a) < 0.05]

    print("\n" + "=" * 74)
    print("  2. VELOCITY SIGNAL QUALITY  (reported vs derivative of position)")
    print("=" * 74)
    print(f"  |reported - derivative|   p50 {pct(err, 50):.4f}   "
          f"p90 {pct(err, 90):.4f}   max {max(err):.4f} rad/s")
    print(f"  samples where position is clearly moving (>0.20 rad/s)")
    print(f"    but reported velocity is near zero (<0.05): {len(drop)} "
          f"of {len(moving)} moving samples"
          f"  ({100.0 * len(drop) / max(1, len(moving)):.1f}%)")
    if len(drop) > 0.02 * max(1, len(moving)):
        print("  -> SIGNAL DROPOUTS CONFIRMED. The firmware velocity register is")
        print("     unreliable here. This also contaminates the human velocity")
        print("     profiling results, which read the same register.")
    else:
        print("  -> No systematic dropout. Reported velocity tracks position.")

    # -- 3. BLEND CHATTER ----------------------------------------------------
    on = [x is not None and x > 0.0 for x in w]
    trans = [i for i in range(1, n) if on[i] != on[i - 1]]
    seg_on, seg_off, start = [], [], 0
    for i in trans:
        (seg_on if on[start] else seg_off).append(t[i] - t[start])
        start = i
    print("\n" + "=" * 74)
    print("  3. BLEND CHATTER")
    print("=" * 74)
    print(f"  assist engaged for {100.0 * sum(on) / n:.1f}% of samples")
    print(f"  on/off transitions: {len(trans)}  ({len(trans) / dur:.2f} per second)")
    if seg_on:
        print(f"  ON  segment length  p50 {pct(seg_on, 50) * 1000:7.1f} ms   "
              f"p90 {pct(seg_on, 90) * 1000:7.1f} ms")
        brief = [s for s in seg_on if s < 0.100]
        print(f"  ON  segments under 100 ms: {len(brief)} of {len(seg_on)}"
              f"  -> felt as stutter")
    if seg_off:
        gaps = [s for s in seg_off if s < 0.100]
        print(f"  OFF segments under 100 ms: {len(gaps)} of {len(seg_off)}"
              f"  -> dropouts mid-lift")

    # Acceleration series, used below to restrict command-tracking validation
    # to near-constant-velocity windows (the model is quasi-static).
    acc = [None] * n
    for i in range(1, n - 1):
        if dt[i] and dt[i] > 0 and vel[i + 1] is not None and vel[i - 1] is not None:
            acc[i] = (vel[i + 1] - vel[i - 1]) / (t[i + 1] - t[i - 1] or 1e-6)

    # -- 4. COMMAND TRACKING -------------------------------------------------
    print("\n" + "=" * 74)
    print("  4. COMMAND TRACKING  (live only)")
    print("=" * 74)
    live = [(iq_app[i], iq_meas[i], acc[i], act[i]) for i in range(n)
            if iq_app[i] is not None and iq_app[i] > 0.05 and iq_meas[i] is not None]
    if not live:
        print("  No commanded samples. Monitor run.")
    else:
        e = [abs(a - m) for a, m, _, _ in live]
        print(f"  commanded samples: {len(live)}")
        print(f"  |iq_applied - iq_measured|  p50 {pct(e, 50):.4f}  "
              f"p90 {pct(e, 90):.4f}  max {max(e):.4f} A")
        if pct(e, 50) > 0.20:
            print("  -> Actuator is NOT following the command. Check it is in")
            print("     torque mode, not left in position mode by a prior script.")
        steady = [(a, m, ang) for a, m, ac, ang in live
                  if ac is not None and abs(ac) < STEADY_ACC_MAX]
        print(f"  near-constant-velocity samples (|acc| < {STEADY_ACC_MAX}): "
              f"{len(steady)}")
        if steady:
            se = [abs(a - m) for a, m, _ in steady]
            print(f"    |applied - measured| here  p50 {pct(se, 50):.4f}  "
                  f"p90 {pct(se, 90):.4f} A")
            print("    (only these windows are valid for model validation --")
            print("     the model is quasi-static and ignores inertia)")

    print()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])