"""
Analyse a velocity_profile CSV. Run locally, paste the OUTPUT, not the data.

TWO QUESTIONS THIS ANSWERS
--------------------------
1. ASSIST LATENCY. Given a candidate trigger threshold, how long after a lift
   actually begins does velocity cross it -- and how many degrees has the arm
   already travelled by then? This is the number that decides whether velocity
   triggering is viable at all. Clean separation between intentional and
   incidental motion is necessary but NOT sufficient: if the user has already
   lifted 20 degrees unassisted before assist engages, they've done the hardest
   part of the stroke alone and the architecture is wrong regardless.

2. FALSE TRIGGERS. How often would that same threshold have fired during the
   static-hold and incidental blocks, where assist must never engage?

Lower threshold -> less latency, more false triggers. Higher -> the reverse.
The table below is that tradeoff, computed from real data instead of guessed.

USAGE
-----
    python3 analyse_velocity_profile.py logs/Velocity_profile/<file>.csv
"""
import csv
import math
import sys

INTENT_LABELS = ('lift_slow', 'lift_fast', 'overhead_task')
QUIET_LABELS = ('static_hold_low', 'static_hold_mid', 'static_hold_high', 'incidental')

CANDIDATE_THRESHOLDS = [0.08, 0.10, 0.15, 0.20, 0.30, 0.45, 0.60]

NOISE_FLOOR = 0.05      # rad/s; below this the arm is treated as not moving
MIN_SUSTAIN = 0.10      # s; a crossing must hold this long to count as real
MAX_BACKTRACK = 2.0     # s; how far back to search for lift onset


def percentile(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] * (hi - k) + s[hi] * (k - lo)


def load(path):
    """Returns {(label, rep): (times, velocities, positions_rad)}"""
    blocks = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            key = (row['trial_label'], row['rep'])
            t, v, p = blocks.setdefault(key, ([], [], []))
            t.append(float(row['elapsed_s']))
            v.append(float(row['velocity_rad_s']))
            p.append(float(row['position_rad']))
    return blocks


def find_crossings(t, v, thresh):
    """Sustained upward crossings. Returns list of (onset_idx, cross_idx)."""
    events = []
    n = len(v)
    i = 0
    while i < n:
        if v[i] <= thresh:
            i += 1
            continue
        j = i
        while j < n and v[j] > thresh:
            j += 1
        if t[j - 1] - t[i] >= MIN_SUSTAIN:
            # Walk back to where the arm was genuinely still.
            k = i
            while k > 0 and v[k] > NOISE_FLOOR and (t[i] - t[k]) < MAX_BACKTRACK:
                k -= 1
            events.append((k, i))
        i = j
    return events


def main(path):
    blocks = load(path)
    if not blocks:
        raise SystemExit(f"No rows read from {path}")

    print(f"\nFile: {path}")
    print(f"Blocks: {len(blocks)}   "
          f"Samples: {sum(len(v[0]) for v in blocks.values())}")

    # Noise floor, measured rather than assumed.
    quiet_peak = 0.0
    for (label, _), (t, v, p) in blocks.items():
        if label in QUIET_LABELS:
            quiet_peak = max(quiet_peak, max(abs(x) for x in v))
    print(f"Worst non-intentional velocity observed: {quiet_peak:.4f} rad/s")

    print("\n" + "=" * 78)
    print("  TRIGGER THRESHOLD TRADEOFF")
    print("=" * 78)
    print(f"{'thresh':>8}{'lifts':>7}{'lat_p50':>10}{'lat_p90':>10}"
          f"{'deg_p50':>10}{'deg_p90':>10}{'false':>8}")
    print(f"{'rad/s':>8}{'':>7}{'ms':>10}{'ms':>10}{'deg':>10}{'deg':>10}{'fires':>8}")
    print("-" * 78)

    for thresh in CANDIDATE_THRESHOLDS:
        latencies, displacements, n_lifts = [], [], 0
        for (label, _), (t, v, p) in blocks.items():
            if label not in INTENT_LABELS:
                continue
            for onset, cross in find_crossings(t, v, thresh):
                n_lifts += 1
                latencies.append((t[cross] - t[onset]) * 1000.0)
                displacements.append(math.degrees(p[cross] - p[onset]))

        false_fires = 0
        for (label, _), (t, v, p) in blocks.items():
            if label in QUIET_LABELS:
                false_fires += len(find_crossings(t, v, thresh))

        print(f"{thresh:>8.2f}{n_lifts:>7}"
              f"{percentile(latencies, 50):>10.1f}{percentile(latencies, 90):>10.1f}"
              f"{percentile(displacements, 50):>10.2f}{percentile(displacements, 90):>10.2f}"
              f"{false_fires:>8}")

    # Lowering: the summary table in the collection script only printed upper
    # percentiles, so the entire descending signal was invisible.
    print("\n" + "=" * 78)
    print("  DESCENDING MOTION (negative velocity)")
    print("=" * 78)
    for label in ('lower_controlled', 'overhead_task'):
        neg = [v for (lab, _), (t, vs, p) in blocks.items() if lab == label
               for v in vs if v < 0]
        if neg:
            print(f"{label:<20} n={len(neg):>6}  "
                  f"p50={percentile(neg, 50):>8.4f}  "
                  f"p10={percentile(neg, 10):>8.4f}  "
                  f"min={min(neg):>8.4f}")

    print("\nHow to read this: pick the lowest threshold with zero false fires,")
    print("then check whether its deg_p90 is an acceptable amount of unassisted")
    print("travel. If it isn't, velocity triggering is the wrong architecture\n"
          "and the trigger needs the current signal.\n")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])