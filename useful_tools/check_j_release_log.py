"""
Data-quality check for Block 4 logs. Offline only: no hardware, commands nothing.
Deliberately computes NOTHING that reveals J (no accelerations, slopes or peak
speeds), so protocol fixes can be decided before the J number is seen.

    python3 useful_tools/check_j_release_log.py RELEASE.csv [RELEASE.csv ...] \\
        [--sweep SWEEP.csv]

Per release (free phase = reads only, after disable()):
  * release command -> first free sample (ms), free-phase sample count, max gap
  * pinned fit window (fit_j_release.py): free rows from the release until the
    first row with vel >= 3 rad/s or past the table end (act 104.6 deg),
    keeping 0.2 < vel < 3 -> whether it has >= 8 samples, and the angle it ends at
  * error timeline: first row with err != 0x80 -> its position, and whether it
    is BEFORE or AFTER the window ends (decoded bits)
  * highest position reached in the free phase
  * current: mean / max |iq| inside the window; after the first error, the
    share of rows with |iq| >= 0.1 A whose iq opposes velocity (damping sign)
  * input_voltage range in the free phase (back-EMF)
Sweep (optional): steady samples, friction bins and the spring-table re-check
(same code as fit_j_release.py).
"""
import argparse
import csv
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from main_controller.controller import PHI_DEG, TAU_SPRING_TABLE  # noqa: E402

V_MIN, V_MAX = 0.2, 3.0
TABLE_END_ACT = TAU_SPRING_TABLE[-1][0] - PHI_DEG      # 104.6 deg
MIN_SAMPLES = 8
BITS = {0: 'Communication (warning)', 1: 'Overheat (warning)', 2: 'Absolute Position',
        3: 'Watchdog Timeout & ESTOP', 4: 'Joint Limit', 5: 'Hardware Fault',
        6: 'Initialization'}


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def decode(err):
    return ', '.join(BITS[b] for b in range(7) if err & (1 << b)) or 'none'


def check_release_file(path):
    with open(path, newline='') as f:
        lines = f.read().splitlines()
    comments = [l for l in lines if l.startswith('#')]
    rows = list(csv.DictReader(l for l in lines if l and not l.startswith('#')))
    print("=" * 78)
    print(f"{os.path.basename(path)}")
    for c in comments:
        if c.startswith('# release'):
            # Drop the peak-speed and time-to-rest parts: they hint at J.
            c = re.sub(r', peak \|vel\| [^,]*', '', c)
            c = re.sub(r', time to rest [^,]*', '', c)
            print(f"  {c[2:]}")
        elif c.startswith('# END') or 'started' in c:
            print(f"  {c[2:]}")
    idxs = sorted({int(r['release_idx']) for r in rows if r['release_idx'] not in ('', '0')})
    for idx in idxs:
        rr = [r for r in rows if r['release_idx'] == str(idx)]
        marks = [fnum(r['t']) for r in rr if r['phase'] == 'release']
        free = [r for r in rr if r['phase'] == 'free']
        target = rr[0]['target_deg']
        print(f"\n  release {idx} (target {target} deg)")
        if not marks or not free:
            print("    no release marker or no free-phase rows")
            continue
        t_cmd = marks[0]
        t = [fnum(r['t']) for r in free]
        pos = [math.degrees(fnum(r['pos'])) for r in free]
        vel = [fnum(r['vel']) for r in free]
        iq = [fnum(r['iq']) for r in free]
        vin = [fnum(r['input_voltage']) for r in free]
        err = [int(r['err'], 16) if r['err'] else None for r in free]
        gaps = [b - a for a, b in zip(t, t[1:])]
        print(f"    release cmd -> first free sample {1000 * (t[0] - t_cmd):.1f} ms; "
              f"{len(free)} free samples; max gap {1000 * max(gaps):.1f} ms"
              if gaps else "    one free sample only")

        # Pinned window (first pass only).
        end = len(free)
        for k in range(len(free)):
            if vel[k] >= V_MAX or pos[k] > TABLE_END_ACT:
                end = k
                break
        win = [k for k in range(end) if V_MIN < vel[k] < V_MAX]
        t_end = t[end] if end < len(free) else t[-1]
        # Counts and times inside the window hint at J, so only yes/no and angles.
        print(f"    fit window: >= {MIN_SAMPLES} samples: {'yes' if len(win) >= MIN_SAMPLES else 'NO'}; "
              f"ends at {pos[min(end, len(pos) - 1)]:.2f} deg")

        bad = [k for k in range(len(free)) if err[k] is not None and err[k] != 0x80]
        if bad:
            k = bad[0]
            when = 'AFTER' if t[k] > t_end else 'BEFORE/INSIDE'
            print(f"    first error: {hex(err[k])} ({decode(err[k])}) at pos {pos[k]:.2f} deg "
                  f"-> {when} the window end; {len(bad)}/{len(free)} free rows flagged")
        else:
            print("    no error flags in the free phase")
        kmax = max(range(len(pos)), key=lambda k: pos[k])
        print(f"    highest position {pos[kmax]:.2f} deg; last {pos[-1]:.2f} deg")
        if win:
            wi = [iq[k] for k in win]
            print(f"    iq in window: mean {sum(wi) / len(wi):+.3f} A, max |iq| "
                  f"{max(abs(x) for x in wi):.3f} A")
        if bad:
            after = [k for k in range(bad[0], len(free)) if abs(iq[k]) >= 0.1]
            opp = [k for k in after if iq[k] * vel[k] < 0]
            print(f"    after the first error: {len(after)} rows with |iq| >= 0.1 A, "
                  f"{len(opp)} of them opposing velocity (damping sign)"
                  + (f"; max |iq| {max(abs(iq[k]) for k in after):.3f} A" if after else ""))
        v_ok = [v for v in vin if not math.isnan(v)]
        if v_ok:
            print(f"    input_voltage in free phase: {min(v_ok):.2f}-{max(v_ok):.2f} V")


def check_sweep(path):
    import fit_j_release as F
    print("=" * 78)
    print(f"sweep {os.path.basename(path)}")
    fr = F.sweep_tables(path)
    yt = F.y_table(fr[:, 0])
    print(f"  {'act':>6}{'friction':>10}{'y_sweep':>9}{'y_table':>9}{'diff':>8}{'n_up':>6}{'n_dn':>6}")
    for (v, f, ys, nu, nd), ytab in zip(fr, yt):
        print(f"  {v - PHI_DEG:6.1f}{f:10.3f}{ys:9.3f}{ytab:9.3f}{ys - ytab:+8.3f}"
              f"{int(nu):6d}{int(nd):6d}")
    print(f"  coverage act {fr[0, 0] - PHI_DEG:.1f}-{fr[-1, 0] - PHI_DEG:.1f} deg; friction "
          f"{fr[:, 1].min():.3f}-{fr[:, 1].max():.3f} Nm")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('releases', nargs='+')
    ap.add_argument('--sweep')
    a = ap.parse_args(argv)
    if a.sweep:
        check_sweep(a.sweep)
    for p in a.releases:
        check_release_file(p)
    return 0


if __name__ == '__main__':
    sys.exit(main())