"""
Compare any number of bare observer_excitation.py sweeps: how much do the
spring-minus-gravity (y) and friction (f) curves move between runs? No hardware;
reads CSVs only, writes nothing unless --out is given.

    python3 useful_tools/compare_sweeps.py <sweep1.csv> <sweep2.csv> [...] [--out file.txt]

Sweeps are listed in the order given (put them in time order). For each one it
uses sweep_tables() (same bins and rules as the hold scripts), then prints:

  1. y per 5-degree vest bin, one column per sweep, then the spread (max - min);
  2. the same for f;
  3. every pair: time apart, RMS change in y and f on shared bins
     (Oct 15 stability gate: y <= 0.06, f <= 0.10 N m);
  4. each sweep against the first, split into
       offset = mean change (the whole curve moved up or down) and
       shape  = RMS left after removing that offset (the curve bent);
  5. current peaks per sweep (|iq| max, and the highest "lowest of 3 in a row",
     which is what the x3 abort sees), against THAT sweep's own limits:
     observer_excitation.py aborts at i_max - 0.5 A for 3 samples and trips at
     i_max - 0.2 A. i_max comes from the header's 'i_max=' (since Oct 28), else
     the 'limit_i_max=set/read' read-back, else the 5.5 A default; the source is
     printed. (Until Oct 29 this column assumed 5.0 A for every sweep and showed
     -2.86 A for an 8.5 A run; STATE 8 #32.)

Why the split in 4: a uniform offset could be corrected from one reading (e.g. one
hold); a change in shape could not.
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sweep_tables import (sweep_tables, merge_sweeps, read_csv, fnum,   # noqa: E402
                          BIN_DEG, STABLE_DY, STABLE_DF)
from main_controller.controller import PHI_DEG                          # noqa: E402

TS = re.compile(r'(\d{8})_(\d{6})')
I_MAX_DEFAULT = 5.5                 # observer_excitation.py's limit_i_max without --i-max
ABORT_BELOW, TRIP_BELOW = 0.5, 0.2  # its abort (x3) and trip, below i_max
RE_IMAX = re.compile(r'\bi_max=([0-9.]+)')
RE_READBACK = re.compile(r'\blimit_i_max=([0-9.]+)/([0-9.]+)')


def label(path):
    m = TS.search(os.path.basename(path))
    if not m:
        return os.path.basename(path)[:13]
    d, t = m.groups()
    return f"{d[4:6]}-{d[6:]} {t[:2]}:{t[2:4]}"


def minutes(path):
    m = TS.search(os.path.basename(path))
    if not m:
        return None
    d, t = m.groups()
    import datetime as dt
    return dt.datetime.strptime(d + t, '%Y%m%d%H%M%S').timestamp() / 60.0


def keyed(table):
    return {int(round(r[0] / BIN_DEG) * BIN_DEG): r for r in table}


def i_max_of(comments):
    """(i_max in A, where it came from) for one sweep's header."""
    for line in comments:
        m = RE_IMAX.search(line)
        if m:
            return float(m.group(1)), 'header'
    for line in comments:
        m = RE_READBACK.search(line)
        if m:
            return float(m.group(1)), 'readback'
    return I_MAX_DEFAULT, 'default'


def peaks(path):
    comments, rows = read_csv(path)
    a = [abs(fnum(r['present_iq'])) for r in rows if r.get('phase') != 'park']
    a = [x for x in a if x == x]
    run3 = max(min(a[i:i + 3]) for i in range(len(a) - 2)) if len(a) >= 3 else float('nan')
    return (max(a) if a else float('nan')), run3, i_max_of(comments)


def report(paths):
    tabs = [sweep_tables(p) for p in paths]
    labs = [label(p) for p in paths]
    keys = [keyed(t) for t in tabs]
    bins = sorted(set().union(*keys))
    out = []
    w = 12

    for col, name in ((2, 'y = spring - rig gravity (N m)'), (1, 'f = friction (N m)')):
        out.append(f"\n{name}")
        out.append(f"{'vest':>6}{'act':>6}" + ''.join(f"{l:>{w}}" for l in labs) + f"{'spread':>{w}}")
        for b in bins:
            vals = [k[b][col] if b in k else None for k in keys]
            have = [v for v in vals if v is not None]
            spread = (max(have) - min(have)) if len(have) > 1 else float('nan')
            out.append(f"{b:6d}{b - PHI_DEG:6.0f}" + ''.join(
                f"{v:{w}.3f}" if v is not None else f"{'-':>{w}}" for v in vals) + f"{spread:{w}.3f}")

    out.append(f"\nEvery pair (shared bins; gate y <= {STABLE_DY}, f <= {STABLE_DF} N m)")
    out.append(f"{'pair':<29}{'apart':>9}{'bins':>6}{'RMS dy':>9}{'max dy':>9}{'RMS df':>9}{'max df':>9}  verdict")
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            _, s = merge_sweeps(tabs[i], tabs[j])
            mi, mj = minutes(paths[i]), minutes(paths[j])
            apart = f"{(mj - mi):.0f} min" if mi is not None and mj is not None else '?'
            ok = s['rms_dy'] <= STABLE_DY and s['rms_df'] <= STABLE_DF
            out.append(f"{labs[i] + ' -> ' + labs[j]:<29}{apart:>9}{s['bins']:6d}{s['rms_dy']:9.3f}"
                       f"{s['max_dy']:9.3f}{s['rms_df']:9.3f}{s['max_df']:9.3f}  "
                       f"{'STABLE' if ok else 'MOVED'}")

    out.append(f"\nEach sweep vs the first ({labs[0]}): offset = mean change, shape = RMS after removing it")
    out.append(f"{'sweep':<14}{'y offset':>10}{'y shape':>10}{'f offset':>10}{'f shape':>10}")
    for j in range(1, len(paths)):
        _, s = merge_sweeps(tabs[0], tabs[j])
        dy, df = s['dy'], s['df']
        out.append(f"{labs[j]:<14}{dy.mean():+10.3f}{np.sqrt(np.mean((dy - dy.mean()) ** 2)):10.3f}"
                   f"{df.mean():+10.3f}{np.sqrt(np.mean((df - df.mean()) ** 2)):10.3f}")

    out.append(f"\nCurrent peaks against each sweep's own limits (abort = i_max - {ABORT_BELOW:g} A x3, "
               f"trip = i_max - {TRIP_BELOW:g} A)")
    out.append(f"{'sweep':<14}{'i_max':>7}{'from':>10}{'max |iq|':>10}{'lowest-of-3':>13}"
               f"{'abort x3':>10}{'margin x3':>11}{'trip':>7}{'margin trip':>13}")
    for p, l in zip(paths, labs):
        mx, r3, (imax, src) = peaks(p)
        ab, tr = imax - ABORT_BELOW, imax - TRIP_BELOW
        out.append(f"{l:<14}{imax:7.2f}{src:>10}{mx:10.2f}{r3:13.2f}{ab:10.2f}{ab - r3:11.2f}"
                   f"{tr:7.2f}{tr - mx:13.2f}")
    out.append("(a negative margin means that limit was reached: the sweep aborted there)")
    return '\n'.join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('sweeps', nargs='+', help='bare sweep CSVs, in time order')
    ap.add_argument('--out', help='also write the report to this file')
    a = ap.parse_args(argv)
    if len(a.sweeps) < 2:
        ap.error('give at least two sweeps')
    text = report(a.sweeps)
    print(text)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            f.write('# compare_sweeps.py ' + ' '.join(a.sweeps) + '\n' + text + '\n')
        print(f"\nWritten: {a.out}")


if __name__ == '__main__':
    main()