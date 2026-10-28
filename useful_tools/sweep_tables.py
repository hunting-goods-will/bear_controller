"""
Spring-minus-gravity (y) and friction (f) per 5-degree bin from a bare
observer_excitation.py sweep. No hardware.

    python3 useful_tools/sweep_tables.py <sweep.csv>
    python3 useful_tools/sweep_tables.py <before.csv> --after <after.csv>

WHAT IT DOES
------------
In a Position-Mode sweep the motor drags the bare arm down, then up, at a
steady 0.10 rad/s. At each angle the motor torque KT*iq is

    going down:  KT*iq = -y - f        going up:  KT*iq = -y + f

where y = tau_spring - tau_rig_gravity (what the spring wins by) and f is
friction (it always opposes the motion). So, per 5-degree vest bin:

    f = (up - down) / 2        y = -(up + down) / 2

Only steady samples count (goal speed within 10 % of the nominal sweep
speed, so no ramps), and a bin needs >= 30 samples on EACH leg.

With --after it also averages two sweeps on their shared bins and reports
how much y and f moved between them (the drift / rig-stability check;
Oct 15 gate: RMS dy <= 0.06, RMS df <= 0.10 N m).

Sign: a bare-rig torque-mode hold at a still angle needs u = -y (+- friction).

Moved verbatim from fit_j_release.py (sweep_tables, merge_sweeps and their
helpers, Oct 26) so the J-release code can be deleted. Same constants.
"""
import argparse
import csv
import os
import re
import sys

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main_controller.controller import KT, PHI_DEG  # noqa: E402

BIN_DEG = 5.0                  # vest-angle bins
MIN_BIN_N = 30                 # samples per bin and leg
STEADY_TOL = 0.10              # goal speed within 10 % of nominal = steady
SG_GOAL_WINDOW = 31            # smoothing for the sweep's goal speed
STABLE_DY = 0.06               # N m, RMS of y(after) - y(before)  (Oct 15 gate)
STABLE_DF = 0.10               # N m, RMS of f(after) - f(before)


def read_csv(path):
    """Return (comment lines, list of row dicts). Comment lines keep their '#'."""
    with open(path, newline='') as f:
        lines = f.read().splitlines()
    comments = [l for l in lines if l.startswith('#')]
    body = [l for l in lines if l and not l.startswith('#')]
    return comments, list(csv.DictReader(body))


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def vest_deg(act_rad):
    return np.degrees(act_rad) + PHI_DEG


def sweep_tables(path):
    comments, rows = read_csv(path)
    hdr = ' '.join(comments)
    if 'observer_excitation.py' not in hdr or 'mode=sweep' not in hdr:
        raise SystemExit(f"{path}: not an observer_excitation.py sweep log")
    m = re.search(r'arm_mass_kg=([-\d.eE]+)', hdr)
    if not m or float(m.group(1)) != 0.0:
        raise SystemExit(f"{path}: not a bare sweep (arm_mass_kg must be 0)")
    m = re.search(r'\bvelocity=([-\d.eE]+)', hdr)
    nominal = float(m.group(1)) if m else None

    legs = [r for r in rows if r.get('phase') in ('up', 'down')]
    if not legs:
        raise SystemExit(f"{path}: no up/down rows")
    t = np.array([fnum(r['t']) for r in legs])
    goal = np.array([fnum(r['goal_pos']) for r in legs])
    pos = np.array([fnum(r['pos']) for r in legs])
    iq = np.array([fnum(r['present_iq']) for r in legs])
    direction = np.array([1.0 if r['phase'] == 'up' else -1.0 for r in legs])

    steady = np.zeros(len(legs), bool)
    for d in (1.0, -1.0):            # each leg on its own, so no cross-leg smoothing
        idx = np.where(direction == d)[0]
        if len(idx) < SG_GOAL_WINDOW:
            continue
        gv = np.gradient(savgol_filter(goal[idx], SG_GOAL_WINDOW, 2), t[idx])
        nom = nominal if nominal else np.nanpercentile(np.abs(gv), 90)
        steady[idx] = np.abs(np.abs(gv) - nom) <= STEADY_TOL * nom

    vest = vest_deg(pos)
    b = np.round(vest / BIN_DEG) * BIN_DEG
    table = []
    for bc in sorted(set(b[steady])):
        up = steady & (b == bc) & (direction > 0)
        dn = steady & (b == bc) & (direction < 0)
        if up.sum() < MIN_BIN_N or dn.sum() < MIN_BIN_N:
            continue
        mu, md = (KT * iq[up]).mean(), (KT * iq[dn]).mean()
        table.append((float(vest[up | dn].mean()), (mu - md) / 2.0,
                      -(mu + md) / 2.0, int(up.sum()), int(dn.sum())))
    if len(table) < 2:
        raise SystemExit(f"{path}: fewer than 2 friction bins with both legs")
    return np.array(table)          # cols: vest_mean, friction, y_sweep, n_up, n_dn


def merge_sweeps(before, after):
    """Average two sweep tables on their common 5-deg bins. Returns (table,
    stats) with table cols as sweep_tables() and stats = RMS/max of the
    after - before differences in y and f."""
    def keyed(t):
        return {round(r[0] / BIN_DEG) * BIN_DEG: r for r in t}
    kb, ka = keyed(before), keyed(after)
    common = sorted(set(kb) & set(ka))
    if len(common) < 2:
        raise SystemExit("before/after sweeps share fewer than 2 bins")
    rows, dy, df = [], [], []
    for k in common:
        b, a = kb[k], ka[k]
        rows.append(((b[0] + a[0]) / 2, (b[1] + a[1]) / 2, (b[2] + a[2]) / 2,
                     min(b[3], a[3]), min(b[4], a[4])))
        dy.append(a[2] - b[2])
        df.append(a[1] - b[1])
    dy, df = np.array(dy), np.array(df)
    stats = dict(rms_dy=float(np.sqrt(np.mean(dy ** 2))), max_dy=float(np.max(np.abs(dy))),
                 rms_df=float(np.sqrt(np.mean(df ** 2))), max_df=float(np.max(np.abs(df))),
                 bins=len(common), dy=dy, df=df)
    return np.array(rows), stats


def interp_inside(table, vest, col):
    """Linear interpolation of a table column (1 = friction, 2 = y) at a vest
    angle. Returns nan OUTSIDE the table's range: never extrapolate."""
    x = table[:, 0]
    if vest < x.min() or vest > x.max():
        return float('nan')
    return float(np.interp(vest, x, table[:, col]))


def print_table(t, title):
    print(f"\n{title}")
    print(f"{'vest deg':>9}{'act deg':>9}{'y N m':>9}{'f N m':>9}{'n up':>7}{'n dn':>7}")
    for r in t:
        print(f"{r[0]:9.2f}{r[0] - PHI_DEG:9.2f}{r[2]:9.3f}{r[1]:9.3f}{int(r[3]):7d}{int(r[4]):7d}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('sweep')
    ap.add_argument('--after', help='a second sweep from the same session')
    a = ap.parse_args(argv)
    before = sweep_tables(a.sweep)
    print_table(before, f"Sweep: {a.sweep}")
    if a.after:
        after = sweep_tables(a.after)
        print_table(after, f"Sweep: {a.after}")
        merged, s = merge_sweeps(before, after)
        print_table(merged, "Average of the two (shared bins)")
        ok = s['rms_dy'] <= STABLE_DY and s['rms_df'] <= STABLE_DF
        print(f"\nDrift over {s['bins']} bins: y RMS {s['rms_dy']:.3f} (max {s['max_dy']:.3f}), "
              f"f RMS {s['rms_df']:.3f} (max {s['max_df']:.3f}) N m -> "
              f"{'STABLE' if ok else 'MOVED'} (gate y <= {STABLE_DY}, f <= {STABLE_DF})")


if __name__ == '__main__':
    main()