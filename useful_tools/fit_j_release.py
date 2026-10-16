"""
Fit the device inertia J from the Block 4 release test (Method C, pinned
2026-10-14 before any data). Offline analysis only: no hardware imports,
commands nothing.

USAGE (from the repo root)
-----
    python3 useful_tools/fit_j_release.py --sweep BEFORE.csv --sweep-after AFTER.csv \\
        RELEASE.csv [RELEASE.csv ...] \\
        [--out logs/summaries/phase2/j_release/<name>.txt] [--plot <name>.png]

  BEFORE.csv  bare slow sweep run just BEFORE the releases, same session:
              observer_excitation.py --mode sweep --arm-mass-kg 0 --arm-com-m 0
              --velocity 0.10 --soft-start-s 0.3
  AFTER.csv   the same sweep run just AFTER the releases (rig-stability check)
  RELEASE.csv one or more j_release_*.csv files (the 1/2/2 invocations)

WHAT IT FITS
------------
Torque balance on the released arm (motor disabled, bare rig, + = up):

    J*theta_ddot = tau_spring(theta) - tau_grav(theta) - f(theta)

    y(theta) = tau_spring - tau_grav   from the same-session sweeps:
                                       -(up + down)/2 of KT*iq per 5-deg vest bin
                                       (the method that built TAU_SPRING_TABLE,
                                       repeated today; no table, no RIG_MGL)
    f(theta)                           per-angle friction from the same sweeps:
                                       (up - down)/2 of KT*iq per 5-deg vest bin
    z = y - f = J * theta_ddot

PINNED METHOD (PLAN Block 4, 2026-10-14; changes of 2026-10-15 declared before
any counted release and before J was computed; do not change after data)
  * y and f: per 5-deg vest bin from the sweeps before and after the releases,
    steady samples only (commanded goal speed within 10 % of nominal,
    analyze_observer.py's rule), >= 30 samples per bin and leg; bins present in
    both sweeps; the average of the two; linear between bin-mean angles.
    Samples outside the sweeps' range are EXCLUDED (no extrapolation).
    [Oct 15: y was TAU_SPRING_TABLE - RIG_MGL*sin(vest), but the Oct 15 sweep
    differs from the table by up to +0.27 N m at act 88-98 deg (Sep 30: <0.05).]
  * rig stability (Oct 15): RMS over common bins of (after - before) must be
    <= STABLE_DY for y and <= STABLE_DF for f, else INCOMPLETE ("rig changed").
    No after-sweep -> INCOMPLETE.
  * theta_ddot: Savitzky-Golay first derivative of present_velocity, window 11
    samples, polyorder 2, over each release's free-phase rows. A sample is
    unusable if any time step in its window exceeds 3 ms.
  * window: free-phase rows from the release until the FIRST row with
    vel >= 3.0 rad/s or actuator angle above the table end (104.6 deg); keep
    0.2 < vel < 3.0 rad/s.
  * per release: J_r = least-squares slope of z vs theta_ddot through zero.
    Usable = completed, not stuck, no free-phase error, >= 8 window samples,
    |mean iq| in the window <= 0.1 A.
  * per height: J_h = median of usable J_r; needs >= 3 usable releases.
  * pass (STATE section 5, unchanged): every J_h within +-25 % of the mean of
    the three J_h; every J_h and that mean inside 0.002-0.03 kg m^2; residual
    RMS per height reported. Fewer than 3 heights or 3 usable releases per
    height -> INCOMPLETE, not PASS or FAIL.
  * reported J = mean of the three J_h.

CROSS-CHECKS (reported, never gating)
  * friction shifted by -0.05 / +0.05 Nm;
  * Method B: one J and one constant f fitted over all usable samples, then
    each height's J with that f;
  * y from TAU_SPRING_TABLE - RIG_MGL*sin(vest) (the Oct 14 primary), to show
    what the table would have given;
  * velocity register vs Savitzky-Golay derivative of position;
  * input_voltage range during free phases (back-EMF).

Units: J comes out in KT units (y and f are both KT*iq), the same units the
observer uses. A physical J inherits KT's error (STATE 4.6).
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main_controller.controller import (  # noqa: E402
    KT, PHI_DEG, RIG_MGL, TAU_SPRING_TABLE, tau_spring)

# ---- Pinned constants (PLAN Block 4, 2026-10-14) ------------------------------
BIN_DEG = 5.0                  # vest-angle bins for the sweep friction
MIN_BIN_N = 30                 # samples per bin and leg
STEADY_TOL = 0.10              # goal speed within 10 % of nominal = steady
SG_GOAL_WINDOW = 31            # smoothing for the sweep's goal speed
SG_WINDOW = 11                 # theta_ddot: SG derivative of present_velocity
SG_ORDER = 2
MAX_STEP_S = 0.003             # a window with a bigger time step is unusable
V_MIN, V_MAX = 0.2, 3.0        # rad/s
TABLE_END_VEST = TAU_SPRING_TABLE[-1][0]          # 176.6 vest = 104.6 act
MIN_SAMPLES = 8                # per release. Was 20 in the Oct 14 proposal; the
                               # offline test showed a true J of 0.006 (the code's
                               # value) gives only 9-19 window samples at 60 deg,
                               # so 20 would make the alternative hypothesis
                               # untestable. Changed before any data.
MAX_MEAN_IQ = 0.1              # A, |mean iq| in the window
MIN_RELEASES = 3               # usable releases per height
CONSIST = 0.25                 # +-25 % of the mean of the three heights
J_RANGE = (0.002, 0.03)        # kg m^2
FRICTION_SHIFT = 0.05          # Nm, sensitivity check
# Rig-stability gate (Oct 15): ~2x the Aug 20 same-session repeatability
# (235718 vs 234335, 23 min apart: RMS dy 0.026, RMS df 0.058 N m).
STABLE_DY = 0.06               # N m, RMS of y(after) - y(before)
STABLE_DF = 0.10               # N m, RMS of f(after) - f(before)


# ---- Reading -----------------------------------------------------------------
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


def y_table(vest):
    """tau_spring - rig gravity, Nm, or nan outside the table."""
    out = np.full(len(vest), np.nan)
    for k, v in enumerate(vest):
        s = tau_spring(float(v))
        if s is not None:
            out[k] = s - RIG_MGL * math.sin(math.radians(v))
    return out


# ---- Friction (and y) from the same-session sweep ------------------------------
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


def interp_held(x, xp, fp):
    """Linear inside [xp0, xpN], held flat outside."""
    return np.interp(x, xp, fp, left=fp[0], right=fp[-1])


# ---- Releases ------------------------------------------------------------------
FOOT = re.compile(r'#\s*release\s+(\d+):\s*target\s+([\d.]+)\s*deg(.*)')


def parse_release_file(path):
    comments, rows = read_csv(path)
    if not comments or 'j_release_test.py' not in comments[0]:
        raise SystemExit(f"{path}: not a j_release_test.py log")
    status = {}
    for line in comments:
        m = FOOT.match(line)
        if not m:
            continue
        idx, target, rest = int(m.group(1)), float(m.group(2)), m.group(3)
        if 'NOT COMPLETED' in rest:
            status[idx] = dict(target=target, completed=False, stuck=None,
                               drop=None, note='NOT COMPLETED')
            continue
        sm = re.search(r'stuck (YES|no)', rest)
        drop = None
        if 'error byte' in rest or 'failed read' in rest:
            drop = rest.split('stuck')[-1].strip(' ,')
        status[idx] = dict(target=target, completed=sm is not None,
                           stuck=(sm.group(1) == 'YES') if sm else None,
                           drop=drop, note='')
    free = {}
    for r in rows:
        if r.get('phase') != 'free':
            continue
        free.setdefault(int(r['release_idx']), []).append(r)
    out = []
    for idx, st in sorted(status.items()):
        rr = free.get(idx, [])
        out.append(dict(file=os.path.basename(path), idx=idx, **st,
                        t=np.array([fnum(r['t']) for r in rr]),
                        pos=np.array([fnum(r['pos']) for r in rr]),
                        vel=np.array([fnum(r['vel']) for r in rr]),
                        iq=np.array([fnum(r['iq']) for r in rr]),
                        vin=np.array([fnum(r['input_voltage']) for r in rr])))
    return comments, out


def window_samples(rel, fric_tab, friction_shift=0.0, y_source='sweep'):
    """Pinned window and z = y - f, theta_ddot. Returns dict of arrays (maybe empty)
    and data-quality numbers."""
    t, pos, vel = rel['t'], rel['pos'], rel['vel']
    good = np.isfinite(t) & np.isfinite(pos) & np.isfinite(vel)
    t, pos, vel, iq = t[good], pos[good], vel[good], rel['iq'][good]
    empty = dict(a=np.array([]), z=np.array([]), f=np.array([]), vest=np.array([]),
                 iq=np.array([]))
    if len(t) < SG_WINDOW:
        return empty, {}
    dt = float(np.median(np.diff(t)))
    acc = savgol_filter(vel, SG_WINDOW, SG_ORDER, deriv=1, delta=dt)
    # A time step bigger than MAX_STEP_S anywhere in a sample's window -> unusable.
    big = np.r_[False, np.diff(t) > MAX_STEP_S]
    half = SG_WINDOW // 2
    bad = np.zeros(len(t), bool)
    for k in np.where(big)[0]:
        bad[max(0, k - half - 1):k + half + 1] = True
    bad[:half] = True                       # SG edge samples
    bad[-half:] = True

    vest = vest_deg(pos)
    # First pass only: stop at the first row at/above 3 rad/s or past the table end.
    stop = np.where((vel >= V_MAX) | (vest > TABLE_END_VEST))[0]
    end = stop[0] if len(stop) else len(t)
    sel = np.zeros(len(t), bool)
    sel[:end] = True
    sel &= (vel > V_MIN) & (vel < V_MAX) & ~bad

    # Inside the sweeps' measured range only: no extrapolation of y or f.
    lo, hi = fric_tab[0, 0], fric_tab[-1, 0]
    cov = (vest >= lo) & (vest <= hi)
    if y_source == 'table':                 # cross-check: the Aug 20 table
        y = y_table(vest)
    else:                                   # pinned (Oct 15): the same-session sweeps
        y = np.where(cov, np.interp(vest, fric_tab[:, 0], fric_tab[:, 2]), np.nan)
    f = np.where(cov, np.interp(vest, fric_tab[:, 0], fric_tab[:, 1]), np.nan) + friction_shift
    sel &= np.isfinite(y) & np.isfinite(f)

    # Velocity register vs SG derivative of position, inside the window.
    vpos = savgol_filter(pos, SG_WINDOW, SG_ORDER, deriv=1, delta=dt)
    q = dict(vel_vs_pos=float(np.median(np.abs(vel[sel] - vpos[sel]))) if sel.any() else float('nan'),
             vin_range=(float(np.nanmin(rel['vin'])), float(np.nanmax(rel['vin'])))
             if np.isfinite(rel['vin']).any() else (float('nan'), float('nan')),
             dt_ms=dt * 1000)
    return dict(a=acc[sel], z=(y - f)[sel], f=f[sel], vest=vest[sel], iq=iq[sel]), q


def slope0(a, z):
    """Least-squares slope through zero."""
    den = float(np.sum(a * a))
    return float(np.sum(a * z)) / den if den > 0 else float('nan')


def analyse(releases, fric_tab, friction_shift=0.0, y_source='sweep'):
    """Per-release and per-height J. Returns (per_release, per_height)."""
    per = []
    for rel in releases:
        w, q = window_samples(rel, fric_tab, friction_shift, y_source)
        n = len(w['a'])
        J = slope0(w['a'], w['z']) if n else float('nan')
        res = w['z'] - J * w['a'] if n else np.array([])
        mean_iq = float(np.mean(w['iq'])) if n else float('nan')
        why = []
        if not rel['completed']:
            why.append('not completed')
        if rel['stuck']:
            why.append('stuck')
        if rel['drop']:
            why.append('free-phase error')
        if n < MIN_SAMPLES:
            why.append(f'{n} < {MIN_SAMPLES} samples')
        if n and abs(mean_iq) > MAX_MEAN_IQ:
            why.append(f'|mean iq| {abs(mean_iq):.3f} A > {MAX_MEAN_IQ}')
        per.append(dict(rel=rel, w=w, q=q, n=n, J=J, mean_iq=mean_iq,
                        rms=float(np.sqrt(np.mean(res ** 2))) if n else float('nan'),
                        usable=not why, why=', '.join(why)))
    heights = {}
    for p in per:
        heights.setdefault(p['rel']['target'], []).append(p)
    ph = {}
    for h, ps in sorted(heights.items()):
        use = [p for p in ps if p['usable']]
        if len(use) >= MIN_RELEASES:
            Jh = float(np.median([p['J'] for p in use]))
            a = np.concatenate([p['w']['a'] for p in use])
            z = np.concatenate([p['w']['z'] for p in use])
            rms = float(np.sqrt(np.mean((z - Jh * a) ** 2)))
        else:
            Jh, rms = float('nan'), float('nan')
        ph[h] = dict(J=Jh, rms=rms, n_use=len(use), n_all=len(ps),
                     Js=[p['J'] for p in use])
    return per, ph


def verdict(ph, stable=True, why_unstable=''):
    """Pinned pass rule. Returns (label, mean J, max deviation, detail)."""
    if not stable:
        return ('INCOMPLETE', float('nan'), float('nan'), why_unstable)
    if len(ph) != 3 or any(v['n_use'] < MIN_RELEASES for v in ph.values()):
        miss = [f"{h:g} deg: {v['n_use']} usable" for h, v in ph.items()
                if v['n_use'] < MIN_RELEASES]
        return ('INCOMPLETE', float('nan'), float('nan'),
                f"need 3 heights with >= {MIN_RELEASES} usable releases each"
                + (f" ({'; '.join(miss)})" if miss else f" (found {len(ph)} heights)"))
    Js = np.array([v['J'] for v in ph.values()])
    mean = float(Js.mean())
    dev = float(np.max(np.abs(Js / mean - 1.0)))
    in_range = bool(np.all((Js >= J_RANGE[0]) & (Js <= J_RANGE[1]))
                    and J_RANGE[0] <= mean <= J_RANGE[1])
    ok = dev <= CONSIST and in_range
    return ('PASS' if ok else 'FAIL', mean, dev,
            f"max deviation from the mean {dev * 100:.1f} % (limit {CONSIST * 100:.0f} %); "
            f"all inside {J_RANGE[0]}-{J_RANGE[1]}: {'yes' if in_range else 'NO'}")


def method_b(per):
    """One J and one constant f over all usable samples; then J per height with that f."""
    use = [p for p in per if p['usable']]
    if not use:
        return None
    # z = y - f_sweep; add the sweep friction back to get y, then fit y = J a + f.
    a = np.concatenate([p['w']['a'] for p in use])
    y = np.concatenate([p['w']['z'] + p['w']['f'] for p in use])
    A = np.c_[a, np.ones(len(a))]
    J, f = np.linalg.lstsq(A, y, rcond=None)[0]
    per_h = {}
    for p in use:
        per_h.setdefault(p['rel']['target'], []).append(p)
    Jh = {}
    for h, ps in sorted(per_h.items()):
        aa = np.concatenate([q['w']['a'] for q in ps])
        yy = np.concatenate([q['w']['z'] + q['w']['f'] for q in ps])
        Jh[h] = slope0(aa, yy - f)
    return float(J), float(f), Jh


def run(sweep_path, release_paths, plot_path=None, sweep_after=None):
    L = []
    say = L.append
    before = sweep_tables(sweep_path)
    say("=" * 78)
    say("  J RELEASE FIT -- Method C (pinned 2026-10-14; y from the sweeps, 2026-10-15)")
    say("=" * 78)
    say(f"sweep before: {sweep_path}")
    say(f"sweep after:  {sweep_after or 'NONE (rig stability unchecked -> INCOMPLETE)'}")
    for p in release_paths:
        say(f"releases:     {p}")
    say(f"KT {KT} Nm/A (datasheet); y and f are both KT*iq (KT units)")
    if sweep_after:
        after = sweep_tables(sweep_after)
        fric, st = merge_sweeps(before, after)
        stable = st['rms_dy'] <= STABLE_DY and st['rms_df'] <= STABLE_DF
        why_unstable = (f"rig changed during the session: RMS dy {st['rms_dy']:.3f} "
                        f"(limit {STABLE_DY}), RMS df {st['rms_df']:.3f} N m (limit {STABLE_DF})")
    else:
        fric, st, stable = before, None, False
        why_unstable = "no after-sweep: rig stability unchecked"
    say("")
    say("Same-session sweeps (y = -(up + down)/2, f = (up - down)/2 of KT*iq):")
    yt = y_table(fric[:, 0])
    if st:
        say(f"  {'act':>6}{'f_avg':>8}{'df':>8}{'y_avg':>8}{'dy':>8}{'y_table':>9}{'avg-tab':>9}")
        for (v, f, ys, nu, nd), ytab, dy, df in zip(fric, yt, st['dy'], st['df']):
            say(f"  {v - PHI_DEG:6.1f}{f:8.3f}{df:+8.3f}{ys:8.3f}{dy:+8.3f}{ytab:9.3f}"
                f"{ys - ytab:+9.3f}")
        say(f"  stability (after - before): RMS dy {st['rms_dy']:.3f} (max {st['max_dy']:.3f}), "
            f"RMS df {st['rms_df']:.3f} (max {st['max_df']:.3f}) N m over {st['bins']} bins "
            f"-> {'STABLE' if stable else 'NOT STABLE'} (limits {STABLE_DY} / {STABLE_DF})")
    else:
        say(f"  {'act':>6}{'friction':>10}{'y_sweep':>9}{'y_table':>9}{'diff':>8}")
        for (v, f, ys, nu, nd), ytab in zip(fric, yt):
            say(f"  {v - PHI_DEG:6.1f}{f:10.3f}{ys:9.3f}{ytab:9.3f}{ys - ytab:+8.3f}")
    say(f"  coverage act {fric[0, 0] - PHI_DEG:.1f}-{fric[-1, 0] - PHI_DEG:.1f} deg "
        f"(samples outside are excluded)")

    releases, headers = [], []
    for p in release_paths:
        c, rs = parse_release_file(p)
        headers.append(c)
        releases.extend(rs)

    per, ph = analyse(releases, fric)

    say("")
    say("Per release (Method C):")
    say(f"  {'file':<28}{'#':>3}{'target':>7}{'n':>5}{'acc range':>13}{'J':>9}"
        f"{'rms':>7}{'mean iq':>9}{'v_reg-v_pos':>12}  use")
    for p in per:
        a = p['w']['a']
        rng = f"{a.min():.1f}-{a.max():.1f}" if len(a) else '-'
        say(f"  {p['rel']['file']:<28}{p['rel']['idx']:>3}{p['rel']['target']:>7g}"
            f"{p['n']:>5}{rng:>13}{p['J']:9.4f}{p['rms']:7.3f}{p['mean_iq']:+9.3f}"
            f"{p['q'].get('vel_vs_pos', float('nan')):12.3f}  "
            f"{'yes' if p['usable'] else 'NO (' + p['why'] + ')'}")

    say("")
    say("Per height (median of usable releases):")
    for h, v in ph.items():
        js = ', '.join(f"{j:.4f}" for j in v['Js'])
        say(f"  {h:5g} deg: J_h {v['J']:.4f}  residual RMS {v['rms']:.3f} Nm  "
            f"({v['n_use']}/{v['n_all']} usable: {js})")
    lab, mean, dev, detail = verdict(ph, stable, why_unstable)
    say("")
    say(f"VERDICT (STATE 5): {lab}" + (f" -- J = {mean:.4f} kg m^2" if np.isfinite(mean) else ""))
    say(f"  {detail}")

    say("")
    say("Cross-checks (reported, NOT gating):")
    for s in (-FRICTION_SHIFT, +FRICTION_SHIFT):
        _, ph2 = analyse(releases, fric, friction_shift=s)
        l2, m2, d2, _ = verdict(ph2, stable, why_unstable)
        say(f"  friction {s:+.2f} Nm: mean J {m2:.4f}, max deviation "
            f"{d2 * 100:.1f} %, " + ', '.join(f"{h:g}: {v['J']:.4f}" for h, v in ph2.items())
            + f" -> {l2}")
    mb = method_b(per)
    if mb:
        J, f, Jh = mb
        say(f"  Method B (one J + one constant f): J {J:.4f}, f {f:.3f} Nm; per height with "
            f"that f: " + ', '.join(f"{h:g}: {j:.4f}" for h, j in Jh.items()))
    _, ph3 = analyse(releases, fric, y_source='table')
    say("  y from TAU_SPRING_TABLE (the Oct 14 primary): "
        + ', '.join(f"{h:g}: {v['J']:.4f} ({v['n_use']} usable)" for h, v in ph3.items()))
    vins = [p['q']['vin_range'] for p in per if p['q']]
    if vins:
        lo = min(v[0] for v in vins)
        hi = max(v[1] for v in vins)
        say(f"  input_voltage during free phases: {lo:.2f}-{hi:.2f} V (back-EMF check)")
    iqs = [abs(p['mean_iq']) for p in per if np.isfinite(p['mean_iq'])]
    if iqs:
        say(f"  |mean iq| in windows: max {max(iqs):.3f} A")

    if plot_path:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 4.5))
            for h in ph:
                ps = [p for p in per if p['rel']['target'] == h and p['usable']]
                if not ps:
                    continue
                a = np.concatenate([p['w']['a'] for p in ps])
                z = np.concatenate([p['w']['z'] for p in ps])
                ax.plot(a, z, '.', ms=3, label=f"{h:g} deg (J_h {ph[h]['J']:.4f})")
            if np.isfinite(mean):
                xs = np.array([0, ax.get_xlim()[1]])
                ax.plot(xs, mean * xs, 'k--', lw=1, label=f"mean J {mean:.4f}")
            ax.set_xlabel('theta_ddot (rad/s^2)')
            ax.set_ylabel('y - f (Nm)')
            ax.legend(frameon=False)
            fig.tight_layout()
            fig.savefig(plot_path, dpi=130)
            say(f"\nplot: {plot_path}")
        except ImportError:
            say("\nplot skipped: matplotlib not installed")
    return '\n'.join(L), lab, mean


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sweep', required=True, help='bare sweep run just BEFORE the releases')
    ap.add_argument('--sweep-after', help='the same sweep run just AFTER the releases '
                    '(required for a PASS/FAIL verdict)')
    ap.add_argument('releases', nargs='+', help='j_release_*.csv files')
    ap.add_argument('--out', help='also write the summary to this text file')
    ap.add_argument('--plot', help='write a y-f vs theta_ddot plot (PNG)')
    a = ap.parse_args(argv)
    text, lab, _ = run(a.sweep, a.releases, a.plot, a.sweep_after)
    print(text)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())