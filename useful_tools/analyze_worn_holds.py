"""
Holds and lowerings in a WORN, MOTOR-OFF log. No hardware; read-only.

    python3 useful_tools/analyze_worn_holds.py <log.csv> [--out summary.txt] [--plot holds.png]
    python3 useful_tools/analyze_worn_holds.py --selftest

Reads velocity_profile.py logs (Aug 18) and worn_logger.py logs (H4).

THREE QUESTIONS (PLAN v4 3.4 Q4, the motor-off half)
----------------------------------------------------
1. DO HOLDS HAPPEN? The arm is "still" when its position moved < 0.1 deg over
   the last 0.3 s -- the same rule the hold controller will use, so this tests
   the actual detector. A hold = an unbroken run of "still"; it is dated from
   the start of its first 0.3 s window. Reported per hold: start, length,
   angle, drift, whether an up-stroke (>= 10 deg rise in the 5 s before) led
   into it, and how it ended. Counted as an overhead hold only above
   --min-angle (actuator deg, default 20). Holds/min and % time held per block.
2. WHAT DOES A REAL LOWERING LOOK LIKE AT THE START? A lowering = a downward
   movement (smoothed velocity < -0.05 rad/s) that drops > 5 deg. From its
   onset: time to pass -0.15 rad/s; drop after 50 / 100 / 200 ms; and the
   PLAN 3.2 gap-5 release rule ("down faster than 0.15 rad/s for 50 ms, OR
   > 3 deg below where it started") -- how long it takes to fire and how far
   the arm has dropped by then. Caveat: motor off, nothing resists, so with a
   hold controller pushing back the arm starts slower and the rule fires LATER
   than here. These are best-case numbers.
3. SMALL ADJUSTMENTS: movements between two holds that stay within 3 deg.
   Their size and peak speed, and whether the release rule would fire on them
   (a wrong release = the "detent" feel, gap 12).

WHAT THIS CAN'T TELL YOU
------------------------
Relaxing vs lowering. Motor off, a relaxing arm simply drops (the spring
alone can't hold it), so "sag" and "lowering" are the same motion here. That
needs worn data with the hold controller running (Dr. Kim's sign-off).

Writes only --out and --plot (if given).
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

STILL_SPAN_DEG = 0.1       # PLAN 3.1 hold detector
STILL_WIN_S = 0.3
MIN_HOLD_S = 1.0           # shortest hold reported
LONG_HOLD_S = 3.0          # PLAN 3.4 "looks good": holds >= 3 s ...
UPSTROKE_DEG = 10.0        # rise in the lead-in window that counts as an up-stroke
UPSTROKE_LOOKBACK_S = 5.0
MOVE_V = 0.05              # rad/s, smoothed: a movement has started
SMOOTH_S = 0.015           # moving average for movement detection only
LOWER_MIN_DROP_DEG = 5.0   # a "lowering" drops more than this
RULE_V = 0.15              # rad/s down ...
RULE_T = 0.050             # ... for this long -> release
RULE_SAG_DEG = 3.0         # ... or this far below the start -> release
ADJUST_MAX_DEG = 3.0
END_MOVE_DEG = 0.5         # how a hold ends: first 0.5 deg away from it

WORK_LABELS = ('overhead_task', 'work', 'mock_work')   # holds/min headline


# ---------------------------------------------------------------- reading
def load(path):
    """Return list of blocks: dict(label, rep, t, pos (rad), vel, iq, err)."""
    with open(path, newline='') as f:
        lines = [l for l in f.read().splitlines() if l and not l.startswith('#')]
    rows = list(csv.DictReader(lines))
    if not rows:
        raise SystemExit(f"{path}: no rows")
    cols = rows[0].keys()
    if 'trial_label' in cols:                       # velocity_profile.py
        lab, tcol = 'trial_label', 'elapsed_s'
    elif 'block' in cols:                           # worn_logger.py
        lab, tcol = 'block', 't_block'
    else:
        raise SystemExit(f"{path}: unknown log format (columns {list(cols)})")
    blocks, order = {}, []
    for r in rows:
        key = (r[lab], r['rep'])
        if key not in blocks:
            blocks[key] = []
            order.append(key)
        blocks[key].append(r)
    out = []
    for key in order:
        rs = blocks[key]
        f = lambda c: np.array([float(r[c]) for r in rs])
        out.append(dict(label=key[0], rep=key[1], t=f(tcol), pos=f('position_rad'),
                        vel=f('velocity_rad_s'), iq=f('iq_A'),
                        err=np.array([int(float(r['error'])) for r in rs])))
    return out


# ---------------------------------------------------------------- detection
def trailing_span(t, x, win):
    """max - min of x over the trailing window (t - win, t]; nan until a full
    window of history exists. Two-pointer with monotonic deques, O(n)."""
    from collections import deque
    n = len(t)
    out = np.full(n, np.nan)
    qmax, qmin = deque(), deque()
    lo = 0
    for i in range(n):
        while qmax and x[qmax[-1]] <= x[i]:
            qmax.pop()
        qmax.append(i)
        while qmin and x[qmin[-1]] >= x[i]:
            qmin.pop()
        qmin.append(i)
        while t[i] - t[lo] > win:
            lo += 1
        while qmax[0] < lo:
            qmax.popleft()
        while qmin[0] < lo:
            qmin.popleft()
        if t[i] - t[0] >= win:
            out[i] = x[qmax[0]] - x[qmin[0]]
    return out


def runs(mask):
    """(start, end) index pairs of True runs, end inclusive."""
    m = np.concatenate([[False], mask, [False]]).astype(int)
    d = np.diff(m)
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


def smooth(t, v, win):
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.003
    k = max(1, int(round(win / dt)))
    return np.convolve(v, np.ones(k) / k, mode='same')


def release_rule(t, pos, vel, i0):
    """Index where the release rule fires for a movement starting at i0, or
    None. Rule: vel < -RULE_V continuously for RULE_T, or pos below
    pos[i0] by more than RULE_SAG_DEG."""
    sag = math.radians(RULE_SAG_DEG)
    below_since = None
    for i in range(i0, len(t)):
        if vel[i] < -RULE_V:
            if below_since is None:
                below_since = t[i]
            if t[i] - below_since >= RULE_T:
                return i
        else:
            below_since = None
        if pos[i0] - pos[i] > sag:
            return i
    return None


def analyse_block(b, min_angle_deg):
    t, p, v = b['t'], b['pos'], b['vel']
    span = trailing_span(t, p, STILL_WIN_S)
    still = span < math.radians(STILL_SPAN_DEG)        # nan -> False
    holds = []
    for s, e in runs(still):
        t0 = t[s] - STILL_WIN_S
        dur = t[e] - t0
        if dur < MIN_HOLD_S:
            continue
        i0 = int(np.searchsorted(t, t0))
        seg = p[i0:e + 1]
        ang = math.degrees(seg.mean())
        lb = (t >= t0 - UPSTROKE_LOOKBACK_S) & (t < t0)
        rise = math.degrees(p[i0] - p[lb].min()) if lb.any() else 0.0
        # how it ended
        ended = 'block end'
        k = np.where(np.abs(p[e:] - p[e]) > math.radians(END_MOVE_DEG))[0]
        if len(k):
            j = e + k[0]
            ended = 'up' if p[j] > p[e] else 'down'
        holds.append(dict(t0=t0, t1=t[e], dur=dur, angle=ang,
                          span=math.degrees(seg.max() - seg.min()),
                          drift=math.degrees(seg[-1] - seg[0]),
                          upstroke=rise >= UPSTROKE_DEG, ended=ended,
                          overhead=ang >= min_angle_deg, i0=i0, i1=e))

    vs = smooth(t, v, SMOOTH_S)
    lowerings = []
    for s, e in runs(vs < -MOVE_V):
        drop = math.degrees(p[s] - p[s:e + 1].min())
        if drop <= LOWER_MIN_DROP_DEG:
            continue
        def drop_at(dt):
            j = int(np.searchsorted(t, t[s] + dt))
            return math.degrees(p[s] - p[min(j, len(p) - 1)])
        k = np.where(v[s:] < -RULE_V)[0]
        t_cross = (t[s + k[0]] - t[s]) if len(k) else float('nan')
        r = release_rule(t, p, v, s)
        lowerings.append(dict(t0=t[s], start_angle=math.degrees(p[s]), drop=drop,
                              t_cross=t_cross, d50=drop_at(0.05), d100=drop_at(0.10),
                              d200=drop_at(0.20),
                              t_rule=(t[r] - t[s]) if r is not None else float('nan'),
                              d_rule=math.degrees(p[s] - p[r]) if r is not None else float('nan'),
                              peak_v=float(-v[s:e + 1].min())))

    adjustments = []
    for a, h in zip(holds, holds[1:]):
        lo, hi = a['i1'] + 1, h['i0']
        if hi <= lo or not (a['overhead'] and h['overhead']):
            continue
        seg = p[lo:hi]
        rng = math.degrees(seg.max() - seg.min()) if len(seg) else 0.0
        if rng >= ADJUST_MAX_DEG:
            continue
        r = release_rule(t[lo:hi], p[lo:hi], v[lo:hi], 0) if hi - lo > 1 else None
        adjustments.append(dict(t0=t[lo], range=rng,
                                net=math.degrees(p[hi - 1] - p[lo]),
                                peak_v=float(np.abs(v[lo:hi]).max()),
                                rule_fires=r is not None))
    dur = t[-1] - t[0]
    held = sum(h['dur'] for h in holds if h['overhead'])
    return dict(holds=holds, lowerings=lowerings, adjustments=adjustments,
                dur=dur, held=held, still=still)


# ---------------------------------------------------------------- data checks
def data_checks(blocks):
    """Is this log trustworthy? Returns (lines, ok)."""
    lines, ok = [], True
    allt = [np.diff(b['t']) for b in blocks if len(b['t']) > 1]
    dt = np.concatenate(allt)
    errs = np.concatenate([b['err'] for b in blocks])
    iq = np.concatenate([b['iq'] for b in blocks])
    bad_err = int(np.sum(errs != 128))
    lines.append(f"rows {len(errs)}, blocks {len(blocks)}; sample dt median "
                 f"{np.median(dt) * 1e3:.2f} ms, max {dt.max() * 1e3:.1f} ms, "
                 f"gaps > 10 ms: {int(np.sum(dt > 0.010))}")
    lines.append(f"error byte != 128: {bad_err} rows;  |iq| max {np.abs(iq).max():.3f} A "
                 f"(motor-off check: must stay < 0.15)")
    disagree = []
    for b in blocks:
        t, p, v = b['t'], b['pos'], b['vel']
        if len(t) < 12:
            continue
        k = 5
        dp = (p[2 * k:] - p[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
        disagree.append(np.abs(v[k:-k] - dp))
    d = np.concatenate(disagree)
    lines.append(f"velocity register vs position slope: median {np.median(d):.3f}, "
                 f"p99 {np.percentile(d, 99):.3f} rad/s")
    if bad_err or np.abs(iq).max() >= 0.15 or np.median(dt) > 0.005 or np.sum(dt > 0.010) > 5 \
            or np.percentile(d, 99) > 0.3:
        ok = False
    lines.append("DATA CHECK: " + ("OK" if ok else "PROBLEM -- read the lines above before "
                                    "trusting anything below"))
    return lines, ok


# ---------------------------------------------------------------- report
def report(path, blocks, res, min_angle):
    L = [f"File: {path}", f"Detector: still = position span < {STILL_SPAN_DEG} deg over "
         f"{STILL_WIN_S} s; holds >= {MIN_HOLD_S} s; overhead = mean angle >= {min_angle} deg "
         f"(actuator)", ""]
    chk, ok = data_checks(blocks)
    L += chk + ["", "1. HOLDS PER BLOCK (overhead holds only)",
                f"{'block':<20}{'rep':>4}{'len s':>7}{'holds':>6}{'>=3 s':>6}"
                f"{'per min':>8}{'% held':>7}{'longest':>8}"]
    work_n = work_long = 0
    work_t = 0.0
    for b, r in zip(blocks, res):
        oh = [h for h in r['holds'] if h['overhead']]
        nl = sum(h['dur'] >= LONG_HOLD_S for h in oh)
        if b['label'] in WORK_LABELS:
            work_n += len(oh); work_long += nl; work_t += r['dur']
        L.append(f"{b['label']:<20}{b['rep']:>4}{r['dur']:7.1f}{len(oh):6d}{nl:6d}"
                 f"{60 * len(oh) / r['dur']:8.1f}{100 * r['held'] / r['dur']:7.0f}"
                 f"{max([h['dur'] for h in oh], default=0):8.1f}")
    if work_t:
        L.append(f"\nMock work ({', '.join(l for l in WORK_LABELS if any(b['label'] == l for b in blocks))}): "
                 f"{work_t:.0f} s total, {work_n} overhead holds ({60 * work_n / work_t:.1f}/min), "
                 f"{work_long} of them >= {LONG_HOLD_S:.0f} s ({60 * work_long / work_t:.1f}/min)")
        if work_t < 180:
            L.append(f"  CAUTION: only {work_t:.0f} s of mock work -- a rate from this is a "
                     f"rough first look, not a measurement.")
    L += ["", "   every overhead hold:",
          f"   {'block':<18}{'rep':>4}{'start s':>8}{'len s':>7}{'angle':>7}{'span':>7}"
          f"{'drift':>7}{'after up':>9}  ended"]
    for b, r in zip(blocks, res):
        for h in r['holds']:
            if h['overhead']:
                L.append(f"   {b['label']:<18}{b['rep']:>4}{h['t0']:8.2f}{h['dur']:7.2f}"
                         f"{h['angle']:7.1f}{h['span']:7.3f}{h['drift']:+7.3f}"
                         f"{'yes' if h['upstroke'] else 'no':>9}  {h['ended']}")

    lows = [(b, w) for b, r in zip(blocks, res) for w in r['lowerings']]
    L += ["", f"2. LOWERINGS (drop > {LOWER_MIN_DROP_DEG:.0f} deg): {len(lows)}. Release rule: "
          f"down > {RULE_V} rad/s for {RULE_T * 1000:.0f} ms, or > {RULE_SAG_DEG:.0f} deg sag. "
          f"Motor off = best case.",
          f"   {'block':<18}{'rep':>4}{'start':>7}{'drop':>7}{'peak v':>8}{'t>0.15':>8}"
          f"{'d@50ms':>8}{'d@100':>7}{'d@200':>7}{'rule ms':>8}{'rule deg':>9}"]
    for b, w in lows:
        L.append(f"   {b['label']:<18}{b['rep']:>4}{w['start_angle']:7.1f}{w['drop']:7.1f}"
                 f"{w['peak_v']:8.2f}{w['t_cross'] * 1000:8.0f}{w['d50']:8.2f}{w['d100']:7.2f}"
                 f"{w['d200']:7.2f}{w['t_rule'] * 1000:8.0f}{w['d_rule']:9.2f}")
    if lows:
        tr = np.array([w['t_rule'] for _, w in lows])
        dr = np.array([w['d_rule'] for _, w in lows])
        L.append(f"   rule fired on {np.sum(~np.isnan(tr))}/{len(lows)}; after median "
                 f"{np.nanmedian(tr) * 1000:.0f} ms (max {np.nanmax(tr) * 1000:.0f}), "
                 f"arm had dropped median {np.nanmedian(dr):.2f} deg (max {np.nanmax(dr):.2f})")

    adj = [(b, a) for b, r in zip(blocks, res) for a in r['adjustments']]
    L += ["", f"3. SMALL ADJUSTMENTS between overhead holds (< {ADJUST_MAX_DEG:.0f} deg): {len(adj)}"]
    for b, a in adj:
        L.append(f"   {b['label']:<18}{b['rep']:>4} t {a['t0']:6.2f}  range {a['range']:.2f} deg  "
                 f"net {a['net']:+.2f}  peak |v| {a['peak_v']:.2f} rad/s  "
                 f"rule fires: {'YES (wrong release)' if a['rule_fires'] else 'no'}")
    L += ["", "NOT ANSWERED HERE: relaxing vs lowering (motor off, a relaxed arm just drops)."]
    return L, ok


def plot(path_png, blocks, res, min_angle):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    keep = [(b, r) for b, r in zip(blocks, res)
            if math.degrees(b['pos'].max()) >= min_angle]
    if not keep:
        return
    fig, axes = plt.subplots(len(keep), 1, figsize=(11, 1.9 * len(keep)), squeeze=False)
    for ax, (b, r) in zip(axes[:, 0], keep):
        ax.plot(b['t'], np.degrees(b['pos']), lw=0.8, color='k')
        for h in r['holds']:
            if h['overhead']:
                ax.axvspan(h['t0'], h['t1'], color='tab:green', alpha=0.25)
        for w in r['lowerings']:
            ax.axvline(w['t0'], color='tab:red', lw=0.8)
            if not math.isnan(w['t_rule']):
                ax.axvline(w['t0'] + w['t_rule'], color='tab:red', lw=0.8, ls='--')
        ax.axhline(min_angle, color='0.6', lw=0.6, ls=':')
        ax.set_ylabel('act deg')
        ax.set_title(f"{b['label']} rep {b['rep']}  (green = hold, red = lowering onset, "
                     f"dashed = release rule fires)", fontsize=8, loc='left')
    axes[-1, 0].set_xlabel('s')
    fig.tight_layout()
    fig.savefig(path_png, dpi=110)


# ---------------------------------------------------------------- self-test
def selftest():
    """Synthetic trace with KNOWN holds, a lowering and a small adjustment."""
    fs = 333.0
    t = np.arange(0, 20, 1 / fs)
    deg = np.zeros_like(t)
    def ramp(a, b, t0, t1):
        m = (t >= t0) & (t < t1)
        x = (t[m] - t0) / (t1 - t0)
        deg[m] = a + (b - a) * (10 * x**3 - 15 * x**4 + 6 * x**5)
        deg[t >= t1] = b
    ramp(0, 80, 1.0, 2.0)              # up-stroke
    # hold 2.0-6.0 at 80
    ramp(80, 78.5, 6.0, 6.6)           # small downward adjustment, 1.5 deg
    # hold 6.6-10.0 at 78.5
    ramp(78.5, 0, 10.0, 11.0)          # lowering
    rng = np.random.default_rng(1)
    pos = np.radians(deg + rng.normal(0, 0.003, len(t)))
    vel = np.gradient(np.radians(deg), t) + rng.normal(0, 0.01, len(t))
    b = dict(label='work', rep='1', t=t, pos=pos, vel=vel, iq=np.zeros_like(t),
             err=np.full(len(t), 128))
    r = analyse_block(b, 20.0)
    oh = [h for h in r['holds'] if h['overhead']]
    checks = []
    def check(name, ok, detail=''):
        checks.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")
    check("two overhead holds found", len(oh) == 2, f"({len(oh)})")
    if len(oh) == 2:
        check("hold 1 ~2.0-6.0 s", abs(oh[0]['t0'] - 2.0) < 0.15 and abs(oh[0]['t1'] - 6.0) < 0.15,
              f"({oh[0]['t0']:.2f}-{oh[0]['t1']:.2f})")
        check("hold 1 led by an up-stroke", oh[0]['upstroke'])
        check("hold 2 ~6.6-10.0 s", abs(oh[1]['t0'] - 6.6) < 0.15 and abs(oh[1]['t1'] - 10.0) < 0.15,
              f"({oh[1]['t0']:.2f}-{oh[1]['t1']:.2f})")
    check("one lowering found", len(r['lowerings']) == 1, f"({len(r['lowerings'])})")
    if r['lowerings']:
        w = r['lowerings'][0]
        check("lowering starts near 10.0 s", abs(w['t0'] - 10.0) < 0.1, f"({w['t0']:.3f})")
        check("release rule fires on it", not math.isnan(w['t_rule']), f"({w['t_rule'] * 1000:.0f} ms)")
    check("one small adjustment, range ~1.5 deg", len(r['adjustments']) == 1 and
          abs(r['adjustments'][0]['range'] - 1.5) < 0.2,
          f"({[round(a['range'], 2) for a in r['adjustments']]})")
    chk, ok = data_checks([b])
    check("data check passes on clean synthetic data", ok)
    bad = dict(b, iq=np.full(len(t), 0.5))
    check("data check FAILS when the motor was on (|iq| 0.5 A)", not data_checks([bad])[1])
    n_ok = sum(checks)
    print(f"\n{n_ok}/{len(checks)} self-checks passed")
    return n_ok == len(checks)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Holds and lowerings in a worn, motor-off log.")
    ap.add_argument('csv', nargs='?')
    ap.add_argument('--min-angle', type=float, default=20.0, help='actuator deg (default 20)')
    ap.add_argument('--out')
    ap.add_argument('--plot')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)
    if a.selftest:
        sys.exit(0 if selftest() else 1)
    if not a.csv:
        ap.error('csv required (or --selftest)')
    blocks = load(a.csv)
    res = [analyse_block(b, a.min_angle) for b in blocks]
    lines, ok = report(a.csv, blocks, res, a.min_angle)
    text = '\n'.join(lines)
    print(text)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    if a.plot:
        plot(a.plot, blocks, res, a.min_angle)
        print(f"\nplot: {a.plot}")


if __name__ == '__main__':
    main()