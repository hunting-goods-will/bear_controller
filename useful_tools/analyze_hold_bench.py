"""
Analyse hold_bench_test.py logs (H1 bare, H2 payload, H3 thermal) against the
same session's sweep. No hardware; read-only.

    python3 useful_tools/analyze_hold_bench.py <hold csv> [<hold csv> ...] \\
        --sweep <H0 sweep before> [--sweep-after <sweep after>] \\
        [--out summary.txt] [--plot holds.png]
    python3 useful_tools/analyze_hold_bench.py --selftest

FIRST: IS EACH LOG TRUSTWORTHY?
  run completed (END line); every error byte 0x80; loop pace (median dt,
  share > 2.5 ms, max); current followed the command (median |iq - iq_sent|);
  at each angle the arm REALLY crept (measured amplitude >= 0.7 deg; less
  means stick-slip and the creep reading is not trusted).

THEN (PLAN v4 3.4; guides, not pass lines -- exploratory):
  Q1 / H1  per angle: creep average u (cycles 2..N, whole cycles; u = KT*iq)
           vs the sweep's -y at the creep's mean angle. Looks good: within
           ~0.1 N m. Still reading (last 5 s) and where it sits inside the
           +-f friction band. Hunting = angle peak-to-peak while still: looks
           good < 0.5 deg, rethink > 1 deg. Several bare runs: spread of the
           creep averages (rethink > 0.3 N m).
  Q1 / H2  payload run minus the mean bare run, per angle: delta-u vs
           m*g*r*sin(vest + hang offset) from the payload run's header.
           Looks good: within ~15 % (and +-0.1 N m repeatable); rethink: off
           by > 30 % with good geometry.
  Q2 / H3  thermal run: fits a first-order model driven by the MEASURED
           current, dT/dt = (Ta + k*iq^2 - T)/tau (the current in a still hold
           wanders inside the friction band, so it is not constant), then
           predicts the temperature after 600 s at 5, 6, 7 A from cold.
           Looks good: 5-7 A stays under 65 C. One-pole model, one sensor:
           a rough guide. Short runs: tau is poorly fixed (reported).
  Drift    with --sweep-after: y and f change over the session (Oct 15 gate:
           RMS dy <= 0.06, df <= 0.10 N m).

Writes only --out and --plot (if given).
"""
import argparse
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sweep_tables as ST                                   # noqa: E402
from main_controller.controller import KT, PHI_DEG          # noqa: E402

G = 9.81
MIN_CREEP_AMP_DEG = 0.7
STILL_LAST_S = 5.0
PREDICT_A = (5.0, 6.0, 7.0)
PREDICT_S = 600.0
GOOD_CREEP_NM, GOOD_HUNT_DEG, BAD_HUNT_DEG = 0.1, 0.5, 1.0
GOOD_PAY, BAD_PAY = 0.15, 0.30
BAD_SPREAD = 0.3
TEMP_GOOD = 65.0


# ---------------------------------------------------------------- reading
def load_run(path):
    comments, rows = ST.read_csv(path)
    if not any('hold_bench_test.py' in c for c in comments):
        raise SystemExit(f"{path}: not a hold_bench_test.py log")
    hdr = ' '.join(comments)

    def meta(key, default=None, cast=float):
        m = re.search(rf'\b{key}=([^\s]+)', hdr)
        return cast(m.group(1)) if m else default
    run = dict(path=path, comments=comments,
               mode=meta('mode', 'holds', str),
               payload_kg=meta('payload_kg', 0.0), payload_r_m=meta('payload_r_m', 0.0),
               payload_angle_deg=meta('payload_angle_deg', 0.0),
               creep_hz=meta('creep_hz', 0.25), sweep=meta('sweep', '', str),
               end=next((c[2:] for c in comments if c.startswith('# END')), 'no END line'))
    cols = ['t', 'loop_dt', 'angle_idx', 'target_deg', 'sp', 'pos', 'vel', 'iq', 'iq_sent',
            'u_des', 'u_cmd', 'integ', 'w_temp', 'p_temp']
    for c in cols:
        run[c] = np.array([ST.fnum(r[c]) for r in rows])
    run['phase'] = np.array([r['phase'] for r in rows])
    run['err'] = np.array([int(r['err'], 16) for r in rows])
    return run


# ---------------------------------------------------------------- per angle
def per_angle(run):
    out = []
    for k in sorted(set(int(x) for x in run['angle_idx'] if x >= 0)):
        a = run['angle_idx'] == k
        d = dict(k=k, deg=float(run['target_deg'][a][0]))
        st = a & (run['phase'] == 'still')
        if st.any():
            last = st & (run['t'] >= run['t'][st].max() - STILL_LAST_S)
            d.update(still_u=KT * run['iq'][last].mean(),
                     still_pp=math.degrees(np.ptp(run['pos'][st])),
                     still_angle=math.degrees(run['pos'][last].mean()))
        cr = a & (run['phase'] == 'creep')
        if cr.any():
            per = 1.0 / run['creep_hz']
            t0, t1 = run['t'][cr].min(), run['t'][cr].max()
            n_whole = int(math.floor((t1 - t0) * run['creep_hz'] + 0.05))
            use = cr & (run['t'] >= t0 + per) & (run['t'] < t0 + n_whole * per)
            if n_whole >= 2 and use.any():
                d.update(creep_u=KT * run['iq'][use].mean(), creep_cycles=n_whole - 1,
                         creep_amp=math.degrees(np.ptp(run['pos'][use])) / 2,
                         creep_angle=math.degrees(run['pos'][use].mean()))
        th = a & (run['phase'] == 'thermal')
        if th.any():
            d.update(thermal=thermal_fit(run['t'][th] - run['t'][th][0], run['w_temp'][th],
                                         run['iq'][th]),
                     thermal_p=thermal_fit(run['t'][th] - run['t'][th][0], run['p_temp'][th],
                                           run['iq'][th]),
                     thermal_u=KT * run['iq'][th].mean())
        out.append(d)
    return out


def _sim_temp(t, i2, T_start, Ta, k, tau):
    """First-order thermal model driven by the MEASURED current:
    dT/dt = (Ta + k*iq^2 - T) / tau, stepped exactly between samples."""
    T = np.empty(len(t))
    T[0] = T_start
    for n in range(len(t) - 1):
        a = 1.0 - math.exp(-(t[n + 1] - t[n]) / tau)
        T[n + 1] = T[n] + a * (Ta + k * i2[n] - T[n])
    return T


def thermal_fit(t, T, iq):
    """Fit Ta (ambient), k (C per A^2 at steady state) and tau to the logged
    temperature, using the measured iq^2 at every sample. Why not a plain
    exponential at the mean current: in a still hold friction lets the
    integral walk the torque around the friction band, so the current is NOT
    constant (1.4-3.3 A in one fake run) and a constant-current fit returned
    tau ~9000 s and k ~13x too big."""
    from scipy.optimize import least_squares
    ok = ~np.isnan(T)
    t, T, i2 = t[ok], T[ok], iq[ok] ** 2
    res = dict(T0=float(T[0]), T_end=float(T[-1]), dur=float(t[-1] - t[0]),
               i_rms=float(math.sqrt(np.mean(i2))), rise=float(T[-1] - T[0]))
    if len(t) < 20 or t[-1] - t[0] < 30:
        res['error'] = 'too short to fit'
        return res
    f = lambda p: _sim_temp(t, i2, T[0], p[0], p[1], p[2]) - T
    try:
        sol = least_squares(f, x0=[T[0] - 1.0, 2.0, 200.0],
                            bounds=([T[0] - 15, 0.0, 10.0], [T[0] + 5, 100.0, 20000.0]))
        Ta, k, tau = sol.x
        rms = float(np.sqrt(np.mean(sol.fun ** 2)))
        res.update(Ta=float(Ta), k=float(k), tau=float(tau), fit_rms=rms,
                   dT=float(k * np.mean(i2)), tau_well_fixed=bool(t[-1] - t[0] >= 0.5 * tau))
    except Exception as e:                                 # noqa: BLE001
        res['error'] = str(e)
    return res


def predict_temp(th, amps, secs=PREDICT_S):
    """Temperature after `secs` of a constant-current hold from cold (ambient Ta)."""
    if 'k' not in th:
        return {}
    return {A: th['Ta'] + th['k'] * A * A * (1 - math.exp(-secs / th['tau'])) for A in amps}


# ---------------------------------------------------------------- checks
def data_check(run):
    L = []
    ok = 'completed' in run['end']
    L.append(f"  end: {run['end']}")
    bad = int(np.sum(run['err'] != 0x80))
    L.append(f"  error bytes != 0x80: {bad}")
    ok &= bad == 0
    dt = run['loop_dt'][run['phase'] != 'thermal']
    if len(dt):
        L.append(f"  loop dt: median {np.median(dt) * 1000:.2f} ms, > 2.5 ms {100 * np.mean(dt > 0.0025):.2f} %, "
                 f"max {dt.max() * 1000:.2f} ms")
        ok &= np.median(dt) < 0.0025
    trk = np.abs(run['iq'] - run['iq_sent'])
    L.append(f"  current tracking |iq - iq_sent|: median {np.median(trk):.3f} A, p99 "
             f"{np.percentile(trk, 99):.3f} A")
    ok &= np.median(trk) < 0.1
    return L, ok


# ---------------------------------------------------------------- report
def payload_torque(run, deg):
    vest = deg + PHI_DEG + run['payload_angle_deg']
    return run['payload_kg'] * G * run['payload_r_m'] * math.sin(math.radians(vest))


def analyse(paths, sweep, sweep_after=None):
    L = []
    table = ST.sweep_tables(sweep)
    if sweep_after:
        table_after = ST.sweep_tables(sweep_after)
        merged, s = ST.merge_sweeps(table, table_after)
        stable = s['rms_dy'] <= ST.STABLE_DY and s['rms_df'] <= ST.STABLE_DF
        L += [f"Sweeps: {sweep}  +  {sweep_after}",
              f"Drift over the session ({s['bins']} bins): y RMS {s['rms_dy']:.3f} (max {s['max_dy']:.3f}), "
              f"f RMS {s['rms_df']:.3f} (max {s['max_df']:.3f}) N m -> {'STABLE' if stable else 'MOVED'}"
              f" (gate {ST.STABLE_DY} / {ST.STABLE_DF})",
              "Reference = the average of the two sweeps."]
        table = merged
    else:
        L += [f"Sweep: {sweep}  (no --sweep-after: drift during the session unknown)"]
    runs = [load_run(p) for p in paths]
    results = []
    for run in runs:
        L += ["", f"=== {os.path.basename(run['path'])}  mode={run['mode']}  payload "
              f"{run['payload_kg']} kg @ {run['payload_r_m']} m, offset {run['payload_angle_deg']} deg"]
        if run['sweep'] and os.path.basename(run['sweep']) != os.path.basename(sweep):
            L.append(f"  NOTE: this run's header names a different sweep ({os.path.basename(run['sweep'])})")
        chk, ok = data_check(run)
        L += ["  DATA CHECK " + ("OK" if ok else "-- PROBLEM, read before trusting the numbers")] + chk
        res = per_angle(run)
        results.append((run, res))
        if run['mode'] != 'thermal':
            L.append(f"  {'angle':>6}{'creep u':>9}{'-y sweep':>9}{'diff':>8}{'amp':>6}{'cyc':>4}"
                     f"{'still u':>9}{'in band':>9}{'hunt p-p':>9}")
        for d in res:
            if 'creep_u' in d:
                vest = d['creep_angle'] + PHI_DEG
                y = ST.interp_inside(table, vest, 2)
                f = ST.interp_inside(table, vest, 1)
                d['ref'] = -y + payload_torque(run, d['creep_angle']) if not math.isnan(y) else float('nan')
                d['f'] = f
                trust = d['creep_amp'] >= MIN_CREEP_AMP_DEG
                d['trust'] = trust
                band = ''
                if 'still_u' in d and not math.isnan(f):
                    pos_in = (d['still_u'] - d['ref']) / f if f else float('nan')
                    band = f"{pos_in:+.2f} f"
                L.append(f"  {d['deg']:6.0f}{d['creep_u']:+9.3f}{d['ref']:+9.3f}"
                         f"{d['creep_u'] - d['ref']:+8.3f}{d['creep_amp']:6.2f}{d['creep_cycles']:4d}"
                         f"{d.get('still_u', float('nan')):+9.3f}{band:>9}{d.get('still_pp', float('nan')):9.3f}"
                         + ("" if trust else "   <- creep < 0.7 deg: stick-slip, reading NOT trusted"))
            if 'thermal' in d:
                th, tp = d['thermal'], d['thermal_p']
                L.append(f"  thermal hold at {d['deg']:.0f} deg: {th['dur']:.0f} s at {th['i_rms']:.2f} A rms "
                         f"(u {d['thermal_u']:+.2f} N m); winding {th['T0']:.1f} -> {th['T_end']:.1f} C, "
                         f"powerstage {tp['T0']:.1f} -> {tp['T_end']:.1f} C")
                for name, x in (('winding', th), ('powerstage', tp)):
                    if 'k' in x:
                        pr = predict_temp(x, PREDICT_A)
                        L.append(f"    {name}: fit ambient {x['Ta']:.1f} C, k {x['k']:.2f} C/A^2, tau {x['tau']:.0f} s, "
                                 f"fit RMS {x['fit_rms']:.2f} C"
                                 f"{'' if x['tau_well_fixed'] else ' (run shorter than tau/2: tau poorly fixed)'}"
                                 f" -> after {PREDICT_S:.0f} s from cold: "
                                 + ', '.join(f"{A:.0f} A {T:.0f} C" for A, T in pr.items())
                                 + f"  [{'all under' if max(pr.values()) < TEMP_GOOD else 'NOT all under'} "
                                   f"{TEMP_GOOD:.0f} C]")
                    else:
                        L.append(f"    {name}: fit failed ({x.get('error', '?')})")

    # ---- H1 summary across bare runs
    bare = [(r, res) for r, res in results if r['mode'] != 'thermal' and r['payload_kg'] == 0]
    pay = [(r, res) for r, res in results if r['mode'] != 'thermal' and r['payload_kg'] > 0]
    L += ["", "=== SUMMARY (exploratory; guides from PLAN 3.4, not pass lines)"]
    bare_by_deg = {}
    for r, res in bare:
        for d in res:
            if d.get('trust'):
                bare_by_deg.setdefault(round(d['deg']), []).append(d)
    for deg, ds in sorted(bare_by_deg.items(), reverse=True):
        diffs = [d['creep_u'] - d['ref'] for d in ds]
        hunt = max(d.get('still_pp', 0) for d in ds)
        verdict = ('looks good' if max(abs(x) for x in diffs) <= GOOD_CREEP_NM and hunt < GOOD_HUNT_DEG
                   else 'rethink' if hunt > BAD_HUNT_DEG else 'in between')
        spread = (max(d['creep_u'] for d in ds) - min(d['creep_u'] for d in ds)) if len(ds) > 1 else None
        L.append(f"  H1 {deg:3d} deg: creep - sweep {', '.join(f'{x:+.3f}' for x in diffs)} N m; "
                 f"hunting max {hunt:.2f} deg"
                 + (f"; spread over {len(ds)} runs {spread:.3f} N m"
                    f"{' (rethink: > 0.3)' if spread > BAD_SPREAD else ''}" if spread is not None else '')
                 + f"  -> {verdict}")
    for r, res in pay:
        for d in res:
            b = bare_by_deg.get(round(d['deg']))
            if not d.get('trust') or not b:
                continue
            du = d['creep_u'] - np.mean([x['creep_u'] for x in b])
            exp = payload_torque(r, d['creep_angle'])
            ratio = du / exp if exp else float('nan')
            off = abs(ratio - 1)
            verdict = 'looks good' if off <= GOOD_PAY else 'rethink' if off > BAD_PAY else 'in between'
            L.append(f"  H2 {d['deg']:3.0f} deg: delta-u {du:+.3f} vs m*g*r*sin {exp:+.3f} N m "
                     f"(ratio {ratio:.2f}) -> {verdict}   [{os.path.basename(r['path'])}]")
    if not bare:
        L.append("  (no bare hold run given: H1 and the H2 baseline need one)")
    return L, results, table


def plot(path_png, results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(results), 1, figsize=(11, 3.0 * len(results)), squeeze=False)
    for ax, (run, res) in zip(axes[:, 0], results):
        t = run['t']
        ax.plot(t, KT * run['iq'], lw=0.6, color='k', label='u = KT*iq (N m)')
        ax2 = ax.twinx()
        ax2.plot(t, np.degrees(run['pos']), lw=0.6, color='tab:blue')
        ax2.plot(t, np.degrees(run['sp']), lw=0.6, color='tab:blue', ls=':')
        ax2.set_ylabel('deg', color='tab:blue')
        for ph, col in (('still', 'tab:green'), ('creep', 'tab:orange'), ('thermal', 'tab:red')):
            m = run['phase'] == ph
            for k in set(run['angle_idx'][m]):
                mm = m & (run['angle_idx'] == k)
                ax.axvspan(t[mm].min(), t[mm].max(), color=col, alpha=0.15)
        for d in res:
            if 'ref' in d and not math.isnan(d['ref']):
                m = (run['angle_idx'] == d['k']) & (run['phase'] == 'creep')
                ax.hlines(d['ref'], t[m].min(), t[m].max(), color='tab:purple', lw=1.5)
        ax.set_ylabel('N m')
        ax.set_title(f"{os.path.basename(run['path'])}  (green still, orange creep, purple = sweep -y "
                     f"(+ payload), dotted = target)", fontsize=8, loc='left')
    axes[-1, 0].set_xlabel('s')
    fig.tight_layout()
    fig.savefig(path_png, dpi=110)


# ---------------------------------------------------------------- self-test
def selftest():
    """Runs the REAL hold_bench_test.py on the fake (test_hold_bench_offline),
    then checks this analyzer recovers the fake's known answers."""
    import tempfile
    import test_hold_bench_offline as T
    tmp = tempfile.mkdtemp(prefix='ahb_selftest_')
    sw = os.path.join(tmp, 'sweep.csv')
    T.make_sweep(sw)
    files = {}
    pay = (0.711, 0.1651, 10.0)
    for name, kw, argv in (
            ('bare', {}, []),
            ('payload', dict(payload=pay), ['--payload-kg', '0.711', '--payload-r-m', '0.1651',
                                            '--payload-angle-deg', '10']),
            ('thermal', {}, ['--thermal', '--thermal-s', '300'])):
        clock = T.Clock()
        fake = T.FakeBear(clock, **kw)
        res = T.invoke(fake, ['--sweep', sw] + argv, clock)
        p = os.path.join(tmp, f'hold_{name}.csv')
        with open(p, 'w') as f:
            f.write(res['csv']['path_text'] + '\n')
        files[name] = p
    L, results, _ = analyse([files['bare'], files['payload'], files['thermal']], sw)
    print('\n'.join(L))
    checks = []

    def check(name, ok, detail=''):
        checks.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")
    (rb, resb), (rp, resp), (rt, rest) = results
    for d in resb:
        truth = -T.y_true(d['creep_angle'])
        check(f"bare {d['deg']:.0f} deg: creep vs fake truth within 0.05", abs(d['creep_u'] - truth) < 0.05,
              f"{d['creep_u']:+.3f} vs {truth:+.3f}")
        check(f"bare {d['deg']:.0f} deg: analyzer's sweep reference vs fake truth within 0.03",
              abs(d['ref'] - truth) < 0.03, f"{d['ref']:+.3f}")
    for d, b in zip(resp, resb):
        du = d['creep_u'] - b['creep_u']
        tp = T.pay_true(d['creep_angle'], *pay)
        check(f"payload {d['deg']:.0f} deg: delta-u vs fake truth within 0.05", abs(du - tp) < 0.05,
              f"{du:+.3f} vs {tp:+.3f}")
    th = rest[0]['thermal']
    check("thermal: k within 25 % of the fake's 3 C/A^2", 'k' in th and abs(th['k'] - 3.0) < 0.75,
          f"k {th.get('k', float('nan')):.2f}")
    check("thermal: tau within 35 % of the fake's 120 s", 'tau' in th and abs(th['tau'] - 120) < 42,
          f"tau {th.get('tau', float('nan')):.0f} s")
    check("summary has H1 and H2 lines", any(l.strip().startswith('H1') for l in L)
          and any(l.strip().startswith('H2') for l in L))
    # a stuck creep must be flagged as not trusted
    rb2 = dict(rb)
    rb2['pos'] = rb['pos'].copy()
    m = rb2['phase'] == 'creep'
    rb2['pos'][m] = rb2['pos'][m][0]
    flagged = all(not d.get('trust', True) for d in per_angle_with_ref(rb2))
    check("a creep that didn't move is flagged 'not trusted'", flagged)
    import shutil
    shutil.rmtree(tmp)
    n = sum(checks)
    print(f"\n{n}/{len(checks)} self-checks passed")
    return n == len(checks)


def per_angle_with_ref(run):
    res = per_angle(run)
    for d in res:
        if 'creep_amp' in d:
            d['trust'] = d['creep_amp'] >= MIN_CREEP_AMP_DEG
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="Analyse hold_bench_test.py logs.")
    ap.add_argument('logs', nargs='*')
    ap.add_argument('--sweep')
    ap.add_argument('--sweep-after')
    ap.add_argument('--out')
    ap.add_argument('--plot')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)
    if a.selftest:
        sys.exit(0 if selftest() else 1)
    if not a.logs or not a.sweep:
        ap.error('give hold logs and --sweep (or --selftest)')
    L, results, _ = analyse(a.logs, a.sweep, a.sweep_after)
    text = '\n'.join(L)
    print(text)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    if a.plot:
        plot(a.plot, results)
        print(f"\nplot: {a.plot}")


if __name__ == '__main__':
    main()