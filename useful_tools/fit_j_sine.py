"""
Fit the device inertia J from the J sine test (PLAN Block 4b, pinned
2026-10-15 before any data). Offline analysis only: no hardware imports,
commands nothing.

USAGE (from the repo root)
-----
    python3 useful_tools/fit_j_sine.py RUN.csv [RUN.csv ...] \\
        [--sweep SWEEP.csv] [--out logs/summaries/phase2/j_sine/<name>.txt] \\
        [--plot <name>.png]

  RUN.csv    observer_jsine_bare_<f>Hz_*.csv from
             observer_excitation.py --mode sine --j-sine --arm-mass-kg 0
             --arm-com-m 0 --center-deg 65 --amp-deg 5 --freq-hz F --cycles N
             --velocity 0.10 --soft-start-s 0.3
             for F, N = 0.5, 6 / 1.0, 8 / 1.5, 10 / 2.0, 12 / 2.5, 14
  SWEEP.csv  optional bare sweep from the same session: cross-check of the
             intercept against the spring slope (never gating)

IDEA
----
Torque balance on the arm (+ = up), motor ON in Position Mode:
    J*theta_ddot = KT*iq + y(theta) - f(theta_dot)
    => KT*iq = J*theta_ddot - y(theta) + f(theta_dot)
With theta = c + A*sin(w*t): theta_ddot = -w^2 * (theta - c), exactly IN STEP
with position. y(theta) ~ y0 + y'*(theta - c) is also in step, but does not
change with w. Friction follows the sign of the velocity, a quarter cycle out
of step, so its fundamental is in QUADRATURE. Hence, per run, the fundamental
of KT*iq divided by the fundamental of theta (complex ratio H) has
    Re(H) = K_in(w) = -y' - J*w^2          (in-phase dynamic stiffness)
    Im(H) = K_q(w)  ~ 4*f/(pi*A) + b*w     (friction and damping)
and across runs K_in is a straight line in w^2 with slope -J.

PINNED METHOD (Oct 15, before data)
-------------
  * Rows: phase 'sine', full-amplitude cycles only: time since the first sine
    row in [1/f, (cycles - 1)/f) (an integer number of cycles).
  * Each signal on its own clock: position at t_pos, current at t_iq (each
    register's reply time), relative to the first sine row's t.
  * Least squares per signal on [1, t, sin kwt, cos kwt for k = 1..5];
    fundamental phasor X = c1 - i*s1. H = T1 / Theta1, T = KT*iq.
  * J = -slope of an ordinary least-squares line K_in vs w^2 over every run.
  * Pair check: J_f = (K_in(0.5 Hz) - K_in(f)) / (w_f^2 - w_0.5^2) for
    f = 1.5, 2.0, 2.5 Hz (per-frequency mean K_in if a frequency was run
    twice).
  * PASS: J inside 0.006-0.03 kg m^2 AND each pair J_f within +-25 % of J.
    (Lower bound 0.006 = the rig's own point-mass m*d^2, 0.0062; raised from
    the release test's 0.002 before this test's data, because the release
    showed 0.002 cannot catch a physically impossible value.)
  * INCOMPLETE: any of 0.5/1.0/1.5/2.0/2.5 Hz has no complete run; a run
    ended early (END not 'complete') or has fewer than cycles - 2 full
    cycles; measured amplitude |Theta1| < 2 deg in any run; t_pos/t_iq
    columns missing.
  * Otherwise FAIL (including J <= 0: a sign or model problem).
Cross-checks, reported, never gating: J with both signals on the cycle-start
clock t (no skew correction); J without the 1.0 Hz run; J from the
fundamental only (no harmonics in the fit); intercept vs the sweep's spring
slope; implied Coulomb friction 4f/(pi*A) from K_q; goal -> position
amplitude ratio and lag; current range; share of samples where the motor
torque changes sign (gear backlash would be crossed there).
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main_controller.controller import KT, PHI_DEG  # noqa: E402

PINNED_FREQS = (0.5, 1.0, 1.5, 2.0, 2.5)
PAIR_REF = 0.5
PAIR_FREQS = (1.5, 2.0, 2.5)
J_RANGE = (0.006, 0.03)
CONSIST = 0.25
MIN_AMP_DEG = 2.0
N_HARM = 5


# ---- Reading -----------------------------------------------------------------
def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return float('nan')


def read_run(path):
    with open(path, newline='') as f:
        lines = f.read().splitlines()
    comments = [l for l in lines if l.startswith('#')]
    body = [l for l in lines if l and not l.startswith('#')]
    rows = list(csv.DictReader(body))
    hdr = ' '.join(comments)
    out = dict(path=path, name=os.path.basename(path), problems=[])
    if 'observer_excitation.py' not in hdr or 'mode=sine' not in hdr:
        raise SystemExit(f"{path}: not an observer_excitation.py sine log")
    if 'j_sine=True' not in hdr:
        raise SystemExit(f"{path}: not a --j-sine run")
    m = re.search(r'arm_mass_kg=([-\d.eE]+)', hdr)
    if not m or float(m.group(1)) != 0.0:
        raise SystemExit(f"{path}: not a bare run (arm_mass_kg must be 0)")
    for key in ('freq_hz', 'cycles', 'amp_deg', 'center_deg'):
        m = re.search(rf'\b{key}=([-\d.eE]+)', hdr)
        out[key] = float(m.group(1)) if m else float('nan')
    ends = [c for c in comments if c.startswith('# END')]
    out['end'] = ends[-1][2:] if ends else 'END missing'
    if not ends or 'complete' not in ends[-1] or 'abort' in ends[-1]:
        out['problems'].append(f"run did not complete ({out['end']})")
    sine = [r for r in rows if r.get('phase') == 'sine']
    if not sine:
        out['problems'].append('no sine rows')
        return out
    if 't_pos' not in sine[0] or 't_iq' not in sine[0]:
        out['problems'].append('t_pos/t_iq columns missing')
    col = lambda k: np.array([fnum(r.get(k)) for r in sine])  # noqa: E731
    out.update(t=col('t'), goal=col('goal_pos'), pos=col('pos'), vel=col('vel'),
               iq=col('present_iq'), t_pos=col('t_pos'), t_iq=col('t_iq'))
    return out


# ---- Harmonic fit ------------------------------------------------------------
def harmonic(t, x, w, n_harm=N_HARM):
    """Least squares on [1, t, sin kwt, cos kwt]; returns (phasor of k=1, rms resid)."""
    cols = [np.ones_like(t), t - t.mean()]
    for k in range(1, n_harm + 1):
        cols += [np.sin(k * w * t), np.cos(k * w * t)]
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    s1, c1 = coef[2], coef[3]
    resid = x - A @ coef
    return complex(c1, -s1), float(np.sqrt(np.mean(resid ** 2)))


def analyse_run(run, clock='own', n_harm=N_HARM):
    """Per-run dynamic stiffness. clock='own' uses t_pos / t_iq; 'cycle' uses t."""
    f = run['freq_hz']
    w = 2 * math.pi * f
    cyc = run['cycles']
    t0 = run['t'][0]
    ts = run['t'] - t0
    keep = (ts >= 1.0 / f) & (ts < (cyc - 1) / f)
    if clock == 'own':
        tp, ti = run['t_pos'] - t0, run['t_iq'] - t0
    else:
        tp = ti = ts
    ok = keep & np.isfinite(tp) & np.isfinite(ti) & np.isfinite(run['pos']) & np.isfinite(run['iq'])
    span = (ts[ok].max() - ts[ok].min()) * f if ok.sum() > 2 else 0.0
    theta1, rp = harmonic(tp[ok], run['pos'][ok], w, n_harm)
    tau = KT * run['iq'][ok]
    tau1, rt = harmonic(ti[ok], tau, w, n_harm)
    goal1, _ = harmonic(ts[ok], run['goal'][ok], w, n_harm)
    H = tau1 / theta1
    return dict(f=f, w=w, n=int(ok.sum()), cycles_in=span,
                amp_deg=math.degrees(abs(theta1)),
                goal_amp_deg=math.degrees(abs(goal1)),
                lag_deg=math.degrees(np.angle(theta1 / goal1)) if abs(goal1) > 0 else float('nan'),
                K_in=H.real, K_q=H.imag, tau=tau, pos_rms=rp, tau_rms=rt,
                tau_max=float(tau.max()), tau_min=float(tau.min()),
                # share of samples on the minority sign of the motor torque:
                # the gear backlash is crossed wherever the torque changes sign
                sign_change=float(min(np.mean(tau > 0), np.mean(tau < 0))),
                iq_max=float(np.max(np.abs(run['iq'][ok]))))


def line_fit(w2, K):
    """K = k0 - J*w^2 by OLS. Returns (J, k0, R^2, residuals)."""
    A = np.column_stack([np.ones_like(w2), -w2])
    (k0, J), *_ = np.linalg.lstsq(A, K, rcond=None)
    res = K - (k0 - J * w2)
    ss = np.sum((K - K.mean()) ** 2)
    r2 = 1.0 - np.sum(res ** 2) / ss if ss > 0 else float('nan')
    return float(J), float(k0), float(r2), res


def pair_js(per):
    """J from each pair (0.5 Hz, f) using per-frequency mean K_in."""
    byf = {}
    for p in per:
        byf.setdefault(round(p['f'], 3), []).append(p['K_in'])
    if PAIR_REF not in byf:
        return {}
    k_ref = float(np.mean(byf[PAIR_REF]))
    w_ref2 = (2 * math.pi * PAIR_REF) ** 2
    out = {}
    for f in PAIR_FREQS:
        if f in byf:
            w2 = (2 * math.pi * f) ** 2
            out[f] = (k_ref - float(np.mean(byf[f]))) / (w2 - w_ref2)
    return out


def verdict(runs, per, J):
    probs = []
    have = {round(r['freq_hz'], 3) for r in runs if not r['problems']}
    miss = [f for f in PINNED_FREQS if f not in have]
    if miss:
        probs.append(f"no complete run at {', '.join(f'{f:g}' for f in miss)} Hz")
    for r in runs:
        for pr in r['problems']:
            probs.append(f"{r['name']}: {pr}")
    for p, r in zip(per, [r for r in runs if not r['problems']]):
        if p['cycles_in'] < r['cycles'] - 2 - 0.05:
            probs.append(f"{r['name']}: only {p['cycles_in']:.2f} full cycles")
        if p['amp_deg'] < MIN_AMP_DEG:
            probs.append(f"{r['name']}: amplitude {p['amp_deg']:.2f} deg < {MIN_AMP_DEG}")
    if probs:
        return 'INCOMPLETE', probs, {}
    pj = pair_js(per)
    in_range = J_RANGE[0] <= J <= J_RANGE[1]
    devs = {f: (v / J - 1.0) if J > 0 else float('nan') for f, v in pj.items()}
    consistent = J > 0 and all(abs(d) <= CONSIST for d in devs.values()) and len(devs) == len(PAIR_FREQS)
    lab = 'PASS' if (in_range and consistent) else 'FAIL'
    why = [f"J {J:.5f} inside {J_RANGE[0]}-{J_RANGE[1]}: {'yes' if in_range else 'NO'}"]
    why += [f"pair 0.5-{f:g} Hz: J {pj[f]:.5f} ({devs[f] * 100:+.1f} % vs fit; limit "
            f"+-{CONSIST * 100:.0f} %)" for f in sorted(pj)]
    return lab, why, pj


# ---- Optional sweep cross-check ------------------------------------------------
def sweep_slope(sweep_path, center_deg):
    """-dy/dtheta at the centre from a bare sweep (N m/rad), via fit_j_release."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import fit_j_release as F
    fr = F.sweep_tables(sweep_path)
    act = fr[:, 0] - PHI_DEG
    y = fr[:, 2]
    k = np.argsort(np.abs(act - center_deg))[:3]
    k = np.sort(k)
    slope = np.polyfit(np.radians(act[k]), y[k], 1)[0]
    return -float(slope), float(np.interp(center_deg, act, fr[:, 1]))


# ---- Main ------------------------------------------------------------------------
def run(paths, sweep=None, plot=None):
    L = []
    say = L.append
    say("=" * 78)
    say("  J SINE FIT -- in-phase stiffness vs w^2 (pinned 2026-10-15)")
    say("=" * 78)
    runs = [read_run(p) for p in paths]
    runs.sort(key=lambda r: (r['freq_hz'], r['name']))
    for r in runs:
        say(f"run: {r['name']}  f {r['freq_hz']:g} Hz  cycles {r['cycles']:g}  "
            f"centre {r['center_deg']:g}  amp {r['amp_deg']:g}  {r['end']}"
            + (f"  PROBLEMS: {'; '.join(r['problems'])}" if r['problems'] else ''))
    say(f"KT {KT} Nm/A (datasheet); J in KT units")
    good = [r for r in runs if not r['problems']]
    per = [analyse_run(r) for r in good]
    say("")
    say(f"  {'f Hz':>5}{'w^2':>8}{'n':>6}{'cyc':>6}{'amp deg':>9}{'goal deg':>9}"
        f"{'lag deg':>8}{'K_in':>9}{'K_q':>8}{'f_coul':>8}{'tau range N m':>18}{'sgn chg':>8}")
    for p in per:
        fc = p['K_q'] * math.pi * math.radians(p['amp_deg']) / 4.0
        say(f"  {p['f']:5.2f}{p['w'] ** 2:8.1f}{p['n']:6d}{p['cycles_in']:6.2f}"
            f"{p['amp_deg']:9.3f}{p['goal_amp_deg']:9.3f}{p['lag_deg']:8.1f}"
            f"{p['K_in']:9.3f}{p['K_q']:8.2f}{fc:8.3f}"
            f"{p['tau_min']:9.3f}..{p['tau_max']:6.3f}{p['sign_change'] * 100:7.1f}%")
    if len(per) < 2:
        say("\nVERDICT: INCOMPLETE (fewer than 2 usable runs)")
        return '\n'.join(L), 'INCOMPLETE', float('nan')
    w2 = np.array([p['w'] ** 2 for p in per])
    K = np.array([p['K_in'] for p in per])
    J, k0, r2, res = line_fit(w2, K)
    say(f"\nLine K_in = k0 - J*w^2 over {len(per)} runs: J {J:.5f} kg m^2, "
        f"k0 {k0:.3f} N m/rad, R^2 {r2:.4f}, residual RMS {np.sqrt(np.mean(res ** 2)):.3f} N m/rad")
    lab, why, pj = verdict(runs, per, J)
    say(f"\nVERDICT (pinned Block 4b): {lab}")
    for w_ in why:
        say(f"  {w_}")

    say("\nCross-checks (reported, NOT gating):")
    per_c = [analyse_run(r, clock='cycle') for r in good]
    Jc, *_ = line_fit(np.array([p['w'] ** 2 for p in per_c]), np.array([p['K_in'] for p in per_c]))
    say(f"  cycle-start clock for both signals (no skew correction): J {Jc:.5f}")
    sel = [i for i, p in enumerate(per) if abs(p['f'] - 1.0) > 1e-6]
    if len(sel) >= 2:
        Jn, *_ = line_fit(w2[sel], K[sel])
        say(f"  without the 1.0 Hz run: J {Jn:.5f}")
    per_1 = [analyse_run(r, n_harm=1) for r in good]
    J1, *_ = line_fit(np.array([p['w'] ** 2 for p in per_1]), np.array([p['K_in'] for p in per_1]))
    say(f"  fundamental only (no harmonics in the fit): J {J1:.5f}")
    if sweep:
        try:
            ks, fs = sweep_slope(sweep, float(np.nanmedian([r['center_deg'] for r in good])))
            say(f"  intercept k0 {k0:.3f} vs sweep -dy/dtheta at the centre {ks:.3f} N m/rad; "
                f"sweep friction there {fs:.3f} N m")
        except SystemExit as e:
            say(f"  sweep cross-check skipped: {e}")
    say(f"  max |iq| in windows: {max(p['iq_max'] for p in per):.3f} A")

    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 4.5))
            ax.plot(w2, K, 'o', label='runs')
            xs = np.linspace(0, w2.max() * 1.05, 50)
            ax.plot(xs, k0 - J * xs, 'k--', lw=1, label=f'K_in = {k0:.2f} - {J:.4f} w^2')
            ax.set_xlabel('w^2 (rad^2/s^2)')
            ax.set_ylabel('K_in, in-phase stiffness (N m/rad)')
            ax.legend(frameon=False)
            fig.tight_layout()
            os.makedirs(os.path.dirname(os.path.abspath(plot)), exist_ok=True)
            fig.savefig(plot, dpi=130)
            say(f"\nplot: {plot}")
        except ImportError:
            say("\nplot skipped: matplotlib not installed")
    return '\n'.join(L), lab, J


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+', help='observer_jsine_*.csv files')
    ap.add_argument('--sweep', help='optional same-session bare sweep (cross-check only)')
    ap.add_argument('--out', help='also write the summary to this text file')
    ap.add_argument('--plot', help='write a K_in vs w^2 plot (PNG)')
    a = ap.parse_args(argv)
    text, lab, _ = run(a.runs, a.sweep, a.plot)
    print(text)
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())