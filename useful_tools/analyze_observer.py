"""
Offline disturbance-observer analysis (Phase 2 observer sprint, 2026-09-30).

Claude-written under a time exception (2026-09-30 observer sprint). Offline
analysis only: no hardware imports, commands nothing.

THE OBSERVER
------------
Torque balance on the joint (actuator frame, + = up):

    J*theta_ddot = KT*i  -  tau_f(theta_dot)  +  tau_ext

so the external-torque estimate is

    tau_ext_hat = J*theta_ddot + tau_f(theta_dot) - KT*i

tau_ext is everything that is not motor, friction or device inertia:
spring (+) minus rig gravity (-), plus any payload or cuff torque.
Sign check against validate_model.py: there residual_measured = KT*iq and
residual_predicted = tau_gravity - tau_spring. Quasi-static and friction-free,
KT*iq = -tau_ext, so tau_ext_true(bare) = -(RIG_MGL*sin(vest) - tau_spring).
Same convention as the sprint doc.

Friction tau_f is signed by direction of motion. Two versions are reported:
  model : sign(v) * (0.910 - 0.208*|v|), |v| clipped to the measured range
          0.05-0.50 rad/s. PROVENANCE: fitted in August from five runs, three of
          which were never committed (handoff 5.3). 234335/235718 are two of
          the five, so on those runs the model is NOT independent.
  sweep : per-angle friction from a bare sweep's own legs, (up - down)/2 of
          KT*iq. On that same sweep this reduces the observer to the
          leg-averaged check, so it tests KT/spring/gravity, not friction.

SUBCOMMANDS
-----------
  bench   P1 (bare, quasi-static), P2 (payload detection + KT), P3 (sine: J)
  human   observer on a Mode 0 human_live run -> estimated cuff torque

Pass thresholds (written in OBSERVER_SPRINT_2026-09-30.md BEFORE any run):
  P1  bare quasi-static, bin-level RMS <= 0.15 Nm over the spring-table range
  P2  delta tau_hat (wrench - bare) within +-10% of m*g*r*sin(vest) in every
      bin, no systematic angle trend
  P3  reported, not gating

Usage:
  python3 useful_tools/analyze_observer.py bench --bare BARE.csv \
      [--wrench WRENCH.csv --mass-kg 1.5025 --r-m 0.2739] \
      [--sine SINE.csv ...] --out logs/summaries/phase2/<name>
  python3 useful_tools/analyze_observer.py human --csv HUMAN.csv \
      --out logs/summaries/phase2/<name>
"""
import argparse
import math
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main_controller.controller import (  # noqa: E402
    KT, PHI_DEG, RIG_MGL, TAU_SPRING_TABLE, SPRING_MIN_DEG, SPRING_MAX_DEG)

G = 9.81
BIN_DEG = 5.0
FRIC_A, FRIC_B = 0.910, 0.208          # August Stribeck fit (provenance above)
FRIC_V_MIN, FRIC_V_MAX = 0.05, 0.50    # measured range; clip, never extrapolate
J_DEVICE_DEFAULT = 0.006               # kg m^2, bare rig. ESTIMATE from the
                                       # validate_model.py comment, unverified.

# Plot colours: validated categorical slots 1/2 (dataviz reference palette),
# grey for reference lines.
C_DOWN, C_UP, C_REF, C_INK = '#2a78d6', '#eb6834', '#8a8984', '#52514e'


# --------------------------------------------------------------------------
# Physics helpers
# --------------------------------------------------------------------------
_TBL = np.array(TAU_SPRING_TABLE)


def tau_spring(vest_deg):
    """Table interpolation; NaN outside the characterized range (never extrapolate)."""
    v = np.asarray(vest_deg, dtype=float)
    out = np.interp(v, _TBL[:, 0], _TBL[:, 1])
    out[(v < SPRING_MIN_DEG) | (v > SPRING_MAX_DEG)] = np.nan
    return out


def tau_ext_true_bare(vest_deg):
    """-(rig gravity - spring). NaN outside the spring table."""
    v = np.asarray(vest_deg, dtype=float)
    return -(RIG_MGL * np.sin(np.radians(v)) - tau_spring(v))


def payload_torque(vest_deg, m, r):
    """Gravity torque magnitude of a point payload m at radius r."""
    return m * G * r * np.sin(np.radians(np.asarray(vest_deg, dtype=float)))


def friction_model(v, direction=None):
    """Signed Stribeck friction. direction overrides sign(v) when given
    (sweeps: the commanded direction is known and cleaner than noisy v)."""
    v = np.asarray(v, dtype=float)
    mag = FRIC_A - FRIC_B * np.clip(np.abs(v), FRIC_V_MIN, FRIC_V_MAX)
    s = np.sign(v) if direction is None else np.asarray(direction, dtype=float)
    return s * mag


# --------------------------------------------------------------------------
# Loading: legacy validate_model CSVs and new observer_excitation CSVs
# --------------------------------------------------------------------------
def _find(df, *names):
    for n in names:
        if n in df.columns:
            return n
    return None


def load_bench(path):
    """Returns a normalized frame:
       t, dt, theta (actuator rad), vest, vel (register, rad/s), iq (A),
       direction (+1 up / -1 down / 0), steady (bool), goal (rad or NaN)."""
    df = pd.read_csv(path, comment='#')
    if 'phase' in df.columns:
        # observer_excitation.py logs approach, dwell and park rows too. Only
        # the excitation itself (sweep legs / sine) is analysis data: park runs
        # at 0.15 rad/s and would otherwise hijack the "nominal speed" below.
        ph = df['phase'].astype(str).str.lower()
        df = df[~ph.str.contains('park|approach|dwell')].reset_index(drop=True)
    out = pd.DataFrame()
    t = df[_find(df, 't', 'time')].to_numpy(float)
    out['t'] = t - t[0]
    out['dt'] = np.r_[np.nan, np.diff(t)]

    if 'act_deg' in df.columns:                      # legacy validate_model
        out['theta'] = np.radians(df['act_deg'].to_numpy(float))
        goal = np.radians(df['goal_deg'].to_numpy(float))
    else:                                            # new excitation script
        pos = df[_find(df, 'pos', 'pos_rad', 'present_position')].to_numpy(float)
        g = df[_find(df, 'goal_pos', 'goal', 'goal_position')].to_numpy(float)
        # Units: radians expected. Actuator travel is 0-2.06 rad; anything
        # beyond ~2*pi can only be degrees.
        if np.nanmax(np.abs(pos)) > 2 * math.pi:
            pos, g = np.radians(pos), np.radians(g)
        out['theta'] = pos
        goal = g
    out['goal'] = goal
    out['vest'] = np.degrees(out['theta']) + PHI_DEG
    out['vel'] = df[_find(df, 'vel', 'vel_rad_s', 'present_velocity')].to_numpy(float)
    out['iq'] = df[_find(df, 'iq_measured', 'present_iq', 'iq')].to_numpy(float)

    if 'leg' in df.columns:
        out['direction'] = np.where(df['leg'] == 'up', 1.0, -1.0)
        out['steady'] = (df['ramp'] >= 0.999).to_numpy() if 'ramp' in df else True
    else:
        # Commanded goal velocity, lightly smoothed. Steady = within 10 % of
        # the run's nominal commanded speed and not reversing.
        gv = np.gradient(savgol_filter(goal, 31, 2), t)
        # Prefer the commanded sweep speed the script logged; fall back to the
        # 90th percentile of goal speed only if it isn't there.
        if 'velocity' in df.columns and df['velocity'].notna().any() \
                and float(df['velocity'].iloc[0]) > 0:
            nominal = float(df['velocity'].iloc[0])
        else:
            nominal = np.nanpercentile(np.abs(gv), 90)
        out['direction'] = np.sign(gv)
        if 'phase' in df.columns:
            ph = df['phase'].astype(str).str.lower()
            known = ph.isin(['up', 'down'])
            out.loc[known.to_numpy(), 'direction'] = np.where(ph[known] == 'up', 1.0, -1.0)
        out['steady'] = np.abs(np.abs(gv) - nominal) <= 0.10 * nominal
    out['mode'] = df['mode'].iloc[0] if 'mode' in df.columns else 'sweep'
    return out


# --------------------------------------------------------------------------
# Differentiation: zero-phase (offline) and causal (what a loop could do)
# --------------------------------------------------------------------------
def accel_zero_phase(t, theta, window_s=0.15):
    """Savitzky-Golay second derivative on a uniform resample. Offline only:
    uses future samples."""
    tu = np.arange(t[0], t[-1], np.median(np.diff(t)))
    th = np.interp(tu, t, theta)
    dt = tu[1] - tu[0]
    w = max(7, int(round(window_s / dt)) | 1)
    acc = savgol_filter(th, w, 3, deriv=2, delta=dt)
    return np.interp(t, tu, acc)


def momentum_observer(t, vel, iq, tau_f, J, K):
    """Generalized-momentum observer (causal; no acceleration needed).

        p = J*w,   r_k = K * ( p_k - p_0 - sum( KT*i - tau_f + r ) * dt )

    r converges to tau_ext through a first-order lag of bandwidth K (rad/s),
    time constant 1/K. [unverified as the canonical form -- standard in the
    literature; check before citing]."""
    r = np.zeros_like(vel)
    integ = 0.0
    p0 = J * vel[0]
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        integ += (KT * iq[k - 1] - tau_f[k - 1] + r[k - 1]) * dt
        r[k] = K * (J * vel[k] - p0 - integ)
    return r


# --------------------------------------------------------------------------
# Binning
# --------------------------------------------------------------------------
def bin_means(d, value_col, min_n=30):
    """Mean of value_col per (5-deg vest bin, direction), steady samples, inside
    the spring table."""
    s = d[d['steady'] & d['vest'].between(SPRING_MIN_DEG, SPRING_MAX_DEG)
          & (d['direction'] != 0)].copy()
    s['bin'] = (np.round(s['vest'] / BIN_DEG) * BIN_DEG)
    g = s.groupby(['bin', 'direction'])
    m = g[value_col].agg(['mean', 'count']).reset_index()
    m['vest_mean'] = g['vest'].mean().to_numpy()
    return m[m['count'] >= min_n]


def sweep_friction_table(d):
    """Per-bin friction from a bare sweep's own legs: (up - down)/2 of KT*iq."""
    d = d.copy()
    d['kti'] = KT * d['iq']
    m = bin_means(d, 'kti').pivot(index='bin', columns='direction', values='mean')
    m = m.dropna()
    return (m[1.0] - m[-1.0]).to_numpy() / 2.0, m.index.to_numpy(float)


# --------------------------------------------------------------------------
# P1
# --------------------------------------------------------------------------
def p1(d, label, fric='model', fric_table=None):
    d = d.copy()
    if fric == 'model':
        d['tau_f'] = friction_model(d['vel'], d['direction'])
    else:
        fr, fb = fric_table
        d['tau_f'] = d['direction'] * np.interp(d['vest'], fb, fr)
    d['tau_hat'] = d['tau_f'] - KT * d['iq']            # quasi-static: J*acc ~ 0
    d['tau_true'] = tau_ext_true_bare(d['vest'])
    d['err'] = d['tau_hat'] - d['tau_true']
    b = bin_means(d, 'err')
    s = d[d['steady'] & d['tau_true'].notna() & (d['direction'] != 0)]
    res = dict(
        label=label, friction=fric,
        bins=len(b), bin_rms=float(np.sqrt(np.mean(b['mean'] ** 2))),
        bin_max=float(np.max(np.abs(b['mean']))), bin_mean=float(b['mean'].mean()),
        sample_rms=float(np.sqrt(np.mean(s['err'] ** 2))), n=len(s),
        vest_lo=float(s['vest'].min()), vest_hi=float(s['vest'].max()))
    res['pass'] = res['bin_rms'] <= 0.15
    return res, d, b


# --------------------------------------------------------------------------
# P2 + KT
# --------------------------------------------------------------------------
def p2(bare, wrench, m, r):
    kb = bare.assign(kti=KT * bare['iq'], i=bare['iq'])
    kw = wrench.assign(kti=KT * wrench['iq'], i=wrench['iq'])
    bb, bw = bin_means(kb, 'i'), bin_means(kw, 'i')
    j = bb.merge(bw, on=['bin', 'direction'], suffixes=('_b', '_w'))
    j['vest'] = (j['vest_mean_b'] + j['vest_mean_w']) / 2
    j['di'] = j['mean_w'] - j['mean_b']
    # Friction uses the same model in both runs, so it cancels in the
    # difference: delta tau_hat = -KT * delta i.
    j['dtau_hat'] = -KT * j['di']
    j['expected'] = -payload_torque(j['vest'], m, r)
    j['ratio'] = j['dtau_hat'] / j['expected']

    # Leg-averaged per bin (cancels any load-dependent friction change too)
    la = j.groupby('bin').agg(di=('di', 'mean'), vest=('vest', 'mean'),
                              n_legs=('di', 'size')).reset_index()
    la = la[la['n_legs'] == 2]
    la['ratio'] = (-KT * la['di']) / (-payload_torque(la['vest'], m, r))
    x = la['di'].to_numpy()
    y = payload_torque(la['vest'], m, r)
    kt = float(np.sum(x * y) / np.sum(x * x))
    resid = y - kt * x
    se = float(np.sqrt(np.sum(resid ** 2) / (len(x) - 1) / np.sum(x * x))) if len(x) > 1 else float('nan')
    slope = np.polyfit(la['vest'], la['ratio'], 1)[0] * 10.0 if len(la) > 2 else float('nan')
    res = dict(bins=len(la), ratio_min=float(la['ratio'].min()),
               ratio_max=float(la['ratio'].max()), ratio_mean=float(la['ratio'].mean()),
               trend_per_10deg=float(slope), kt=kt, kt_se=se,
               kt_pct_vs_datasheet=100 * (kt / KT - 1))
    res['pass'] = bool((np.abs(la['ratio'] - 1) <= 0.10).all())
    return res, j, la


# --------------------------------------------------------------------------
# P3: J from sine runs
# --------------------------------------------------------------------------
def p3(d, m=0.0, r=0.0, K=30.0):
    d = d.copy()
    tt = d['t'].to_numpy()
    d['acc'] = accel_zero_phase(tt, d['theta'].to_numpy())
    d['tau_f'] = friction_model(d['vel'])
    d['tau_true'] = tau_ext_true_bare(d['vest']) - payload_torque(d['vest'], m, r)
    # Exclude near-zero velocity: friction sign is undefined there (stiction).
    ok = d['tau_true'].notna() & (d['vel'].abs() > FRIC_V_MIN)
    s = d[ok]
    y = (KT * s['iq'] - s['tau_f'] + s['tau_true']).to_numpy()   # = J*acc
    x = s['acc'].to_numpy()
    J = float(np.sum(x * y) / np.sum(x * x))
    e0 = (s['tau_f'] - KT * s['iq']) - s['tau_true']
    e1 = e0 + J * s['acc']
    # Causal estimate via the momentum observer, and its lag vs the offline one
    mo = momentum_observer(tt, d['vel'].to_numpy(), d['iq'].to_numpy(),
                           d['tau_f'].to_numpy(), J, K)
    off = (J * d['acc'] + d['tau_f'] - KT * d['iq']).to_numpy()
    dt = np.median(np.diff(tt))
    lags = np.arange(0, int(0.3 / dt))
    a = off[ok.to_numpy()] - off[ok.to_numpy()].mean()
    best = max(lags, key=lambda L: np.dot(a[:len(a) - L] if L else a,
                                          (mo[ok.to_numpy()] - mo[ok.to_numpy()].mean())[L:]))
    e_mo = mo[ok.to_numpy()] - s['tau_true'].to_numpy()
    return dict(J=J, acc_rms=float(np.sqrt(np.mean(x ** 2))),
                Jacc_rms=float(np.sqrt(np.mean((J * x) ** 2))),
                rms_no_J=float(np.sqrt(np.mean(e0 ** 2))),
                rms_with_J=float(np.sqrt(np.mean(e1 ** 2))),
                rms_causal=float(np.sqrt(np.mean(e_mo ** 2))),
                causal_K=K, causal_lag_ms=1000.0 / K,
                causal_lag_xcorr_ms=float(best * dt * 1000), n=int(ok.sum()))


# --------------------------------------------------------------------------
# Human run (Mode 0)
# --------------------------------------------------------------------------
def human(path, J=J_DEVICE_DEFAULT, K=30.0):
    df = pd.read_csv(path)
    t = df['t'].to_numpy(float)
    t = t - t[0]
    theta = df['pos_rad'].to_numpy(float)
    vest = df['vest_deg'].to_numpy(float)
    vel = df['vel_rad_s'].to_numpy(float)
    i_meas = df['iq_measured'].to_numpy(float)
    i_cmd = df['iq_applied'].to_numpy(float)
    acc = accel_zero_phase(t, theta, window_s=0.05)
    tf = friction_model(vel)
    tau_ext = J * acc + tf - KT * i_meas            # offline, zero-phase accel
    tau_ext_cmd = J * acc + tf - KT * i_cmd         # Mode 0 form: commanded current
    mo = momentum_observer(t, vel, i_cmd, tf, J, K) # causal
    ref = tau_ext_true_bare(vest)                   # spring - rig gravity
    cuff = tau_ext - ref                            # what the wearer's arm adds
    cuff_causal = mo - ref
    # Arm model the run used (logged tau_gravity includes the arm estimate)
    with np.errstate(divide='ignore', invalid='ignore'):
        arm_mgl = np.nanmedian(df['tau_gravity'] / np.sin(np.radians(vest)) - RIG_MGL)
    out = pd.DataFrame(dict(t=t, vest=vest, vel=vel, acc=acc, i_meas=i_meas,
                            i_cmd=i_cmd, tau_f=tf, tau_ext=tau_ext,
                            tau_ext_cmd=tau_ext_cmd, tau_ext_causal=mo,
                            cuff=cuff, cuff_causal=cuff_causal,
                            arm_relaxed=-arm_mgl * np.sin(np.radians(vest)),
                            latched=df['latched'].astype(str).eq('True')))
    return out, dict(J=J, K=K, arm_mgl_logged=float(arm_mgl),
                     dt_med_ms=float(np.median(np.diff(t)) * 1000),
                     dt_p99_ms=float(np.percentile(np.diff(t), 99) * 1000),
                     i_cmd_vs_meas_rms=float(np.sqrt(np.mean((i_cmd - i_meas) ** 2))),
                     Jacc_p99=float(np.percentile(np.abs(J * acc), 99)))


def strokes(v, t, v_on=0.15, min_s=0.3):
    """Contiguous upward (v > v_on) and downward (v < -v_on) segments."""
    segs = []
    for sgn in (1, -1):
        m = (sgn * v) > v_on
        edges = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            if t[b - 1] - t[a] >= min_s:
                segs.append((sgn, a, b))
    return sorted(segs, key=lambda s: s[1])


# --------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------
def _ax_style(ax):
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(C_REF)
    ax.tick_params(colors=C_INK, labelsize=8)
    ax.grid(True, color='#e6e5e0', linewidth=0.6)
    ax.set_axisbelow(True)


def plot_p1(d, b, res, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True,
                                 gridspec_kw=dict(height_ratios=[3, 2]))
    s = d[d['steady'] & d['tau_true'].notna()]
    x = np.linspace(SPRING_MIN_DEG, SPRING_MAX_DEG, 200)
    a1.plot(x, tau_ext_true_bare(x), color=C_INK, lw=2, label='truth: spring − rig gravity')
    for dirn, c, lab in ((-1.0, C_DOWN, 'observer, down leg'), (1.0, C_UP, 'observer, up leg')):
        ss = s[s['direction'] == dirn]
        a1.plot(ss['vest'], ss['tau_hat'], '.', ms=1.5, alpha=0.35, color=c, label=lab)
        bb = b[b['direction'] == dirn]
        a2.plot(bb['vest_mean'], bb['mean'], 'o-', ms=5, lw=2, color=c, label=lab)
    a1.set_ylabel('external torque (Nm)', color=C_INK)
    a1.legend(frameon=False, fontsize=8, markerscale=6)
    a1.set_title(f"P1 — bare rig, quasi-static ({res['label']}, friction: {res['friction']})",
                 fontsize=10, color=C_INK, loc='left')
    for yv in (0.15, -0.15):
        a2.axhline(yv, color=C_REF, lw=1, ls='--')
    a2.axhline(0, color=C_REF, lw=0.8)
    a2.set_ylabel('bin error (Nm)', color=C_INK)
    a2.set_xlabel('vest angle (deg)', color=C_INK)
    a2.text(0.01, 0.95, f"bin RMS {res['bin_rms']:.3f} Nm (limit 0.15), max {res['bin_max']:.3f} Nm",
            transform=a2.transAxes, fontsize=8, color=C_INK, va='top')
    for a in (a1, a2):
        _ax_style(a)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_p2(j, la, res, m, r, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    x = np.linspace(SPRING_MIN_DEG, SPRING_MAX_DEG, 200)
    a1.plot(x, -payload_torque(x, m, r), color=C_INK, lw=2, label='expected −m·g·r·sin θ')
    for dirn, c, lab in ((-1.0, C_DOWN, 'Δτ̂ down leg'), (1.0, C_UP, 'Δτ̂ up leg')):
        jj = j[j['direction'] == dirn]
        a1.plot(jj['vest'], jj['dtau_hat'], 'o', ms=5, color=c, label=lab)
    a1.set_ylabel('Δ external torque (Nm)', color=C_INK)
    a1.legend(frameon=False, fontsize=8)
    a1.set_title(f'P2 — payload detection (m={m} kg, r={r} m)', fontsize=10, color=C_INK, loc='left')
    a2.axhspan(0.9, 1.1, color='#e6e5e0')
    a2.plot(la['vest'], la['ratio'], 'o-', color=C_DOWN, lw=2, ms=5, label='leg-averaged ratio')
    a2.axhline(1.0, color=C_REF, lw=0.8)
    a2.set_ylabel('measured / expected', color=C_INK)
    a2.set_xlabel('vest angle (deg)', color=C_INK)
    a2.text(0.01, 0.95, f"KT_measured {res['kt']:.3f} ± {res['kt_se']:.3f} Nm/A "
            f"({res['kt_pct_vs_datasheet']:+.1f}% vs 0.67)", transform=a2.transAxes,
            fontsize=8, color=C_INK, va='top')
    for a in (a1, a2):
        _ax_style(a)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_human(h, segs, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (a0, a1, a2) = plt.subplots(3, 1, figsize=(8, 8.5),
                                     gridspec_kw=dict(height_ratios=[1, 1.4, 2]))
    a0.plot(h['t'], h['vel'], color=C_INK, lw=0.8)
    a0.axhline(0, color=C_REF, lw=0.8)
    a0.set_ylabel('velocity (rad/s)', color=C_INK)
    a0.set_title('Aug 21 human run (Mode 0, self-worn) — observer applied offline',
                 fontsize=10, color=C_INK, loc='left')
    # Blind at rest: below ~0.1 rad/s the friction sign is undefined (stiction
    # anywhere in +-0.8 Nm), so the estimate is masked there.
    blind = h['vel'].abs() < 0.10
    a1.plot(h['t'], h['cuff'].where(~blind), color=C_DOWN, lw=1.0, label='offline (zero-phase)')
    a1.plot(h['t'], h['cuff_causal'].where(~blind), color=C_UP, lw=1.0,
            label='causal momentum observer, K=30/s')
    a1.axhline(0, color=C_REF, lw=0.8)
    a1.set_ylabel('cuff torque (Nm)', color=C_INK)
    a1.set_xlabel('time (s)  — masked where |v| < 0.1 rad/s', color=C_INK)
    a1.legend(frameon=False, fontsize=8, loc='lower left')
    a0.sharex(a1)
    for sgn, a, b in segs:
        s = h.iloc[a:b]
        s = s[s['cuff'].notna()]
        a2.plot(s['vest'], s['cuff'], lw=1.2, color=C_UP if sgn > 0 else C_DOWN, alpha=0.8)
    a2.plot([], [], color=C_UP, label='upward strokes (assist on)')
    a2.plot([], [], color=C_DOWN, label='downward strokes (~0 A commanded)')
    a2.axhline(0, color=C_REF, lw=0.8)
    a2.set_xlabel('vest angle (deg)', color=C_INK)
    a2.set_ylabel('estimated cuff torque (Nm)', color=C_INK)
    a2.legend(frameon=False, fontsize=8, loc='lower right')
    for a in (a0, a1, a2):
        _ax_style(a)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('bench')
    b.add_argument('--bare', required=True)
    b.add_argument('--wrench')
    b.add_argument('--mass-kg', type=float)
    b.add_argument('--r-m', type=float)
    b.add_argument('--sine', nargs='*', default=[],
                   help='sine CSVs; payload read from their arm_mass_kg/arm_com_m columns')
    b.add_argument('--out', required=True)
    h = sub.add_parser('human')
    h.add_argument('--csv', required=True)
    h.add_argument('--J', type=float, default=J_DEVICE_DEFAULT)
    h.add_argument('--out', required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    lines = []
    say = lambda s='': (print(s), lines.append(s))

    if a.cmd == 'bench':
        bare = load_bench(a.bare)
        say(f"# Observer bench analysis\nbare: {os.path.basename(a.bare)}")
        say(f"  samples {len(bare)}, dt median {np.nanmedian(bare['dt'])*1000:.2f} ms, "
            f"vest {bare['vest'].min():.1f}-{bare['vest'].max():.1f} deg, "
            f"steady {bare['steady'].mean():.0%}")
        say("\n## P1 (limit: bin RMS <= 0.15 Nm)")
        fr = sweep_friction_table(bare)
        for fric in ('model', 'sweep'):
            res, d, bb = p1(bare, os.path.basename(a.bare), fric, fr)
            say(f"  friction={fric:5s}: bins {res['bins']}, bin RMS {res['bin_rms']:.3f}, "
                f"bin max {res['bin_max']:.3f}, bin mean {res['bin_mean']:+.3f}, "
                f"sample RMS {res['sample_rms']:.3f} Nm (n={res['n']}) -> "
                f"{'PASS' if res['pass'] else 'FAIL'}")
            plot_p1(d, bb, res, os.path.join(a.out, f'p1_{fric}.png'))
        say(f"  own-sweep friction: {fr[0].min():.3f}-{fr[0].max():.3f} Nm "
            f"(mean {fr[0].mean():.3f}); model at 0.10 rad/s: "
            f"{FRIC_A - FRIC_B*0.10:.3f} Nm")
        if a.wrench:
            if a.mass_kg is None or a.r_m is None:
                ap.error('--wrench needs --mass-kg and --r-m')
            w = load_bench(a.wrench)
            res, j, la = p2(bare, w, a.mass_kg, a.r_m)
            say(f"\n## P2 (limit: every bin within +-10 %, no trend)\nwrench: "
                f"{os.path.basename(a.wrench)}, m={a.mass_kg} kg, r={a.r_m} m")
            say(f"  bins {res['bins']}, ratio {res['ratio_min']:.3f}-{res['ratio_max']:.3f} "
                f"(mean {res['ratio_mean']:.3f}), trend {res['trend_per_10deg']:+.4f} per 10 deg "
                f"-> {'PASS' if res['pass'] else 'FAIL'}")
            say(f"  KT_measured {res['kt']:.4f} +- {res['kt_se']:.4f} Nm/A "
                f"({res['kt_pct_vs_datasheet']:+.1f} % vs datasheet 0.67). "
                f"NB: only as good as m*g*r.")
            plot_p2(j, la, res, a.mass_kg, a.r_m, os.path.join(a.out, 'p2.png'))
        for sp in a.sine:
            raw = pd.read_csv(sp, comment='#')
            m = float(raw['arm_mass_kg'].iloc[0]) if 'arm_mass_kg' in raw else 0.0
            r = float(raw['arm_com_m'].iloc[0]) if 'arm_com_m' in raw else 0.0
            d = load_bench(sp)
            res = p3(d, m, r)
            say(f"\n## P3 {os.path.basename(sp)} (m={m}, r={r})")
            say(f"  J fit {res['J']:.4f} kg m^2; rms accel {res['acc_rms']:.3f} rad/s^2 "
                f"-> rms J*acc {res['Jacc_rms']:.4f} Nm")
            say(f"  error RMS without J {res['rms_no_J']:.3f}, with J {res['rms_with_J']:.3f}, "
                f"causal momentum observer (K={res['causal_K']:.0f}/s, tau={res['causal_lag_ms']:.0f} ms, "
                f"x-corr lag {res['causal_lag_xcorr_ms']:.0f} ms) {res['rms_causal']:.3f} Nm")
            say("  CAUTION: in a single-frequency sine, acc = -w^2*(theta - centre), so J is"
                " collinear with ANY angle-dependent model error (KT, spring table, gravity)."
                " Synthetic test: a 7 % KT error flips J's sign at 0.2 Hz. Trust J only if"
                " rms J*acc is well above the P1 bin errors.")
            if m > 0:
                say(f"  m*r^2 = {m*r*r:.4f} kg m^2 (compare to J_wrench - J_bare)")
    else:
        hh, info = human(a.csv, J=a.J)
        segs = strokes(hh['vel'].to_numpy(), hh['t'].to_numpy())
        say(f"# Observer on human run {os.path.basename(a.csv)}")
        say(f"  dt median {info['dt_med_ms']:.2f} ms, p99 {info['dt_p99_ms']:.2f} ms; "
            f"J={info['J']} (estimate); p99 |J*acc| {info['Jacc_p99']:.3f} Nm")
        say(f"  commanded vs measured current RMS diff {info['i_cmd_vs_meas_rms']:.3f} A")
        say(f"  arm model used in the run: m*g*L = {info['arm_mgl_logged']:.2f} Nm")
        up = [s for s in segs if s[0] > 0]
        dn = [s for s in segs if s[0] < 0]
        say(f"  strokes: {len(up)} up, {len(dn)} down (|v| > 0.15 rad/s, >= 0.3 s)")
        for sgn, lab in ((1, 'up'), (-1, 'down')):
            vals = [np.nanmedian(hh['cuff'].iloc[a0:b0]) for s, a0, b0 in segs if s == sgn]
            vals = [v for v in vals if np.isfinite(v)]
            if vals:
                say(f"  {lab:4s} stroke median cuff torque: " +
                    ', '.join(f'{v:+.2f}' for v in vals) + ' Nm')
        hh.to_csv(os.path.join(a.out, 'human_observer_timeseries.csv'), index=False)
        plot_human(hh, segs, os.path.join(a.out, 'human_cuff.png'))

    with open(os.path.join(a.out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()