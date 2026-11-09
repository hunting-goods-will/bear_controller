"""
hold_feasibility_sim.py -- desk simulation, NO hardware, NO imports from the repo.

Question 1 (authority, F4): at which vest angles can the motor's torque ceiling cover the
residual R(theta) that a hold has to carry?   R = (A + RIG) sin(theta) - L * spring1(theta)

Question 2 (identifiability, F3): from a few *hold readings* (what the controller will see),
can it learn the arm parameter A and/or the spring level L well enough to size the assist at
the OTHER angles, when readings above the torque ceiling are censored (saturated)?

Inputs and their status (see STATE sections 2.4, 2.5):
  * spring1(theta): Level 1 table in controller.py (vest 81.8-176.6 deg), measured [verified].
  * Level 2 = 1.99 x Level 1 over vest 135-170 [verified for that range only]; this script applies the
    1.99 factor at ALL angles (below 135 deg it is an extrapolation). L3/L4 = HYPOTHETICAL
    multiples (3, 4): untested. Treated as planning numbers only.
  * RIG = 1.1012 N m (2.041 kg at 55 mm CoM, CoM an estimate).
  * A = arm gravity torque at horizontal. Sam's anthropometric estimate: 20.29 N m, never validated.
    TRUE value is drawn as 20.29 x U(0.8, 1.2) (+-20 %: an assumption, unverified).
  * Reading noise sigma = 0.1 N m per hold (the value used on Oct 29) [inferred].
  * Muscle tone: the wearer is never fully relaxed, reading = R - tone, tone ~ U(0, TONE) N m, with
    TONE = 0.3 default (assumption, unverified). Readings are therefore lower bounds on R.
  * KT = 0.67 N m/A (datasheet, unverified on this unit). Ceiling = I_clamp * KT.
  * Friction is assumed cancelled by the creep (that is the design); residual friction error is
    inside sigma. If it is not, the results are optimistic.

Run:  python3 -I hold_feasibility_sim.py         (about 1 s)
"""
import numpy as np

KT = 0.67
RIG = 1.1012
A0 = 20.29                    # N m, anthropometric estimate (Sam)
SIGMA = 0.10
TONE = 0.30
SAT_MARGIN = 0.15             # a reading within this of the ceiling is treated as saturated

VEST = np.array([81.8, 87.1, 92.1, 97.0, 102.0, 107.1, 112.0, 117.0, 122.0, 127.0, 132.0,
                 137.0, 142.0, 147.0, 152.0, 157.0, 162.0, 167.0, 172.0, 176.6])
S1 = np.array([3.162, 3.034, 3.061, 3.011, 2.924, 2.876, 2.745, 2.638, 2.539, 2.461, 2.273,
               2.048, 1.903, 1.723, 1.595, 1.490, 1.373, 1.302, 1.192, 1.090])


SPRING_VARIANT = "table"      # "table" = controller.py Level 1 table (older data); "oct29" = +0.054 at vest 105
                              # rising to +0.154 at 170, to match the Oct 29 Level 1 sweeps (2.95 -> 1.39), STATE 2.5


def spring1(th):
    base = np.interp(th, VEST, S1)
    if SPRING_VARIANT == "oct29":
        return base + 0.054 + np.clip((np.asarray(th, float) - 105.0) / 65.0, 0, 1) * 0.100
    return base


def R(th, A, L):
    return (A + RIG) * np.sin(np.radians(th)) - L * spring1(th)


def hr(title):
    print("\n" + "=" * 100 + "\n" + title + "\n" + "=" * 100)


# ---------------------------------------------------------------- Part 1: authority
def part1():
    hr("PART 1 - AUTHORITY: residual R(theta) the motor must carry (N m), A = 20.29 (Sam's estimate)")
    ths = [90, 105, 120, 135, 150, 157, 163, 170]
    print("vest deg   " + "".join("%8d" % t for t in ths))
    for L, tag in ((0, "no spring"), (1, "L1 (measured)"), (2, "L2 (measured, 1.99x)"),
                   (3, "L3 (HYPOTHETICAL 3x)"), (4, "L4 (HYPOTHETICAL 4x)")):
        print("%-22s" % tag + "".join("%8.1f" % R(t, A0, L) for t in ths))
    print("\nCeilings: 3.0 A = %.2f | 5.5 A = %.2f | 8.0 A = %.2f | 10 A = %.2f N m (KT 0.67, unverified)"
          % (3 * KT, 5.5 * KT, 8 * KT, 10 * KT))

    hr("Lowest vest angle above which a HOLD is fully carried by the motor (R <= ceiling, everywhere above it up to 170 deg)")
    grid = np.arange(85, 170.01, 0.5)
    print("            ceiling:   3.0 A     5.5 A     8.0 A    10.0 A")
    for A in (10.0, 15.0, 20.29, 25.0):
        for L in (1, 2, 3, 4):
            row = []
            for I in (3.0, 5.5, 8.0, 10.0):
                ok = R(grid, A, L) <= I * KT
                # lowest angle such that all angles >= it are ok
                if not ok[-1]:
                    row.append("   never")
                else:
                    k = len(ok) - 1
                    while k > 0 and ok[k - 1]:
                        k -= 1
                    row.append("%8.1f " % grid[k] if k > 0 else "   <=85 ")
            print("A=%5.2f  L%d%s        %s" % (A, L, " (hyp)" if L > 2 else "      ", "".join(row)))
    global SPRING_VARIANT
    SPRING_VARIANT = "oct29"
    print("\nSensitivity: same, A=20.29, spring table shifted to the Oct 29 Level 1 sweeps (+0.05..+0.15 N m):")
    for L in (2, 3, 4):
        row = []
        for I in (3.0, 5.5, 8.0, 10.0):
            ok = R(grid, 20.29, L) <= I * KT
            if not ok[-1]:
                row.append("   never")
            else:
                k = len(ok) - 1
                while k > 0 and ok[k - 1]:
                    k -= 1
                row.append("%8.1f " % grid[k])
        print("A=20.29  L%d%s        %s" % (L, " (hyp)" if L > 2 else "      ", "".join(row)))
    SPRING_VARIANT = "table"
    print("\nReading: for Sam's A the motor can carry the whole residual only in a thin band near the")
    print("top of travel at L2; the table shows how far a stronger spring or a higher ceiling widens it.")


# ---------------------------------------------------------------- Part 2: identifiability
def run_scenario(rng, n, A0s, L_true, I_clamp, n_holds, th_lo, th_hi, L_known):
    ceil = I_clamp * KT
    A_true = A0s * rng.uniform(0.8, 1.2, n)
    th = rng.uniform(th_lo, th_hi, (n, n_holds))
    true = R(th, A_true[:, None], L_true)
    tone = rng.uniform(0, TONE, (n, n_holds))
    read = true - tone + rng.normal(0, SIGMA, (n, n_holds))
    sat = read >= ceil - SAT_MARGIN
    read = np.where(sat, ceil, read)
    A_hat, L_hat = estimate(th, read, sat, L_true if L_known else None, A0s)
    nun = (~sat).sum(axis=1)
    thu = np.where(~sat, th, np.nan)
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        spread = np.nanmax(thu, axis=1) - np.nanmin(thu, axis=1)
    spread = np.where(nun >= 2, spread, 0.0)
    return A_true, A_hat, L_hat, nun, spread


def estimate(th, read, sat, L_known, A0s, L_prior_mean=2.0):
    """Per trial: least squares of read_k = (A+RIG) sin_k - L s1_k on UNSATURATED holds, with a prior
    A ~ N(A0s, 0.3 A0s) and (if L unknown) L ~ N(2, 1). No unsaturated hold -> the prior alone."""
    n, m = th.shape
    psd = 0.30 * A0s
    A_hat = np.empty(n)
    L_hat = np.empty(n)
    sn = np.sin(np.radians(th))
    s1 = spring1(th)
    for i in range(n):
        u = ~sat[i]
        if L_known is not None:
            y = read[i, u] + L_known * s1[i, u]
            a = sn[i, u]
            w0 = 1.0 / psd ** 2
            A_hat[i] = ((a * y).sum() / SIGMA ** 2 + w0 * (A0s + RIG)) / ((a * a).sum() / SIGMA ** 2 + w0) - RIG
            L_hat[i] = L_known
        else:
            H = np.column_stack([sn[i, u], -s1[i, u]]) if u.any() else np.zeros((0, 2))
            P = np.diag([1.0 / psd ** 2, 1.0])
            x0 = np.array([A0s + RIG, L_prior_mean])
            x = np.linalg.solve(H.T @ H / SIGMA ** 2 + P, H.T @ read[i, u] / SIGMA ** 2 + P @ x0)
            A_hat[i], L_hat[i] = x[0] - RIG, x[1]
    return A_hat, L_hat


def part2(seed=7, n=4000):
    rng = np.random.default_rng(seed)
    hr("PART 2 - IDENTIFIABILITY: do N hold readings predict R at the working angles better than the prior?")
    print("A_true = A0 x U(0.8,1.2); sigma %.2f N m; tone U(0,%.1f); readings within %.2f of the ceiling are SATURATED" %
          (SIGMA, TONE, SAT_MARGIN))
    print("(censored: they only say R >= ceiling and are not used).  Prior-only = A0 and the known level, no learning.")
    print("Scored at vest 105/120/135/150 deg.  n = %d per row.  err = |R_hat - R_true| in N m;  over = P(R_hat - R_true > 1 N m)." % n)
    print("'learned' = rows where >= 1 hold was unsaturated; 'level learned' needs >= 2 unsaturated holds >= 10 deg apart.\n")
    work = np.array([105.0, 120.0, 135.0, 150.0])
    rows = [
        ("Sam A0=20.3", 20.29, 2, 5.5, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 2, 8.0, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 2, 10.0, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 2, 8.0, 8, 100, 170, True),
        ("Sam A0=20.3", 20.29, 2, 8.0, 8, 140, 170, True),
        ("light A0=12", 12.0, 2, 5.5, 8, 110, 150, True),
        ("light A0=12", 12.0, 2, 8.0, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 3, 8.0, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 4, 8.0, 8, 110, 150, True),
        ("Sam A0=20.3", 20.29, 4, 8.0, 8, 110, 150, False),
        ("Sam A0=20.3", 20.29, 4, 10.0, 8, 130, 170, False),
        ("light A0=12", 12.0, 3, 8.0, 8, 110, 150, False),
    ]
    print("wearer        L   ceiling  holds        L known  P(>=1 unsat)  P(level learned)   err learned (105/120/135/150)    err prior-only (same trials)      over (same trials): learned / prior-only   prior-only over, ALL trials")
    for tag, A0s, L, I, nh, lo, hi, known in rows:
        A_true, A_hat, L_hat, nun, spread = run_scenario(rng, n, A0s, L, I, nh, lo, hi, known)
        learned = nun >= 1
        Rt = np.array([R(w, A_true, L) for w in work])
        Rh = np.array([R(w, A_hat, L_hat) for w in work])
        Rp = np.array([R(w, A0s, L) for w in work])[:, None]
        e_l = np.abs(Rh - Rt)[:, learned]
        e_p_all = np.abs(Rp - Rt)
        e_p = e_p_all[:, learned] if learned.any() else np.full((4, 1), np.nan)   # SAME trials as 'learned'
        ov_l = ((Rh - Rt) > 1.0)[:, learned].mean(axis=1) if learned.any() else np.full(4, np.nan)
        ov_p = ((Rp - Rt) > 1.0)[:, learned].mean(axis=1) if learned.any() else np.full(4, np.nan)
        ov_p_all = ((Rp - Rt) > 1.0).mean()
        med_l = np.median(e_l, axis=1) if learned.any() else np.full(4, np.nan)
        print("%-12s L%d%s  %4.1f A   %3d-%3d x%d  %-5s    %5.2f        %5.2f           %s         %s        %.2f / %.2f      %.2f" % (
            tag, L, "*" if L > 2 else " ", I, lo, hi, nh, "yes" if known else "no", learned.mean(),
            ((nun >= 2) & (spread >= 10)).mean(),
            "/".join("%4.1f" % v for v in med_l), "/".join("%4.1f" % v for v in np.median(e_p, axis=1)),
            np.nanmean(ov_l) if learned.any() else float("nan"), np.nanmean(ov_p) if learned.any() else float("nan"), ov_p_all))
    print("\n* = hypothetical spring level (untested).")

    hr("Unsaturated vest range for Sam's arm at L2 (R <= ceiling - %.2f)" % SAT_MARGIN)
    for I in (5.5, 8.0, 10.0):
        ok = np.arange(100, 170.1, 0.5)
        ok = ok[R(ok, A0, 2) <= I * KT - SAT_MARGIN]
        print("  ceiling %4.1f A (%.2f N m): %s" % (I, I * KT, "none" if len(ok) == 0 else
              "%.1f to 170 deg (%.1f deg wide)" % (ok[0], 170 - ok[0])))


def selftest():
    """Checks of the simulation itself (planted-bug tests are for Opus to extend)."""
    assert abs(R(105, 20.29, 2) - 14.9) < 0.3
    assert abs(R(90, 20.29, 0) - (20.29 + RIG)) < 1e-9
    rng = np.random.default_rng(1)
    # noiseless, no tone, one unsaturated hold, L known: A recovered to < 0.5 % (prior weight is tiny)
    global SIGMA, TONE
    s0, t0 = SIGMA, TONE
    SIGMA, TONE = 1e-4, 0.0
    A_true, A_hat, _, nun, _ = run_scenario(rng, 300, 20.29, 2, 10.0, 6, 150, 170, True)
    ok = nun >= 1
    assert ok.mean() > 0.5 and np.max(np.abs(A_hat - A_true)[ok]) < 0.1, np.max(np.abs(A_hat - A_true)[ok])
    # with no unsaturated hold the estimate IS the prior
    A_true, A_hat, _, nun, _ = run_scenario(rng, 300, 20.29, 2, 3.0, 4, 100, 120, True)
    assert (nun == 0).all() and np.allclose(A_hat, 20.29)
    # level learned when two unsaturated holds are far apart and the level is unknown
    A_true, A_hat, L_hat, nun, spread = run_scenario(rng, 300, 15.0, 3, 10.0, 8, 120, 170, False)
    m = (nun >= 2) & (spread >= 20)
    assert m.sum() > 20 and np.max(np.abs(L_hat - 3)[m]) < 0.2, (m.sum(), np.max(np.abs(L_hat - 3)[m]))
    SIGMA, TONE = s0, t0
    print("selftest: ok")


# ---------------------------------------------------------------- Part 3: wearer carries part of the arm
def part3(seed=11, n=4000):
    """Sam's objection (Nov 5): the wearer will carry part of the arm, so the motor will not saturate.
    True -- but then the hold reading is u = R - tau_muscle, and the controller cannot tell a heavy arm
    partly held from a light arm fully relaxed.  Here the wearer carries a FRACTION f of R (not a small
    tone), f ~ U(f_lo, f_hi), and the estimator assumes a relaxed wearer (as the hold design does)."""
    rng = np.random.default_rng(seed)
    hr("PART 3 - WEARER CARRIES A FRACTION f OF THE ARM: no saturation, but what does the reading measure?")
    print("Sam's arm A0 = 20.29, L2, 8 holds in vest 110-150, L known, A_true = A0 x U(0.8,1.2), sigma %.2f." % SIGMA)
    print("Option A's worn run (Aug 21) supplied 6 % of the residual + friction (alpha 0.06): the wearer carried ~90 %.\n")
    print("f range        ceiling   P(>=1 unsat)   median R_hat/R_true at 120 deg   P(R_hat > R_true + 1 N m)   median err at 120 (N m)")
    work = 120.0
    for flo, fhi in ((0.0, 0.0), (0.2, 0.5), (0.5, 0.9), (0.85, 0.95)):
        for I in (5.5, 8.0, 10.0):
            ceil = I * KT
            A_true = A0 * rng.uniform(0.8, 1.2, n)
            th = rng.uniform(110, 150, (n, 8))
            f = rng.uniform(flo, fhi, (n, 1)) * np.ones((1, 8))
            read = (1 - f) * R(th, A_true[:, None], 2) + rng.normal(0, SIGMA, (n, 8))
            sat = read >= ceil - SAT_MARGIN
            read = np.where(sat, ceil, read)
            A_hat, _ = estimate(th, read, sat, 2, A0)
            ok = (~sat).any(axis=1)
            Rt, Rh = R(work, A_true, 2), R(work, A_hat, 2)
            if ok.any():
                ratio = np.median((Rh / Rt)[ok]); over = ((Rh - Rt) > 1.0)[ok].mean(); err = np.median(np.abs(Rh - Rt)[ok])
                print("%.2f-%.2f      %4.1f A     %5.2f           %5.2f                           %5.2f                       %5.2f" %
                      (flo, fhi, I, ok.mean(), ratio, over, err))
            else:
                print("%.2f-%.2f      %4.1f A     %5.2f           (no unsaturated hold: estimate = prior)" % (flo, fhi, I, ok.mean()))
    print("\nReading: carrying part of the arm keeps the motor out of saturation, but the reading then measures what")
    print("the wearer let go of, not the arm. R_hat/R_true ~ (1 - f): an under-estimate (the safe direction), and the")
    print("assist it produces depends on how much the wearer chose to relax during the holds.")


# ---------------------------------------------------------------- Part 4: learn where the motor CAN carry it
def part4(seed=13, n=4000):
    """Nov 5: overhead work holds near the TOP of travel, where the residual is small enough for the motor.
    Learn A (arm + any tool, at horizontal) from holds in vest 150-170 only, L known (the spring setting),
    and use the arm-gravity shape to predict R at 105/120/135. Compare with the table prior, which cannot
    know a tool. Tool = extra torque at horizontal (1 kg at 0.6 m ~ 5.9 N m)."""
    rng = np.random.default_rng(seed)
    hr("PART 4 - LEARN FROM OVERHEAD HOLDS (vest 150-170), L2 known, predict R lower down; with and without a tool")
    print("6 holds, A_true = 20.29 x U(0.8,1.2) + tool, sigma %.2f, wearer carries f of R.  Errors in N m at vest 105/120/135." % SIGMA)
    print("\ntool    f range     ceiling  P(>=1 unsat)   learned: median |err|     table prior: median |err|   learned over by >1 N m")
    for tool in (0.0, 3.0, 5.9):
        for flo, fhi in ((0.0, 0.0), (0.0, 0.3)):
            for I in (5.5, 8.0, 10.0):
                ceil = I * KT
                A_true = A0 * rng.uniform(0.8, 1.2, n) + tool
                th = rng.uniform(150, 170, (n, 6))
                f = rng.uniform(flo, fhi, (n, 1))
                read = (1 - f) * R(th, A_true[:, None], 2) + rng.normal(0, SIGMA, (n, 6))
                sat = read >= ceil - SAT_MARGIN
                read = np.where(sat, ceil, read)
                A_hat, _ = estimate(th, read, sat, 2, A0)
                ok = (~sat).any(axis=1)
                w = np.array([105.0, 120.0, 135.0])[:, None]
                el = np.median(np.abs(R(w, A_hat, 2) - R(w, A_true, 2))[:, ok], axis=1)
                ep = np.median(np.abs(R(w, A0, 2) - R(w, A_true, 2))[:, ok], axis=1)
                ov = ((R(w, A_hat, 2) - R(w, A_true, 2)) > 1.0)[:, ok].mean()
                print("%4.1f    %.1f-%.1f     %4.1f A     %5.2f          %s            %s               %.2f" % (
                    tool, flo, fhi, I, ok.mean(), "/".join("%4.1f" % v for v in el), "/".join("%4.1f" % v for v in ep), ov))


# ---------------------------------------------------------------- Part 5: F3 check (rule pinned in PLAN Nov 8 16:58)
# Nov 8 measured springs (sweep_compare_20261108_L1/L2.txt): spring = y + RIG sin(vest).
_V1 = np.array([110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160.])
_Y1 = np.array([1.684, 1.665, 1.620, 1.576, 1.458, 1.364, 1.281, 1.230, 1.137, 1.048, 0.998])
_V2 = np.array([140, 145, 150, 155, 160, 165, 170.])
_Y2 = np.array([3.288, 3.1335, 2.9555, 2.803, 2.7545, 2.742, 2.744])   # 165/170: Oct 29 (y within 0.03 of Nov 8)
_SP1 = _Y1 + RIG * np.sin(np.radians(_V1))
_SP2 = _Y2 + RIG * np.sin(np.radians(_V2))
MODEL_RATIO = 2.01


def spring1_nov8(th):
    th = np.asarray(th, float)
    slope = (_SP1[1] - _SP1[0]) / 5.0
    return np.where(th < 110, _SP1[0] + slope * (th - 110), np.interp(th, _V1, _SP1))


def spring2(th, ratio_low):
    """Level 2 spring: measured at vest >= 140; below, ratio_low x Level 1, blended to the 140 value."""
    th = np.asarray(th, float)
    hi = np.interp(th, _V2, _SP2)
    lo = ratio_low * spring1_nov8(th)
    lo = lo + (_SP2[0] - ratio_low * spring1_nov8(140.0)) * np.clip((th - 120.0) / 20.0, 0, 1)   # continuous at 140
    return np.where(th >= 140, hi, lo)


def R2(th, A, ratio_low):
    return (A + RIG) * np.sin(np.radians(th)) - spring2(th, ratio_low)


SHRINK = 0.0   # N m subtracted from A_hat (conservative learner); 0 = the design as pinned


def f3_trials(rng, n, tool, flo, fhi, I, n_holds=6):
    ceil = I * KT
    A_true = A0 * rng.uniform(0.8, 1.2, n) + tool
    ratio = rng.uniform(1.85, 2.15, n)
    th = rng.uniform(150, 170, (n, n_holds))
    f = rng.uniform(flo, fhi, (n, 1))
    fric = rng.uniform(-0.15, 0.15, (n, 1))
    read = (1 - f) * R2(th, A_true[:, None], ratio[:, None]) + fric + rng.normal(0, SIGMA, (n, n_holds))
    sat = read >= ceil - SAT_MARGIN
    # estimator: model spring (ratio 2.01), relaxed-wearer assumption, prior A0 +/- 30 %, unsaturated holds only
    sn = np.sin(np.radians(th))
    y = read + spring2(th, MODEL_RATIO) - RIG * sn          # = A * sin + noise, if the model were right
    psd = 0.30 * A0
    w0 = 1.0 / psd ** 2
    u = ~sat
    num = ((sn * y) * u).sum(axis=1) / SIGMA ** 2 + w0 * A0
    den = ((sn * sn) * u).sum(axis=1) / SIGMA ** 2 + w0
    A_hat = num / den - SHRINK
    return A_true, ratio, A_hat, u.any(axis=1)


def part5(seed=17, n=20000):
    rng = np.random.default_rng(seed)
    hr("PART 5 - F3 CHECK against the rule pinned in PLAN (Nov 8 16:58), Nov 8 measured Level 2 spring, SHRINK %.2f" % SHRINK)
    work = np.array([105.0, 120.0, 135.0, 150.0])[:, None]
    verdict = {}
    print("(a) SAFETY  P(R_hat - R > 0.5 N m), worst of the 4 angles; must be <= 0.05")
    print("tool   wearer f   " + "".join("   %4.1f A" % I for I in (5.5, 8.0, 10.0)))
    for tool in (0.0, 3.0, 5.9):
        for flo, fhi in ((0.0, 0.0), (0.0, 0.3)):
            row = []
            for I in (5.5, 8.0, 10.0):
                A_true, ratio, A_hat, ok = f3_trials(rng, n, tool, flo, fhi, I)
                Rt = R2(work, A_true, ratio)
                Rh = R2(work, A_hat, MODEL_RATIO)
                p = ((Rh - Rt) > 0.5).mean(axis=1).max()
                row.append(p)
                verdict.setdefault(('a', I), []).append(p <= 0.05)
            print("%4.1f   %.1f-%.1f    " % (tool, flo, fhi) + "".join("   %6.3f" % p for p in row))
    print("\n(b) USEFULNESS  relaxed wearer, trials with >= 1 unsaturated hold: median |err| per angle 105/120/135/150 (N m)")
    print("    must be <= 0.5 at every angle, and <= half the table prior's error when a tool is present")
    for tool in (0.0, 3.0, 5.9):
        for I in (5.5, 8.0, 10.0):
            A_true, ratio, A_hat, ok = f3_trials(rng, n, tool, 0.0, 0.0, I)
            Rt = R2(work, A_true[ok], ratio[ok])
            el = np.median(np.abs(R2(work, A_hat[ok], MODEL_RATIO) - Rt), axis=1)
            ep = np.median(np.abs(R2(work, A0, MODEL_RATIO) - Rt), axis=1)
            good = (el <= 0.5).all() and (tool == 0 or (el <= 0.5 * ep).all())
            verdict.setdefault(('b', I), []).append(good)
            print("  tool %3.1f  %4.1f A   learned %s   table %s   %s" % (
                tool, I, "/".join("%4.2f" % v for v in el), "/".join("%4.2f" % v for v in ep), "ok" if good else "FAIL"))
    print("\n(c) AVAILABILITY  P(>= 1 unsaturated hold of 6), relaxed wearer; must be >= 0.90")
    for I in (5.5, 8.0, 10.0):
        ps = []
        for tool in (0.0, 3.0, 5.9):
            _, _, _, ok = f3_trials(rng, n, tool, 0.0, 0.0, I)
            ps.append(ok.mean())
        verdict[('c', I)] = [min(ps) >= 0.90]
        print("  %4.1f A   tool 0 / 3.0 / 5.9: %s" % (I, " / ".join("%.3f" % p for p in ps)))
    print("\nVERDICT per clamp (all of a, b, c must pass):")
    for I in (5.5, 8.0, 10.0):
        parts = {k: all(verdict[(k, I)]) for k in 'abc'}
        print("  %4.1f A:  a %s   b %s   c %s   ->  F3 %s" % (I, *("pass" if parts[k] else "FAIL" for k in 'abc'),
                                                        "PASS" if all(parts.values()) else "FAIL"))


if __name__ == "__main__":
    selftest()
    part1()
    part2()
    part3()
    part4()
    part5()
    for SHRINK in (0.3, 0.4):      # module-level loop variable: part5 reads the global
        part5()