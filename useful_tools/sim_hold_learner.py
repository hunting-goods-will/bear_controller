"""Feasibility simulations behind PLAN v4 §3.2 and STATE §4.7 / §4.9 (Oct 24, 2026).

Simple 1-DOF models with made-up spring and rig curves, Karnopp stiction and a toy wearer.
Exploratory only: this checks the logic, not the hardware.

  python3 sim_hold_learner.py            # runs every check and prints the numbers quoted in the docs

Part 1 - residual learner (old Phase 2 design): a wearer who repeats the same lift.
Part 2 - hold-phase load estimation: friction bias, creep, rate limit, arm leaving the cuff.
"""
import numpy as np

KT = 0.67
J0 = 0.016          # vest moving parts (physics estimate)
JARM = 0.80         # whole arm about the shoulder (rough)
FS, FC, BV = 0.85, 0.80, 0.05   # static / Coulomb friction (N·m), viscous (N·m·s/rad)
VEST = np.deg2rad(72)


def spring(th):
    return 3.0 * np.sin(th + 0.4)


def rig(th):
    return 1.1 * np.sin(th + VEST + np.deg2rad(5))


# ------------------------------------------------------------------ part 1: learner
def learner_run(alpha, strokes=30):
    """Wearer follows the same min-jerk lift every stroke; muscles supply whatever is missing."""
    dt, K, kf, J = 0.002, 30.0, 0.75, 0.016
    tf = lambda v: 0.85 * np.tanh(v / 0.02)
    sp = lambda th: 3.0 * np.sin(th + 0.4)
    gr = lambda th: 1.1 * np.cos(th - 0.3)
    bins = np.linspace(0.2, 1.8, 17)
    R = np.zeros(len(bins) - 1)
    th, v, i_prev, I, r, on, t_on = 0.3, 0.0, 0.0, 0.0, 0.0, False, 0.0
    for _ in range(strokes):
        for k in range(int(2.6 / dt)):
            t = k * dt
            if t < 1.0:
                x = t; thd = 0.3 + 1.3 * (10*x**3 - 15*x**4 + 6*x**5); thdd = 1.3 * (60*x - 180*x**2 + 120*x**3)
            elif t < 1.3:
                thd, thdd = 1.6, 0.0
            elif t < 2.3:
                x = t - 1.3; thd = 1.6 - 1.3 * (10*x**3 - 15*x**4 + 6*x**5); thdd = -1.3 * (60*x - 180*x**2 + 120*x**3)
            else:
                thd, thdd = 0.3, 0.0
            fh = tf(v)
            I += (KT * i_prev - fh + r) * dt
            r = K * (J * v - I)                      # momentum observer, p0 = 0
            if v > 0.15:
                on, t_on = True, 0.0
            if on:
                t_on += dt
            if on and v < 0.08 and t_on > 0.15:
                on = False
            w = 1.0 if (on and v > 0) else 0.0
            b = np.searchsorted(bins, th) - 1
            Rb = R[b] if 0 <= b < len(R) else 0.0
            if w > 0 and 0 <= b < len(R):
                R[b] += 0.02 * (-r - R[b])           # binned learner fitting -tau_ext_hat
            i_prev = w * (alpha * max(0.0, Rb) + kf * fh) / KT
            tau_cuff = J * thdd + tf(v) - KT * i_prev - sp(th) + gr(th) + 200 * (thd - th) * J
            acc = (KT * i_prev - tf(v) + sp(th) - gr(th) + tau_cuff) / J
            v += acc * dt
            th += v * dt
    return R


# ------------------------------------------------------------------ part 2: hold
def hold_run(A=9.0, rho_final=1.0, relax_tau=2.0, umax=11.8, kp=40.0, ki=60.0, kd=1.5,
             human_k=15.0, human_b=1.0, remove_at=None, v_trip=1.0, T=10.0,
             th_h=np.deg2rad(60), rate=50.0, seed=0, creep_amp=0.0, creep_f=0.25):
    """Torque-mode PI hold (upward-only) at th_h, 500 Hz controller, 10 kHz plant.
    Wearer: feedforward effort (1 - rho(t))·need, rho -> rho_final with time constant relax_tau,
    plus the wearer's own stiffness about th_h. remove_at: the arm leaves the cuff at that time."""
    rng = np.random.default_rng(seed)
    dtc, sub = 0.002, 20
    dt = dtc / sub
    armg = lambda th: A * np.sin(th + VEST)
    th, v, u, I = th_h, 0.0, 0.0, 0.0
    attached, tripped, t_trip, t_rem = True, False, None, None
    log = []
    for k in range(int(T / dtc)):
        t = k * dtc
        vm = v + rng.normal(0, 0.026)                 # velocity register noise
        e = th_h + creep_amp * np.sin(2 * np.pi * creep_f * t) - th
        if not tripped:
            I = np.clip(I + ki * e * dtc, 0, umax)
            u_des = np.clip(kp * e - kd * vm + I, 0, umax)
            if vm > v_trip:
                tripped, t_trip, u = True, t, 0.0
            else:
                u += np.clip(u_des - u, -rate * dtc, rate * dtc)
        else:
            u = 0.0
        if remove_at is not None and t >= remove_at and attached:
            attached, t_rem = False, t
        rho = rho_final * (1 - np.exp(-t / relax_tau))
        for _ in range(sub):
            if attached:
                need = armg(th) + rig(th) - spring(th)
                muscle = (1 - rho) * need + human_k * (th_h - th) - human_b * v
                Jt, other = J0 + JARM, u + spring(th) - rig(th) - armg(th) + muscle
            else:
                Jt, other = J0, u + spring(th) - rig(th)
            if abs(v) < 1e-3:
                if abs(other) <= FS:
                    v, acc = 0.0, 0.0
                else:
                    acc = (other - np.sign(other) * FC) / Jt
            else:
                acc = (other - np.sign(v) * FC - BV * v) / Jt
            v += acc * dt
            th += v * dt
            if th > np.deg2rad(116.5):
                th, v = np.deg2rad(116.5), 0.0
        log.append((t, th, v, u))
    L = np.array(log)
    out = {"R_true": armg(th_h) + rig(th_h) - spring(th_h)}
    last = L[L[:, 0] > T - 2.0]
    out["u_last2s"] = last[:, 3].mean()
    out["u_pp"] = np.ptp(last[:, 3])
    out["th_pp_deg"] = np.rad2deg(np.ptp(last[:, 1]))
    if remove_at is not None:
        post = L[L[:, 0] >= remove_at]
        out["peak_v_after_removal"] = post[:, 2].max()
        out["trip_delay_ms"] = None if t_trip is None else (t_trip - t_rem) * 1000
    return out, L


def cycle_avg(L, f, T, n):
    m = L[:, 0] > T - n / f
    return L[m, 3].mean()


if __name__ == "__main__":
    print("PART 1: residual learner, wearer repeats the same lift (true need 7-18 N·m)")
    for a in (0.06, 0.3, 0.6):
        R = learner_run(a)
        nz = R[R != 0]
        print(f"  alpha {a}: R_hat range {nz.min():.2f} .. {nz.max():.2f} N·m -> max(0,R_hat) = {max(0, R.max()):.2f}")

    print("\nPART 2: hold-phase estimation (residual at 60 deg, made-up curves)")
    for s in range(5):
        o, _ = hold_run(seed=s)
        print(f"  relaxed, seed {s}: u = {o['u_last2s']:.2f}  R_true = {o['R_true']:.2f}  (friction offset {o['u_last2s'] - o['R_true']:+.2f})")
    for rho in (0.5, 0.0):
        o, _ = hold_run(rho_final=rho)
        print(f"  rho_final {rho}: u = {o['u_last2s']:.2f}")
    o, _ = hold_run(umax=3.7)
    print(f"  clamp 3.7 N·m: u = {o['u_last2s']:.2f}")
    for amp in (0.5, 1.0, 2.0):
        for f in (0.25, 0.5):
            o, L = hold_run(creep_amp=np.deg2rad(amp), creep_f=f, T=14.0)
            print(f"  creep +-{amp} deg @ {f} Hz: cycle-average u = {cycle_avg(L, f, 14.0, 2):.2f} (R {o['R_true']:.2f})")
    for tau in (1.0, 2.0, 4.0):
        o, L = hold_run(rate=4.0, relax_tau=tau, creep_amp=np.deg2rad(1.0), creep_f=0.5, T=16.0)
        t, u = L[:, 0], L[:, 3]
        w = int(2.0 / 0.002)
        ua = np.convolve(u, np.ones(w) / w, mode="same")
        idx = np.where(ua >= 0.9 * o["R_true"])[0]
        print(f"  4 N·m/s rate limit, relax tau {tau} s: u = {cycle_avg(L, 0.5, 16.0, 2):.2f}, 90% at ~{t[idx[0]]:.1f} s")
    for kd, ki in ((1.5, 60), (3.0, 120), (0.5, 20)):
        o, _ = hold_run(kd=kd, ki=ki)
        print(f"  gains kp 40 ki {ki} kd {kd}: theta p-p {o['th_pp_deg']:.2f} deg, u p-p {o['u_pp']:.2f} N·m")
    o, _ = hold_run(remove_at=8.0, T=9.0)
    print(f"  arm leaves cuff, 1 rad/s trip: peak {o['peak_v_after_removal']:.1f} rad/s, trip after {o['trip_delay_ms']:.0f} ms")
    o, _ = hold_run(remove_at=8.0, v_trip=1e9, T=9.0)
    print(f"  arm leaves cuff, no trip: peak {o['peak_v_after_removal']:.1f} rad/s")
    o, _ = hold_run(remove_at=8.0, rho_final=0.0, v_trip=1e9, T=9.0)
    print(f"  arm leaves cuff, motor already ~0 (spring alone): peak {o['peak_v_after_removal']:.1f} rad/s")