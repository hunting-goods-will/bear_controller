"""
Offline test for fit_j_release.py. No hardware: writes synthetic logs in the
exact formats of observer_excitation.py (sweep) and j_release_test.py
(releases) with a KNOWN J and friction, runs the fit, and checks it recovers
them. Also parses the real Sep 30 bare sweep if given.

Run from the repo root:
    python3 useful_tools/test_fit_j_release_offline.py [--real-sweep SWEEP.csv]

Synthetic physics (released arm, + = up):
    J*acc = y(theta) - f(theta, v),   y = TAU_SPRING_TABLE - RIG_MGL*sin(vest)
                                          + an optional offset (a stale table)
    SIM ONLY: above the table end the last spring value is held so the arm can
    reach the 116.5 deg stop. Noise: vel 0.02 rad/s, pos 2e-4 rad, iq 0.037 A
    (STATE 4.2), loop jitter, 3.5 ms overruns on 0.2 % of loops (Oct 13 RT runs:
    a few loops > 3 ms per 5,000).
"""
import argparse
import math
import os
import shutil
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import fit_j_release as F                                           # noqa: E402
from main_controller.controller import (TAU_SPRING_TABLE, RIG_MGL,  # noqa: E402
                                        KT, PHI_DEG)

T = np.array(TAU_SPRING_TABLE)
TOP_STOP = math.radians(116.5)
# Per-angle friction measured in the Aug 20 table run (235718), vest deg -> Nm.
F718 = np.array([(81.8, .777), (87.1, .890), (92.1, .925), (97.0, .934),
                 (102.0, .932), (107.1, .949), (112.0, .922), (117.0, .918),
                 (122.0, .918), (127.0, .959), (132.0, .917), (137.0, .852),
                 (142.0, .842), (147.0, .808), (152.0, .798), (157.0, .808),
                 (162.0, .815), (167.0, .830), (172.0, .806), (176.6, .780)])


# The rig's TRUE spring-minus-gravity may differ from the table (as on Oct 15):
# SPRING_OFFSET(act_deg) -> N m added to the table value, set per scenario.
SPRING_OFFSET = [lambda act_deg: 0.0]


def y_true(act):
    v = math.degrees(act) + PHI_DEG
    return (float(np.interp(v, T[:, 0], T[:, 1])) - RIG_MGL * math.sin(math.radians(v))
            + SPRING_OFFSET[0](math.degrees(act)))


def oct15_offset(act_deg):
    """Shape of the Oct 15 sweep-minus-table gap: +0.04 at 33 deg rising to
    +0.27 at 98 deg (held beyond)."""
    return 0.04 + 0.23 * min(1.0, max(0.0, (act_deg - 33.0) / 65.0))


def fric_true(act, shift=0.0):
    return float(np.interp(math.degrees(act) + PHI_DEG, F718[:, 0], F718[:, 1])) + shift


# ---- Synthetic sweep, observer_excitation.py format --------------------------
SWEEP_COLS = ['t', 'loop_dt', 'goal_pos', 'pos', 'vel', 'present_iq', 'w_temp',
              'p_temp', 'mode', 'phase', 'freq_hz', 'amp_deg', 'center_deg',
              'velocity', 'soft_start_s', 'arm_mass_kg', 'arm_com_m']


def write_sweep(path, rng, shift=0.0, vel=0.10, soft=0.3, mass=0.0):
    rows, t = [], 0.0
    dt = 0.005

    def leg(phase, a, b):
        nonlocal t
        d = 1.0 if b > a else -1.0
        g, t0 = a, t
        decel = vel * soft
        while True:
            rem = abs(b - g)
            ramp = max(0.15, min(1.0, (t - t0) / soft, rem / decel))
            g += d * vel * ramp * dt
            arrived = (g - b) * d >= 0 or rem < math.radians(0.5)
            if arrived:
                g = b
            pos = g - d * 0.002
            fr = fric_true(pos, shift) * (d if phase in ('up', 'down') else d)
            iq = (fr - y_true(pos)) / KT + rng.normal(0, 0.037)
            rows.append([f"{t:.6f}", f"{dt:.6f}", f"{g:.6f}", f"{pos:.6f}",
                         f"{d * vel * ramp + rng.normal(0, 0.01):.6f}", f"{iq:.5f}",
                         '25.0', '27.0', 'sweep', phase, '', '', '', f"{vel:.4f}",
                         f"{soft:.3f}", f"{mass:.4f}", '0.0000'])
            t += dt
            if arrived:
                return g
    g = leg('approach', math.radians(116.3), math.radians(100))
    g = leg('down', g, math.radians(30))
    g = leg('up', g, math.radians(100))
    with open(path, 'w') as f:
        f.write("# observer_excitation.py  started 2026-10-15T10:00:00-0600\n")
        f.write("# argv: observer_excitation.py --mode sweep --arm-mass-kg 0 "
                "--arm-com-m 0 --velocity 0.10 --soft-start-s 0.3\n")
        f.write(f"# mode=sweep arm_mass_kg={mass} arm_com_m=0.0 velocity={vel} "
                f"soft_start_s={soft} freq_hz=None amp_deg=None center_deg=None cycles=None\n")
        f.write(','.join(SWEEP_COLS) + '\n')
        for r in rows:
            f.write(','.join(r) + '\n')
        f.write("# END: complete\n# PARK: parked\n")


# ---- Synthetic releases, j_release_test.py format -----------------------------
REL_COLS = ['t', 'loop_dt', 'release_idx', 'target_deg', 'phase', 'torque_enabled',
            'goal_pos', 'pos', 'vel', 'iq', 'input_voltage', 'w_temp', 'p_temp', 'err']


def free_phase(h_deg, J, rng, shift=0.0, speed_coef=0.0, iq_bias=0.0, stuck=False):
    """Integrate one release; return rows sampled at ~500 Hz."""
    th, w, ts, sim_t = math.radians(h_deg), 0.0, 0.0, 0.0
    sub = 1e-4
    out = []
    next_sample = 0.0
    while sim_t < 1.2:
        if sim_t >= next_sample:
            step = 0.0035 if rng.random() < 0.002 else 0.002 + rng.normal(0, 5e-5)
            next_sample = sim_t + step
            out.append((sim_t, th + rng.normal(0, 2e-4), w + rng.normal(0, 0.02),
                        iq_bias + rng.normal(0, 0.037)))
        if stuck:
            acc = 0.0
        else:
            f = fric_true(th, shift) + speed_coef * w
            drive = y_true(th) - (f if w > 1e-6 else min(f, y_true(th)))
            acc = drive / J
        w += acc * sub
        th += w * sub
        if th >= TOP_STOP:
            th, w = TOP_STOP, 0.0
        sim_t += sub
    return out


def write_releases(path, heights, repeats, J, rng, start_idx=1, **kw):
    """kw: shift, speed_coef, special={idx: 'stuck'|'notdone'|'iq'|'drop'}"""
    special = kw.pop('special', {})
    rows, foot, t = [], [], 0.0
    idx = start_idx - 1
    end = 'complete -- ' + str(len(heights) * repeats) + ' releases'
    for _ in range(repeats):
        for h in heights:
            idx += 1
            sp = special.get(idx)
            for k in range(5):        # a few hold rows
                rows.append([f"{t:.6f}", '0.002000', idx, f"{h:g}", 'hold', 1,
                             f"{math.radians(h):.6f}", f"{math.radians(h):.6f}",
                             '0.00000', '-1.5000', '24.00', '25.0', '27.0', '0x80'])
                t += 0.002
            rows.append([f"{t:.6f}", '', idx, f"{h:g}", 'release', 1] + [''] * 8)
            t += 0.003
            rows.append([f"{t:.6f}", '', idx, f"{h:g}", 'release', 0] + [''] * 8)
            if sp == 'notdone':
                foot.append(f"release {idx:2d}: target {h:g} deg -- NOT COMPLETED (abort -- test)")
                end = 'abort -- test'
                break
            ph = free_phase(h, J, rng, stuck=(sp == 'stuck'),
                            iq_bias=0.3 if sp == 'iq' else 0.0, **kw)
            t0 = t
            for st, p, v, iq in ph:
                rows.append([f"{t0 + st:.6f}", '0.002000', idx, f"{h:g}", 'free', 0, '',
                             f"{p:.6f}", f"{v:.5f}", f"{iq:.4f}", '24.10', '25.0', '27.0',
                             '0x84' if sp == 'drop' else '0x80'])
            t = t0 + ph[-1][0] + 0.01
            foot.append(f"release {idx:2d}: target {h:g} deg, hold {h:.2f} deg, mean iq "
                        f"last 0.2s -1.500 A, t_release_cmd {t0:.4f} s, disable() 3.0 ms "
                        f"attempts 1 (confirmed at 3.0 ms), peak |vel| 5.000 rad/s, time "
                        f"to rest 0.300 s, stuck {'YES' if sp == 'stuck' else 'no'}"
                        + (', error byte 0x84 in free phase' if sp == 'drop' else ''))
        else:
            continue
        break
    with open(path, 'w') as f:
        f.write("# j_release_test.py  started 2026-10-15T10:05:00-0600  git test dirty: no\n")
        f.write(','.join(REL_COLS) + '\n')
        for r in rows:
            f.write(','.join(str(x) for x in r) + '\n')
        for l in foot:
            f.write(f"# {l}\n")
        f.write(f"# END: {end}\n")


# ---- Checks ------------------------------------------------------------------
BAD = []


def check(name, cond, detail=''):
    print(f"    [{'PASS' if cond else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ''))
    if not cond:
        BAD.append(name)


def fit(sweep, rels, after=None, y_source='sweep'):
    b = F.sweep_tables(sweep)
    if after:
        fric, st = F.merge_sweeps(b, F.sweep_tables(after))
        stable = st['rms_dy'] <= F.STABLE_DY and st['rms_df'] <= F.STABLE_DF
        why = f"rig changed: RMS dy {st['rms_dy']:.3f}, RMS df {st['rms_df']:.3f}"
    else:
        fric, stable, why = b, False, "no after-sweep"
    releases = []
    for p in rels:
        releases.extend(F.parse_release_file(p)[1])
    per, ph = F.analyse(releases, fric, y_source=y_source)
    return per, ph, F.verdict(ph, stable, why)


def standard_set(d, J, rng, prefix='r', **kw):
    paths = []
    for k, reps in enumerate((1, 2, 2)):
        p = os.path.join(d, f"j_release_{prefix}{k}.csv")
        write_releases(p, (90, 75, 60), reps, J, rng, **kw)
        paths.append(p)
    return paths


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--real-sweep', help='a real observer_excitation sweep CSV to parse')
    a = ap.parse_args()
    rng = np.random.default_rng(7)
    d = tempfile.mkdtemp(prefix='fit_j_test_')
    try:
        sweep = os.path.join(d, 'sweep.csv')
        after = os.path.join(d, 'sweep_after.csv')
        write_sweep(sweep, rng)
        write_sweep(after, rng)

        print("\n=== 0. friction and y recovered from a synthetic sweep ===")
        fr = F.sweep_tables(sweep)
        true = np.array([fric_true(math.radians(v - PHI_DEG)) for v in fr[:, 0]])
        err = np.abs(fr[:, 1] - true)
        check("friction per bin within 0.01 Nm of truth", err.max() < 0.01,
              f"max {err.max():.4f} Nm over {len(fr)} bins, act "
              f"{fr[0, 0] - PHI_DEG:.1f}-{fr[-1, 0] - PHI_DEG:.1f} deg")
        ytrue = np.array([y_true(math.radians(v - PHI_DEG)) for v in fr[:, 0]])
        yerr = np.abs(fr[:, 2] - ytrue)
        check("y_sweep within 0.01 Nm of truth", yerr.max() < 0.01, f"max {yerr.max():.4f}")
        check("coverage reaches act >= 95 deg with --soft-start-s 0.3",
              fr[-1, 0] - PHI_DEG >= 95, f"top bin mean act {fr[-1, 0] - PHI_DEG:.1f}")

        for J in (0.015, 0.006, 0.005, 0.020):
            print(f"\n=== 1. true J = {J}; table correct; before + after sweeps ===")
            rels = standard_set(d, J, rng)
            per, ph, (lab, mean, dev, det) = fit(sweep, rels, after)
            for h, v in ph.items():
                print(f"      {h:g} deg: J_h {v['J']:.4f} ({v['n_use']}/{v['n_all']}), "
                      f"rms {v['rms']:.3f}")
            check("PASS", lab == 'PASS', det)
            check("mean J within 5 % of truth", abs(mean / J - 1) < 0.05,
                  f"{mean:.4f} vs {J}")
            ns = [p['n'] for p in per]
            check("window samples per release >= 8", min(ns) >= 8, f"{min(ns)}-{max(ns)}")
            vmax = max(p['w']['vest'].max() for p in per if p['n'])
            check("no sample outside the sweep coverage", vmax <= fr[-1, 0] + 1e-6,
                  f"max vest {vmax:.1f} vs coverage {fr[-1, 0]:.1f}")

        print("\n=== 2. table STALE like Oct 15 (true spring +0.04 -> +0.27 N m) ===")
        SPRING_OFFSET[0] = oct15_offset
        s15, a15 = os.path.join(d, 's15.csv'), os.path.join(d, 'a15.csv')
        write_sweep(s15, rng)
        write_sweep(a15, rng)
        rels = standard_set(d, 0.015, rng, prefix='st')
        per, ph, (lab, mean, dev, det) = fit(s15, rels, a15)
        print("      sweep y: " + ', '.join(f"{h:g}: {v['J']:.4f}" for h, v in ph.items())
              + f" -> {lab}, mean {mean:.4f}")
        check("y from the sweeps: PASS and within 5 % of truth",
              lab == 'PASS' and abs(mean / 0.015 - 1) < 0.05, det)
        _, ph_t, (lab_t, mean_t, dev_t, _) = fit(s15, rels, a15, y_source='table')
        print("      table y: " + ', '.join(f"{h:g}: {v['J']:.4f}" for h, v in ph_t.items())
              + f" -> {lab_t}, mean {mean_t:.4f}, max dev {dev_t * 100:.0f} %")
        check("y from the stale table is badly biased (what Oct 14's pin would have done)",
              abs(mean_t / 0.015 - 1) > 0.25 or lab_t != 'PASS')

        print("\n=== 3. rig changes during the session (after-sweep spring +0.15 N m) ===")
        SPRING_OFFSET[0] = lambda act_deg: 0.0
        s3, a3 = os.path.join(d, 's3.csv'), os.path.join(d, 'a3.csv')
        write_sweep(s3, rng)
        SPRING_OFFSET[0] = lambda act_deg: 0.15
        write_sweep(a3, rng)
        SPRING_OFFSET[0] = lambda act_deg: 0.0
        rels = standard_set(d, 0.015, rng, prefix='rc')
        per, ph, (lab, mean, dev, det) = fit(s3, rels, a3)
        check("INCOMPLETE (rig changed)", lab == 'INCOMPLETE' and 'changed' in det, det)

        print("\n=== 4. no after-sweep ===")
        rels = standard_set(d, 0.015, rng, prefix='na')
        per, ph, (lab, mean, dev, det) = fit(sweep, rels)
        check("INCOMPLETE (no after-sweep)", lab == 'INCOMPLETE' and 'after' in det, det)

        print("\n=== 5. release friction 0.05 Nm below the sweeps' (limitation) ===")
        rels = standard_set(d, 0.015, rng, prefix='fl', shift=-0.05)
        per, ph, (lab, mean, dev, det) = fit(sweep, rels, after)
        print("      " + ', '.join(f"{h:g}: {v['J']:.4f}" for h, v in ph.items())
              + f" -> {lab}, mean {mean:.4f}")
        check("biased LOW, as expected (fit subtracts too much friction)", mean < 0.015)

        print("\n=== 6. friction falls with speed (0.03 Nm per rad/s; limitation) ===")
        rels = standard_set(d, 0.015, rng, prefix='sp', speed_coef=-0.03)
        per, ph, (lab, mean, dev, det) = fit(sweep, rels, after)
        print("      " + ', '.join(f"{h:g}: {v['J']:.4f}" for h, v in ph.items())
              + f" -> {lab}, mean {mean:.4f}")
        check("biased LOW", mean < 0.015)

        print("\n=== 7. exclusions: stuck, NOT COMPLETED, iq bias, free-phase error ===")
        p1 = os.path.join(d, 'j_release_x0.csv')
        write_releases(p1, (90, 75, 60), 2, 0.015, rng,
                       special={2: 'stuck', 3: 'iq', 4: 'drop'})
        p2 = os.path.join(d, 'j_release_x1.csv')
        write_releases(p2, (90, 75, 60), 2, 0.015, rng,
                       special={6: 'notdone'})
        per, ph, (lab, mean, dev, det) = fit(sweep, [p1, p2], after)
        why = {(p['rel']['file'], p['rel']['idx']): p['why'] for p in per}
        check("stuck excluded", 'stuck' in why[('j_release_x0.csv', 2)], why[('j_release_x0.csv', 2)])
        check("iq bias excluded", 'mean iq' in why[('j_release_x0.csv', 3)], why[('j_release_x0.csv', 3)])
        check("free-phase error excluded", 'free-phase error' in why[('j_release_x0.csv', 4)],
              why[('j_release_x0.csv', 4)])
        check("NOT COMPLETED excluded", 'not completed' in why[('j_release_x1.csv', 6)],
              why[('j_release_x1.csv', 6)])
        check("INCOMPLETE when a height has < 3 usable", lab == 'INCOMPLETE', det)

        print("\n=== 8. CLI end to end, summary + plot ===")
        rels = standard_set(d, 0.015, rng, prefix='cli')
        out = os.path.join(d, 'summary.txt')
        png = os.path.join(d, 'fit.png')
        rc = F.main(['--sweep', sweep, '--sweep-after', after, *rels, '--out', out,
                     '--plot', png])
        check("rc 0 and summary written", rc == 0 and os.path.getsize(out) > 0)
        txt = open(out).read()
        check("summary has verdict, stability and cross-checks",
              'VERDICT (STATE 5): PASS' in txt and 'STABLE' in txt and 'Method B' in txt
              and 'friction +0.05' in txt and 'TAU_SPRING_TABLE' in txt)

        print("\n=== 9. a non-bare sweep is refused ===")
        bad = os.path.join(d, 'sweep_wrench.csv')
        write_sweep(bad, rng, mass=0.711)
        try:
            F.sweep_tables(bad)
            check("refused", False)
        except SystemExit as e:
            check("refused", 'bare' in str(e), str(e))

        if a.real_sweep:
            print(f"\n=== 10. real sweep: {a.real_sweep} ===")
            fr = F.sweep_tables(a.real_sweep)
            yt = F.y_table(fr[:, 0])
            for r, y in zip(fr, yt):
                print(f"      act {r[0] - PHI_DEG:5.1f}  friction {r[1]:.3f}  y_sweep "
                      f"{r[2]:.3f}  y_table {y:.3f}")
            check("parsed", len(fr) >= 5, f"{len(fr)} bins")
    finally:
        shutil.rmtree(d)
    print(f"\n{'ALL CHECKS PASSED' if not BAD else 'FAILED: ' + '; '.join(BAD)}")
    return 1 if BAD else 0


if __name__ == '__main__':
    sys.exit(main())