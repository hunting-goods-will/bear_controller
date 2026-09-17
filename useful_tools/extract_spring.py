"""
Extract tau_spring(theta) from a saved validation CSV. No hardware needed.

    python3 useful_tools/extract_spring.py logs/model_validation_YYYYMMDD_HHMMSS.csv

WHY THIS IS SEPARATE
--------------------
validate_model.py's summary crashed on rows outside the spring table -- those
return early from compute() with tau_gravity = None, and the summary tried to
average them. But gravity does not need to come from the controller: it is
fully determined by vest angle and the known load, so it can be recomputed here
from the logged data. Nothing needs re-running.

METHOD
------
In Position Mode at constant slow velocity the actuator supplies

    tau_act = tau_gravity(theta) - tau_spring(theta) +/- tau_friction

so, averaging the two sweep directions to cancel friction:

    tau_spring = tau_gravity - mean(residual_up, residual_down)
    tau_friction = (residual_up - residual_down) / 2

Only fully-ramped samples are used; soft-start regions are not steady state.
"""
import csv
import math
import sys

RIG_MGL = 1.1012             # Nm; bare rig, 2.041 kg at 0.055 m
BIN_DEG = 5.0
RAMP_MIN = 0.999

# Must match what the run was actually configured with.
ARM_MASS_KG = 0.0
ARM_COM_M = 0.2739


def rig_gravity(vest_deg, mgl=RIG_MGL):
    """Gravitational torque from a known mass at vest angle. vest=0 is the
    arm/rig hanging at the side (lever arm vertical, torque zero), hence sin().
    Recomputed here rather than imported from controller.py -- see module
    docstring: it only needs vest angle and a known load, not the controller.
    """
    return mgl * math.sin(math.radians(vest_deg))


def main(path):
    tot_mgl = RIG_MGL + ARM_MASS_KG * 9.81 * ARM_COM_M
    bins = {'down': {}, 'up': {}}
    n = 0

    for r in csv.DictReader(open(path, newline='')):
        try:
            if float(r['ramp']) < RAMP_MIN:
                continue
            act = float(r['act_deg'])
            vest = float(r['vest_deg'])
            res = float(r['residual_measured'])
        except (ValueError, KeyError):
            continue
        leg = r['leg']
        if leg not in bins:
            continue
        n += 1
        grav = rig_gravity(vest, tot_mgl)
        b = round(act / BIN_DEG) * BIN_DEG
        bins[leg].setdefault(b, []).append((res, grav, vest))

    print(f"\nFile: {path}")
    print(f"Load: {ARM_MASS_KG:.4f} kg at {ARM_COM_M * 1000:.1f} mm  "
          f"-> total mgL {tot_mgl:.4f} Nm")
    print(f"Fully-ramped samples used: {n}")

    keys = sorted(set(bins['down']) | set(bins['up']))
    print("\n" + "=" * 70)
    print(f"{'act':>5}{'vest':>6}{'n_dn':>6}{'n_up':>6}{'grav':>8}"
          f"{'res_dn':>9}{'res_up':>9}{'SPRING':>9}{'frict':>8}")
    print("=" * 70)
    rows = []
    for b in keys:
        dn, up = bins['down'].get(b, []), bins['up'].get(b, [])
        if not dn or not up:
            continue
        grav = sum(g for _, g, _ in dn + up) / len(dn + up)
        vest = sum(v for _, _, v in dn + up) / len(dn + up)
        mdn = sum(x for x, _, _ in dn) / len(dn)
        mup = sum(x for x, _, _ in up) / len(up)
        spring = grav - (mdn + mup) / 2.0
        fric = (mup - mdn) / 2.0
        rows.append((vest, spring, fric))
        print(f"{b:5.0f}{vest:6.1f}{len(dn):6}{len(up):6}{grav:8.3f}"
              f"{mdn:9.3f}{mup:9.3f}{spring:9.3f}{fric:8.3f}")

    print("\n" + "-" * 70)
    print("  PASTE-READY TAU_SPRING_TABLE entries:")
    for vest, spring, _ in rows:
        print(f"    ({round(vest / 10.0) * 10.0:.1f}, {spring:.3f}),")
    if rows:
        f = [x for _, _, x in rows]
        print(f"\n  friction across bins: mean {sum(f) / len(f):.3f}, "
              f"range {min(f):.3f} to {max(f):.3f} Nm")
    print()

    validation_error_table(path)


def validation_error_table(path):
    """Reproduces validate_model.py's own end-of-run 'delta' column (its
    live summary: spring = tau_gravity - mean(residual_measured, both legs
    averaged to cancel friction); delta = spring - tau_spring_logged) from
    the SAVED CSV, crash-safely -- validate_model.py's live version requires
    residual_predicted/tau_spring/tau_gravity non-blank, which fails outside
    the characterized spring range; this just excludes those specific blank
    values from the average instead of raising.

    Uses each row's OWN logged tau_gravity/tau_spring -- NOT recomputed from
    the spring table currently in controller.py, and NOT extract_spring.py's
    own ARM_MASS_KG-based rig_gravity() (see the caution printed below: that
    constant may not match what this particular run was actually configured
    with). A run's logged tau_spring may also be from an earlier version of
    the table than what's live now, so this additionally reports, per bin,
    the gap between the logged value and what the CURRENT table would give
    at that angle -- making it visible which table version the run actually
    validated against. Requires both directions present in a bin, and at
    least one non-blank tau_gravity/tau_spring pair in that bin.
    """
    from main_controller.controller import tau_spring as current_tau_spring

    per_leg = {'down': {}, 'up': {}}
    n = 0
    for r in csv.DictReader(open(path, newline='')):
        try:
            if float(r['ramp']) < RAMP_MIN:
                continue
            vest = float(r['vest_deg'])
            meas = float(r['residual_measured'])
        except (ValueError, KeyError):
            continue
        leg = r['leg']
        if leg not in per_leg:
            continue
        try:
            grav = float(r['tau_gravity'])
        except (ValueError, KeyError):
            grav = None
        try:
            spring_logged = float(r['tau_spring'])
        except (ValueError, KeyError):
            spring_logged = None
        n += 1
        b = round(vest / BIN_DEG) * BIN_DEG
        per_leg[leg].setdefault(b, []).append((meas, grav, spring_logged, vest))

    print("=" * 78)
    print("  VALIDATION ERROR  (spring = logged tau_gravity - measured "
          "residual, both legs averaged)")
    print("=" * 78)
    print("  CAUTION: uses the CSV's own logged tau_gravity, which reflects")
    print("  whatever arm mass this run was actually configured with -- this")
    print("  may not be bare rig despite extract_spring.py's ARM_MASS_KG "
          "default above.")
    print(f"Fully-ramped samples used: {n}")
    keys = sorted(set(per_leg['down']) & set(per_leg['up']))
    if not keys:
        print("  No bins with both directions present.")
        print()
        return

    print(f"{'vest':>6}{'n_dn':>6}{'n_up':>6}{'grav':>8}{'meas':>9}"
          f"{'spr_meas':>10}{'logged_spr':>12}{'error':>9}{'cur_spr':>9}"
          f"{'spr_diff':>9}")
    all_err = []
    sign_mismatch = 0
    skipped_bins = 0
    for b in keys:
        dn, up = per_leg['down'][b], per_leg['up'][b]
        meas_dn = sum(m for m, _, _, _ in dn) / len(dn)
        meas_up = sum(m for m, _, _, _ in up) / len(up)
        meas_avg = (meas_dn + meas_up) / 2.0
        gravs = [g for _, g, _, _ in dn + up if g is not None]
        springs = [s for _, _, s, _ in dn + up if s is not None]
        if not gravs or not springs:
            skipped_bins += 1
            continue
        grav_avg = sum(gravs) / len(gravs)
        logged_spr = sum(springs) / len(springs)
        implied_pred = grav_avg - logged_spr   # == this run's own residual_predicted
        spring_meas = grav_avg - meas_avg
        err = spring_meas - logged_spr
        all_err.append(err)
        if implied_pred * meas_avg < 0:
            sign_mismatch += 1
        vest_mean = sum(v for _, _, _, v in dn + up) / len(dn + up)
        cur = current_tau_spring(vest_mean)
        cur_str = f"{cur:9.3f}" if cur is not None else "     None"
        diff_str = f"{logged_spr - cur:9.3f}" if cur is not None else "      n/a"
        print(f"{b:6.0f}{len(dn):6}{len(up):6}{grav_avg:8.3f}{meas_avg:9.3f}"
              f"{spring_meas:10.3f}{logged_spr:12.3f}{err:9.3f}{cur_str}"
              f"{diff_str}")
    if skipped_bins:
        print(f"  ({skipped_bins} bins skipped: no tau_gravity/tau_spring in "
              f"either direction, i.e. outside the range characterized at "
              f"run time)")
    if all_err:
        print(f"\n  mean error: {sum(all_err) / len(all_err):+.3f} Nm   "
              f"max |error|: {max(abs(x) for x in all_err):.3f} Nm   "
              f"(n={len(all_err)} bins)")
    if all_err and sign_mismatch >= 0.8 * len(all_err):
        print(f"\n  -> SIGN MISMATCH in {sign_mismatch} of {len(all_err)} bins:")
        print( "     measured residual is the opposite sign from what")
        print( "     tau_gravity - tau_spring predicts for this run. Per")
        print( "     CONTROL_DESIGN_STATE_v4.md's own falsifiable-prediction")
        print( "     note: this means either the load wasn't what the run")
        print( "     assumed, or the sign convention was inverted for this")
        print( "     run. The error/mean-error numbers above are not")
        print( "     physically meaningful until that's resolved.")
    print()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])