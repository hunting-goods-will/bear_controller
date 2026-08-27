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
        grav = tot_mgl * math.sin(math.radians(vest))
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


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])