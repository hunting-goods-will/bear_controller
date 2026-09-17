"""
Extract bare-rig spring torque and friction from a bidirectional sweep CSV.
No hardware needed.

    python3 useful_tools/analyze_bidirectional_sweep.py logs/spring_characterization/bidirectional_sweep_YYYYMMDD_HHMMSS.csv

Bidirectional_sweep.py logs holding current (iq_hold_A), not torque, and has
no residual_measured/vest_deg columns -- it isn't the schema extract_spring.py
reads. This is the equivalent for that schema.

METHOD
------
In Position Mode, holding current times KT is the same "measured residual"
quantity used elsewhere (see validate_model.py):

    residual = iq_hold_A * KT

Averaging the down and up leg at the same target angle cancels friction, same
method as extract_spring.py:

    bare-rig spring = rig_gravity(vest_deg) - mean(residual_down, residual_up)
    friction = KT * |iq_hold_down - iq_hold_up| / 2

"Bare-rig" because rig_gravity() (imported from extract_spring.py) uses only
RIG_MGL -- these sweeps have no arm/wrench load. Grouped by target_deg (the
commanded grid point), not the actual reached angle, since down/up legs share
the same grid.
"""
import csv
import sys

from extract_spring import RIG_MGL, rig_gravity
from main_controller.controller import KT, actuator_to_vest_deg, tau_spring as current_tau_spring


def main(path):
    per_dir = {'down': {}, 'up': {}}
    n = 0
    for r in csv.DictReader(open(path, newline='')):
        d = r.get('sweep_direction')
        if d not in per_dir:
            continue
        try:
            target = float(r['target_deg'])
            angle_rad = float(r['angle_rad'])
            iq = float(r['iq_hold_A'])
        except (ValueError, KeyError):
            continue
        n += 1
        vest_deg = actuator_to_vest_deg(angle_rad)
        b = round(target)
        per_dir[d].setdefault(b, []).append((iq, vest_deg))

    print(f"\nFile: {path}")
    print(f"Bare-rig assumption: gravity from RIG_MGL={RIG_MGL:.4f} Nm only "
          f"(no arm/wrench load)")
    print(f"Rows used: {n}")

    keys = sorted(set(per_dir['down']) & set(per_dir['up']))
    if not keys:
        print("  No target angles with both directions present.")
        return

    print("\n" + "=" * 78)
    print(f"{'target':>7}{'vest':>7}{'iq_dn':>8}{'iq_up':>8}{'grav':>8}"
          f"{'SPRING':>9}{'frict':>8}{'cur_tbl':>9}{'diff':>8}")
    print("=" * 78)
    rows = []
    for b in keys:
        dn = per_dir['down'][b]
        up = per_dir['up'][b]
        iq_dn = sum(x[0] for x in dn) / len(dn)
        iq_up = sum(x[0] for x in up) / len(up)
        vest_deg = sum(x[1] for x in dn + up) / len(dn + up)
        res_dn = iq_dn * KT
        res_up = iq_up * KT
        grav = rig_gravity(vest_deg)
        spring = grav - (res_dn + res_up) / 2.0
        fric = KT * abs(iq_dn - iq_up) / 2.0
        cur = current_tau_spring(vest_deg)
        cur_str = f"{cur:9.3f}" if cur is not None else "     None"
        diff_str = f"{spring - cur:8.3f}" if cur is not None else "     n/a"
        rows.append((vest_deg, spring, fric))
        print(f"{b:7.0f}{vest_deg:7.1f}{iq_dn:8.3f}{iq_up:8.3f}{grav:8.3f}"
              f"{spring:9.3f}{fric:8.3f}{cur_str}{diff_str}")

    print("\n" + "-" * 78)
    print("  PASTE-READY (vest_deg, bare-rig spring Nm) entries:")
    for vest_deg, spring, _ in rows:
        print(f"    ({round(vest_deg / 10.0) * 10.0:.1f}, {spring:.3f}),")
    fricts = [f for _, _, f in rows]
    print(f"\n  friction across bins: mean {sum(fricts) / len(fricts):.3f}, "
          f"range {min(fricts):.3f} to {max(fricts):.3f} Nm")
    print()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
