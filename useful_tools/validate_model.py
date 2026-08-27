"""
Model validation harness. The ACTUATOR drives the arm; you do not.

WHY THIS EXISTS
---------------
Every run so far had ZERO samples at near-constant velocity. Hand-moving an arm
produces continuous acceleration and deceleration, never steady motion. Since
controller.py is quasi-static (no J*alpha term), it can only be checked in
steady-state windows -- so the model has never actually been validated against
anything. This produces those windows.

It also removes the need to hold the arm. Position Mode holds it against the
load, so there is no drop risk and no repeated slamming of 1.5 kg at 274 mm
into the bracket stop.

WHAT IS ACTUALLY BEING VALIDATED
--------------------------------
Not the assist command -- that is not directly measurable. The RESIDUAL is.

In Position Mode at constant slow velocity the actuator supplies exactly:

    tau_act = tau_gravity(theta) - tau_spring(theta) +/- tau_friction

which is the model's tau_residual plus a friction term whose sign is set by
direction of travel. So:

    measured residual = iq_measured * KT

Sweeping both directions and averaging cancels friction, exactly as in the
bidirectional sweep. The comparison of predicted vs measured residual is the
first real test of phi, the spring table, the gravity model, and KT together.

FALSIFIABLE PREDICTION
----------------------
With the wrench fitted, residual is POSITIVE (unlike every prior sweep, where
the bare rig made it negative and iq_hold was negative throughout). Measured iq
should be POSITIVE across the range, peaking near +3.2 A around actuator 40 deg
and crossing zero near actuator 90 deg. If iq comes back negative, either the
load is not what we think or the sign convention is inverted.

NO TORQUE IS COMMANDED BY THE CONTROLLER. The assist model computes and logs
only. Position Mode does the driving.
"""
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import LOG_DIR, MIN_ANGLE, MAX_ANGLE, TEMP_WARN, TEMP_MAX
from main_controller.controller import (
    AssistController, KT, actuator_to_vest_deg, tau_gravity_total)

# --- Load under test --------------------------------------------------------
ARM_MASS_KG = 1.5025
ARM_COM_M = 0.2739

# --- Sweep ------------------------------------------------------------------
TOP_DEG = 108.0
BOTTOM_DEG = 8.0             # was 22. Extended DOWN because with a human arm the
                             # rig rests at the bottom stop (the spring cannot
                             # hold ~13 Nm), so the wearer works from act ~0
                             # upward -- below the characterized range, where the
                             # controller silently commands zero. The first live
                             # human run spanned act -0.2 to 17.3 and produced no
                             # torque at all for exactly this reason.
                             # MIN_ANGLE is 5 deg, so 8 leaves 3 deg of margin.
SWEEP_VELOCITY = 0.10        # rad/s. Slow enough that J*alpha is negligible.
SOFT_START = 1.5             # s to ramp velocity up at the start of a leg
DECEL_RAD = SWEEP_VELOCITY * SOFT_START   # distance over which to decelerate
RAMP_FLOOR = 0.15            # minimum ramp; without it the step decays to zero
ARRIVE_EPS = math.radians(0.5)
DWELL_AT_END = 2.0           # s pause between legs
BIN_DEG = 10.0

# --- Gains: same set that produced 12/12 clean targets on the bare rig ------
# NOTE: these were tuned WITHOUT the test load. Rotational inertia is now
# roughly 20x higher (~0.139 vs ~0.006 kg*m^2), so the position loop may be
# sluggish or oscillatory. Watch the first descent; abort if it hunts.
IQ_P, IQ_I, IQ_D = 0.02, 0.02, 0.0
VEL_P, VEL_I, VEL_D = 4.5, 0.001, 0.0
POS_P, POS_I, POS_D = 5.0, 0.0, 0.2
LIMIT_I_MAX = 5.5
LIMIT_VELOCITY_MAX = 1.0
LIMIT_ACC_MAX = 5.0

TRACKING_ABORT = math.radians(15.0)   # goal vs actual divergence

# The arm rests against the top hard stop (measured 115.0-118.8 deg) and CANNOT
# be repositioned by hand -- the test load overpowers the spring and drops. So
# the script must be able to start from above MAX_ANGLE: Position Mode grabs
# wherever it is, and the first goal is clamped into range.
HARD_STOP_DEG = 119.5

# Park before disabling. Disabling at height drops 1.5 kg on a 274 mm lever
# into the bottom bracket stop. Park low first, then release.
PARK_DEG = 24.0
PARK_VELOCITY = 0.15


def pct(vals, p):
    if not vals:
        return float('nan')
    s = sorted(vals)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] * (hi - k) + s[hi] * (k - lo)


iface = BearInterface()
ctrl = AssistController(arm_mass_kg=ARM_MASS_KG, arm_com_m=ARM_COM_M)
collected = {'down': {}, 'up': {}}


def read():
    pos, err = iface.bear.get_present_position(iface.id)[0]
    vel, _ = iface.bear.get_present_velocity(iface.id)[0]
    iq, _ = iface.bear.get_present_iq(iface.id)[0]
    return pos[0], vel[0], iq[0], err


def thermal_ok(w, p):
    eff = max(w, p)
    if eff >= TEMP_MAX:
        raise RuntimeError(f"Effective temp {eff:.1f}C >= TEMP_MAX")
    return eff


def run_leg(writer, f, label, start, target, temps):
    """Drives goal_position at constant velocity. Logs every sample."""
    direction = 1.0 if target > start else -1.0
    goal = start
    t0 = time.monotonic()
    t_prev = t0
    n = 0
    print(f"\n--- {label.upper()} leg: {math.degrees(start):.1f} -> "
          f"{math.degrees(target):.1f} deg at {SWEEP_VELOCITY:.2f} rad/s ---")

    while True:
        now = time.monotonic()
        dt = now - t_prev
        t_prev = now
        elapsed = now - t0

        # Soft start/stop so acceleration transients don't contaminate the
        # steady-state windows. The ramp is FLOORED: scaling the step purely by
        # remaining distance gives exponential decay that never reaches the
        # target (this hung the first run at 110.00 indefinitely).
        remaining = abs(target - goal)
        ramp = min(1.0, elapsed / SOFT_START, remaining / DECEL_RAD)
        ramp = max(ramp, RAMP_FLOOR)
        goal += direction * SWEEP_VELOCITY * ramp * dt
        goal = max(MIN_ANGLE, min(goal, MAX_ANGLE))
        if (goal - target) * direction >= 0 or remaining < ARRIVE_EPS:
            goal = target
            iface.bear.set_goal_position((iface.id, goal))
            break
        iface.bear.set_goal_position((iface.id, goal))

        pos, vel, iq, err = read()
        n += 1
        if err != 128:
            raise RuntimeError(f"Error byte {err}")
        if abs(goal - pos) > TRACKING_ABORT:
            raise RuntimeError(
                f"Position loop diverged: goal {math.degrees(goal):.1f} vs "
                f"actual {math.degrees(pos):.1f} deg. Gains likely wrong for "
                f"this load.")

        if now - temps['t'] > 1.0:
            w, _ = iface.bear.get_winding_temperature(iface.id)[0]
            p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
            temps.update(w=w[0], p=p[0], t=now)
            thermal_ok(temps['w'], temps['p'])

        # Model computes only. Nothing is commanded from it.
        tau_pred, iq_pred, d = ctrl.compute(pos, vel, now=now)
        residual_meas = iq * KT
        residual_pred = d['tau_residual']

        writer.writerow([
            f"{time.time():.4f}", label, f"{math.degrees(goal):.3f}",
            f"{math.degrees(pos):.3f}", f"{d['vest_deg']:.3f}",
            f"{vel:.5f}", f"{iq:.4f}", f"{residual_meas:.4f}",
            f"{residual_pred:.4f}" if residual_pred is not None else '',
            f"{d['tau_spring']:.4f}" if d['tau_spring'] is not None else '',
            f"{d['tau_gravity']:.4f}" if d['tau_gravity'] is not None else '',
            f"{ramp:.3f}", f"{temps['w']:.1f}", f"{temps['p']:.1f}",
        ])

        # Bin every fully-ramped sample. Previously this required
        # residual_pred, which silently discarded everything below the table's
        # lower edge -- exactly the region that needs extending.
        if ramp >= 0.999:
            b = round(math.degrees(pos) / BIN_DEG) * BIN_DEG
            # Recompute gravity here rather than taking d['tau_gravity']:
            # compute() returns early outside the spring table and leaves it
            # None, which is precisely the extended region we care about.
            grav_here = tau_gravity_total(d['vest_deg'], ARM_MASS_KG, ARM_COM_M)
            collected[label].setdefault(b, []).append(
                (residual_meas, residual_pred, grav_here, d['vest_deg']))

        if n % 200 == 0:
            print(f"  act {math.degrees(pos):6.2f}  vel {vel:+6.3f}  "
                  f"iq {iq:+6.3f}  meas {residual_meas:+6.3f}  "
                  f"pred {residual_pred if residual_pred is None else round(residual_pred, 3)}"
                  f"  {temps['w']:.0f}/{temps['p']:.0f}C")

    f.flush()
    return goal


def park_and_disable():
    """Bring the load down under control, THEN disable.

    Disabling at height lets the test load free-fall into the bottom stop. This
    runs on every exit path -- clean finish, Ctrl+C, or exception. It is wrapped
    defensively because if the exit was caused by a comm failure, the park
    itself may not work; releasing without parking is still better than hanging.
    """
    try:
        pos, _, _, _ = read()
        target = math.radians(PARK_DEG)
        if pos <= target + math.radians(1.0):
            print("Already low. Releasing.")
        else:
            print(f"Parking from {math.degrees(pos):.1f} to {PARK_DEG:.0f} deg "
                  f"before release...")
            goal = pos
            t_prev = time.monotonic()
            deadline = t_prev + 60.0
            while goal > target and time.monotonic() < deadline:
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                goal = max(target, goal - PARK_VELOCITY * dt)
                iface.bear.set_goal_position((iface.id, goal))
                time.sleep(0.005)
            print("Parked.")
    except Exception as e:
        print(f"Park failed ({e}). Releasing anyway -- support the arm.")
    finally:
        iface.disable()
        print("Actuator disabled.")


os.makedirs(LOG_DIR, exist_ok=True)
log_path = os.path.join(LOG_DIR, f"model_validation_{time.strftime('%Y%m%d_%H%M%S')}.csv")

try:
    pos, vel, iq, err = read()
    w, _ = iface.bear.get_winding_temperature(iface.id)[0]
    p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
    temps = {'w': w[0], 'p': p[0], 't': time.monotonic()}

    print("=" * 74)
    print(f"  MODEL VALIDATION -- actuator drives, model logs only")
    print(f"  load {ARM_MASS_KG:.4f} kg at {ARM_COM_M * 1000:.1f} mm")
    print("=" * 74)
    print(f"pos {math.degrees(pos):.2f} deg (vest {actuator_to_vest_deg(pos):.2f})  "
          f"err {err}  w {temps['w']:.1f}C  p {temps['p']:.1f}C")

    if MAX_ANGLE < math.radians(TOP_DEG + 5.0):
        raise SystemExit(
            f"BLOCKED: MAX_ANGLE is {math.degrees(MAX_ANGLE):.1f} deg, need at "
            f"least {TOP_DEG + 5.0:.1f}. Set math.radians(118) in config.py.")
    if pos <= MIN_ANGLE:
        raise SystemExit("Start position below MIN_ANGLE.")
    if pos >= math.radians(HARD_STOP_DEG):
        raise SystemExit(
            f"Start position {math.degrees(pos):.2f} deg is above the expected "
            f"hard stop ({HARD_STOP_DEG} deg). Check the homing offset.")
    if pos >= MAX_ANGLE:
        print(f"\nNote: resting at {math.degrees(pos):.2f} deg, above MAX_ANGLE "
              f"({math.degrees(MAX_ANGLE):.1f}). Position Mode will grab here and "
              f"the first goal is clamped into range. This is expected -- the "
              f"load cannot be repositioned by hand.")

    for setter, val in [
        (iface.bear.set_p_gain_iq, IQ_P), (iface.bear.set_i_gain_iq, IQ_I),
        (iface.bear.set_d_gain_iq, IQ_D), (iface.bear.set_p_gain_id, IQ_P),
        (iface.bear.set_i_gain_id, IQ_I), (iface.bear.set_d_gain_id, IQ_D),
        (iface.bear.set_p_gain_velocity, VEL_P), (iface.bear.set_i_gain_velocity, VEL_I),
        (iface.bear.set_d_gain_velocity, VEL_D), (iface.bear.set_p_gain_position, POS_P),
        (iface.bear.set_i_gain_position, POS_I), (iface.bear.set_d_gain_position, POS_D),
        (iface.bear.set_limit_i_max, LIMIT_I_MAX),
        (iface.bear.set_limit_velocity_max, LIMIT_VELOCITY_MAX),
        (iface.bear.set_limit_acc_max, LIMIT_ACC_MAX),
        (iface.bear.set_limit_position_min, MIN_ANGLE),
        (iface.bear.set_limit_position_max, MAX_ANGLE),
    ]:
        setter((iface.id, val))

    legs = (2 * abs(TOP_DEG - BOTTOM_DEG) * math.pi / 180.0) / SWEEP_VELOCITY
    print(f"\nTwo legs, roughly {legs:.0f}s total. The actuator holds the arm "
          f"throughout.\nKeep a hand near it but do NOT support it -- your hand "
          f"would corrupt the measurement.")
    input("Press Enter to enable and begin: ")

    iface.enable_position_mode(pos)

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'leg', 'goal_deg', 'act_deg', 'vest_deg', 'vel_rad_s',
                         'iq_measured', 'residual_measured', 'residual_predicted',
                         'tau_spring', 'tau_gravity', 'ramp', 'w_temp', 'p_temp'])

        cur = run_leg(writer, f, 'down', pos, math.radians(TOP_DEG), temps) \
            if math.degrees(pos) > TOP_DEG else pos
        cur = run_leg(writer, f, 'down', cur, math.radians(BOTTOM_DEG), temps)
        print(f"\nPausing {DWELL_AT_END:.0f}s before the up leg...")
        time.sleep(DWELL_AT_END)
        run_leg(writer, f, 'up', cur, math.radians(TOP_DEG), temps)

    print(f"\nComplete. Raw data: {log_path}")

except KeyboardInterrupt:
    print("\nInterrupted -- parking before release.")
finally:
    park_and_disable()

    bins = sorted(set(collected['down']) | set(collected['up']))
    if bins:
        print("\n" + "=" * 74)
        print("  MEASURED SPRING TORQUE  (tau_spring = tau_gravity - residual)")
        print("=" * 74)
        print(f"{'act':>5}{'vest':>6}{'grav':>8}{'meas_res':>10}{'SPRING':>9}"
              f"{'frict':>8}{'table':>8}{'delta':>8}")
        rows = []
        for b in bins:
            dn = collected['down'].get(b, [])
            up = collected['up'].get(b, [])
            if not dn or not up:
                continue
            grav = sum(g for _, _, g, _ in dn + up) / len(dn + up)
            vest = sum(v for _, _, _, v in dn + up) / len(dn + up)
            mdn = sum(m for m, _, _, _ in dn) / len(dn)
            mup = sum(m for m, _, _, _ in up) / len(up)
            mean = (mdn + mup) / 2.0
            fric = (mup - mdn) / 2.0
            spring = grav - mean
            tbl = [p for _, p, _, _ in dn + up if p is not None]
            tval = (grav - (sum(tbl) / len(tbl))) if tbl else None
            rows.append((vest, spring))
            ds = f"{spring - tval:+8.3f}" if tval is not None else "     new"
            tv = f"{tval:8.3f}" if tval is not None else "      --"
            print(f"{b:5.0f}{vest:6.0f}{grav:8.3f}{mean:10.3f}{spring:9.3f}"
                  f"{fric:8.3f}{tv}{ds}")

        print("\n" + "-" * 74)
        print("  PASTE-READY TAU_SPRING_TABLE entries (vest deg, Nm):")
        # Round to the BIN size, not to 10 -- rounding 5-degree bins to the
        # nearest 10 collapses pairs onto duplicate keys.
        for vest, spring in rows:
            print(f"    ({round(vest / BIN_DEG) * BIN_DEG:.1f}, {spring:.3f}),")
        print("\n  'delta' near zero confirms the existing table.")
        print("  Rows marked 'new' extend it below the previous lower edge --")
        print("  the region a wearer actually works in, where the controller")
        print("  has been silently commanding zero.")
        print("  'frict' falls with velocity (Stribeck): compare like for like.\n")