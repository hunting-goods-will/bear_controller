"""
Human-in-vest assist. THIS APPLIES TORQUE TO A PERSON.

APPROVAL
--------
The earlier PI approval covered a READ-ONLY script with the actuator disabled.
This is a different risk category. Confirm it is separately cleared.

WHY THE PARAMETERS ARE WHAT THEY ARE
------------------------------------
ASSIST_RATIO is 0.15, not the 0.30 used on the bench. With a real arm the
residual reaches ~8.9 Nm; at alpha=0.30 the command would be 4.89 A, over the
3.0 A clamp, so the controller would sit saturated and the characterized spring
curve would do nothing. At 0.15 the peak is 2.89 A and the curve shapes the
output.

THE ACTUATOR CANNOT OVERPOWER THE WEARER. Peak command is ~1.94 Nm against a
gravitational demand of ~8.9 Nm at horizontal. Even if the arm-mass estimate is
50% high, the actuator supplies a fraction of what the arm's own weight already
demands. The failure mode is under-assist, which is the safe one.

ARM MASS IS AN ESTIMATE AND CANNOT BE MEASURED HERE
---------------------------------------------------
Position Mode would need ~13 A to hold a human arm at horizontal, against a
5.5 A limit -- it would saturate and drop. So there is no bench-style
characterization pass for the wearer. Mass and CoM come from Winter's
anthropometric tables, scaled by the wearer's height and body mass. Treat every
torque number downstream as carrying that uncertainty.

MODES
-----
  --monitor  Actuator DISABLED. Wearer moves freely. Logs what WOULD be
             commanded. Run this first with the wearer in the vest -- it
             validates the whole path with their actual arm, at zero risk.
  --live     Applies torque.

STOPPING
--------
Ctrl+C disables immediately. There is no park routine and none is needed: the
wearer supports their own arm throughout, so disabling only removes assist.
"""
import argparse
import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import LOG_DIR, MIN_ANGLE, TEMP_WARN, TEMP_MAX
from main_controller.controller import (
    AssistController, KT, TAU_FRICTION, actuator_to_vest_deg,
    V_ON, V_OFF, V_HI, MIN_ON_TIME)

ASSIST_RATIO = 0.10          # Was 0.15. I checked the peak by sampling only the
                             # 10 spring-table angles and got 2.89 A -- but the
                             # interpolated peak is 3.40 A at vest 97, which
                             # pinned against the 3.0 A clamp for the whole first
                             # live run (every engagement logged tau 2.010, the
                             # ceiling). Saturated means constant torque and the
                             # characterized curve does nothing. 0.10 peaks at
                             # 2.57 A, inside the clamp, so the curve shapes it.

# Velocity sanity check. The first live run showed the velocity register
# reporting +7 rad/s while position FELL, and -9.9 rad/s while position ROSE --
# so the blend held at full command through motion in the wrong direction. A
# position derivative is an independent estimate from a different register.
# Prior validation runs put reported-vs-derivative disagreement at p50 0.02,
# p90 0.07, max 0.43 rad/s, so 1.0 is far outside normal and catches the garbage.
VEL_SANITY_WINDOW = 5        # samples for the derivative (~15 ms at 333 Hz)
VEL_SANITY_TOLERANCE = 1.0   # rad/s disagreement before commanding zero
FRICTION_COMP_RATIO = 0.75

TORQUE_MODE_MAX_ANGLE = math.radians(105.0)
TAPER_BAND = math.radians(15.0)
LIMIT_I_MAX = 5.5
IQ_HARD_CLAMP = 3.0
VELOCITY_ABORT = 12.0        # rad/s. Was 6.0 -- a false positive by construction:
                             # measured lift_fast peaks at 5.55 rad/s, so a brisk
                             # normal lift cleared it. It also protects nothing.
                             # Peak command is 1.94 Nm into a ~13 Nm limb; the
                             # actuator CANNOT produce 6 rad/s. High velocity with
                             # a wearer means the person moved, not a fault.
MAX_RUN_SECONDS = 60.0
MAX_LOOP_PERIOD = 0.050


def winters_arm(body_mass_kg, height_m):
    """Whole upper limb, Winter's anthropometric tables.
    Mass ~5.0% of body mass; CoM ~0.19 * height from the shoulder.
    ESTIMATE ONLY -- never measured on this wearer."""
    return 0.050 * body_mass_kg, 0.19 * height_m


def thermal_ceiling(w, p):
    if w is None or p is None:
        return 0.0
    eff = max(w, p)
    if eff < TEMP_WARN:
        return IQ_HARD_CLAMP * KT
    if eff >= TEMP_MAX:
        return 0.0
    return IQ_HARD_CLAMP * KT * (1.0 - (eff - TEMP_WARN) / (TEMP_MAX - TEMP_WARN))


def joint_limit_scale(pos):
    """Asymmetric: only the TOP limit is a hazard for positive torque.
    Firmware position limits DO NOT apply in torque mode -- this is the only
    thing stopping a command near the top of travel."""
    if pos >= TORQUE_MODE_MAX_ANGLE:
        return 0.0
    if pos > TORQUE_MODE_MAX_ANGLE - TAPER_BAND:
        return (TORQUE_MODE_MAX_ANGLE - pos) / TAPER_BAND
    return 1.0


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--monitor', action='store_true')
    g.add_argument('--live', action='store_true')
    ap.add_argument('--mass', type=float, required=True, help='wearer body mass, kg')
    ap.add_argument('--height', type=float, required=True, help='wearer height, m')
    ap.add_argument('--alpha', type=float, default=ASSIST_RATIO)
    ap.add_argument('--kf', type=float, default=FRICTION_COMP_RATIO)
    args = ap.parse_args()

    arm_m, arm_L = winters_arm(args.mass, args.height)
    ctrl = AssistController(arm_mass_kg=arm_m, arm_com_m=arm_L,
                            assist_ratio=args.alpha, friction_ratio=args.kf)
    mode = 'LIVE' if args.live else 'MONITOR'

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(
        LOG_DIR, f"human_{mode.lower()}_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    iface = BearInterface()

    def read():
        pos, err = iface.bear.get_present_position(iface.id)[0]
        vel, _ = iface.bear.get_present_velocity(iface.id)[0]
        iq, _ = iface.bear.get_present_iq(iface.id)[0]
        return pos[0], vel[0], iq[0], err

    try:
        iface.disable()
        time.sleep(0.3)
        pos, vel, iq, err = read()
        w, _ = iface.bear.get_winding_temperature(iface.id)[0]
        p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
        w, p = w[0], p[0]

        print("=" * 74)
        print(f"  HUMAN {mode}   alpha={args.alpha}  k_f={args.kf}")
        print(f"  wearer {args.mass:.0f} kg, {args.height:.2f} m")
        print(f"  arm ESTIMATE: {arm_m:.2f} kg at {arm_L * 1000:.0f} mm "
              f"(Winter's tables -- not measured)")
        print(f"  friction {TAU_FRICTION:.2f} Nm, compensated at {args.kf:.0%}")
        print(f"  blend: on {V_ON}, off {V_OFF}, full {V_HI} rad/s, "
              f"min on-time {MIN_ON_TIME * 1000:.0f} ms")
        print("=" * 74)
        print(f"pos {math.degrees(pos):.2f} deg (vest {actuator_to_vest_deg(pos):.2f})  "
              f"iq {iq:+.4f}  err {err}  w {w:.1f}C  p {p:.1f}C")

        if abs(iq) > 0.15:
            raise SystemExit(f"ABORT: iq {iq:.3f} A while disabled. Investigate.")

        tau, iq_c, d = ctrl.compute(pos, 1.0, now=0.0)
        ctrl._latched = False
        print(f"\nAt this angle, at full blend: {tau:.3f} Nm / {iq_c:.3f} A"
              f"{'  [' + d['reason'] + ']' if d['reason'] else ''}")

        if args.live:
            print("\n" + "!" * 74)
            print("  LIVE. Torque will be applied to the wearer.")
            print("  Brief them BEFORE enabling:")
            print("   - assist engages only on deliberate UPWARD movement")
            print("   - nothing happens on the way down or while holding still")
            print("   - it is a light assist, roughly a fifth of the arm's weight")
            print("   - say STOP at any time; a second person holds Ctrl+C")
            print(f"  Auto-stop after {MAX_RUN_SECONDS:.0f}s.")
            print("!" * 74)
            if input("\nType LIVE to confirm: ").strip() != "LIVE":
                raise SystemExit("Not confirmed.")
            iface.bear.set_limit_i_max((iface.id, LIMIT_I_MAX))
            iface.enable()
        else:
            print("\nMONITOR: nothing commanded. Have the wearer move normally.")
            input("Press Enter to start: ")

        with open(log_path, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['t', 'loop_dt', 'pos_rad', 'act_deg', 'vest_deg',
                         'vel_rad_s', 'blend_w', 'latched', 'tau_spring',
                         'tau_gravity', 'tau_residual', 'tau_requested',
                         'iq_requested', 'iq_applied', 'iq_measured',
                         'joint_scale', 'saturated', 'vel_derivative',
                         'vel_ok', 'w_temp', 'p_temp', 'err',
                         # Run conditions, repeated on every row so the log
                         # records its own configuration and can't be read
                         # against the wrong constants later.
                         'alpha', 'k_f', 'arm_mass_kg', 'arm_com_m'])

            t0 = time.monotonic()
            t_prev = t0
            n = 0
            temp_t = 0.0
            hist = []
            vel_rejects = 0

            while True:
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                elapsed = now - t0
                if elapsed > MAX_RUN_SECONDS:
                    print(f"\nAuto-stop at {MAX_RUN_SECONDS:.0f}s.")
                    break

                pos, vel, iq_meas, err = read()
                n += 1
                if now - temp_t > 1.0:
                    w, _ = iface.bear.get_winding_temperature(iface.id)[0]
                    p, _ = iface.bear.get_powerstage_temperature(iface.id)[0]
                    w, p = w[0], p[0]
                    temp_t = now
                if err != 128:
                    raise RuntimeError(f"Error byte {err}")
                if abs(vel) > VELOCITY_ABORT:
                    raise RuntimeError(f"Velocity {vel:.2f} rad/s over abort limit")
                if dt > MAX_LOOP_PERIOD and n > 5:
                    raise RuntimeError(f"Loop period {dt * 1000:.1f} ms over limit")

                # Independent velocity estimate from the position register.
                hist.append((now, pos))
                if len(hist) > VEL_SANITY_WINDOW + 1:
                    hist.pop(0)
                vel_ok = True
                vel_deriv = None
                if len(hist) > VEL_SANITY_WINDOW:
                    span = hist[-1][0] - hist[0][0]
                    if span > 0:
                        vel_deriv = (hist[-1][1] - hist[0][1]) / span
                        if abs(vel - vel_deriv) > VEL_SANITY_TOLERANCE:
                            vel_ok = False
                            vel_rejects += 1

                # A rejected sample is fed to the controller as zero velocity,
                # not skipped -- that way the rate limiter still runs and the
                # command ramps DOWN rather than being held at its last value.
                tau_req, iq_req, d = ctrl.compute(
                    pos, vel if vel_ok else 0.0,
                    tau_ceiling=thermal_ceiling(w, p), now=now)
                jscale = joint_limit_scale(pos)
                iq_final = max(0.0, min(iq_req * jscale, IQ_HARD_CLAMP))

                if args.live:
                    iface.bear.set_goal_iq((iface.id, iq_final))
                else:
                    iq_final = 0.0

                wr.writerow([
                    f"{time.time():.4f}", f"{dt:.5f}", f"{pos:.5f}",
                    f"{math.degrees(pos):.3f}", f"{d['vest_deg']:.3f}",
                    f"{vel:.5f}", f"{d['w']:.4f}", d['latched'],
                    f"{d['tau_spring']:.4f}" if d['tau_spring'] is not None else '',
                    f"{d['tau_gravity']:.4f}" if d['tau_gravity'] is not None else '',
                    f"{d['tau_residual']:.4f}" if d['tau_residual'] is not None else '',
                    f"{tau_req:.4f}", f"{iq_req:.4f}", f"{iq_final:.4f}",
                    f"{iq_meas:.4f}", f"{jscale:.3f}", d['saturated'],
                    f"{vel_deriv:.5f}" if vel_deriv is not None else '', vel_ok,
                    f"{w:.1f}", f"{p:.1f}", err,
                    f"{args.alpha:.4f}", f"{args.kf:.4f}",
                    f"{arm_m:.4f}", f"{arm_L:.4f}",
                ])

                if n % 100 == 0:
                    print(f"  {elapsed:5.1f}s  act {math.degrees(pos):6.2f}  "
                          f"vel {vel:+6.3f}  w {d['w']:.2f}  "
                          f"tau {tau_req:5.3f}  cmd {iq_final:5.3f}  "
                          f"meas {iq_meas:+6.3f}  {w:.0f}/{p:.0f}C")

        if vel_rejects:
            print(f"\n{vel_rejects} samples rejected on velocity sanity "
                  f"({100.0 * vel_rejects / max(1, n):.1f}%). If this is more "
                  f"than a fraction of a percent, the velocity register is "
                  f"unreliable at speed and the blend band derived from the "
                  f"human profiling needs re-examining.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            iface.bear.set_goal_iq((iface.id, 0.0))
        except Exception:
            pass
        iface.disable()
        print(f"Actuator disabled. Log: {log_path}")


if __name__ == '__main__':
    main()