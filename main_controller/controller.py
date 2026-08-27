"""
Upward-assist control model. PURE — no hardware imports, cannot actuate.

This module computes a torque command from (angle, velocity). It does not talk
to the actuator. Wiring it to a control loop is a separate, later step, done
only after the command curve below has been reviewed on paper.

CONTROL LAW
-----------
    tau_cmd = w(theta_dot) * [ alpha * max(0, tau_residual) + k_f * tau_friction ]
    tau_residual = tau_gravity_total(theta_vest) - tau_spring(theta_vest)

Each term, and why it is the shape it is:

w(theta_dot) -- BLEND, not a threshold.
    A hard velocity threshold has two failure modes: chatter (noise crossing
    the line toggles assist on and off) and a torque discontinuity (0 -> several
    Nm instantly, felt by the wearer as a kick). w() ramps smoothly over a
    velocity band instead. Band edges come from measured data: the worst
    non-intentional velocity observed was 0.106 rad/s, and slow intentional
    lifts sustain 0.87 rad/s (p90). V_LO/V_HI sit in that gap.
    Direction handling is implicit -- w is zero for all negative velocity, so
    downward strokes command nothing without a separate branch.

alpha -- ASSIST RATIO, and it is not optional.
    Residual torque with a human arm reaches ~8.9 Nm. The actuator ceiling at
    5.5 A is 3.685 Nm. Commanding full compensation saturates below roughly
    vest 155 deg, at which point the output is just constant max torque and the
    characterized spring curve does nothing. alpha keeps the system inside its
    real authority so the curve actually shapes the output.
    THIS IS A PI DECISION. It sets how much load comes off the wearer.

max(0, ...) -- never command negative.
    On the bare rig, or near the top of travel, or for a light wearer, the
    residual goes negative: the spring alone over-provides. Commanding that
    negative value would drive the arm DOWN against the user. Clamped to zero.

k_f * tau_friction -- deliberately UNDER-compensated.
    Measured direction-dependent torque is 0.67 Nm. Compensating 100% risks
    self-driving instability: assist overcomes the friction that was holding
    the arm, which produces motion, which triggers more assist. k_f ~ 0.75 is
    standard practice. Note this term is blended too -- applying
    sign(theta_dot) * 0.67 unblended reintroduces the exact discontinuity
    w() exists to remove.

WHAT THIS MODEL DOES NOT KNOW
-----------------------------
- Arm mass and CoM are ESTIMATES from anthropometric tables, not measurements.
  They dominate the gravity term. Everything downstream inherits their error.
- Viscous friction is unmeasured. The 0.67 Nm figure is static, taken at
  |v| < 0.05 rad/s. At operating speeds (measured up to 5.5 rad/s) there is an
  additional velocity-dependent term this model ignores entirely.
- KT = 0.67 Nm/A is the datasheet value, never verified on this unit. It
  converts every torque here into a current command.
- tau_spring is characterized for ONE configuration: Level 1 spring, Low
  activation zone. Using this model on any other configuration is invalid.
"""
import math

# --- Verified / measured ----------------------------------------------------
KT = 0.67                    # Nm/A -- DATASHEET, unverified on this unit
PHI_DEG = 72.0               # actuator -> vest frame offset (342 - 270)
TAU_FRICTION = 0.80          # Nm. See note below -- friction is NOT constant.
RIG_MGL = 1.1012             # Nm; 2.041 kg at 0.055 m (CoM is an estimate)

# FRICTION IS VELOCITY-DEPENDENT (Stribeck), measured across five validation
# runs at 0.05 / 0.10 / 0.20 / 0.35 / 0.50 rad/s, bare rig:
#
#     friction = 0.910 - 0.208 * v      (R^2 = 0.97, residual max 0.009 Nm)
#
# Friction DECREASES with speed -- the falling branch of a Stribeck curve as
# the contact leaves boundary lubrication. This is the opposite of viscous
# damping, and it is why the earlier 0.67 Nm figure disagreed: that came from
# settle points, where the position loop stops as soon as friction can hold it,
# recording a LOWER BOUND on stiction rather than a measurement of it.
#
# DO NOT extrapolate the line. It predicts negative friction above 4.4 rad/s,
# which is impossible. A real Stribeck curve flattens to a minimum and then
# rises again as viscous drag takes over; only the falling branch was measured.
# 0.80 is the value at the fastest velocity actually tested (0.50 rad/s), which
# is closest to real lift speeds and errs on the low side -- under-compensating
# friction is stable, over-compensating self-drives.

# --- Tunables that need PI sign-off -----------------------------------------
ASSIST_RATIO = 0.30          # alpha. Fraction of residual gravity to supply.
FRICTION_COMP_RATIO = 0.75   # k_f. Deliberately < 1.0 (see module docstring).

# --- Blend band, from measured human data -----------------------------------
V_ON = 0.15                  # rad/s; engage. Above worst incidental motion (0.106)
V_OFF = 0.08                 # rad/s; disengage. Hysteresis gap prevents chatter.
V_HI = 0.45                  # rad/s; full assist. Below slow-lift p90 (0.867)
MIN_ON_TIME = 0.15           # s; minimum engaged duration once latched

# TORQUE RATE LIMIT. Without this the command steps from 0 to whatever the
# blend calls for in a single loop iteration -- at 333 Hz that is effectively
# instantaneous. In the first live human run the command jumped straight to a
# saturated 2.010 Nm on engagement, which the wearer felt as a jerk on the way
# UP, and which contributed to breaking the mounting flange. Symmetric, so
# disengagement ramps down too rather than dropping out.
# NOTE: this is a rate limit inside compute(). Emergency stops bypass it --
# the loop's finally block writes iq = 0 directly.
MAX_TAU_RATE = 4.0           # Nm/s; ~0.5 s from zero to full assist

# Hysteresis exists because measured hand-driven motion crosses a single
# threshold constantly. A MONITOR run -- zero torque commanded, so the
# controller could not have caused it -- showed 100 transitions in 44s, with
# 30 of 50 ON segments under 100ms. Human movement is simply not smooth around
# any single value, so a one-threshold gate stutters no matter how it's tuned.
V_LO = V_ON                  # backwards-compat alias for blend()

# --- Wearer parameters ------------------------------------------------------
# DEFAULT IS ZERO ON PURPOSE. With no arm mass the residual is negative
# everywhere and the model commands zero torque -- so a first bare-rig run is
# safe by construction. These must be set deliberately, never inherited.
ARM_MASS_KG = 0.0
ARM_COM_M = 0.0

# --- Actuator limits --------------------------------------------------------
LIMIT_I_MAX = 5.5
TAU_MAX = LIMIT_I_MAX * KT   # 3.685 Nm

# --- Characterized spring, Level 1 / Low activation zone --------------------
# Vest degrees -> Nm. Continuous bidirectional sweep at 0.10 rad/s, bare rig,
# 5-degree bins, ~290 samples per bin per direction. Friction cancelled by
# averaging the two sweep directions.
#
# This supersedes the earlier settle-at-points table. It is better in three
# ways: 5-degree resolution instead of 10, roughly 30x more samples per point,
# and it extends down to vest 82 -- 10 degrees below the old lower edge. That
# matters because with an arm in it the rig rests at the bottom stop, so the
# wearer works from the BOTTOM of travel upward. The first live human run
# spanned vest 72-89 and commanded zero torque the whole time, because the old
# table simply had no entries there.
#
# The old table agrees with this one to within +-0.05 Nm typical, 0.12 max --
# an independent confirmation, since the two were produced by different sweep
# methods.
#
# CAVEATS:
#  - vest 81.8 came from a bin with only 29 down-leg samples against 322 up.
#    Friction cancellation needs balanced legs, so treat it as the weakest
#    point in the table. It is also the most useful one for coverage.
#  - vest 87.1 / 92.1 are slightly non-monotonic (3.034 then 3.061). The gap is
#    0.027 Nm, inside run-to-run scatter, and it suggests the spring's peak
#    support lies around vest 82-92 rather than at the 105 deg the manual
#    quotes for the Low setting.
#  - Below vest 82 is still uncharacterized. That is the bracket-stop region,
#    a mechanical limit, not a gap that more data can close.
TAU_SPRING_TABLE = [
    (81.8, 3.162),   # weakest point -- see caveat above
    (87.1, 3.034),
    (92.1, 3.061),
    (97.0, 3.011),
    (102.0, 2.924),
    (107.1, 2.876),
    (112.0, 2.745),
    (117.0, 2.638),
    (122.0, 2.539),
    (127.0, 2.461),
    (132.0, 2.273),
    (137.0, 2.048),
    (142.0, 1.903),
    (147.0, 1.723),
    (152.0, 1.595),
    (157.0, 1.490),
    (162.0, 1.373),
    (167.0, 1.302),
    (172.0, 1.192),
    (176.6, 1.090),
]
# The old table's vest-182 entry (0.741) is dropped: it was flagged uncorrected,
# it disagrees with this curve's trend by ~0.23 Nm, and TORQUE_MODE_MAX_ANGLE
# (act 105 = vest 177) means the controller never reaches it anyway.
SPRING_MIN_DEG = TAU_SPRING_TABLE[0][0]
SPRING_MAX_DEG = TAU_SPRING_TABLE[-1][0]


def actuator_to_vest_deg(theta_actuator_rad):
    return math.degrees(theta_actuator_rad) + PHI_DEG


def tau_spring(theta_vest_deg):
    """Linear interpolation over the characterized table.

    Returns None outside the characterized range. The caller must treat that as
    'do not assist' rather than extrapolating. Extrapolating below 92 deg would
    UNDER-estimate a still-rising spring curve, which OVER-estimates the
    residual, which over-assists -- the failure direction that pushes a wearer.
    """
    if theta_vest_deg < SPRING_MIN_DEG or theta_vest_deg > SPRING_MAX_DEG:
        return None
    for i in range(len(TAU_SPRING_TABLE) - 1):
        d0, t0 = TAU_SPRING_TABLE[i]
        d1, t1 = TAU_SPRING_TABLE[i + 1]
        if d0 <= theta_vest_deg <= d1:
            if d1 == d0:
                return t0
            f = (theta_vest_deg - d0) / (d1 - d0)
            return t0 + f * (t1 - t0)
    return TAU_SPRING_TABLE[-1][1]


def tau_gravity_total(theta_vest_deg, arm_mass_kg=None, arm_com_m=None):
    """Gravitational torque from rig plus wearer's arm, about the pivot.

    theta_vest = 0 is the arm hanging at the side, where the lever arm is
    vertical and the torque is zero. Hence sin().
    """
    m = ARM_MASS_KG if arm_mass_kg is None else arm_mass_kg
    L = ARM_COM_M if arm_com_m is None else arm_com_m
    return (RIG_MGL + m * 9.81 * L) * math.sin(math.radians(theta_vest_deg))


def blend(theta_dot, v_lo=V_LO, v_hi=V_HI):
    """Smoothstep over [v_lo, v_hi]. Zero for all non-positive velocity.

    Smoothstep (3x^2 - 2x^3) rather than a linear ramp because its derivative
    is zero at both ends -- no kink in commanded torque at the band edges.
    """
    if theta_dot <= v_lo:
        return 0.0
    if theta_dot >= v_hi:
        return 1.0
    x = (theta_dot - v_lo) / (v_hi - v_lo)
    return x * x * (3.0 - 2.0 * x)


class AssistController:
    """Stateless. Given (angle, velocity), returns a torque command.

    Stateless is deliberate: it makes the whole control law unit-testable with
    no hardware, and means there is no internal state to get stuck in a bad
    configuration during a live run.
    """

    def __init__(self, arm_mass_kg=ARM_MASS_KG, arm_com_m=ARM_COM_M,
                 assist_ratio=ASSIST_RATIO, friction_ratio=FRICTION_COMP_RATIO):
        self.arm_mass_kg = arm_mass_kg
        self.arm_com_m = arm_com_m
        self.assist_ratio = assist_ratio
        self.friction_ratio = friction_ratio
        # The ONLY mutable state. All the physics above stays pure and testable.
        self._latched = False
        self._on_since = 0.0
        self._last_tau = 0.0
        self._last_t = None

    def _update_latch(self, theta_dot, now):
        """Schmitt trigger. Engage above V_ON, disengage below V_OFF, and only
        after MIN_ON_TIME has elapsed."""
        if not self._latched:
            if theta_dot > V_ON:
                self._latched = True
                self._on_since = now
        else:
            if theta_dot < V_OFF and (now - self._on_since) >= MIN_ON_TIME:
                self._latched = False
        return self._latched

    def _rate_limit(self, tau, now, limit_rate=True):
        """Clamp dtau/dt. Applied to EVERY return path including the zero
        returns, so disengagement ramps down instead of dropping out."""
        if not limit_rate:
            self._last_tau = tau
            self._last_t = now
            return tau
        # First call has no elapsed time. Use a nominal loop period rather than
        # passing through unlimited -- passing through would let the very first
        # iteration command full torque, which is the exact failure this guards.
        dt = 0.003 if self._last_t is None else (now - self._last_t)
        self._last_t = now
        if dt <= 0.0:
            dt = 0.003
        step = MAX_TAU_RATE * dt
        tau = max(self._last_tau - step, min(self._last_tau + step, tau))
        self._last_tau = tau
        return tau

    def compute(self, theta_actuator_rad, theta_dot_rad_s, tau_ceiling=TAU_MAX,
                now=0.0, limit_rate=True):
        """Returns (tau_cmd_Nm, iq_cmd_A, diagnostics_dict).

        tau_ceiling lets a caller pass a thermally-derated limit in. It is NOT
        computed here -- this module never reads hardware.
        """
        vest_deg = actuator_to_vest_deg(theta_actuator_rad)
        spring = tau_spring(vest_deg)

        # While latched, blend over [V_OFF, V_HI] rather than [V_ON, V_HI].
        # Otherwise a dip to 0.10 rad/s would give w = 0 even while engaged,
        # reproducing the stutter with extra bookkeeping.
        latched = self._update_latch(theta_dot_rad_s, now)
        w_val = blend(theta_dot_rad_s, V_OFF, V_HI) if latched else 0.0

        diag = {
            'vest_deg': vest_deg,
            'w': w_val,
            'latched': latched,
            'tau_spring': spring,
            'tau_gravity': None,
            'tau_residual': None,
            'saturated': False,
            'reason': None,
        }

        if spring is None:
            diag['reason'] = 'outside characterized spring range'
            t = self._rate_limit(0.0, now, limit_rate)
            return t, t / KT, diag

        gravity = tau_gravity_total(vest_deg, self.arm_mass_kg, self.arm_com_m)
        residual = gravity - spring
        diag['tau_gravity'] = gravity
        diag['tau_residual'] = residual

        w = diag['w']
        if w <= 0.0:
            diag['reason'] = 'below blend band (not an upward stroke)'
            t = self._rate_limit(0.0, now, limit_rate)
            return t, t / KT, diag

        # GATED friction compensation. If there is nothing to assist, command
        # nothing -- do not apply friction feedforward on its own.
        #
        # The alternative (unconditional friction comp) is defensible and
        # arguably better long-term: it makes the device transparent to the
        # wearer regardless of assist level, which is the actual point of
        # friction compensation. But it means no configuration ever outputs
        # zero, which removes "commands nothing on a bare rig" as a testable
        # property. Gated for the first live controller; revisit with the PI.
        if residual <= 0.0:
            diag['reason'] = 'no positive residual (spring already over-provides)'
            t = self._rate_limit(0.0, now, limit_rate)
            return t, t / KT, diag

        tau_assist = self.assist_ratio * residual
        tau_fric = self.friction_ratio * TAU_FRICTION
        tau_cmd = w * (tau_assist + tau_fric)

        if tau_cmd > tau_ceiling:
            tau_cmd = tau_ceiling
            diag['saturated'] = True
            diag['reason'] = 'clamped to torque ceiling'

        tau_cmd = self._rate_limit(tau_cmd, now, limit_rate)
        return tau_cmd, tau_cmd / KT, diag


def _dry_run():
    """Prints the command curve across the reachable range. No hardware."""
    print("\nDRY RUN -- no hardware touched.\n")
    for label, m, L in [("BARE RIG (arm=0)", 0.0, 0.0),
                        ("WITH ARM (3.5 kg @ 0.32 m, estimated)", 3.5, 0.32)]:
        c = AssistController(arm_mass_kg=m, arm_com_m=L)
        print("=" * 76)
        print(f"  {label}   alpha={c.assist_ratio}  k_f={c.friction_ratio}  "
              f"tau_max={TAU_MAX:.3f} Nm")
        print("=" * 76)
        print(f"{'act deg':>9}{'vest deg':>10}{'grav':>8}{'spring':>8}"
              f"{'resid':>8}{'tau@full':>10}{'iq@full':>9}  note")
        for act_deg in range(10, 111, 10):
            th = math.radians(act_deg)
            tau, iq, d = c.compute(th, 1.0, limit_rate=False)
            c._latched = False   # velocity 1.0 -> w = 1
            g = d['tau_gravity']
            s = d['tau_spring']
            r = d['tau_residual']
            fmt = lambda x: f"{x:8.2f}" if x is not None else f"{'--':>8}"
            note = d['reason'] or ''
            print(f"{act_deg:>9}{d['vest_deg']:>10.1f}{fmt(g)}{fmt(s)}{fmt(r)}"
                  f"{tau:>10.3f}{iq:>9.3f}  {note}")
        print()

    print("=" * 76)
    print("  BLEND RESPONSE (at actuator 60 deg, with arm)")
    print("=" * 76)
    c = AssistController(arm_mass_kg=3.5, arm_com_m=0.32)
    print(f"{'vel rad/s':>11}{'w':>8}{'tau Nm':>10}{'iq A':>9}")
    for v in [0.0, 0.05, 0.106, 0.15, 0.20, 0.30, 0.45, 0.60, 1.00, 2.00]:
        tau, iq, d = c.compute(math.radians(60), v, limit_rate=False)
        print(f"{v:>11.3f}{d['w']:>8.3f}{tau:>10.3f}{iq:>9.3f}")
    print("\nSanity checks that must hold before this goes anywhere near hardware:")
    print("  - BARE RIG commands 0.000 at every angle (residual is negative)")
    print("  - velocity 0.106 (worst measured incidental motion) commands 0.000")
    print("  - no iq exceeds", f"{LIMIT_I_MAX:.1f} A")
    print("  - actuator 10 deg is outside the characterized range -> 0.000\n")


if __name__ == '__main__':
    _dry_run()