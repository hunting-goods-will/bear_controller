"""Controller v2 parameters: V2_ARCHITECTURE v1.4 in code (Nov 12, 2026).

One place for every number the v2 run loop (run_v2.py) and the v2 controller
(v2_control.py) use. Pure data and small pure functions: no bus, no I/O.
Change a value here only together with V2_ARCHITECTURE (the doc wins).

Units: torque N m (+ = up), angles in DEGREES unless the name ends in _rad,
vest angle = actuator angle + PHI_DEG.
"""
import math

import numpy as np

from main_controller.controller import KT, PHI_DEG, RIG_MGL

RIG = RIG_MGL                     # 1.1012 N m: rig gravity at horizontal

# ---- Spring tables S_L(vest), N m, Low zone, healthy unit (V2 §2) -------------------
VEST_GRID = np.arange(105.0, 171.0, 5.0)        # 105 ... 170
SPRING = {
    1: np.array([2.775, 2.716, 2.663, 2.574, 2.478, 2.302, 2.143, 1.990, 1.862, 1.687,
                 1.514, 1.375, 1.236, 1.097]),
    2: np.array([5.394, 5.153, 5.031, 4.930, 4.761, 4.495, 4.316, 3.994, 3.765, 3.510,
                 3.274, 3.139, 3.004, 2.862]),
    3: np.array([8.52, 8.17, 7.83, 7.485, 7.141, 6.786, 6.458, 6.126, 5.832, 5.509,
                 5.229, 5.071, 4.916, 4.754]),
}
# ---- Sliding friction f_L(vest), N m, from the healthy-unit sweeps at 0.1 rad/s -----
# [verified, sweep_tables method on raw-data, Nov 11]: L1 observer_sweep_bare_20261108_162028
# (110-160; held flat outside); L2 bare 20261108_181155 + 20261110_191001 (140-165),
# L2 plate 20261108_195616 below 140; L3 plate 20261108_201818 + 20261110_201024
# (125-165; held flat outside). 170 = the 165 value.
FRICTION = {
    1: np.array([0.92, 0.92, 0.92, 0.92, 0.94, 0.92, 0.92, 0.91, 0.90, 0.90, 0.92, 0.92,
                 0.92, 0.92]),
    2: np.array([1.31, 1.31, 1.35, 1.36, 1.35, 1.31, 1.29, 1.25, 1.20, 1.17, 1.18, 1.17,
                 1.16, 1.16]),
    3: np.array([1.34, 1.34, 1.34, 1.34, 1.34, 1.33, 1.29, 1.28, 1.22, 1.19, 1.20, 1.19,
                 1.20, 1.20]),
}
# Before the level is known the creep and the reading checks use the MEAN of the three
# tables [proposal, Nov 11; review #11]. The friction help in LIFT uses the identified
# level's table, or the LOWEST of the three while unidentified (under-assist side).
FRICTION_REF = (FRICTION[1] + FRICTION[2] + FRICTION[3]) / 3.0
LEVELS = (1, 2, 3)


def spring(level, vest_deg):
    """S_L at a vest angle (np.interp: held flat outside 105-170)."""
    return np.interp(vest_deg, VEST_GRID, SPRING[level])


def friction(level, vest_deg):
    """f_L at a vest angle; level None or 0 -> FRICTION_REF."""
    tab = FRICTION[level] if level in LEVELS else FRICTION_REF
    return np.interp(vest_deg, VEST_GRID, tab)


def friction_min(vest_deg):
    return min(float(np.interp(vest_deg, VEST_GRID, FRICTION[L])) for L in LEVELS)


def residual_model(A, level, vest_deg):
    """R(theta) = (A + RIG) sin(vest) - S_L(vest), N m (V2 §2)."""
    return (A + RIG) * math.sin(math.radians(vest_deg)) - float(spring(level, vest_deg))


def arm_value(u, level, vest_deg):
    """A_k^c = (u + S_c - RIG sin) / sin  (V2 §6.2)."""
    s = math.sin(math.radians(vest_deg))
    return (u + float(spring(level, vest_deg)) - RIG * s) / s


# ---- Current numbers (V2 §7). The cap is per run: 5.5 (first worn runs) or 8.0 A. -----
CAP_CHOICES_A = (5.5, 8.0)
TRIP_3X_ABOVE_CAP_A = 0.2          # over-current: cap + 0.2 A for 3 samples
TRIP_1X_ABOVE_CAP_A = 0.5          # or cap + 0.5 A once
LIMIT_ABOVE_CAP_A = 0.7            # firmware limit_i_max = cap + 0.7 (8.7 A at 8.0)
I_MAX_RESTORE_A = 5.5              # config.MAX_IQ, restored after a confirmed disable

# ---- Shaping (V2 §7) ------------------------------------------------------------------
RATE_UP_NM_S = 4.0                 # applied to the FINAL output (review #14, Nov 11)
RATE_DOWN_NM_S = 20.0
TAPER_ACT_LO_DEG, TAPER_ACT_HI_DEG = 90.0, 105.0   # = config MODE0 taper (vest 162-177)


def taper_up(act_deg):
    """Upward-torque joint-limit scale at an actuator angle (same as joint_limit_scale)."""
    return min(1.0, max(0.0, (TAPER_ACT_HI_DEG - act_deg) / (TAPER_ACT_HI_DEG - TAPER_ACT_LO_DEG)))


def effective_ceiling(vest_deg, cap_a, derate):
    """Command cap x KT x taper(at that vest angle) x thermal derate, N m (V2 §6.1 check 5).
    Pass the vest angle at the creep's TOP."""
    return cap_a * KT * taper_up(vest_deg - PHI_DEG) * derate


# ---- Sensing (V2 §3) ------------------------------------------------------------------
VEL_FILTER_S = 0.010               # first-order filter on the position derivative
ACC_FILTER_S = 0.010               # and on the derivative of that
PERIOD_S = 0.002                   # 500 Hz

# ---- States (V2 §4) --------------------------------------------------------------------
IDLE, LIFT, SETTLE, HOLD, LOWER, FAULT = 'IDLE', 'LIFT', 'SETTLE', 'HOLD', 'LOWER', 'FAULT'
STATES = (IDLE, LIFT, SETTLE, HOLD, LOWER, FAULT)
STATE_CODE = {s: i for i, s in enumerate(STATES)}

LATCH_ON_RAD_S, LATCH_OFF_RAD_S, LATCH_MIN_ON_S = 0.15, 0.08, 0.150
STILL_SPAN_DEG, STILL_WINDOW_S, HOLD_MIN_VEST_DEG = 0.1, 0.3, 120.0
SETTLE_TIMEOUT_S = 1.0
LOWER_BELOW_ANCHOR_DEG = 2.0
LOWER_SPEED_RAD_S, LOWER_SPEED_S = 0.15, 0.050
UNLOAD_ABOVE_CREEP_DEG = 1.0       # theta > theta_a + creep amplitude + this
UNLOAD_SPEED_RAD_S, UNLOAD_SPEED_S = 0.15, 0.100
UNLOAD_LIFT_BLOCK_S = 1.0

# ---- Hold law (V2 §5) -----------------------------------------------------------------
KP, KI, KD = 40.0, 60.0, 1.5
INTEG_SAT_BELOW_DEG = 1.0          # the integral's input saturates this far below the setpoint
                                   # (V2 v1.4; v1.3 froze it beyond 0.3 deg: no pick-up)
HOLD_CEILING_MARGIN_NM = 1.0       # after identification: R_hat + f + 1.0
CREEP_HZ = 0.25
CREEP_DEG_PER_NM = 1.0             # amplitude = 1.0 deg x f_ref / 1 N m (as --creep-scale-f 1)
CREEP_DEG_RANGE = (0.5, 2.0)
CREEP_FF_GAIN = 0.7                # x f_ref [proposal, Nov 11; bench 0.8 x the true f]
CREEP_CYCLES_DISCARD, CREEP_CYCLES_READ = 3, 3   # read = cycles 4-6 (~24 s)
CREEP_EASE_CYCLES = 2.0            # amplitude and feedforward ease in over cycles 1-2 (v1.4; was
                                   # cycle 1): leaves the first seconds quiet for the load pick-up

# ---- Reading checks (V2 §6.1) ---------------------------------------------------------
WINDOW_VEST_DEG = (150.0, 170.0)
TRAVEL_MIN_DEG, TRAVEL_ASYM_MAX_DEG = 0.5, 0.5
SPLIT_BAND = (0.25, 1.4)           # x f_ref
CYCLE_AGREE_NM = 0.1
HEADROOM_NM = 0.15

# ---- Learner (V2 §6.3-6.5) ------------------------------------------------------------
WIN = 6
BIN_DEG = 4.0
SEP_DEG = 10.0
TOL_NM = 1.5
DROP_NM = 2.0
OFFSET_NM = 0.4                    # arm units, envelope term
FAST_MARGIN_READING_NM = 0.25      # reading units, single-reading term (C1)
A_CAP_NM = 28.0
A_LOW_NM = 6.0                     # unidentified: A_low on the L3 table

# ---- Assist (V2 §7) -------------------------------------------------------------------
ALPHA_STEPS = (0.0, 0.10, 0.20, 0.35, 0.50)    # 0 = hold-only session (no LIFT torque)
KF = 0.75
G_LOW_VEST_DEG = (95.0, 105.0)     # model term tapered 0 -> 1
BLEND_HI_RAD_S = 0.45              # w(theta_dot): 0 at LATCH_OFF, 1 at this (Phase 1 V_HI)

# ---- Trips (V2 §8; every trip -> FAULT: zero, confirmed disable, run ends) -------------
TRIP_SPEED_UP_RAD_S = 1.0          # every state except LIFT, command > 0
TRIP_ACC_UP_RAD_S2, TRIP_ACC_COUNT = 120.0, 3   # LIFT, SETTLE, LOWER, command > 0
TRIP_LOOP_DT_S = 0.010
RUN_CAP_WORN_S = 60.0