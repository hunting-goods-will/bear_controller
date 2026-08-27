# CONTROL_DESIGN_STATE_v4

**Status: Option A complete and demonstrated on a human wearer.**
Supersedes v3. Everything here is either measured, or explicitly flagged as an
estimate. Where a number is an estimate, that is stated at the point of use.

---

## 1. What was built

A feedforward gravity-compensation controller for the EksoVest shoulder
exoskeleton, augmented with a Westwood Robotics BEAR PB02 actuator.

**Control law:**

```
tau_cmd = w(theta_dot) * [ alpha * max(0, tau_residual) + k_f * tau_friction ]

tau_residual = tau_gravity_total(theta_vest) - tau_spring(theta_vest)
theta_vest   = theta_actuator + phi
```

The vest's passive spring already supplies part of the lift. The actuator
supplies only the *difference* between total gravitational demand and what the
spring provides, and only during deliberate upward movement.

Each term and why it has the form it does:

| Term | Purpose |
|---|---|
| `w(theta_dot)` | Smooth blend, not a threshold. Prevents chatter and torque discontinuity. Zero for all negative velocity, so downward strokes need no separate branch. |
| `alpha` | Assist ratio. Bounded by actuator authority — see §6. |
| `max(0, ...)` | Never command negative. On a bare rig or near the top of travel the spring over-provides; commanding the negative residual would drive the arm *down* against the user. |
| `k_f * tau_friction` | Friction feedforward, deliberately under-compensated. Over-compensating self-drives: assist overcomes the friction holding the arm, producing motion, triggering more assist. |

---

## 2. Measured constants

| Parameter | Value | Provenance |
|---|---|---|
| `phi` (frame offset) | **72.0°** | Inclinometer: 342° at bracket hard stop, 270° at vest true zero. Independently corroborated — see §5. |
| `RIG_MGL` | **1.1012 Nm** | 2.041 kg at 0.055 m. **CoM is an estimate**, not a balance-point measurement. Sensitivity ±0.22 Nm at ±20%, small relative to other terms. |
| `KT` | **0.67 Nm/A** | **Datasheet. Never verified on this unit.** Multiplies every torque figure in this document. |
| `TAU_FRICTION` | **0.80 Nm** | See §4. This is a simplification of a velocity-dependent function. |
| `MIN_ANGLE` / hard stop | 5° / ~119° actuator | Measured. |

### Spring table — Level 1 spring, Low activation zone

Vest degrees → Nm. Continuous bidirectional sweep at 0.10 rad/s, bare rig, 5°
bins, ~290 samples per bin per direction. Friction cancelled by averaging the
two sweep directions.

| vest° | Nm | | vest° | Nm |
|---|---|---|---|---|
| 81.8 | 3.162 | | 132.0 | 2.273 |
| 87.1 | 3.034 | | 137.0 | 2.048 |
| 92.1 | 3.061 | | 142.0 | 1.903 |
| 97.0 | 3.011 | | 147.0 | 1.723 |
| 102.0 | 2.924 | | 152.0 | 1.595 |
| 107.1 | 2.876 | | 157.0 | 1.490 |
| 112.0 | 2.745 | | 162.0 | 1.373 |
| 117.0 | 2.638 | | 167.0 | 1.302 |
| 122.0 | 2.539 | | 172.0 | 1.192 |
| 127.0 | 2.461 | | 176.6 | 1.090 |

**Caveats:**
- `vest 81.8` came from a bin with only 29 down-leg samples against 322 up.
  Friction cancellation requires balanced legs. Weakest point in the table, and
  also the most useful one for coverage.
- `vest 87.1 / 92.1` are slightly non-monotonic (3.034 then 3.061). Gap is
  0.027 Nm, inside run-to-run scatter.
- **Below vest 82 is uncharacterized.** That is the bracket-stop region — a
  mechanical limit, not a data gap.
- Table coverage is **actuator 9.8° to 104.6°**.

**Notable finding:** the curve flattens across vest 82–92 rather than continuing
to climb, bracketing the spring's peak support at roughly **vest 82–92**. The
EksoVest manual quotes 105° for the Low setting. First data on both sides of it.

---

## 3. Blend, latch, rate limit

| Parameter | Value | Basis |
|---|---|---|
| `V_ON` | 0.15 rad/s | Above worst measured incidental motion (0.106) |
| `V_OFF` | 0.08 rad/s | Hysteresis gap |
| `V_HI` | 0.45 rad/s | Below slow-lift p90 (0.867) |
| `MIN_ON_TIME` | 0.15 s | Minimum engaged duration |
| `MAX_TAU_RATE` | 4.0 Nm/s | ~0.4 s from zero to full |

Band edges come from human velocity profiling: 70,596 samples at 333 Hz across
labelled conditions (static holds, incidental motion, slow/fast lifts, lowering,
simulated overhead work). Separation between intentional and incidental motion
was close to an order of magnitude — worst non-intentional 0.106 rad/s against
slow-lift sustained 0.867 rad/s p90. Assist engages within **0.15° of travel**
at threshold 0.15–0.30, with zero false triggers.

**Why hysteresis exists:** a *monitor* run — zero torque commanded, so the
controller could not have caused it — showed 100 latch transitions in 44 s, with
30 of 50 engaged segments under 100 ms. Human movement is not smooth around any
single value. A one-threshold gate stutters no matter how it is tuned.

**Why the rate limit exists:** without it the command steps to its full value in
one loop iteration (3 ms at 333 Hz). In the first live human run the command
jumped straight to a saturated 2.010 Nm on engagement. This was felt as a jerk
on the upward stroke and **broke the mounting flange**. The limit is symmetric,
so disengagement ramps down rather than dropping out. Emergency stops bypass it
— the loop's `finally` block writes `iq = 0` directly.

---

## 4. Friction — velocity-dependent, not constant

Measured across five bench validation runs, bare rig:

```
friction = 0.910 - 0.208 * v        R^2 = 0.97, residual max 0.009 Nm
```

| sweep velocity (rad/s) | friction (Nm) |
|---|---|
| 0.05 | 0.8902 |
| 0.10 | 0.8942 |
| 0.20 | 0.8758 |
| 0.35 | 0.8327 |
| 0.50 | 0.8048 |

**Friction decreases with speed** — the falling branch of a Stribeck curve as
the contact leaves boundary lubrication. This is the opposite of viscous
damping.

**Do not extrapolate the line.** It predicts negative friction above 4.4 rad/s,
which is impossible. A real Stribeck curve flattens to a minimum and rises again
as viscous drag dominates. Only the falling branch was measured, over
0.05–0.5 rad/s; real lift speeds reach 5.5 rad/s.

`TAU_FRICTION = 0.80` is the value at the fastest velocity actually tested. It
errs low, which is the stable direction.

**Additional scatter not captured by the constant:**
- Within one run, friction varied **0.777–0.959** across angle (±11%).
- Between sessions at the same 0.10 rad/s, **0.82 vs 0.89**.

**Terminology caution:** what was measured is *direction-dependent torque*.
It lumps gearbox friction, spring/gas-strut hysteresis, and linkage friction —
the experiment cannot separate them. Backlash was ruled out quantitatively: with
`dtau_spring/dtheta ≈ 0.027 Nm/deg`, faking 0.67 Nm would need ~25° of lost
motion. For a paper, "gearbox friction" is a claim not yet earned.

**Why the earlier 0.67 Nm figure was wrong:** it came from settle points, where
the position loop stops as soon as friction can hold the arm. That records a
*lower bound* on stiction, not a measurement of it.

---

## 5. What is validated, and how

### Bench validation harness (`validate_model.py`)

The actuator drives the arm at constant velocity in Position Mode while the
model computes and logs **without applying anything**. In Position Mode at
constant slow velocity the actuator supplies exactly:

```
tau_act = tau_gravity(theta) - tau_spring(theta) +/- tau_friction
```

so `measured residual = iq_measured * KT`, and sweeping both directions cancels
friction. This is a comparison of prediction against **directly measured
actuator current** — independent of the sweep that produced the spring table.

**Result: mean error +0.126 Nm, max 0.211 Nm, across eight angle bins spanning
70° of travel, in five runs at a 10× velocity spread.**

- **No angle trend** in the error → rules out `phi` and the spring table as
  error sources.
- Error is **additive, not proportional** (+0.132 at predicted −1.902, +0.131 at
  −1.135) → rules out `KT` scaling error.
- Per-bin error reproducible to **±0.006 Nm across five independent runs**.

That per-bin structure was the spring table's own measurement error, and
correcting the table against it **independently made the curve monotonic**,
removing a physically implausible bump that nothing in the validation runs knew
about. Independent corroboration that both the correction and `phi` are real.

### What validation does NOT cover

**The human arm mass estimate cannot be validated with this hardware.** Holding
a human arm at horizontal requires ~13 A against a 5.5 A limit — Position Mode
would saturate and drop. There is no bench-style characterization pass for a
wearer.

**User feel cannot validate a physics model.** A wrong model that pushes upward
at roughly the right moments feels identical to a correct one.

---

## 6. Human result — first successful run

Wearer 122.5 kg, 1.778 m. `alpha = 0.06`, `k_f = 0.75`.
19,961 samples over 60 s at 333 Hz.

| Check | Result |
|---|---|
| Engagement | 35% of samples, actuator 9.8–95.7° |
| Peak torque | **1.691 Nm / 2.524 A** (predicted 1.694 / 2.54) |
| Angle variation | 1.669 → 1.124 Nm across the stroke |
| Saturation | 0 samples |
| Rate limiter | max 4.03 Nm/s against 4.0 limit, 0 leaks |
| Velocity rejects | 0 of 19,961 at 3.81 rad/s peak |
| Latch | 66 transitions, p50 segment 492 ms, min 150 ms |
| Thermal | winding 26–27 °C, powerstage 29–31 °C |
| Zero on descent / while still | Confirmed |

Subjective: noticeable assistance on the upward stroke, no jerk.

### The defensible claim

> Feedforward gravity compensation with an empirically characterized spring
> model and friction correction. The model was validated on a bench rig to
> ±0.13 Nm against directly measured actuator current. In human testing the
> controller delivered angle-varying assist tracking the characterized curve,
> peaking within 0.2% of prediction, with no jerk, no stutter, and correct
> directional gating. Assist magnitude was limited to ~9% of the wearer's arm
> weight by actuator torque authority.

### Torque authority — the binding constraint

For a 122.5 kg wearer, Winter's tables give a 6.12 kg arm at 338 mm →
**20.29 Nm**. Residual at vest 90 is **18.34 Nm**. Actuator ceiling at 5.5 A is
**3.685 Nm**.

**The actuator can supply at most ~20% of the residual. The run delivered ~9%.**

This is inherently a partial-assist, torque-limited system. `alpha` is bounded by
actuator authority — chosen so peak current stays inside the 3.0 A clamp — with
wearer mass as an input. It is *not* derived from weight.

**Roughly half the commanded torque is the constant friction term**
(`k_f * TAU_FRICTION = 0.60 Nm`). The angle-varying part is 1.07 → 0.52 Nm. This
is a design choice, not a law — `k_f` could be zero, though it shouldn't be.

---

## 7. Safety architecture

| Layer | Detail |
|---|---|
| Torque rate limit | 4.0 Nm/s, symmetric |
| Hysteresis latch | 0.15 on / 0.08 off, 150 ms min-on |
| Velocity sanity check | Reported register vs position derivative over ~15 ms; >1.0 rad/s disagreement feeds zero velocity to the controller |
| Joint limit taper | **Asymmetric** — only the top limit is a hazard for positive torque |
| Thermal derating | `max(winding, powerstage)`, matching firmware fault logic |
| Hard current clamp | 3.0 A, well under the 5.5 A limit |
| Loop period guard | Abort above 50 ms |
| Velocity abort | 12 rad/s |
| Run duration cap | 60 s (human), 120 s (bench) |
| Park before disable | Bench only — the wearer supports their own arm |

**FIRMWARE POSITION LIMITS DO NOT APPLY IN TORQUE MODE.**
`limit_position_min/max` are enforced by the firmware's *position* loop. In
direct torque mode there is no position loop. The software taper is the only
thing preventing a positive command from driving into the top hard stop.
`TORQUE_MODE_MAX_ANGLE = 105°` is defined explicitly in the run script, not
imported from `config.py`.

The taper is asymmetric on purpose: commanded torque is always positive, so
upward torque moves the arm *away* from the bottom limit.

---

## 8. Known limitations and unvalidated assumptions

1. **`KT = 0.67 Nm/A` is a datasheet value**, never measured on this unit. It
   multiplies every torque figure in this document. **This becomes critical in
   Option B**, where current *is* the sensor rather than just an output scale.
2. **Wearer arm mass is an anthropometric estimate.** Winter's tables scale arm
   mass linearly at 5% of body mass — reasonable for lean mass, less so across
   body compositions where the difference is largely trunk.
3. **Rig CoM (55 mm) is an estimate**, not a balance-point measurement.
4. **Friction is treated as constant** in the controller despite being measured
   as velocity- and angle-dependent.
5. **Viscous friction unmeasured above 0.5 rad/s.** Real lifts reach 5.5.
6. **One configuration of twelve** characterized (Level 1 spring, Low zone).
7. **Below actuator 10° the controller commands zero** — bracket-stop region.
   The wearer lifts from the bottom, so the first ~10° of every stroke is
   unassisted.
8. **Sensing deadband ±0.80 Nm.** Disturbances below stiction are absorbed
   without moving `present_iq`. At a hand ~0.6 m out that is ~140 g of payload.
   **Directly limits Option B.**
9. **Homing offset drifts if the bracket is disassembled.** Resting position
   varied 114.5–122.6° across sessions during a period of repeated bracket
   removal; stable otherwise, and a homing reset restored ~119°. **Re-home after
   any bracket disassembly** — `phi` is referenced to the hard stop, so a moved
   reference moves every vest angle in the dataset.

---

## 9. Repository state

**Core:**
- `main_controller/controller.py` — model, blend, latch, rate limit. Pure, no
  hardware imports. `python3 controller.py` runs a dry-run command table.
- `main_controller/config.py`, `bear_interface.py`

**Characterization and validation:**
- `useful_tools/bidirectional_sweep.py` — separates spring from friction
- `useful_tools/validate_model.py` — bench harness; also extracts spring values
- `useful_tools/extract_spring.py` — offline extraction from a saved CSV
- `useful_tools/position_hold_characterization.py` — historical, produced the
  original 12-point dataset

**Human:**
- `useful_tools/velocity_profile.py` — read-only, actuator disabled
- `useful_tools/analyze_velocity_profile.py` — latency/false-trigger tradeoff
- `useful_tools/run_assist_human.py` — `--monitor` / `--live`
- `useful_tools/analyze_human_run.py` — post-run diagnostics
- `useful_tools/analyze_assist_run.py` — general assist-run diagnostics

**Config as of this document:** `MAX_ANGLE = radians(125)`, `MIN_ANGLE =
radians(5)`, `SAFETY_CHECKS_CONFIRMED = True`.
**`MAX_ANGLE` is above the ~119° hard stop and therefore provides no firmware
backstop even in Position Mode. See NEXT_STEPS_v4 item 1.**

**Raw CSVs are the evidence base for every number in `controller.py`.** Without
them the spring table and the Stribeck fit become unreproducible assertions.

---

## 10. Analyzer caveats

Two diagnostics in the repo produce misleading output and should not be trusted
without reading the code:

- **`analyze_assist_run.py` section 4 ("positive feedback")** is a selection
  artifact. It defines onset as the blend crossing threshold, which by
  construction occurs early in the acceleration phase of a hand-driven lift, so
  acceleration is always higher after. It reports "POSITIVE FEEDBACK CONFIRMED"
  on monitor runs where zero torque was commanded. Sections 1, 2, 3 and 5 are
  sound.
- **`analyze_human_run.py` section 2** averages over all engaged samples,
  including partial-blend and mid-ramp ones, which dilutes toward zero and
  destroys the correlation. Filter to `blend_w >= 0.99` and settled torque.
  Note also that comparing commanded torque against the logged `tau_residual` is
  **circular** — it recomputes the controller's arithmetic from its own inputs.
  The human run cannot validate the physics; §5 is where that happened.

Its "commanded while moving DOWN" flag also fires spuriously: the rate limiter
ramps down over ~0.4 s, so nonzero torque during early descent is correct.
