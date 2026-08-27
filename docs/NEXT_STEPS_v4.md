# NEXT_STEPS_v4

Ordered by real dependency, not by size. Companion to `CONTROL_DESIGN_STATE_v4.md`.

**Scope decision made:** Option A stops at one configuration (Level 1 spring,
Low activation zone). The full 4×3 matrix is **not** being characterized.
Rationale in Group D.

---

## Group A — Before any further hardware work

Small, and two of them are genuine safety items.

### 1. Fix `MAX_ANGLE`

`config.py` currently has `MAX_ANGLE = math.radians(125)`. The measured hard
stop is ~119°. **A firmware position limit set beyond the physical stop can
never fire**, so Position Mode currently has no backstop.

Set to `math.radians(118)`. Nothing in current use needs more — sweeps top out
at actuator 108, and `TORQUE_MODE_MAX_ANGLE` is 105.

Also correct the stale comment: it reads `# 1.8326 rad` (= 105°) next to a
different value.

### 2. Document the re-homing procedure

Resting position varied **114.5–122.6°** across sessions — an 8° spread on what
should be a fixed mechanical stop. Cause identified: repeated bracket
disassembly moved the motor relative to the frame. A homing reset restored
~119°.

This matters because **`phi = 72°` is referenced to the hard stop.** If the
reference moves, every vest angle in the dataset moves with it, and the spring
table silently shifts.

Add to the repo README: *re-run homing calibration after any bracket
disassembly, and record the resting position at the start of every session as a
drift check.*

### 3. Commit the raw CSVs

The bidirectional sweep, five validation runs, velocity profile, and successful
human run are the evidence base for every number in `controller.py`. If space is
a concern, `gzip` (text compresses ~10×) or push to a branch — do not delete.
The velocity profile data is the most expensive to recollect: it needs a human,
a session, and PI approval.

### 4. Fix or annotate the two broken analyzers

`analyze_assist_run.py` section 4 and `analyze_human_run.py` section 2 both
produce confidently wrong output. Either fix them or add a header warning. Left
as-is they will mislead whoever reads them next, including you in three months.
Details in `CONTROL_DESIGN_STATE_v4.md` §10.

---

## Group B — Option B foundations

### 5. Verify `KT` empirically — **blocking**

`KT = 0.67 Nm/A` is the datasheet value and has never been measured on this
unit.

**In Option A this was a scale factor on the output.** A 10% error meant 10%
less assist — harmless.

**In Option B it is the sensor calibration.** Current *is* the torque
measurement. A 10% `KT` error becomes a 10% error in every disturbance estimate
the impedance loop acts on, and that error feeds directly into a closed loop
rather than an open one.

This has been deferred since early in the project. It should not survive into
Option B.

Method: fish-scale at a known radius, several current levels, both directions to
cancel friction. An afternoon on the bench, no human, no new hardware.

### 6. Measure arm inertia `J`

Impedance control needs a `J*alpha` term. The Option A controller was
quasi-static and had none.

Loaded rig estimate: ~0.139 kg·m² with the test mass at 274 mm, against
~0.006 kg·m² bare. Human arm inertia is unmeasured.

Method: pendulum ring-down with the actuator disabled, or step-torque response
on the bench.

### 7. Characterize the sensing deadband directly

`CONTROL_DESIGN_STATE_v4` §8 states ±0.80 Nm inferred from the friction
measurement. Option B's viability depends on it, so measure it rather than
inferring: apply known external torques at increasing magnitude and record the
threshold at which `present_iq` responds.

**If the deadband is materially worse than 0.80 Nm, that is a finding that could
reshape Option B's design** — better known before building than after.

### 8. Reuse, do not rebuild

These carry into Option B **unchanged**:
- Friction model (§4) — impedance control still overcomes the same friction
- Blend, hysteresis latch, rate limit — all three fixed real observed failures
- Full safety layer stack (§7)
- **`validate_model.py` methodology** — the harness that proved Option A's
  physics. Option B needs proving too, and this is the pattern.

These are **not needed** by Option B:
- `tau_spring(theta)` table
- Wearer arm mass estimate
- `phi` for the gravity term (still useful for joint limits and logging)

---

## Group C — Deferred, documented

### 9. Viscous friction above 0.5 rad/s
Only the Stribeck falling branch was measured, over 0.05–0.5 rad/s. Real lifts
reach 5.5. The minimum and the viscous rise are both unmeasured.
`limit_velocity_max` is 1.0, so higher sweeps need that raised first.

### 10. Angle-dependence of friction
Varied 0.777–0.959 across angle within one run (±11%). Treated as constant.

### 11. Between-session friction reproducibility
0.82 vs 0.89 at the same 0.10 rad/s in different sessions — larger than
within-session scatter. Possibly temperature or wear.

### 12. Rig CoM measurement
55 mm is an estimate. Sensitivity ±0.22 Nm at ±20%. Low priority; a balance-point
measurement takes minutes.

### 13. Bracket redesign
The bracket stop leaves the lowest ~10° of the wearer's stroke both unassisted
and unmeasurable — and that is where the arm is heaviest. Redesign was already
under consideration. **Any redesign requires re-measuring `phi` and re-running
homing.**

### 14. Downward spring-cancellation assist
The descending data exists (`lower_controlled` p10 −0.912, min −1.379 rad/s) and
the same blend structure works with the sign flipped. Not started.

### 15. `MAX_IQ` vs `limit_i_max` reconciliation
`config.MAX_IQ = 6.0` against firmware `limit_i_max = 5.5`. Given the torque
shortfall in §6, whether the limit can be raised against thermal headroom is
worth a deliberate look — the PB02's continuous vs peak rating is the real
constraint, not an arbitrary choice.

### 16. Loop rate and PREEMPT_RT
Measured 333 Hz consistently, zero iterations over 50 ms across every run. No
longer obviously blocking. `cyclictest` numbers still pending PI review.

### 17. `SAFETY_CHECKS_CONFIRMED` architecture
Committed as `True`. The gate belongs inside `BearInterface.enable()` so all
scripts are protected by construction rather than each remembering to check.

---

## Group D — For the PI

### 18. Scope: stopping Option A at one configuration
Communicated in the weekly update; confirm he accepts it.

**The argument:** Option B measures the disturbance directly rather than
predicting it from a characterized spring curve and an estimated arm mass, so it
should generalize across all twelve configurations without per-configuration
characterization. The one term in Option A that cannot be validated on a human
is exactly the term Option B removes.

**The risk, stated plainly:** if Option B's deadband (item 7) proves
disqualifying, the fallback is Option A across all twelve configurations — and
that matrix would then be needed after all. Worth naming now rather than
discovering it.

### 19. Partial-assist scope
The actuator supplies at most ~20% of residual gravitational torque for a
heavier wearer; the successful run delivered ~9%. Confirm this is the intended
scope rather than a shortfall to engineer around.

### 20. Spring peak vs manual specification
Measured peak support at **vest 82–92**; the manual quotes 105° for the Low
setting. First data on both sides of the peak. Worth raising — it is a
discrepancy with the stock specification on an actuator-modified vest, and this
project has now found three of those (180° ROM assumption, mass/CoM figures,
this).

---

## Suggested order

1. Item 1 (`MAX_ANGLE`) — five minutes, safety
2. Item 3 (commit CSVs) — five minutes, irreplaceable
3. Item 2 (re-homing procedure) — documentation
4. **Item 5 (`KT` verification)** — an afternoon, blocking for Option B
5. Item 7 (deadband) — determines whether Option B is viable as designed
6. Item 6 (inertia) — needed for the impedance law itself
7. Then build

Items 5 and 7 are both cheap, both bench-only, and both can invalidate Option B's
design if they come back badly. Do them before writing the controller, not after.
