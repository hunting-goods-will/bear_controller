# Raw run data — `raw-data` branch

This branch preserves raw bench/field logs for the Eksovest BEAR-actuator
controller project. It does not track `main` — it exists solely to hold
`logs/` at whatever state it was in when each snapshot was taken, so `main`
can stay lean (source + generated summaries only).

Retrieve any file with:

```
git show raw-data:<path> | gunzip
```

e.g. `git show raw-data:logs/human_live_csvs/human_live_20260821_002111.csv.gz | gunzip > run.csv`

## File index

### `human_live_csvs/` — live-torque human runs (assist actually applied)
Producer: `useful_tools/run_assist_human.py --live`. Dates 2026-08-18 and 2026-08-21.

| File | Notes |
|---|---|
| `human_live_20260818_223842.csv.gz` | 2026-08-18 session |
| `human_live_20260818_223956.csv.gz` | 2026-08-18 session |
| `human_live_20260818_224009.csv.gz` | 2026-08-18 session |
| `human_live_20260818_224632.csv.gz` | 2026-08-18 session |
| `human_live_20260821_001640.csv.gz` | 2026-08-21 session |
| `human_live_20260821_002111.csv.gz` | 2026-08-21 session |

### `human_monitor_csvs/` — zero-torque monitor-mode human runs
Producer: `useful_tools/run_assist_human.py --monitor`. Logs what would be
commanded at zero applied torque — same date range as the live runs above.

| File | Notes |
|---|---|
| `human_monitor_20260818_223506.csv.gz` | 2026-08-18 session |
| `human_monitor_20260821_000840.csv.gz` | 2026-08-21 session |
| `human_monitor_20260821_001025.csv.gz` | 2026-08-21 session |

### `model_validation_csvs/` — scripted open-loop model validation
Producer: `useful_tools/validate_model.py`. Actuator drives the arm at
constant velocity in Position Mode while the model computes and logs without
applying anything; used to compare predicted vs. measured residual torque.
Dated 2026-08-20.

| File | Notes |
|---|---|
| `model_validation_20260820_234335.csv.gz` | 2026-08-20 session. **Anomalous, see below** — kept here for reference but removed from `main` and excluded from generated summaries. |
| `model_validation_20260820_235718.csv.gz` | 2026-08-20 session. Clean bare-rig validation run. |

**Missing data**: Five validation runs were made on 2026-08-20/21 at 0.05,
0.10, 0.20, 0.35, and 0.50 rad/s. Only two CSVs were retained, both at
~0.08–0.10 rad/s, and their logged tau_spring values match neither the
original nor the final spring table at every angle, so it is unclear whether
they are two of the five runs or later re-runs. No retained data exists at
0.05, 0.20, 0.35, or 0.50 rad/s. The friction-vs-velocity fit and the
±0.006 Nm reproducibility claim in CONTROL_DESIGN_STATE_v4.md are not
reproducible from retained raw data; re-collection is planned.

**Update (repo cleanup, 2026-09-17)**: of the two retained CSVs, only
`model_validation_20260820_235718.csv.gz` is actually consistent with a
bare-rig run described above — its logged `tau_gravity` matches
`RIG_MGL` alone, and comparing its logged `residual_measured` against
`tau_gravity − tau_spring` (both legs averaged to cancel friction) gives a
clean mean error of +0.009 Nm, max 0.142 Nm, in line with
CONTROL_DESIGN_STATE_v4.md's aggregate "+0.126 Nm mean, 0.211 Nm max"
claim.

`model_validation_20260820_234335.csv.gz` is a **different, unresolved
experiment**: its logged `tau_gravity` (~5.1 Nm at vest 90°) implies a
~1.5 kg loaded arm, matching `validate_model.py`'s wrench-load
"falsifiable prediction" test, not the bare-rig sweep. That test's own
docstring states the prediction fails if `iq_measured` comes back negative
("either the load is not what we think or the sign convention is
inverted") — and in this file `iq_measured` is negative throughout,
opposite in sign from what `tau_gravity − tau_spring` predicts in 15 of 17
angle bins. This was apparently never diagnosed or resolved. The file is
kept on this branch for that future diagnosis, but was removed from `main`
(git rm, not `git filter` — still in this branch's history) and is
excluded from `logs/summaries/` since its numbers aren't physically
meaningful as-is.

### `velocity_profile_csvs/` — velocity-threshold characterization sweep
Producer: `useful_tools/Velocity_profile.py`. Actuator disabled throughout;
collects labeled velocity distributions across trial conditions (intentional
lifts vs. incidental/quiet motion) used to tune `V_ON`/`V_HI`. Dated
2026-08-18.

| File | Notes |
|---|---|
| `velocity_profile_20260818_002929.csv.gz` | 2026-08-18 session |

### `spring_characterization/` — position-mode spring/friction sweeps
Two producers, both single-CSV-per-run:

| File | Producer | Notes |
|---|---|---|
| `bidirectional_sweep_20260817_203602.csv.gz` | `useful_tools/Bidirectional_sweep.py` | 2026-08-17, descend-then-ascend sweep; direction-separated to help cancel friction |
| `bidirectional_sweep_20260817_204335.csv.gz` | `useful_tools/Bidirectional_sweep.py` | 2026-08-17, second sweep |
| `position_sweep_FINAL_20260804_203506.csv.gz` | `useful_tools/position_hold_characterization.py` | 2026-08-04, single-direction only (friction and spring torque are confounded in this file — see `Bidirectional_sweep.py`'s own docstring) |

### `cyclictest/` — stock-kernel latency baselines
Idle/loaded PREEMPT-style latency measurements taken 2026-07-18. **Kernel was
stock at the time, not yet PREEMPT_RT** — these are baseline numbers, not
RT-kernel results.

| File | Notes |
|---|---|
| `idle_20260718_015245.log` | `cyclictest` histogram, idle system |
| `loaded_20260718_020500.log` | `cyclictest` histogram, loaded system |
| `loaded_v220260718_021707.log` | `cyclictest` histogram, loaded system, second run |
| `Initial_Cyclic_Values.txt` | Summary of `cyclictest -t` output across 500Hz/800Hz idle/loaded conditions |
| `README.md` | Short human-written note recording max-latency numbers for the above runs |

### Other

| File | Notes |
|---|---|
| `Encoder_values.txt` | Standalone note recording an encoder homing-offset value; not a timestamped run |
