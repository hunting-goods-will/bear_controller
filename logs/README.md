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
| `model_validation_20260820_234335.csv.gz` | 2026-08-20 session. Bare-rig run, **held-out** check of the current spring table. Its logged prediction used the wrong load constant (1.5025 kg); see the correction below. Summarized on `main` with gravity recomputed as bare rig. |
| `model_validation_20260820_235718.csv.gz` | 2026-08-20 session. Bare-rig run, **in-sample**: the current `TAU_SPRING_TABLE` was extracted from this run. |

**Missing data**: Five validation runs were made on 2026-08-20/21 at 0.05,
0.10, 0.20, 0.35, and 0.50 rad/s. Only two CSVs were retained, both at
~0.08–0.10 rad/s, and their logged tau_spring values match neither the
original nor the final spring table at every angle, so it is unclear whether
they are two of the five runs or later re-runs. No retained data exists at
0.05, 0.20, 0.35, or 0.50 rad/s. The friction-vs-velocity fit and the
±0.006 Nm reproducibility claim in CONTROL_DESIGN_STATE_v4.md are not
reproducible from retained raw data; re-collection is planned.

**Update (corrected 2026-09-25)**: both retained CSVs are bare-rig runs from
the same session at the same speed.

- `model_validation_20260820_235718.csv.gz`: logged `tau_gravity` matches
  `RIG_MGL` alone. Its error against its own logged spring values is
  +0.009 Nm mean, 0.142 Nm max. The current `TAU_SPRING_TABLE` was
  extracted from this run (all 20 entries identical), so it is **in-sample**:
  its near-zero error against the current table is fit residual, not
  predictive accuracy.
- `model_validation_20260820_234335.csv.gz`: the **held-out** check. It is
  **not anomalous**. It was logged while `validate_model.py` hardcoded
  `ARM_MASS_KG = 1.5025` at 0.2739 m, so its logged `tau_gravity` (~5.1 Nm
  at vest 90°, implied mgL 5.1384 Nm) and `residual_predicted` assume a
  wrench that was not fitted. Evidence that the rig was bare: its
  leg-averaged `KT*iq_measured` matches `RIG_MGL`-only gravity minus spring,
  and matches `235718` within 0.044 Nm at every 5° bin; a 1.5 kg load at
  274 mm would have shifted iq by about +6 A. With gravity recomputed from
  `RIG_MGL` alone, its error against the current table is **+0.021 Nm mean,
  0.054 Nm max, 17 bins**.

The 2026-09-17 version of this note called `234335` a "sign-mismatch
anomaly" (negative `iq_measured` against a positive loaded prediction).
That was a misdiagnosis: the sign mismatch came from the wrong load
constant in the logged prediction, not from the measurement. `234335` was
removed from `main` at that time; `main` now carries its summary
(`logs/summaries/model_validation_20260820_234335.txt`). To stop this
recurring, `validate_model.py` now requires `--arm-mass-kg` and
`--arm-com-m` and writes both into every CSV row.

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

### `cyclictest/` — scheduling-latency measurements, grouped by campaign

#### `cyclictest/2026-07_stock_initial/`
Idle/loaded stock-kernel latency measurements taken 2026-07-18. **Kernel was
stock at the time, not PREEMPT_RT**, so these are baseline numbers, not
RT-kernel results. The protocol differs from 2026-09 (100k cycles per
thread, different load), so they are not directly comparable with it.

| File | Notes |
|---|---|
| `idle_20260718_015245.log` | `cyclictest` histogram, idle system |
| `loaded_20260718_020500.log` | `cyclictest` histogram, loaded system |
| `loaded_v220260718_021707.log` | `cyclictest` histogram, loaded system, second run |
| `Initial_Cyclic_Values.txt` | Summary of `cyclictest -t` output across 500Hz/800Hz idle/loaded conditions |
| `README.md` | Short human-written note recording max-latency numbers for the above runs |

#### `cyclictest/2026-09_stock_vs_rt/`
Stock vs PREEMPT_RT comparison on the same 6.18.50 source, 800 Hz interval,
10-minute idle and loaded runs. `README.md` here is identical to the one on
`main` and holds the results and percentile table.

| Folder / file | Notes |
|---|---|
| `README.md` | Protocol, results for both kernels, and the stock-vs-RT comparison |
| `stock_6.18.50/` | Stock `6.18.50+rpt-rpi-v8`, recorded 2026-09-24: idle and loaded histograms, conditions snapshots, and a README with the full test record (commands, clock lock, power, background processes, percentiles) |
| `rt_6.18.50-v8-rt1/` | PREEMPT_RT `6.18.50-v8-rt1`, recorded 2026-09-25: idle and loaded histograms, stress-ng log, four conditions snapshots, and a README with the full test record, results interpretation and procedural differences from stock |

### Other

| File | Notes |
|---|---|
| `Encoder_values.txt` | Standalone note recording an encoder homing-offset value; not a timestamped run |
