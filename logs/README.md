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

### `phase2/observer_csvs/` — Phase 2 observer excitation runs
Producer: `useful_tools/observer_excitation.py`. Actuator drives the arm in
Position Mode (sweep: constant-velocity down/up legs; sine: smoothstep-ramped
sinusoid) and logs goal, position, velocity register, present_iq and
temperatures every loop, with `loop_dt`. Every file starts with `#` header
lines (argv, all limits, set/readback of gains and limits, park target) and
ends with `# END:` / `# PARK:` outcome lines; read with
`pd.read_csv(path, comment='#')`. The `# ... started` header line (time of
enable) is the authoritative start time. Dated 2026-09-30.

| File | Notes |
|---|---|
| `observer_sweep_bare_20260930_144400_ABORTED.csv.gz` | Bare rig, sweep at 0.10 rad/s, **aborted** on the down leg: \|present_iq\| > 4.0 A for 3 samples (4.095 A at act 59.6°). The 4.0 A threshold came from a friction-free estimate; normal bare down-leg current is 3.2–4.8 A (August `model_validation` runs), so it was raised to 5.0 A (trip 5.3 A). Run with the old park: parked LOW at 24° (up to −4.8 A pulling against the spring), and on release the bare arm sprang into the top stop. The park now goes HIGH (110°) on a bare rig. |
| `observer_sweep_bare_20260930_150800.csv.gz` | Bare rig, sweep at 0.10 rad/s, **complete**, with the fixed settings: current abort 5.0 A ×3 / trip 5.3 A, sweep 100→30→100°, park HIGH at 110°. Peak \|present_iq\| 4.62 A. |
| `observer_sweep_wrench_20260930_162234.csv.gz` | Light wrench, **0.711 kg @ 0.1651 m** (per-row `arm_mass_kg`/`arm_com_m`; "wrench" in the name only means mass ≠ 0), sweep at 0.10 rad/s, **complete**, same limits as `150800`. Park chosen from the torque model: net = τ_spring 3.021 − τ_gravity 2.240 = +0.781 Nm at vest 96°, so the spring wins and it parked HIGH at 110°. Peak \|present_iq\| 2.75 A (approach leg, act 107°). |

### `phase2/latency/` — serial comm-budget timing
Producer: `useful_tools/measure_comm_budget.py` (added on `main` in
`8738d4d`). Read-only: actuator never enabled; the only register written is
`goal_iq = 0.0`. Times 5000 calls each (+20 warmup) of `ping`,
`get_present_position`, `get_state` (4 single reads), `bulk_read_3`
(pos/vel/iq), `bulk_rw_3_goal_iq0` and `bulk_rw_state4_goal_iq0` (get_state's
four registers), with a `--gap-ms` sleep between calls outside the timed
region. One row per call: `method, i, dt_us, ok, err, fresh_boot, sched`.
`#` header lines record config-register readback, ping scan,
torque_enable/goal_iq at start and end, present_position vs. the 2.0328 rad
top stop, scheduler/affinity, xhci IRQ counts, latency_timer, and the stats
table; read with `pd.read_csv(path, comment='#')`. Both runs log
`git: 501db9b dirty: yes` because the script was uncommitted at the time.
The committed `8738d4d` file is the one that ran: it was last modified at
20:08, before either run. Dated 2026-10-06.

| File | Notes |
|---|---|
| `comm_budget_20261006_201203.csv.gz` | SCHED_FIFO 80, CPU 3, `--gap-ms 0.5`, fresh_boot yes. **Complete**, 0 failures and 0 non-0x80 error bytes on every method; torque_enable 0 and goal_iq 0.0 at start and end. Medians: single-transaction methods 489.5–489.7 µs, get_state 3487.9 µs. Max ≤ 1397.9 µs except get_state 4273.3 µs. present_position 2.0214 rad (115.82°), −0.65° from the expected top stop. |
| `comm_budget_20261006_203436.csv.gz` | Same as `201203` but `--gap-ms 0.25`. **Complete**, 0 failures, 0 non-0x80 error bytes. Medians: single-transaction 739.5–740.2 µs, get_state 3738.3 µs. Max: ping 2742.7, bulk_rw_state4 4119.4, get_state 5736.2 µs, others ≤ 1423.6 µs. |

In both runs, single-transaction median + gap ≈ 1 ms, and get_state's median
≈ that median + 3 × 1 ms. That fits replies being paced by a 1 ms cycle
(FTDI `latency_timer` = 1 ms is logged in the header), so per-call medians
here reflect the phase within that cycle, not the actuator's turnaround. No
SCHED_OTHER (plain) run yet.

### Other

| File | Notes |
|---|---|
| `Encoder_values.txt` | Standalone note recording an encoder homing-offset value; not a timestamped run |
