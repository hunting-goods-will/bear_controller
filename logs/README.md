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
enable) is the authoritative start time. Dated 2026-09-30 to 2026-10-15.

Provenance: observer_excitation.py logs no git hash. The Oct 14–15 sweeps
have the 17-column schema of the committed `501db9b` version. The `jsine`
runs have the 20-column schema (adds `t_pos, t_vel, t_iq`) of the version
committed later in `94a319f`; the file was last modified Oct 15 21:42,
before the first sine run.

| File | Notes |
|---|---|
| `observer_sweep_bare_20260930_144400_ABORTED.csv.gz` | Bare rig, sweep at 0.10 rad/s, **aborted** on the down leg: \|present_iq\| > 4.0 A for 3 samples (4.095 A at act 59.6°). The 4.0 A threshold came from a friction-free estimate; normal bare down-leg current is 3.2–4.8 A (August `model_validation` runs), so it was raised to 5.0 A (trip 5.3 A). Run with the old park: parked LOW at 24° (up to −4.8 A pulling against the spring), and on release the bare arm sprang into the top stop. The park now goes HIGH (110°) on a bare rig. |
| `observer_sweep_bare_20260930_150800.csv.gz` | Bare rig, sweep at 0.10 rad/s, **complete**, with the fixed settings: current abort 5.0 A ×3 / trip 5.3 A, sweep 100→30→100°, park HIGH at 110°. Peak \|present_iq\| 4.62 A. |
| `observer_sweep_wrench_20260930_162234.csv.gz` | Light wrench, **0.711 kg @ 0.1651 m** (per-row `arm_mass_kg`/`arm_com_m`; "wrench" in the name only means mass ≠ 0), sweep at 0.10 rad/s, **complete**, same limits as `150800`. Park chosen from the torque model: net = τ_spring 3.021 − τ_gravity 2.240 = +0.781 Nm at vest 96°, so the spring wins and it parked HIGH at 110°. Peak \|present_iq\| 2.75 A (approach leg, act 107°). |
| `observer_sweep_bare_20261014_220311.csv.gz` | Bare rig, sweep at 0.10 rad/s, `--soft-start-s 0.3`, **complete**, parked. Peak \|present_iq\| 4.67 A. Same-session sweep before `j_release_20261014_220710`. |
| `observer_sweep_bare_20261015_000248.csv.gz` | As `220311`, **complete**, parked. Peak \|present_iq\| 4.62 A. Same-session sweep before `j_release_20261015_000441`. |
| `observer_sweep_bare_20261015_202707.csv.gz` | As `220311`, **complete**, parked. Peak \|present_iq\| 4.64 A. The **before** sweep of the J release fit (`main`: `logs/summaries/phase2/j_release/j_release_fit_20261015.txt`). |
| `observer_sweep_bare_20261015_205603.csv.gz` | As `220311`, **complete**, parked. Peak \|present_iq\| 4.59 A. The **after** sweep of the J release fit. Against `202707`: RMS dy 0.017 (max 0.030), RMS df 0.053 (max 0.087) N m over 14 bins, act 33–98°, so STABLE. Their average y sits 0.01–0.11 N m below `TAU_SPRING_TABLE` − rig gravity at act 43–98°. |
| `observer_jsine_bare_0.5Hz_20261015_214657.csv.gz` | J sine test (`--j-sine`): bare, centre 65°, amp 5°, 6 cycles, limit_velocity_max 2.0 / limit_acc_max 30 for the run. **Complete**, parked. Peak \|present_iq\| 3.77 A. Measured amplitude 2.52° of the 5° goal. |
| `observer_jsine_bare_1Hz_20261015_214859.csv.gz` | As `0.5Hz`, 1.0 Hz, 8 cycles. **Complete**, parked. Peak 3.60 A. Measured amplitude 1.50°. |
| `observer_jsine_bare_1.5Hz_20261015_214948.csv.gz` | As `0.5Hz`, 1.5 Hz, 10 cycles. **Complete**, parked. Peak 3.66 A. Measured amplitude 1.24°. |
| `observer_jsine_bare_2Hz_20261015_215057.csv.gz` | As `0.5Hz`, 2.0 Hz, 12 cycles. **Complete**, parked. Peak 3.79 A. Measured amplitude 1.04°. |
| `observer_jsine_bare_2.5Hz_20261015_215156.csv.gz` | As `0.5Hz`, 2.5 Hz, 14 cycles. **Complete**, parked. Peak 3.64 A. Measured amplitude 0.95°. J sine fit over the five runs (`main`: `logs/summaries/phase2/j_sine/j_sine_fit_20261015.txt`): J −0.025 kg m², R² 0.36, verdict **INCOMPLETE**. The arm tracked only 19–50 % of the goal amplitude (stick-slip). |

### `phase2/j_release/` — J release test (Block 4, abandoned method)
Producer: `useful_tools/j_release_test.py` (on `main` since `fdf0c7a`, last
changed in `c971b22`; each file logs the git hash it ran at). **Enables torque (Position Mode).** Bare rig: holds
the arm at 90 / 75 / 60° in Position Mode, then `disable()` releases it and
the spring drives it free into the top stop while the script only reads. One
row per bus cycle; `#` header (argv, plan, settle/free rules, script git hash
and dirty flag); `#` footer with one line per release and an `END` line. Read
with `pd.read_csv(path, comment='#')`. Fitted by `fit_j_release.py`, verdict
**INCOMPLETE** (`main`: `logs/summaries/phase2/j_release/j_release_fit_20261015.txt`).
Dated 2026-10-14 to 2026-10-15.

| File | Notes |
|---|---|
| `j_release_20261014_220710.csv.gz` | git `7612843`, clean. `--repeats 1`. **Aborted** before the first release: the hold at 90° did not settle within 3.0 s (\|vel\| 0.001). No release. |
| `j_release_20261015_000441.csv.gz` | git `7612843`, **dirty**. `--repeats 1`. One release from 90° (hold −3.005 A, peak \|vel\| 5.03 rad/s), then **stopped**: error byte 0x90 (bit 4, Joint Limit) in the free phase. |
| `j_release_20261015_202949.csv.gz` | git `c971b22`, clean. `--repeats 1`, **complete**, 3 releases (90 / 75 / 60°). Hold current −2.55 / −2.95 / −3.56 A; peak \|vel\| 3.07 / 4.27 / 5.37 rad/s; `disable()` confirmed on attempt 1 in ≤ 2.2 ms each. Used in the fit. |
| `j_release_20261015_204934.csv.gz` | git `c971b22`, clean. `--repeats 2`, **complete**, 6 releases. Peak \|vel\| 3.78–5.46 rad/s (highest from 60°). Used in the fit. |
| `j_release_20261015_205429.csv.gz` | git `c971b22`, clean. `--repeats 2`, **complete**, 6 releases. Peak \|vel\| 3.49–5.78 rad/s (highest from 60°). Used in the fit. In all three fitted files the 60° releases had only 4–5 window samples (< 8), so they were unusable. |

### `phase2/probes/` — one-off register probes
| File | Notes |
|---|---|
| `goal_write_probe_20261014_214721.csv.gz` | `goal_write_probe.py` (added on `main` in `7612843`). **Never enables.** Trials A (mode already 2), B (goal written right after the mode change) and C (50 ms wait), 5 each; rows `trial, k, mode_ms, present, goal_before, first_read, last_read, reads, match_ms`. 14 of 15 writes read back after 58.9 ms (±0.05 ms) in every trial type. A1 (a 0.0325 rad step) was only 0.0232 rad along after 100 ms. So the read-back is a motion-profiled setpoint, not a lost write. This is the basis for `enable_position_mode()`'s settle poll and `GOAL_SETTLE_S`. |

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

### `phase2/watchdog/` — Mode 0 watchdog bench test (T1–T5)
Producer: `useful_tools/watchdog_bench_test.py` (committed on `main` in
`0e33864`). **Enables torque (Mode 0).** Bare rig, arm resting on the top
stop, watchdog 10 ms, 500 Hz loop with one bulk `exchange()` per loop. Tests:
T1 feed `goal_iq = 0` for 10 s; T2 total silence → trip with bit 3 within
12 ms; T3 reads only, no writes → RECORD whether it trips; T4 command −0.5 A
then silence → after the trip \|iq\| < 0.1 A; T5 `disable()` clears the trip
and re-arms. Pass/fail lines are in the `#` header; rows are
`test, t_s, dt_ms, action, pos_rad, vel_rad_s, iq_A, iq_sent_A, err, note`;
read with `pd.read_csv(path, comment='#')`. Dated 2026-10-13.

Provenance: the script file was last edited at 15:43:08, so runs 2–4 used
the committed `0e33864` version. Run 1 used an earlier, uncommitted version.
`bear_interface.py` was last edited at 17:27:47, so only run 4 used the
committed `0e33864` version.

| File | Notes |
|---|---|
| `watchdog_test_20261013_153347.csv.gz` | Run 1, SCHED_OTHER, no pinning. T1 PASS (max dt 5.65 ms), T2 PASS (bit 3 after 12.8 ms). T5 **FAIL**: read once ~1 ms after `disable()`, still torque_enable 3 / 0x88. The T5 procedure was then changed to poll every 5 ms for up to 500 ms. T3/T4 not run. |
| `watchdog_test_20261013_154341.csv.gz` | Run 2, SCHED_FIFO 80, CPU 3. T1 PASS (max dt 3.14 ms), T2 PASS (12.6 ms), T5 PASS (0.9 ms). T3 RECORD: reads alone **do** keep the watchdog alive (no trip in 100 ms). T5 PASS (0.8 ms). Then **ABORT** at T4: enable failed (torque_enable read 0). |
| `watchdog_test_20261013_155117.csv.gz` | Run 3, FIFO 80, CPU 3. T1/T2/T3/T5 as run 2. T4 **FAIL (review)**: max \|iq\| 0.196 A over 17 post-trip reads (max \|vel\| 0.088 rad/s). Reviewed 2026-10-13: PASS by the ‘damping only’ clause pinned before review (every post-trip read with \|iq\| ≥ 0.1 A opposes velocity with \|vel\| > 0.01 rad/s). The CSV header’s ‘FAIL (review)’ is the script’s automatic < 0.1 A check only. T5 after T4 **FAIL**: `disable()` did not clear within 500 ms; a lone `torque_enable = 0` cleared it in 1.0 ms. Aborted, re-arm not tested. |
| `watchdog_test_20261013_173054.csv.gz` | Run 4, FIFO 80, CPU 3, committed `bear_interface.py`. T1 PASS (max dt 3.24 ms), T2 PASS (12.6 ms), T3 RECORD as before, T5 PASS (2.9 / 2.8 ms). T4 **FAIL (review)**: max \|iq\| 0.191 A over 17 reads (max \|vel\| 0.105 rad/s). Reviewed 2026-10-13: PASS by the ‘damping only’ clause pinned before review (every post-trip read with \|iq\| ≥ 0.1 A opposes velocity with \|vel\| > 0.01 rad/s). The CSV header’s ‘FAIL (review)’ is the script’s automatic < 0.1 A check only. T5 after T4 PASS (23.9 ms), re-arm PASS, final T5 PASS (3.8 ms). Only run to reach the end. |

### Other

| File | Notes |
|---|---|
| `Encoder_values.txt` | Standalone note recording an encoder homing-offset value; not a timestamped run |
