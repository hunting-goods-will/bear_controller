# logs/

Raw run data (the `.csv.gz` files under `human_live_csvs/`, `human_monitor_csvs/`,
`model_validation_csvs/`, `velocity_profile_csvs/`, `spring_characterization/`,
`phase2/observer_csvs/`, `phase2/latency/`, `phase2/watchdog/`, `phase2/j_release/`,
`phase2/hold_bench/`, `phase2/probes/`)
does not live on `main` — it lives on the orphan branch `raw-data`, which
mirrors this directory's paths exactly. Retrieve any file with:

```
git show raw-data:logs/<path> | gunzip > <path minus .gz>
```

e.g. `git show raw-data:logs/human_live_csvs/human_live_20260821_002111.csv.gz | gunzip > run.csv`

`raw-data`'s own `logs/README.md` has the full file index, including a
"Missing data" section on the model-validation gap and the correction for
`model_validation_20260820_234335` (see below).

## `summaries/`

One text file per raw `.csv.gz`, in one folder per section, named after its
source (`<source name>.txt`). Each summary's header states its source file,
the exact `git show` command to retrieve that source from `raw-data`, the
analyzer script used, and the commit hash of that script at generation
time -- so a summary can always be reproduced or checked against a newer
analyzer version. Generated 2026-09-17 (the two `model_validation_*` summaries
were regenerated 2026-09-25; everything under `phase2/` was generated
2026-10-13).

| Summary folder | Source on `raw-data` | Analyzer(s) |
|---|---|---|
| `human_live/` | `human_live_csvs/*.csv.gz` | `analyze_human_run.py` + `analyze_assist_run.py` |
| `human_monitor/` | `human_monitor_csvs/*.csv.gz` | `analyze_assist_run.py` |
| `model_validation/` | `model_validation_csvs/*.csv.gz` | `extract_spring.py` (includes its validation-error table; `234335` run with `--arm-mass-kg 0`, see below) |
| `velocity_profile/` | `velocity_profile_csvs/*.csv.gz` | `Analyse_velocity_profile.py` |
| `spring_characterization/` | `spring_characterization/bidirectional_sweep_*.csv.gz` | `analyze_bidirectional_sweep.py` |
| `spring_characterization/` | `spring_characterization/position_sweep_FINAL_*.csv.gz` | none -- single-direction sweep, no schema-compatible analyzer exists (see that summary file for why) |
| `phase2/observer/` | `phase2/observer_csvs/*.csv.gz` | `analyze_observer.py bench` (the wrench summary is the P2 run, paired with bare sweep `150800`; the `_ABORTED` run has no up leg, so the analyzer stops before P1 -- its summary says so and keeps the CSV's outcome lines) |
| `phase2/latency/` | `phase2/latency/comm_budget_*.csv.gz` | none -- the CSV's own `#` header (written by `measure_comm_budget.py`) plus per-method stats recomputed from the rows and checked against the header's stats table |
| `phase2/watchdog/` | `phase2/watchdog/watchdog_test_*.csv.gz` | none -- the CSV's own `#` header (T1-T5 pass/fail lines written by `watchdog_bench_test.py`) plus per-test loop timing, current and event timeline computed from the rows |
| `phase2/j_release/` | `phase2/j_release/j_release_*.csv.gz` + the same-session bare sweeps `phase2/observer_csvs/observer_sweep_bare_20261015_{202707,205603}.csv.gz` | `fit_j_release.py` (removed from `main` in `617158f`; last version `c971b22`). One summary per fit, not per CSV. Verdict INCOMPLETE (abandoned method) |
| `phase2/j_sine/` | `phase2/observer_csvs/observer_jsine_bare_*_20261015_*.csv.gz` | `fit_j_sine.py` (removed from `main` in `617158f`; last version `94a319f`). One summary per fit. Verdict INCOMPLETE (abandoned method) |
| `phase2/hold_bench/` | `phase2/hold_bench/hold_bench_*.csv.gz`, `hold_thermal_*.csv.gz` + that session's bare sweeps in `phase2/observer_csvs/` | `analyze_hold_bench.py` (one summary `.txt` + `.png` per session: `hold_bench_<date>[_L2]`; the run list and sweeps are in its first line and section headers; `hold_bench_20261028` uses `--payload-r-m 0.213`, the measured r) and `compare_sweeps.py` (`sweep_compare_<date>.txt`; its margin column assumes a 5.0 A abort, wrong for `--i-max` runs) |

The `phase2/` summaries were generated with pandas 3.0.6 / numpy 2.5.4 in a
throwaway venv, because pandas is not installed in the project venv.

## Model validation: in-sample vs held-out

- `model_validation_20260820_235718` is **in-sample**: the current
  `TAU_SPRING_TABLE` was extracted from this run (all 20 entries match), so
  its near-zero current-table error is fit residual, not predictive accuracy.
- `model_validation_20260820_234335` is the **held-out** check (same session,
  same speed). Against the current table: mean +0.021 Nm, max 0.054 Nm,
  17 bins.

Correction (2026-09-25): on 2026-09-17, `234335` was labelled a
"sign-mismatch anomaly" and left unsummarized. That was a misdiagnosis. It is
a bare-rig run logged while `validate_model.py` hardcoded
`ARM_MASS_KG = 1.5025`, so its logged `tau_gravity`/`residual_predicted`
assume a wrench that was not fitted. Its leg-averaged `KT*iq_measured`
matches `RIG_MGL`-only gravity minus spring, and matches `235718` within
0.044 Nm at every 5° bin; a 1.5 kg load at 274 mm would have shifted iq by
about +6 A. Its summary recomputes gravity as bare rig (`--arm-mass-kg 0`).
`validate_model.py` now requires the load on the command line and writes it
into every CSV row, so this can't recur.

## Not summarized (still on `main`)

`Encoder_values.txt` and `cyclictest/` are small, already-readable text
files, so no separate summary is generated for them. `cyclictest/` is
grouped by campaign:

| Folder | Contents |
|---|---|
| `cyclictest/2026-07_stock_initial/` | July 2026 stock-kernel runs and their note: three 4-thread histogram runs and a single-thread 500/800 Hz summary, all 100k cycles per thread. Different protocol and duration -- not directly comparable with 2026-09. |
| `cyclictest/2026-09_stock_vs_rt/` | `README.md` with the stock 6.18.50 and PREEMPT_RT 6.18.50-v8-rt1 results, percentile tables and comparison. Raw histograms are on `raw-data` only, under the same path. |
