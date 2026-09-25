# Stock vs PREEMPT_RT cyclictest comparison — 2026-09

Scheduling-latency comparison of the stock Raspberry Pi OS kernel against a
PREEMPT_RT build of the **same source** (raspberrypi/linux commit
`cff533aec2fa`, 6.18.50), on bearpi (Pi 4 Model B Rev 1.5), under identical
test conditions. The test interval matches the 800 Hz control-loop target:
one period is **1,250 µs**.

This file is identical on `main` and `raw-data`. The raw histograms and the
full conditions record live on `raw-data` only:

| Folder (on `raw-data`) | Kernel (`uname -r`) | Status |
|---|---|---|
| `stock_6.18.50/` | `6.18.50+rpt-rpi-v8` (`#1 SMP PREEMPT`) | done 2026-09-24 |
| `rt_6.18.50-v8-rt1/` | `6.18.50-v8-rt1` (`SMP PREEMPT_RT`) | done 2026-09-25 |

Retrieve a histogram with, e.g.:

```
git show raw-data:logs/cyclictest/2026-09_stock_vs_rt/stock_6.18.50/cyclictest_stock_loaded.txt
```

Each kernel folder's `README.md` records the exact commands, CPU clock lock,
power and physical setup, background processes, temperatures, and throttling
state needed to repeat the run.

## Protocol (both kernels)

```
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q                   # idle
stress-ng --cpu 4 --io 2 --vm 2 --vm-bytes 256M --timeout 11m            # load...
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q                   # ...start ~18 s later
```

CPU locked at 1.2 GHz, `performance` governor; `vcgencmd get_throttled` must
read 0x0 after each run or the run is invalid.

## Results — stock 6.18.50

Latencies in µs. Percentiles are computed from the committed histograms (1 µs
bins); "≥ 1250" counts samples that took at least one full 800 Hz period.

| Run | CPU | Avg | p99 | p99.9 | p99.99 | Max | ≥ 1250 |
|---|---|---|---|---|---|---|---|
| Idle   | 0 | 5  | 10  | 17  | 32  | 103  | 0 |
| Idle   | 1 | 5  | 10  | 17  | 34  | 81   | 0 |
| Idle   | 2 | 5  | 10  | 18  | 39  | 98   | 0 |
| Idle   | 3 | 5  | 10  | 17  | 29  | 79   | 0 |
| Loaded | 0 | 29 | 125 | 179 | 413 | 981  | 0 |
| Loaded | 1 | 23 | 91  | 133 | 390 | 1296 | 1 |
| Loaded | 2 | 20 | 62  | 95  | 427 | 1321 | 1 |
| Loaded | 3 | 21 | 70  | 101 | 436 | 1430 | 1 |

~480,000 samples per CPU per run. Loaded total ≥ 1250 µs: **3 samples**, one
each on CPUs 1–3. Throttling 0x0 after both runs.

## Results — PREEMPT_RT 6.18.50-v8-rt1

| Run | CPU | Avg | p99 | p99.9 | p99.99 | Max | ≥ 1250 |
|---|---|---|---|---|---|---|---|
| Idle   | 0 | 5  | 6   | 8   | 14  | 26  | 0 |
| Idle   | 1 | 5  | 7   | 10  | 14  | 27  | 0 |
| Idle   | 2 | 5  | 7   | 9   | 13  | 37  | 0 |
| Idle   | 3 | 5  | 6   | 9   | 16  | 59  | 0 |
| Loaded | 0 | 21 | 72  | 109 | 137 | 220 | 0 |
| Loaded | 1 | 28 | 131 | 178 | 207 | 240 | 0 |
| Loaded | 2 | 26 | 113 | 166 | 196 | 232 | 0 |
| Loaded | 3 | 21 | 75  | 113 | 143 | 165 | 0 |

~480,000 samples per CPU per run. Loaded total ≥ 1250 µs: **0 samples**.
Throttling 0x0 in every snapshot.

## Stock vs RT

Under load, the worst case fell from 981–1,430 µs to 165–240 µs (from 114 %
of the 1,250 µs period to 19 %), p99.99 fell from 390–436 µs to 137–207 µs,
and no sample reached a full period (stock had 3). The average (about
20–30 µs loaded, 5 µs idle) and p99 are essentially unchanged: PREEMPT_RT
bounds the rare long non-preemptible delays; it doesn't speed up the normal
wake-up path.

The two runs were not perfectly matched. The RT run had no VS Code server
or Copilot running, started idle about 8 °C cooler, and ran stress-ng from
the same shell. Each condition was measured once for 10 minutes. See
`rt_6.18.50-v8-rt1/README.md` for the full interpretation and its limits.

## Planned

- **Rerun stock 6.18.50 without VS Code** before these numbers are used in a
  paper: no VS Code server or Copilot running (plain SSH + tmux only), with an
  identical protocol, saved as `stock_6.18.50_no_vscode/`. Reason: the RT run
  had no VS Code while stock had two VS Code servers plus Copilot, a confound
  that favors RT.
