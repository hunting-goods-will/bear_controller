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
| `rt_6.18.50-v8-rt1/` | `6.18.50-v8-rt1` (`SMP PREEMPT_RT`) | pending |

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

Pending.
