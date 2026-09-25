# Stock kernel cyclictest baseline — 6.18.50, 2026-09-24

Baseline scheduling latency of the **stock (non-RT)** Raspberry Pi OS kernel on
bearpi, idle and under synthetic load. Recorded so the PREEMPT_RT kernel
(`6.18.50-v8-rt1`, built from the same source commit) can be compared against
it under identical conditions. The test interval matches the 800 Hz control
loop target (period 1,250 µs).

All times are local (MDT, UTC−6).

## Results

| | Min (µs) | Avg (µs), CPU 0–3 | Max (µs), CPU 0–3 | Overflows |
|---|---|---|---|---|
| Idle   | 4    | 5, 5, 5, 5     | 103, 81, 98, 79       | 0 |
| Loaded | 5–6  | 29, 23, 20, 21 | 981, 1296, 1321, 1430 | 0 |

Each thread completed ~480,000 cycles (10 min × 800 Hz). Summary lines are at
the bottom of each `cyclictest_*.txt`; the rest of each file is the
per-microsecond histogram (0–9999 µs, one column per CPU).

### Percentiles

Computed from the committed histograms (1 µs bins; a percentile is the
smallest latency whose cumulative count reaches that fraction of the
samples). "≥ 1250" counts samples that took at least one full 800 Hz period.

| Run | CPU | p99 | p99.9 | p99.99 | Max | ≥ 1250 µs |
|---|---|---|---|---|---|---|
| Idle   | 0 | 10  | 17  | 32  | 103  | 0 |
| Idle   | 1 | 10  | 17  | 34  | 81   | 0 |
| Idle   | 2 | 10  | 18  | 39  | 98   | 0 |
| Idle   | 3 | 10  | 17  | 29  | 79   | 0 |
| Loaded | 0 | 125 | 179 | 413 | 981  | 0 |
| Loaded | 1 | 91  | 133 | 390 | 1296 | 1 |
| Loaded | 2 | 62  | 95  | 427 | 1321 | 1 |
| Loaded | 3 | 70  | 101 | 436 | 1430 | 1 |

Loaded total ≥ 1250 µs: 3 samples out of ~1.92 million. Idle: none.

## System

- **Hardware:** Raspberry Pi 4 Model B Rev 1.5, 8 GB (7.6 GiB usable)
- **Kernel:** `6.18.50+rpt-rpi-v8` — `#1 SMP PREEMPT Debian 1:6.18.50-1+rpt1 (2026-09-11)`
  (Debian package `linux-image-6.18.50+rpt-rpi-v8`, source commit
  `cff533aec2fa601846766b32ff57204e0a61bed7` of raspberrypi/linux; `CONFIG_PREEMPT=y`, `CONFIG_HZ=250`)
- **Tools:** cyclictest V 2.60 (rt-tests), stress-ng 0.19.02
- **CPU clock:** locked at 1.2 GHz on all four cores (hardware max is 1.8 GHz).
  Set through cpufreq sysfs on `cpu0`, in this order: governor
  `performance`, then `scaling_max_freq` = 1200000, then `scaling_min_freq` =
  1200000. All four cores share one cpufreq policy (`policy0`,
  `related_cpus` = 0 1 2 3), so setting `cpu0` locks all of them; verified by
  all four cores reading 1200000 in `conditions_before_idle.txt`. No script
  for this was saved (`~/lock_clock.sh` does not exist); equivalent commands:

  ```
  echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
  echo 1200000     | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
  echo 1200000     | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
  ```
- **Power:** USB-C into the UPS HAT (E); the HAT powers the Pi through its
  pogo pins. VBUS read nonzero before the runs, so external power was
  present. During both runs `ups-monitor.service` logged "Fast Charging
  state", pack at ~16.81 V, reaching 100 % at 23:12.
- **Physical setup:** Pi out of its case, no heatsink, no fan.
- **Background processes (part of the "idle" condition — the RT runs must match):**
  - `ups-monitor.service` (Python, polls the UPS over I²C, ~6–7 log lines/s)
  - **two** VS Code server builds running side by side, `7debcd0e2acdea1c52de81bf9ee1620444407dda`
    (server-main, extension host, pty host, file watcher) and
    `2242ebbb54efeeb0129e08e919e7e8d43033cd83` (server-main, agent host)
  - the GitHub Copilot runtime (`copilot-sdk-linux-arm64`, under the `2242ebb…` build)
  - sshd sessions

  Full process list at start: `conditions_before_idle.txt`. For the RT runs,
  connect the same way so the same set is running, and capture the process
  list again to confirm.

## Test commands (exactly as run, from the system journal)

Idle run — started 22:49:43, finished 22:59:43:

```
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q > cyclictest_stock_idle.txt
```

Loaded run — load started first, then cyclictest 18 s later:

```
stress-ng --cpu 4 --io 2 --vm 2 --vm-bytes 256M --timeout 11m      # 23:10:08 → 23:21:08
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q > cyclictest_stock_loaded.txt   # 23:10:26 → 23:20:26
```

cyclictest flags: `-m` lock memory, `-S` one thread per CPU (SMP), `-p 80`
SCHED_FIFO priority 80, `-i 1250` 1,250 µs interval (800 Hz), `-D 10m`
10-minute duration, `-h 10000` histogram up to 10 ms, `-q` summary only.

### Load timing — evidence the load covered the whole loaded run

- stress-ng invocation logged by the journal at **23:10:08** with `--timeout 11m` → load ends ~23:21:08.
- cyclictest `sudo` invocation logged at **23:10:26**; output file last written **23:20:26** (10 min).
- The full measurement window (23:10:26–23:20:26) sits inside the load window,
  starting 18 s after load began and ending ~42 s before it stopped.

## Conditions

| Snapshot | Time | SoC temp | `get_throttled` |
|---|---|---|---|
| Before idle   | 22:49:07 | 49.7 °C | 0x0 |
| After idle    | 23:00:40 | 49.7 °C | 0x0 |
| Before loaded | — (not captured, see below) | — | — |
| After loaded  | 23:27:05 (~6 min after load ended) | 52.1 °C | 0x0 |

Temperature is `/sys/class/thermal/thermal_zone0/temp` (m°C in the files).

- **Temperature under load:** a separate stress check under the same setup leveled off at
  about 70–71 °C. Temperature was not logged during the loaded cyclictest run itself.
- **Throttling:** 0x0 after both runs — no under-voltage, frequency capping, or
  thermal throttling occurred, so the 1.2 GHz clock held throughout.

## Known gaps

- **No "before" snapshot for the loaded run** (`conditions_before_loaded.txt` is
  missing). Partial substitute: the stress-ng journal entry at 23:10:08 records
  the kernel string and free memory (6654 MB free of 7820 MB, swap unused).
- **No temperature trace during either run** — only point snapshots before/after.

## Files

| File | Contents |
|---|---|
| `cyclictest_stock_idle.txt`   | Idle histogram + summary |
| `cyclictest_stock_loaded.txt` | Loaded histogram + summary |
| `conditions_before_idle.txt`  | Date, kernel, uptime/load, temp, throttled, per-core freq, process list |
| `conditions_after_idle.txt`   | Date, temp, throttled |
| `conditions_after_loaded.txt` | Date, temp, throttled |

## Reproducing

1. Boot the kernel under test; confirm with `uname -r` / `uname -v`.
2. Lock all cores to 1.2 GHz with the `performance` governor; verify
   `scaling_min_freq`, `scaling_max_freq` and `scaling_cur_freq` all read 1200000.
3. Same power and physical setup and same background processes as above.
4. Capture a conditions snapshot, run the idle command, snapshot again.
5. Capture a conditions snapshot, start stress-ng, wait ~18 s, run the loaded
   cyclictest command, snapshot again after stress-ng exits.
6. Check `sudo vcgencmd get_throttled` is 0x0 after each run; if not, the run is invalid.
