# PREEMPT_RT kernel cyclictest — 6.18.50-v8-rt1, 2026-09-25

Scheduling latency of the **PREEMPT_RT** build of the same 6.18.50 source as
the stock baseline in `../stock_6.18.50/`, idle and under the same synthetic
load. The test interval matches the 800 Hz control-loop target: one period is
**1,250 µs**.

All times are local (MDT, UTC−6).

## Results

| | Min (µs) | Avg (µs), CPU 0–3 | Max (µs), CPU 0–3 | Overflows |
|---|---|---|---|---|
| Idle   | 4–5 | 5, 5, 5, 5     | 26, 27, 37, 59     | 0 |
| Loaded | 5–7 | 21, 28, 26, 21 | 220, 240, 232, 165 | 0 |

~480,000 samples per CPU per run (10 min × 800 Hz). Summary lines are at the
bottom of each `cyclictest_*.txt`; the rest of each file is the
per-microsecond histogram (0–9999 µs, one column per CPU).

### Percentiles

Computed from the committed histograms the same way as for stock (1 µs bins;
a percentile is the smallest latency whose cumulative count reaches that
fraction of the samples). "≥ 1250" counts samples that took at least one full
800 Hz period.

| Run | CPU | p99 | p99.9 | p99.99 | Max | ≥ 1250 µs |
|---|---|---|---|---|---|---|
| Idle   | 0 | 6   | 8   | 14  | 26  | 0 |
| Idle   | 1 | 7   | 10  | 14  | 27  | 0 |
| Idle   | 2 | 7   | 9   | 13  | 37  | 0 |
| Idle   | 3 | 6   | 9   | 16  | 59  | 0 |
| Loaded | 0 | 72  | 109 | 137 | 220 | 0 |
| Loaded | 1 | 131 | 178 | 207 | 240 | 0 |
| Loaded | 2 | 113 | 166 | 196 | 232 | 0 |
| Loaded | 3 | 75  | 113 | 143 | 165 | 0 |

### Against stock (loaded, range across the four CPUs)

| | Stock 6.18.50 | PREEMPT_RT 6.18.50-v8-rt1 |
|---|---|---|
| Avg      | 20–29 µs      | 21–28 µs     |
| p99      | 62–125 µs     | 72–131 µs    |
| p99.99   | 390–436 µs    | 137–207 µs   |
| Max      | 981–1,430 µs  | 165–240 µs   |
| ≥ 1250 µs | 3 samples    | 0 samples    |
| Worst case as share of the 1,250 µs period | 114 % | 19 % |

### Interpretation

**What changed** is the tail. Under load, the worst wake-up fell from
981–1,430 µs to 165–240 µs, p99.99 fell from 390–436 µs to 137–207 µs, and
no sample reached a full 800 Hz period (stock had three). The worst case now
uses under a fifth of the 1,250 µs period, where on stock it could exceed the
whole period. Idle maxima also dropped, from 79–103 µs to 26–59 µs. **What
didn't change** is the typical case: the average (about 20–30 µs loaded,
5 µs idle) and the minimum are the same on both kernels, and p99 is roughly
the same, a little higher on CPUs 1 and 2 under RT. **Why:** PREEMPT_RT
doesn't make the CPU or the normal wake-up path any faster. What it removes
is the long stretches where the kernel can't be interrupted: spinlocks
become sleeping locks, and interrupt handlers and softirqs run as
schedulable threads (the `ksoftirqd/N` and `ktimers/N` threads in the
conditions snapshots). So a priority-80 SCHED_FIFO task can preempt the
kernel work stress-ng generates (I/O syscalls, page reclaim) instead of
waiting for it to finish. That caps the rare long delays that set the stock
maximum, while the ordinary timer → scheduler → context-switch path, which
sets the average, is unchanged. The extra thread hand-offs are also the
usual reason RT costs slightly more in the middle of the distribution.

**Limits of this comparison.** Each kernel was measured once, for 10 minutes
per condition. Max is a single sample, and p99.99 rests on about 48 samples
per CPU, so the exact values would shift on a rerun. The RT run was also not
perfectly matched to stock (see *Procedural differences*): in particular,
the VS Code servers and Copilot runtime were not running. Those processes
were mostly idle (under 1 % CPU each in the stock snapshot) against 8
stress-ng workers on 4 cores, so they are unlikely to account for the
roughly six-fold drop in the overall worst case (1,430 → 240 µs). But the RT environment was
slightly lighter, and a strict comparison would rerun one kernel with the
other's background.

## System

- **Hardware:** Raspberry Pi 4 Model B Rev 1.5, 8 GB — same board as stock.
- **Kernel:** `6.18.50-v8-rt1` — `#1 SMP PREEMPT_RT Thu Sep 24 06:20:56 BST 2026`,
  built on bearpi from raspberrypi/linux commit
  `cff533aec2fa601846766b32ff57204e0a61bed7` (the same source as stock) with
  the stock config plus `PREEMPT_RT=y`, the options Kconfig adjusts because of
  it, and `LOCALVERSION="-v8-rt1"`; `CONFIG_HZ=250` as stock.
- **Boot:** booted once via **tryboot** at 11:47:52. `/boot/firmware/tryboot.txt`
  is `config.txt` plus `[all]` / `kernel=kernel8-rt.img`; `config.txt` itself
  was not changed, so a normal reboot returns to stock. `auto_initramfs=1`
  loaded `initramfs8-rt` ("Freeing initrd memory: 11580K" in the boot log).
- **Tools:** cyclictest V 2.60, stress-ng 0.19.02 — same versions as stock.
- **CPU clock:** locked at 1.2 GHz, governor `performance`. The journal shows
  the same sequence as stock at 11:57:31: `sudo tee` to `cpu0`'s
  `scaling_governor`, then `scaling_max_freq`, then `scaling_min_freq`. All four
  cores read 1200000 in every conditions snapshot.
- **Power:** UPS HAT (E) on external power for both runs: `ups-monitor.service`
  logged "Fast Charging state", battery 100 %, VBUS ≈ 12.07 V, pack ≈ 16.81 V.
- **Physical setup:** not re-recorded for this run.
- **Background processes:** `ups-monitor.service`
  (`/home/samuel/projects/ups_hat/UPS_HAT_E/ups.py`), two plain SSH sessions
  (logged in 11:49:57 and 11:58:13), and system daemons. **No VS Code server
  or Copilot runtime:** they were not in any snapshot and started only at
  15:49:31, after the tests.

## Test commands (exactly as run, from the system journal)

Idle run — started 12:02:27, finished 12:12:27:

```
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q > cyclictest_rt_idle.txt
```

Loaded run — stress-ng started in the background of the same shell, output to
a log, then cyclictest 18 s later:

```
stress-ng --cpu 4 --io 2 --vm 2 --vm-bytes 256M --timeout 11m > stressng_rt_loaded.log 2>&1 &   # 14:47:53 → 14:58:53 (redirection reconstructed)
sudo cyclictest -m -S -p 80 -i 1250 -D 10m -h 10000 -q > cyclictest_rt_loaded.txt              # 14:48:11 → 14:58:11
```

(The stress-ng arguments are from the journal. The journal doesn't record
the shell redirection, so the `> … 2>&1 &` part is reconstructed; the log
contains stress-ng's normal info output, ending with "successful run
completed in 11 mins".)

### Load timing — evidence the load covered the whole loaded run

- stress-ng invocation logged at **14:47:53** with `--timeout 11m`; its log
  reports "passed: 8: cpu (4) io (2) vm (2)", "failed: 0", "successful run
  completed in 11 mins", and was last written **14:58:53**.
- cyclictest `sudo` invocation logged at **14:48:11**; output file last written
  **14:58:11** (10 min).
- The measurement window (14:48:11–14:58:11) sits inside the load window,
  starting 18 s after load began and ending 42 s before it stopped — the same
  offsets as stock.

## Conditions

| Snapshot | Time | SoC temp | `get_throttled` |
|---|---|---|---|
| Before idle   | 12:02:27 | 41.9 °C | 0x0 |
| After idle    | 12:12:27 | 40.4 °C | 0x0 |
| Before loaded | 14:47:53 | 40.9 °C | 0x0 |
| After loaded  | 14:58:54 (1 s after load ended) | 62.8 °C | 0x0 |

Throttling was 0x0 in every snapshot, so the 1.2 GHz clock held throughout.
Each snapshot has date, kernel, uptime/load, temperature (m°C), throttled,
per-core frequency and the top of `ps aux --sort=-%cpu`.

## Procedural differences from stock

1. **stress-ng ran in the background of the same shell** (`&`, output to
   `stressng_rt_loaded.log`) instead of in a separate window. Load parameters,
   duration and the 18 s head start are the same.
2. **The idle run started cooler:** 41.9 °C at the before-idle snapshot and
   40.4 °C at the end, against 49.7 °C for stock.
3. **The "before loaded" snapshot exists this time**
   (`conditions_before_loaded.txt`), which closes the stock run's known gap.
4. **No VS Code server or Copilot runtime was running** (stock had two VS Code
   server builds plus Copilot). See *Limits of this comparison*.

Minor: the after-loaded snapshot was taken 1 s after the load ended (62.8 °C),
against about 6 minutes after for stock (52.1 °C), so those two temperatures
aren't comparable. No temperature was logged during either loaded run.

## Files

| File | Contents |
|---|---|
| `cyclictest_rt_idle.txt`       | Idle histogram + summary |
| `cyclictest_rt_loaded.txt`     | Loaded histogram + summary |
| `stressng_rt_loaded.log`       | stress-ng output for the loaded run |
| `conditions_before_idle.txt`   | Snapshot before the idle run |
| `conditions_after_idle.txt`    | Snapshot after the idle run |
| `conditions_before_loaded.txt` | Snapshot before the loaded run (new for RT) |
| `conditions_after_loaded.txt`  | Snapshot after the loaded run |
