# PREEMPT_RT kernel cyclictest — 6.18.50-v8-rt1 (pending)

Placeholder for the RT half of the 2026-09 stock-vs-RT comparison. Nothing
has been run yet.

- **Kernel:** `6.18.50-v8-rt1` (`uname -v` shows `SMP PREEMPT_RT`), built on
  bearpi from raspberrypi/linux commit `cff533aec2fa` — the same source as
  the stock baseline in `../stock_6.18.50/`. The only config changes from
  stock are `PREEMPT_RT=y`, the options Kconfig adjusts because of it, and
  `LOCALVERSION="-v8-rt1"`.
- **Boot files:** `/boot/firmware/kernel8-rt.img` and `/boot/firmware/initramfs8-rt`;
  modules in `/lib/modules/6.18.50-v8-rt1`.
- **Protocol:** identical to `../stock_6.18.50/README.md`: same commands,
  1.2 GHz clock lock, power and physical setup, and background processes.

When the runs are done, add the histograms, the conditions snapshots, and a
README in the same format as the stock one, then fill in the RT results in
`../README.md`.
