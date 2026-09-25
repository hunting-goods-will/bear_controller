Westwood robotics actuator that produces intelligent torque to aid in making Eksovest an active system

## System

- **Kernel:** bearpi runs the PREEMPT_RT kernel `6.18.50-v8-rt1`
  (`#1 SMP PREEMPT_RT`), built on the Pi from raspberrypi/linux `cff533aec2fa`
  with the stock config plus `PREEMPT_RT=y`.
- **Boot:** `kernel=kernel8-rt.img` in `/boot/firmware/config.txt`.
- **Fallback:** the stock 6.18.50 kernel is still installed and boots once via
  `sudo reboot '0 tryboot'` (`tryboot.txt` is the stock config). The stock
  config is backed up as `/boot/firmware/config.txt.stock-backup`.
- **Held packages:** kernel and firmware packages are held, so apt can't
  change the tested kernel. `apt-mark showhold` lists `linux-image-rpi-v8`,
  `linux-image-6.18.50+rpt-rpi-v8`, `linux-headers-rpi-v8`,
  `linux-headers-6.18.50+rpt-rpi-v8`, `linux-headers-6.18.50+rpt-common-rpi`
  and `raspi-firmware`. Updating the RT kernel means rebuilding it from
  `~/rt-kernel`.
- **Latency results:** `logs/cyclictest/2026-09_stock_vs_rt/`.
