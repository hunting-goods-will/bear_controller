"""
Probe: does disable() clear a latched watchdog trip, and how?
NEVER ENABLES. The only writes are torque_enable = 0 (disable) and goal_iq = 0.

Step A  read torque_enable + error byte as they are right now
Step B  if still latched: write torque_enable = 0 ALONE, then read it back
        at 0, 10, 50, 200 and 1000 ms
Step C  if still latched: call disable() (goal_iq = 0, then torque_enable = 0)
        three times, 100 ms apart, reading after each

Run from the repo root:
  cd ~/projects/bear_controller
  python3 useful_tools/watchdog_clear_probe.py
"""
import sys
import time

sys.dont_write_bytecode = True
from main_controller.bear_interface import BearInterface


def status(iface, label):
    (te,), err = iface.bear.get_torque_enable(iface.id)[0]
    (wd,), _ = iface.bear.get_watchdog_timeout(iface.id)[0]
    (mode,), _ = iface.bear.get_mode(iface.id)[0]
    e = 'none' if err is None else hex(err)
    print(f"{label:<38} torque_enable={te}  error={e}  watchdog={wd} us  mode={mode}")
    return te, err


def clear(te, err):
    return te == 0 and err == 0x80


iface = BearInterface()
try:
    print("Step A: as it is now")
    te, err = status(iface, "  now")
    if clear(te, err):
        print("\nAlready clear: the earlier disable() calls DID clear it, given time.")
        sys.exit(0)

    print("\nStep B: torque_enable = 0 written alone")
    t0 = time.monotonic()
    iface.bear.set_torque_enable((iface.id, 0))
    for ms in (0, 10, 50, 200, 1000):
        while (time.monotonic() - t0) * 1000 < ms:
            time.sleep(0.001)
        te, err = status(iface, f"  {ms:>4} ms after the write")
        if clear(te, err):
            print(f"\nCleared by a lone torque_enable = 0, read back at {ms} ms.")
            sys.exit(0)

    print("\nStep C: disable() three times")
    for i in range(3):
        iface.disable()
        time.sleep(0.1)
        te, err = status(iface, f"  after disable() #{i + 1} + 100 ms")
        if clear(te, err):
            print(f"\nCleared by disable() #{i + 1}.")
            sys.exit(0)

    print("\nSTILL LATCHED. Do not try anything else; power-cycle the actuator,"
          " then run this probe once more and paste both outputs.")
finally:
    iface.bear.close_port()