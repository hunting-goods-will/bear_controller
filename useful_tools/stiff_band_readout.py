"""
stiff_band_readout.py -- READ-ONLY. Find where an actuator feels stiff when the
flange is turned by hand.

What it does
  * Opens the port, pings the actuator, reads torque_enable and the error byte.
  * Refuses to continue unless torque_enable == 0 (a damped or latched motor
    feels stiff by itself and would fool the hand test).
  * Then reads present_position and present_velocity at ~20 Hz while you turn the
    flange by hand, and shows them live. Press Enter at the angle where the
    stiff band STARTS and press Enter again where it ENDS (one pair per pass).
  * Ctrl-C stops it. Writes a CSV and prints a summary.

What it never does
  * It sends NOTHING to the actuator except reads (get_* and ping). Every
    other call is blocked by the ReadOnly wrapper below and raises.
  * It does not enable, disable, clear errors, change the ID, or save config.

Run (from the repo root, actuator and vest in place, motor OFF):
    venv/bin/python3 useful_tools/stiff_band_readout.py --id 1 --label "right, sticky unit, no spring"

Notes
  * Position is the actuator angle after the homing offset in flash (0 = bottom
    bracket stop, top stop about 116.5 deg on the unit that was homed Oct 2).
    A unit that has not been homed in this vest will read an arbitrary angle.
  * Keep the arm off the stops: slow, smooth turns, do not slam the top stop.
"""
import argparse
import csv
import math
import os
import select
import sys
import time
from datetime import datetime

from pybear import Manager
from main_controller.config import LOG_DIR, PORT, BAUDRATE, COMM_TIMEOUT, COMM_TRY_NUM

LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'stiff_band')
RATE_HZ = 20.0
SHOW_HZ = 10.0


class ReadOnly:
    """Only ping() and get_*() reach the real PyBEAR object. Anything else raises."""

    def __init__(self, bear):
        object.__setattr__(self, '_bear', bear)

    def __getattr__(self, name):
        if name == 'ping' or name.startswith('get_'):
            return getattr(self._bear, name)
        raise RuntimeError("stiff_band_readout is read-only; blocked call: %s" % name)

    def __setattr__(self, name, value):
        raise RuntimeError("stiff_band_readout is read-only; blocked attribute write: %s" % name)


def read1(bear, getter, aid):
    """One register read -> (value or None, error byte or None). Never raises on a bad read."""
    try:
        (v,), err = getattr(bear, getter)(aid)[0]
        return v, err
    except Exception:
        return None, None


def main():
    ap = argparse.ArgumentParser(description="Read-only stiff-band locator")
    ap.add_argument('--id', type=int, default=1, help="actuator ID on the bus (default 1)")
    ap.add_argument('--label', default='', help="free text saved in the CSV header")
    args = ap.parse_args()

    raw = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
    raw.single_timeout = COMM_TIMEOUT
    raw.single_try_num = COMM_TRY_NUM
    bear = ReadOnly(raw)
    aid = args.id

    if not bear.ping(aid)[0]:
        print("ID %d did not answer. Nothing was written. Exiting." % aid)
        return 1

    te, err = read1(bear, 'get_torque_enable', aid)
    pos, _ = read1(bear, 'get_present_position', aid)
    print("ID %d: torque_enable %s, error byte %s, position %s deg" % (
        aid, te, hex(err) if err is not None else None,
        "%.2f" % math.degrees(pos) if pos is not None else None))
    if te != 0:
        print("REFUSED: torque_enable is not 0. A damped or latched motor feels stiff by itself,\n"
              "so a hand test now would not tell us anything. Nothing was written.\n"
              "Clear it with the disable snippet first, then run this again.")
        return 2

    os.makedirs(LOG_SUBDIR, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_SUBDIR, 'stiff_band_id%d_%s.csv' % (aid, stamp))

    print("\nTurn the flange slowly by hand through the whole range, down and up, several passes.")
    print("Press Enter where the stiff band STARTS, and Enter again where it ENDS.")
    print("Ctrl-C to stop.\n")

    marks = []          # (t, act_deg, direction_guess)
    rows = []
    n_bad = 0
    t0 = time.time()
    next_t = t0
    next_show = t0
    last_deg = None
    stdin_closed = False
    try:
        while True:
            now = time.time()
            p, e1 = read1(bear, 'get_present_position', aid)
            v, e2 = read1(bear, 'get_present_velocity', aid)
            if p is None or v is None:
                n_bad += 1
            else:
                deg = math.degrees(p)
                dps = math.degrees(v)
                if not stdin_closed and select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.readline() == '':
                        stdin_closed = True          # EOF: stop looking for Enter presses
                    else:
                        direction = '' if last_deg is None else ('up' if deg > last_deg else 'down')
                        marks.append((now - t0, deg, direction))
                        print("\n  mark %d at act %.2f deg (%s)" % (len(marks), deg, direction or '?'))
                last_deg = deg
                rows.append((now - t0, deg, dps, e1, len(marks)))
                if now >= next_show:
                    print("\r  act %7.2f deg   vel %+7.1f deg/s   torque_enable %s  marks %d   " % (
                        deg, dps, te, len(marks)), end='', flush=True)
                    next_show = now + 1.0 / SHOW_HZ
            next_t += 1.0 / RATE_HZ
            time.sleep(max(0.0, next_t - time.time()))
    except KeyboardInterrupt:
        print("\n\nStopped.")

    with open(path, 'w', newline='') as f:
        f.write("# stiff_band_readout (read-only) id=%d label=%s start=%s\n" % (aid, args.label, stamp))
        w = csv.writer(f)
        w.writerow(['t_s', 'act_deg', 'vel_deg_s', 'err_byte', 'n_marks'])
        for r in rows:
            w.writerow(["%.3f" % r[0], "%.3f" % r[1], "%.2f" % r[2], r[3], r[4]])
        f.write("# marks (t_s, act_deg, direction)\n")
        for m in marks:
            f.write("# %.3f, %.2f, %s\n" % m)

    print("Samples %d, failed reads %d, CSV %s" % (len(rows), n_bad, path))
    if rows:
        degs = [r[1] for r in rows]
        print("Range covered: %.1f to %.1f deg" % (min(degs), max(degs)))
    if marks:
        print("Marks (act deg, direction):")
        for i, m in enumerate(marks, 1):
            print("  %d: %.2f  %s" % (i, m[1], m[2]))
    else:
        print("No marks pressed.")
    return 0


if __name__ == '__main__':
    sys.exit(main())