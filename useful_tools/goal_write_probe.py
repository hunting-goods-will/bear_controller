"""
goal_position write probe (Oct 14). NEVER ENABLES TORQUE.

Why: on Oct 14 the confirmed enable_position_mode() refused at its goal
read-back: wrote goal_position 2.03519 rad, read back 2.02087 rad straight
after (observer_excitation.py sweep, arm resting at ~116.6 deg). The check
worked (no torque was enabled); this probe finds out WHY the write did not
show, so the fix is chosen from data, not guessed.

Hypotheses it separates:
  H1  the write lands, but a read straight after it is too early (as seen with
      torque_enable on Oct 13) -> the value appears after a few ms of polling.
  H2  a goal write straight after a mode change is lost or overwritten -> fails
      in trial B, works in trials A and C.
  H3  goal writes do not take while disabled at all -> never matches anywhere.

Trials (5 each, torque_enable stays 0 throughout):
  A  mode already 2: write goal = present position, poll the read-back.
  B  mode 0 -> mode 2 (each confirmed), then IMMEDIATELY write goal, poll.
  C  as B, but wait 50 ms after the mode change before writing the goal.
Each trial writes a goal that DIFFERS from what the register holds (present
position, or present - 0.005 rad = 0.29 deg, alternating), so a lost write can't
look like a match. Torque is never enabled, so the goal moves nothing; the last
goal written in finally is the present position. Polls every read for up to
100 ms and records time-to-match and the first value seen.

Writes, in order: mode (0 or 2), goal_position (present, or present - 0.005
rad); in finally: goal_position = present (mode 2), mode restored to what it was
at the start, then disable() (goal_iq 0, torque_enable 0, confirmed). No
save_config, no torque_enable 1, ever.

Run from the repo root (no sudo needed; no control loop):
  ~/projects/bear_controller/venv/bin/python3 useful_tools/goal_write_probe.py
Output: printed table + logs/phase2/probes/goal_write_probe_<timestamp>.csv
"""
import sys
sys.dont_write_bytecode = True

import csv
import math
import os
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import LOG_DIR

TRIALS = 5
POLL_S = 0.100          # poll the goal read-back for up to this long
MATCH_TOL = 1e-4        # rad, same as enable_position_mode()
MODE_POLL_S = 0.020     # mode read-back poll
WAIT_C_S = 0.050        # trial C: wait after the mode change
GAP_S = 0.100           # between trials
OFFSET = 0.005          # rad; alternate targets so each write changes the register


def read1(fn, iface):
    (v,), err = fn(iface.id)[0]
    return v, err


def set_mode_confirmed(iface, mode):
    iface.bear.set_mode((iface.id, mode))
    t0 = time.monotonic()
    while time.monotonic() - t0 < MODE_POLL_S:
        m, _ = read1(iface.bear.get_mode, iface)
        if m == mode:
            return (time.monotonic() - t0) * 1000
    raise RuntimeError(f"mode did not read back {mode} within {MODE_POLL_S * 1000:.0f} ms")


def goal_trial(iface):
    """Write a goal that differs from the register's current value (present
    position, or present - OFFSET), poll the read-back. Returns a dict."""
    present, _ = read1(iface.bear.get_present_position, iface)
    before, _ = read1(iface.bear.get_goal_position, iface)
    pos = present
    if before is not None and abs(before - present) <= MATCH_TOL:
        pos = present - OFFSET
    iface.bear.set_goal_position((iface.id, pos))
    t0 = time.monotonic()
    first, reads, match_ms, last = None, 0, None, None
    while time.monotonic() - t0 < POLL_S:
        g, _ = read1(iface.bear.get_goal_position, iface)
        reads += 1
        if first is None:
            first = g
        last = g
        if g is not None and abs(g - pos) <= MATCH_TOL:
            match_ms = (time.monotonic() - t0) * 1000
            break
    return dict(pos=pos, before=before, first=first, last=last, reads=reads,
                match_ms=match_ms)


def main():
    iface = BearInterface()
    rows = []
    mode0 = None
    try:
        te, err = read1(iface.bear.get_torque_enable, iface)
        mode0, _ = read1(iface.bear.get_mode, iface)
        pos, _ = read1(iface.bear.get_present_position, iface)
        goal, _ = read1(iface.bear.get_goal_position, iface)
        print(f"start: torque_enable {te}, error {err}, mode {mode0}, present "
              f"{pos:.5f} rad ({math.degrees(pos):.2f} deg), goal_position {goal} "
              f"({'' if goal is None else f'{math.degrees(goal):.2f} deg'})")
        if te != 0 or err != 0x80:
            raise SystemExit("REFUSED: actuator must be disabled with no fault (te 0, 0x80).")

        for kind in ('A', 'B', 'C'):
            for k in range(1, TRIALS + 1):
                mode_ms = None
                if kind == 'A':
                    if read1(iface.bear.get_mode, iface)[0] != 2:
                        set_mode_confirmed(iface, 2)
                        time.sleep(GAP_S)
                else:
                    set_mode_confirmed(iface, 0)
                    time.sleep(0.02)
                    mode_ms = set_mode_confirmed(iface, 2)
                    if kind == 'C':
                        time.sleep(WAIT_C_S)
                r = goal_trial(iface)
                r.update(trial=kind, k=k, mode_ms=mode_ms)
                rows.append(r)
                ok = r['match_ms'] is not None
                print(f"  {kind}{k}: wrote {r['pos']:.5f}  goal before {r['before']}  "
                      f"first read {r['first']}  "
                      f"{'MATCH at ' + format(r['match_ms'], '.1f') + ' ms (read ' + str(r['reads']) + ')' if ok else 'NO MATCH in 100 ms (last ' + str(r['last']) + ')'}")
                time.sleep(GAP_S)
    finally:
        try:                      # leave the goal register at the present position
            if read1(iface.bear.get_mode, iface)[0] != 2:
                set_mode_confirmed(iface, 2)
            p_now, _ = read1(iface.bear.get_present_position, iface)
            iface.bear.set_goal_position((iface.id, p_now))
            read1(iface.bear.get_goal_position, iface)
        except Exception as e:
            print(f"WARNING: could not reset goal_position: {e}")
        try:
            if mode0 is not None:
                set_mode_confirmed(iface, mode0)
        except Exception as e:
            print(f"WARNING: could not restore mode {mode0}: {e}")
        iface.disable()
        if rows:
            d = os.path.join(LOG_DIR, 'phase2', 'probes')
            os.makedirs(d, exist_ok=True)
            p = os.path.join(d, f"goal_write_probe_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            with open(p, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['trial', 'k', 'mode_ms', 'present', 'goal_before', 'first_read',
                            'last_read', 'reads', 'match_ms'])
                for r in rows:
                    w.writerow([r['trial'], r['k'], r['mode_ms'], r['pos'], r['before'],
                                r['first'], r['last'], r['reads'], r['match_ms']])
            print(f"Log: {p}")
    print("\nSummary (matches / trials; first-read matches):")
    for kind in ('A', 'B', 'C'):
        rr = [r for r in rows if r['trial'] == kind]
        if rr:
            m = [r for r in rr if r['match_ms'] is not None]
            first_ok = [r for r in rr if r['first'] is not None and abs(r['first'] - r['pos']) <= MATCH_TOL]
            ms = ', '.join(f"{r['match_ms']:.1f}" for r in m)
            print(f"  {kind}: {len(m)}/{len(rr)} matched ({ms} ms); "
                  f"{len(first_ok)}/{len(rr)} on the first read")


if __name__ == '__main__':
    main()