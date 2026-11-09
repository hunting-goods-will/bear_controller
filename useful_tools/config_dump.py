"""
config_dump.py -- READ-ONLY. Print and save every configuration register of one actuator.

Use it
  * before touching a unit you haven't used yet (the healthy one): its ID, homing offset,
    gains and limits go on record before anything else is done to it;
  * before and after an ID change or any other config write, to see exactly what changed;
  * with --compare-sticky, to check a unit against the Oct 6 flash readback of the sticky unit.

Sends NOTHING except pings and reads. A wrapper raises on every other PyBEAR call.
It does not enable, disable, clear errors or save anything.

  venv/bin/python3 useful_tools/config_dump.py --id 1 --label "healthy unit, bench, before anything"
  venv/bin/python3 useful_tools/config_dump.py --id 1 --compare-sticky

Only ONE actuator should be on the bus if both are ID 1 (two answers on top of each other).
The script pings 5 times and stops if the replies are not clean and identical.

Reading right after power-up shows flash. Reading after another script has run shows RAM, which can
differ from flash (a script may have changed mode, limits or gains for its run). For a flash readback,
power-cycle the actuator first and run nothing in between.

Saves logs/phase2/config/config_dump_id<ID>_<time>.json
Exit: 0 done; 1 no clean answer; 3 done, but --compare-sticky found differences.
"""
import argparse
import json
import math
import os
import sys
from datetime import datetime

from pybear import Manager
from pybear.CONTROL_TABLE import REGISTER
from main_controller.config import LOG_DIR, PORT, BAUDRATE, COMM_TIMEOUT, COMM_TRY_NUM

LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'config')
CFG_NAMES = [n for n in REGISTER['REG_TYPE'] if REGISTER['REG_TYPE'][n] == 'cfg']

# Flash readback of the sticky unit, Oct 6 (STATE section 2.2): (value, absolute tolerance)
STICKY_FLASH = {
    'mode': (0, 0), 'homing_offset': (-20.3786, 2e-4),
    'p_gain_iq': (0.02, 1e-6), 'i_gain_iq': (0.02, 1e-6), 'd_gain_iq': (0.0, 1e-9),
    'p_gain_id': (0.02, 1e-6), 'i_gain_id': (0.02, 1e-6), 'd_gain_id': (0.0, 1e-9),
    'p_gain_velocity': (4.5, 1e-5), 'i_gain_velocity': (0.001, 1e-8), 'd_gain_velocity': (0.0, 1e-9),
    'p_gain_position': (5.0, 1e-5), 'i_gain_position': (0.0, 1e-9), 'd_gain_position': (0.2, 1e-6),
    'limit_i_max': (5.5, 1e-5), 'limit_velocity_max': (1.0, 1e-6), 'limit_acc_max': (5.0, 1e-5),
    'limit_position_min': (-0.0873, 1e-4), 'limit_position_max': (2.1817, 1e-4),
    'watchdog_timeout': (0, 0), 'min_voltage': (6.0, 1e-5), 'max_voltage': (60.0, 1e-4),
    'temp_limit_low': (80.0, 1e-4), 'temp_limit_high': (100.0, 1e-4),
}


class ReadOnly:
    """Only ping, single_read and get_* reach PyBEAR. Anything else raises."""

    def __init__(self, bear):
        object.__setattr__(self, '_bear', bear)

    def __getattr__(self, name):
        if name in ('ping', 'single_read') or name.startswith('get_'):
            return getattr(self._bear, name)
        raise RuntimeError("config_dump is read-only; blocked call: %s" % name)

    def __setattr__(self, name, value):
        raise RuntimeError("config_dump is read-only; blocked attribute write: %s" % name)


def ping_word(bear, aid):
    try:
        (data,), err = bear.ping(aid)[0]
    except Exception:
        return None
    return data if (data is not None and err is not None) else None


def read_reg(bear, aid, name):
    try:
        (v,), err = bear.single_read(aid, [name])
    except Exception:
        return None, None
    return v, err


def main(argv=None):
    ap = argparse.ArgumentParser(description="Read-only configuration dump")
    ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--label', default='')
    ap.add_argument('--compare-sticky', action='store_true',
                    help="compare with the Oct 6 flash readback of the sticky unit")
    args = ap.parse_args(argv)

    raw = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
    raw.single_timeout = COMM_TIMEOUT
    raw.single_try_num = COMM_TRY_NUM
    bear = ReadOnly(raw)
    aid = args.id

    words = [ping_word(bear, aid) for _ in range(5)]
    if None in words or len(set(words)) != 1:
        print("ID %d did not give 5 clean, identical pings (%s). Two actuators with the same ID on the bus,\n"
              "a bad connection, or the wrong ID. Nothing was written." % (aid, words))
        return 1
    te, err = read_reg(bear, aid, 'torque_enable')
    pos, _ = read_reg(bear, aid, 'present_position')
    print("ID %d: ping word %s, torque_enable %s, error byte %s, position %s" % (
        aid, words[0], te, hex(err) if err is not None else None,
        "%.2f deg" % math.degrees(pos) if pos is not None else None))

    vals, bad = {}, []
    for n in CFG_NAMES:
        v, e = read_reg(bear, aid, n)
        if v is None:
            bad.append(n)
        else:
            vals[n] = v
    print("\n%-22s %s" % ("register", "value"))
    for n in CFG_NAMES:
        v = vals.get(n)
        extra = ''
        if n == 'homing_offset' and v is not None:
            extra = '   (%.2f deg)' % math.degrees(v)
        elif n.startswith('limit_position') and v is not None:
            extra = '   (%.2f deg)' % math.degrees(v)
        print("%-22s %s%s" % (n, 'UNREADABLE' if v is None else repr(v), extra))
    if bad:
        print("\nCould not read: %s" % ', '.join(bad))

    mism = []
    if args.compare_sticky:
        for n, (want, tol) in STICKY_FLASH.items():
            got = vals.get(n)
            if got is None or abs(got - want) > tol:
                mism.append((n, got, want))
        print()
        if mism:
            print("Differs from the Oct 6 flash readback of the sticky unit:")
            for n, got, want in mism:
                print("   %-20s read %s, Oct 6 %s" % (n, got, want))
        else:
            print("Matches the Oct 6 flash readback of the sticky unit on all 24 checked registers.")

    os.makedirs(LOG_SUBDIR, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_SUBDIR, 'config_dump_id%d_%s.json' % (aid, stamp))
    with open(path, 'w') as f:
        json.dump(dict(script='config_dump.py', time=stamp, id=aid, label=args.label, ping_word=words[0],
                       torque_enable=te, error_byte=err, present_position_rad=pos, config=vals,
                       unreadable=bad, vs_sticky_oct6=mism if args.compare_sticky else None), f, indent=1)
    print("\nSaved %s" % path)
    return 3 if mism else 0


if __name__ == '__main__':
    sys.exit(main())