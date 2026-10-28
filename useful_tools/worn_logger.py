"""
Worn, MOTOR-OFF logger (PLAN v4 3.4 Q4, H4). NEVER ENABLES TORQUE and makes
NO WRITES to the actuator (not even a disable): one bulk READ per sample.
Needs Dr. Kim's OK before a worn session.

    python3 useful_tools/worn_logger.py                      # default protocol
    python3 useful_tools/worn_logger.py --blocks mock_work:180,lower_controlled:8x3
    sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 useful_tools/worn_logger.py
                                                             # RT: steadier 500 Hz (optional)
    python3 useful_tools/worn_logger.py --selftest           # fake actuator, no port

WHAT IT RECORDS
---------------
Angle, velocity, current (to prove the motor is off) and both temperatures,
target 500 Hz, in labelled blocks you start with Enter. Default (~8 min):
  rest 10 s; mock_work 180 s x2 (realistic overhead tasks at your own pace --
  don't hold still on purpose, don't hurry); lower_controlled 8 s x3.
Columns match analyze_worn_holds.py (block, rep, t_block, position_rad, ...).

CHECKS
------
Before the vest goes on: torque_enable 0 and no fault (else REFUSED -- it will
not disable it for you, because that would be a write); |iq| < 0.15 A; a 3 s
rate test must reach >= 250 samples/s.
While logging: |iq| > 0.15 A (something is commanding torque), a real error
bit (2-6), a failed read, or torque_enable != 0 (re-read before and after every
block) -> stop at once, save what was logged, and tell
you to take the vest off.

WRITES: none to the actuator. Its own CSV only, appended between blocks
(never inside the sampling loop): logs/phase2/worn/worn_<YYYYmmdd_HHMMSS>.csv,
chowned to SUDO_UID:SUDO_GID under sudo. Human data: keep this repo public
only while Sam is the only wearer (STATE 6.3).
"""
import sys
sys.dont_write_bytecode = True

import argparse
import csv
import math
import os
import subprocess
import time

from main_controller.bear_interface import BearInterface
from main_controller.config import LOG_DIR

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'worn')
PERIOD_S = 0.002
IQ_OFF_A = 0.15
MIN_RATE_HZ = 250.0
RATE_TEST_S = 3.0
READ_REGS = ['present_position', 'present_velocity', 'present_iq',
             'winding_temperature', 'powerstage_temperature']
ERROR_BITS = 0x7C

INSTRUCTIONS = {
    'rest': "Stand relaxed, arm resting at your side / bottom of travel. Don't move.",
    'mock_work': ("Do overhead work as you would on the job, at your own pace: hold a part "
                  "up to the ceiling, drive screws or 'drill' at overhead points, reach, "
                  "place, adjust, lower when you'd naturally lower. Don't hold still on "
                  "purpose and don't hurry."),
    'lower_controlled': "From overhead, lower the arm under control back to the bottom.",
    'static_hold_high': "Raise the arm overhead and HOLD it still.",
}
DEFAULT_BLOCKS = 'rest:10,mock_work:180x2,lower_controlled:8x3'
COLUMNS = ['block', 'rep', 't_block', 't', 'loop_dt', 'position_rad', 'position_deg',
           'velocity_rad_s', 'iq_A', 'winding_temp_C', 'powerstage_temp_C', 'error']


class Stop(Exception):
    pass


_REAL_IFACE = BearInterface


def parse_blocks(spec):
    """'label:seconds[xN],...' -> [(label, seconds, reps)]"""
    out = []
    for item in spec.split(','):
        item = item.strip()
        if not item:
            continue
        label, _, rest = item.partition(':')
        secs, _, reps = rest.partition('x')
        label = label.strip()
        secs, reps = float(secs), int(reps or 1)
        if not label.replace('_', '').isalnum():
            raise ValueError(f"bad block label {label!r}")
        if not 1.0 <= secs <= 600.0 or not 1 <= reps <= 10:
            raise ValueError(f"block {item!r}: seconds 1-600, reps 1-10")
        out.append((label, secs, reps))
    if not out:
        raise ValueError("no blocks")
    return out


def _git(*argv):
    try:
        r = subprocess.run(['git', '-c', f'safe.directory={REPO_DIR}', '-C', REPO_DIR, *argv],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else 'unavailable'
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


def read(iface):
    values, err = iface.bear.bulk_read([iface.id], READ_REGS)[0]
    if err is None or any(v is None for v in values):
        return None, None
    return values, err


def check_disabled(iface, when):
    """torque_enable can't be bulk-read, so it is re-read (a single READ) before and after
    every block, outside the sampling loop. Something else enabling the motor while it
    draws < 0.15 A would otherwise go unnoticed (Claude Code review, Oct 27)."""
    (te,), err = iface.bear.get_torque_enable(iface.id)[0]
    if te != 0:
        raise Stop(f"torque_enable reads {te} {when}: something enabled the motor")


def preflight(iface):
    (te,), err = iface.bear.get_torque_enable(iface.id)[0]
    if te != 0 or err != 0x80:
        raise SystemExit(f"REFUSED: torque_enable={te}, error byte "
                         f"{err if err is None else hex(err)}. This logger never writes, so it "
                         f"won't disable it: run test_connection.py / power-cycle first.")
    v, err = read(iface)
    if v is None:
        raise SystemExit("REFUSED: could not read state.")
    if abs(v[2]) >= IQ_OFF_A:
        raise SystemExit(f"REFUSED: |iq| {abs(v[2]):.3f} A with the motor disabled.")
    n, t0 = 0, time.monotonic()
    nxt = t0
    while time.monotonic() - t0 < RATE_TEST_S:
        if read(iface)[0] is not None:
            n += 1
        nxt += PERIOD_S
        s = nxt - time.monotonic()
        if s > 0:
            time.sleep(s)
        else:
            nxt = time.monotonic()
    rate = n / (time.monotonic() - t0)
    if rate < MIN_RATE_HZ:
        raise SystemExit(f"REFUSED: {rate:.0f} samples/s < {MIN_RATE_HZ:.0f}. Try RT priority "
                         f"(sudo chrt -f 80 taskset -c 3 ...).")
    return [f"worn_logger.py  git {_git('rev-parse', '--short', 'HEAD')}",
            f"started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
            f"preflight: pos {math.degrees(v[0]):.2f} deg, iq {v[2]:+.3f} A, winding {v[3]:.1f} C, "
            f"powerstage {v[4]:.1f} C, rate test {rate:.0f}/s, sched {os.sched_getscheduler(0)}",
            "motor OFF throughout (no writes of any kind)"]


def record_block(iface, label, rep, secs, t_run0):
    rows = []
    t0 = time.monotonic()
    nxt, prev = t0, t0
    while True:
        now = time.monotonic()
        if now - t0 >= secs:
            break
        v, err = read(iface)
        t = time.monotonic()
        if v is None:
            raise Stop("read failed")
        rows.append([label, rep, f"{t - t0:.6f}", f"{t - t_run0:.6f}", f"{t - prev:.6f}",
                     f"{v[0]:.6f}", f"{math.degrees(v[0]):.4f}", f"{v[1]:.5f}", f"{v[2]:.4f}",
                     f"{v[3]:.2f}", f"{v[4]:.2f}", err])
        prev = t
        if abs(v[2]) > IQ_OFF_A:
            raise Stop(f"|iq| {abs(v[2]):.3f} A > {IQ_OFF_A} A: something is commanding torque")
        if err & ERROR_BITS:
            raise Stop(f"error byte {hex(err)}")
        nxt += PERIOD_S
        s = nxt - time.monotonic()
        if s > 0:
            time.sleep(s)
        else:
            nxt = time.monotonic()
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--blocks', default=DEFAULT_BLOCKS,
                    help=f"label:seconds[xN],... (default {DEFAULT_BLOCKS})")
    ap.add_argument('--wearer', default='sam')
    ap.add_argument('--note', default='', help='free text into the header (tool, task, ...)')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)
    if a.selftest:
        return selftest()
    try:
        blocks = parse_blocks(a.blocks)
    except ValueError as e:
        ap.error(str(e))
    if a.wearer.lower() != 'sam':
        print("REMINDER: someone other than Sam -> human data must go to a PRIVATE repo, and "
              "this needs its own sign-off (STATE 6.3).")
    iface = BearInterface()
    path, header, n_rows, why = None, [], 0, 'completed'
    try:
        print("PREFLIGHT -- the vest should NOT be worn yet.")
        header = preflight(iface)
        header += [f"wearer={a.wearer} note={a.note!r}", f"blocks={a.blocks}"]
        for l in header:
            print(l)
        made, p = [], os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        path = os.path.join(LOG_SUBDIR, f"worn_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            for l in header:
                f.write(f"# {l}\n")
            csv.writer(f).writerow(COLUMNS)
        _chown(made + [LOG_SUBDIR, path])
        input("\nPut the vest on now. Press Enter when ready: ")
        t_run0 = time.monotonic()
        for label, secs, reps in blocks:
            for rep in range(1, reps + 1):
                print(f"\n--- {label} (rep {rep}/{reps}, {secs:.0f} s)\n    "
                      f"{INSTRUCTIONS.get(label, '(no preset instruction)')}")
                input("    Press Enter to start recording: ")
                check_disabled(iface, 'before the block')
                print("    RECORDING ...")
                try:
                    rows = record_block(iface, label, rep, secs, t_run0)
                except Stop as e:
                    why = f"STOPPED: {e}"
                    raise
                with open(path, 'a', newline='') as f:
                    csv.writer(f).writerows(rows)
                check_disabled(iface, 'after the block')
                n_rows += len(rows)
                print(f"    {len(rows)} samples ({len(rows) / secs:.0f}/s) saved.")
    except Stop as e:
        if not why.startswith('STOPPED'):
            why = f"STOPPED: {e}"
        print(f"\n{why}. TAKE THE VEST OFF and check the actuator before anything else.")
    except KeyboardInterrupt:
        why = 'interrupted (Ctrl-C); completed blocks are saved'
        print(f"\n{why}")
    finally:
        if path:
            with open(path, 'a') as f:
                f.write(f"# END {why}; {n_rows} rows\n")
            print(f"\nLog: {path}")
        iface.bear.close_port()
    return 0 if why == 'completed' else 1


def _chown(paths):
    uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
    if os.geteuid() == 0 and uid and gid:
        for q in paths:
            try:
                os.chown(q, int(uid), int(gid))
            except OSError as e:
                print(f"WARNING: chown {q} failed: {e}")


# ---------------------------------------------------------------- self-test
def selftest():
    """Fake actuator (test_hold_bench_offline.FakeBear, disabled) with a
    scripted arm: up-stroke, 4 s hold, lowering. Checks: zero writes; the CSV
    is read by analyze_worn_holds.py, which finds the hold and the lowering;
    the |iq| stop and a refused enabled actuator."""
    import builtins
    import contextlib
    import io
    import shutil
    import tempfile
    from unittest import mock
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import test_hold_bench_offline as T
    import analyze_worn_holds as W
    me = sys.modules[__name__]
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")

    def scripted(fake):
        t = fake.clock.t - fake.t_start
        def deg(t):
            if t < 1: return 0.0
            if t < 2: x = t - 1; return 80 * (10 * x**3 - 15 * x**4 + 6 * x**5)
            if t < 6: return 80.0
            if t < 7: x = t - 6; return 80 - 80 * (10 * x**3 - 15 * x**4 + 6 * x**5)
            return 0.0
        return math.radians(deg(t))

    def run(fake, argv, typed=''):
        clock = fake.clock
        logdir = tempfile.mkdtemp(prefix='worn_selftest_')
        out = io.StringIO()

        real_cls = me.BearInterface if isinstance(me.BearInterface, type) else _REAL_IFACE

        def factory():
            iface = real_cls.__new__(real_cls)
            iface.bear, iface.id = fake, 1
            return iface
        with mock.patch.object(me, 'BearInterface', factory), \
                mock.patch.object(me, 'LOG_SUBDIR', logdir), \
                mock.patch.object(time, 'monotonic', clock.monotonic), \
                mock.patch.object(time, 'sleep', clock.sleep), \
                mock.patch.object(builtins, 'input', lambda p='': (setattr(fake, 't_start', clock.t), typed)[1]), \
                contextlib.redirect_stdout(out):
            try:
                rc = main(argv)
            except SystemExit as e:
                rc = f"SystemExit({e.code})"
        files = os.listdir(logdir)
        p = os.path.join(logdir, files[0]) if files else None
        return rc, out.getvalue(), p, logdir

    # 1. normal run with a scripted hold
    clock = T.Clock()
    fake = T.FakeBear(clock, start_deg=0.0)
    fake.t_start = clock.t
    real_read = fake._read
    fake._read = lambda name: (scripted(fake) if name == 'present_position' else
                               (0.0 if name == 'present_velocity' else real_read(name)))
    rc, out, p, logdir = run(fake, ['--blocks', 'mock_work:9'])
    check("completed", rc == 0, out.strip().splitlines()[-1] if out else '')
    check("ZERO writes to the actuator", not fake.writes, str(fake.writes[:3]))
    if p:
        # velocity column: rebuild from position for the analyzer (the fake's velocity is 0)
        blocks = W.load(p)
        b = blocks[0]
        import numpy as np
        b['vel'] = np.gradient(b['pos'], b['t'])
        r = W.analyse_block(b, 20.0)
        oh = [h for h in r['holds'] if h['overhead']]
        check("analyze_worn_holds reads it and finds the 4 s hold", len(oh) == 1 and
              abs(oh[0]['dur'] - 4.0) < 0.4, f"{[round(h['dur'], 2) for h in oh]}")
        check("...and the lowering", len(r['lowerings']) == 1)
        rate = len(b['t']) / (b['t'][-1] - b['t'][0])
        check("~500 samples/s", 450 < rate < 520, f"{rate:.0f}/s")
    shutil.rmtree(logdir)

    # 2. motor current appears mid-block -> stop
    clock = T.Clock()
    fake = T.FakeBear(clock, start_deg=0.0)
    fake.t_start = clock.t
    fake.iq_override = lambda x: 0.5 if x.clock.t - x.t_start > 2.0 else None
    rc, out, p, logdir = run(fake, ['--blocks', 'mock_work:5'])
    check("|iq| > 0.15 A -> stops, says take the vest off", rc == 1 and 'TAKE THE VEST OFF' in out)
    check("...still zero writes", not fake.writes)
    shutil.rmtree(logdir)

    # 2b. something enables the motor mid-block (drawing no current) -> caught after the block
    clock = T.Clock()
    fake = T.FakeBear(clock, start_deg=0.0)
    fake.t_start = clock.t
    orig = fake._read
    fake._read = lambda name: (1 if name == 'torque_enable' and clock.t - fake.t_start > 2.0
                               else orig(name))
    rc, out, p, logdir = run(fake, ['--blocks', 'mock_work:5'])
    check("motor enabled by something else mid-block -> stops after the block",
          rc == 1 and 'torque_enable reads 1' in out and not fake.writes)
    shutil.rmtree(logdir)

    # 3. actuator enabled -> refused, no writes
    clock = T.Clock()
    fake = T.FakeBear(clock, start_deg=0.0)
    fake.te_visible = 1
    rc, out, p, logdir = run(fake, ['--blocks', 'mock_work:5'])
    check("enabled actuator -> REFUSED, no writes", 'REFUSED' in str(rc) and not fake.writes)
    shutil.rmtree(logdir)
    check("bad --blocks refused", _bad_blocks_refused())
    n = sum(results)
    print(f"\n{n}/{len(results)} self-checks passed")
    return 0 if n == len(results) else 1


def _bad_blocks_refused():
    for spec in ('mock_work:700', 'mock_work:10x20', 'bad label:10', ''):
        try:
            parse_blocks(spec)
            return False
        except ValueError:
            pass
    return True


if __name__ == '__main__':
    sys.exit(main())