"""
Serial comm-budget measurement for the BEAR actuator. READ-ONLY TOOLING: it
never enables the actuator and never writes a configuration register. The one
register it writes is goal_iq, always exactly 0.0 -- see WRITES below.

Times six ways of talking to the actuator, N calls each, with
time.perf_counter() around each call:
  ping                   bear.ping(ID)
  get_present_position   bear.get_present_position(ID)            (1 register)
  get_state              BearInterface.get_state()                (4 single reads)
  bulk_read_3            bear.bulk_read([ID], [pos, vel, iq])
  bulk_rw_3_goal_iq0     bear.bulk_read_write([ID], [pos, vel, iq],
                                              ['goal_iq'], [[0.0]])
  bulk_rw_state4_goal_iq0  bulk_read_write with exactly the registers
                         get_state() reads (present_position, present_velocity,
                         present_iq, winding_temperature) + goal_iq = 0.0
and reports min / median / p99 / p99.9 / max (over ALL calls, failures
included -- a failed call still spends that time in a control loop) and the
failure count per method. Raw timings go to
logs/phase2/latency/comm_budget_<timestamp>.csv.

USAGE
-----
Run it twice, pinned to the same CPU both times, so the only difference
between the runs is the scheduling policy:

  1. plain
       taskset -c 3 python3 useful_tools/measure_comm_budget.py --fresh-boot yes

  2. real-time priority. sudo resets PATH, so name the venv's python
     explicitly (your shell expands ~ before sudo runs):
       sudo chrt -f 80 taskset -c 3 \\
           ~/projects/bear_controller/venv/bin/python3 \\
           ~/projects/bear_controller/useful_tools/measure_comm_budget.py \\
           --fresh-boot yes

Why taskset -c 3: pybear busy-waits on ser.in_waiting for every reply. At
SCHED_FIFO 80 that spin outranks the xhci IRQ thread (FIFO 50) and the
SCHED_OTHER workers that move USB-serial data into the tty buffer, so if it
shares a CPU with them it can delay the very reply it is waiting for. The xhci
interrupt has so far been serviced on CPU0. The script records its own
affinity and the per-CPU xhci IRQ counts before/after, and warns if it runs
SCHED_FIFO/RR on a CPU that has serviced xhci interrupts.

--fresh-boot yes|no is REQUIRED (no default), parsed before the port opens,
and written into the header and every CSV row. --gap-ms (default 0.5) sleeps
between calls, outside the timed region, so the process is not 100% busy:
back-to-back spinning at FIFO 80 would trip RT throttling / the fair-server
and show up as ~50 ms spikes that a periodic control loop would not see.
Use --gap-ms 0 for strictly back-to-back calls.

Order: argument checks and static code checks (no hardware) -> open port ->
FIRST hardware action: read torque_enable, exit with no writes unless it is 0
-> pybear version + bulk_read fix check -> ping scan IDs 0..10 -> read back
configuration registers and present_position -> timing -> read torque_enable
and goal_iq again -> write CSV -> under sudo, chown output to the invoking
user.

WRITES
------
To the actuator: goal_iq = 0.0, to ACTUATOR_ID only, by the two
bulk_read_write methods: 2 x (--warmup + --n) packets (10040 by default), plus
one resend of the same packet per bulk timeout (pybear retries by resending
the whole instruction, write included; the retry cap is single_try_num, not
bulk_try_num -- an SDK quirk). Nothing else: no torque_enable, no mode, no
other goal register, no configuration register, no save_config, no set_posi.
Enforced two ways: the bulk write payload is checked to be ['goal_iq'] /
[[+0.0]] before the port opens, and once BearInterface() is built every other
pybear write path on that instance (single_write -- which every set_* goes
through -- bulk_write, set_register, save_config, set_posi) is replaced by a
function that raises.

To disk: the CSV and any directories created for it. sys.dont_write_bytecode
is set so a sudo run leaves no root-owned __pycache__ in the repo.
"""
import argparse
import ast
import csv
import gc
import importlib.metadata
import inspect
import math
import os
import subprocess
import sys
import textwrap
import time

# Before importing main_controller: under sudo, don't leave root-owned
# __pycache__ directories in the repo.
sys.dont_write_bytecode = True

import numpy as np
from pybear import Manager
from pybear.CONTROL_TABLE import REGISTER

from main_controller.bear_interface import BearInterface
from main_controller.config import ACTUATOR_ID, BAUDRATE, LOG_DIR, PORT

EXPECTED_TOP_STOP_RAD = 2.0328   # rad, reference top-stop reading to compare against
SCAN_IDS = range(0, 11)          # ping-scan IDs 0..10 inclusive
CLEAN_ERR = 0x80                 # error byte with no fault bits set

BULK_READ_REGS = ['present_position', 'present_velocity', 'present_iq']
# Exactly what BearInterface.get_state() reads, in its order. Checked against
# the source below; the script refuses to run if they drift apart.
GET_STATE_REGS = ['present_position', 'present_velocity', 'present_iq',
                  'winding_temperature']
WRITE_REGS = ['goal_iq']
WRITE_DATA = [[0.0]]             # the ONLY value this script ever writes

CONFIG_REGS = [
    'p_gain_iq', 'i_gain_iq', 'd_gain_iq',
    'p_gain_id', 'i_gain_id', 'd_gain_id',
    'p_gain_velocity', 'i_gain_velocity', 'd_gain_velocity',
    'p_gain_position', 'i_gain_position', 'd_gain_position',
    'limit_i_max', 'limit_velocity_max', 'limit_acc_max',
    'limit_position_min', 'limit_position_max',
    'watchdog_timeout', 'min_voltage', 'max_voltage',
    'temp_limit_low', 'temp_limit_high',
    'homing_offset', 'mode',
]
MODE_NAMES = {0: 'torque', 1: 'velocity', 2: 'position', 3: 'force'}

# pybear call names that put a write on the wire (every set_* also counts).
WRITE_CALL_NAMES = {'single_write', '_single_write', 'bulk_write',
                    'bulk_read_write', '_bulk_read_write_stat', 'save_config',
                    '_save_config', 'set_posi', '_set_posi', 'set_register',
                    'send_packet', 'write'}
# Write paths blocked on the live BEAR instance. bulk_read_write stays open
# for the goal_iq=0.0 methods; its payload is the constant checked below.
BLOCKED_WRITE_METHODS = ('single_write', '_single_write', 'bulk_write',
                         'set_register', 'save_config', '_save_config',
                         'set_posi', '_set_posi')

OUT_DIR = os.path.join(LOG_DIR, 'phase2', 'latency')
REPO_DIR = os.path.dirname(LOG_DIR)


# --- Arguments and static checks: all BEFORE BearInterface() opens the port --
def parse_args():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fresh-boot', choices=('yes', 'no'), required=True,
                    help='actuator power-cycled, and no script has written '
                         'config registers since')
    ap.add_argument('--n', type=int, default=5000,
                    help='timed calls per method (default 5000, min 1000)')
    ap.add_argument('--warmup', type=int, default=20,
                    help='untimed calls per method before timing (default 20)')
    ap.add_argument('--gap-ms', type=float, default=0.5,
                    help='sleep between calls, outside the timed region, ms '
                         '(default 0.5; 0 = back-to-back)')
    args = ap.parse_args()
    if args.n < 1000:
        ap.error("--n must be >= 1000 (p99.9 needs at least 1000 samples)")
    if args.warmup < 0:
        ap.error("--warmup must be >= 0")
    if not math.isfinite(args.gap_ms) or args.gap_ms < 0:
        ap.error("--gap-ms must be finite and >= 0")
    return args


def _attr_calls(fn):
    """Names of attribute calls (x.name(...)) in fn's body, in source order.
    Parsed with ast so docstrings and comments can't match."""
    tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    calls.sort(key=lambda n: (n.lineno, n.col_offset))
    return [n.func.attr for n in calls]


def _is_write_call(name):
    return name.startswith('set_') or name in WRITE_CALL_NAMES


def static_checks():
    """Code checks that need no hardware. Exits on any failure."""
    # BearInterface() and get_state() must write nothing, or timing get_state()
    # through BearInterface would put extra writes on the wire.
    init_calls = _attr_calls(BearInterface.__init__)
    state_calls = _attr_calls(BearInterface.get_state)
    writes = [c for c in init_calls + state_calls if _is_write_call(c)]
    if writes:
        sys.exit(f"REFUSED: BearInterface.__init__/get_state() now call {writes}. "
                 f"Re-review this script: replicate get_state()'s reads directly "
                 f"instead of calling it.")
    print("BearInterface.__init__ calls:  " + ", ".join(init_calls)
          + "   -> no writes")
    print("BearInterface.get_state calls: " + ", ".join(state_calls)
          + "   -> no writes")

    state_regs = [c[len('get_'):] for c in state_calls if c.startswith('get_')]
    if state_regs != GET_STATE_REGS:
        sys.exit(f"REFUSED: get_state() reads {state_regs}, but GET_STATE_REGS "
                 f"is {GET_STATE_REGS}. Update GET_STATE_REGS.")

    # pybear bulk instructions (BULK_STAT) only address STATUS registers.
    print("bulk-readability (installed pybear CONTROL_TABLE):")
    not_bulk = []
    for reg in dict.fromkeys(BULK_READ_REGS + GET_STATE_REGS + WRITE_REGS):
        reg_type = REGISTER['REG_TYPE'].get(reg)
        ok = reg_type == 'stat'
        print(f"  {reg:<22} type={reg_type}  "
              f"{'bulk OK' if ok else 'CANNOT be bulk-read/written'}")
        if not ok:
            not_bulk.append(reg)
    if not_bulk:
        sys.exit(f"REFUSED: {not_bulk} are not status registers; pybear's bulk "
                 f"instruction can't address them.")
    # The R/W count byte packs reads in the high nibble, writes in the low.
    if max(len(BULK_READ_REGS), len(GET_STATE_REGS), len(WRITE_REGS)) > 15:
        sys.exit("REFUSED: more than 15 registers in one bulk packet.")

    if not (WRITE_REGS == ['goal_iq'] and len(WRITE_DATA) == 1
            and len(WRITE_DATA[0]) == 1 and WRITE_DATA[0][0] == 0.0
            and math.copysign(1.0, WRITE_DATA[0][0]) > 0):
        sys.exit(f"REFUSED: bulk write payload is {WRITE_REGS} {WRITE_DATA}; "
                 f"this script may only write goal_iq = +0.0.")


def install_write_guard(bear):
    """Replace every write path on this BEAR instance except bulk_read_write
    with one that raises. Every set_* goes through single_write."""
    def blocked(name):
        def _raise(*_a, **_k):
            raise RuntimeError(f"write guard: {name}() called. "
                               f"measure_comm_budget.py only writes goal_iq = 0.0.")
        return _raise
    for name in BLOCKED_WRITE_METHODS:
        setattr(bear, name, blocked(name))


# --- Environment snapshot (read-only) ----------------------------------------
def _sched():
    names = {os.SCHED_OTHER: 'SCHED_OTHER', os.SCHED_FIFO: 'SCHED_FIFO',
             os.SCHED_RR: 'SCHED_RR'}
    for attr in ('SCHED_BATCH', 'SCHED_IDLE'):
        if hasattr(os, attr):
            names[getattr(os, attr)] = attr
    pol = os.sched_getscheduler(0)
    return names.get(pol, str(pol)), os.sched_getparam(0).sched_priority


def _xhci_irq_counts():
    """{irq: [count per CPU]} for xhci lines in /proc/interrupts."""
    try:
        with open('/proc/interrupts') as f:
            lines = f.read().splitlines()
    except OSError:
        return {}
    ncpu = len(lines[0].split())
    out = {}
    for line in lines[1:]:
        if 'xhci' in line:
            parts = line.split()
            out[parts[0].rstrip(':')] = [int(x) for x in parts[1:1 + ncpu]]
    return out


def _read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError as e:
        return f"unavailable ({e.strerror})"


def _git(*argv):
    # -c safe.directory: under sudo, git refuses a repo owned by another user.
    try:
        r = subprocess.run(['git', '-c', f'safe.directory={REPO_DIR}',
                            '-C', REPO_DIR, *argv],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else 'unavailable'
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


# --- Timing --------------------------------------------------------------------
def _check(r):
    """ping / single read / bulk read: r = [([values...], error)]."""
    vals, err = r[0]
    return (err is not None and all(v is not None for v in vals)), err


def _check_state(r):
    """get_state() returns a dict and discards the error bytes."""
    return all(v is not None for v in r.values()), None


def time_method(fn, check, n, warmup, gap_s, out):
    """Appends into out so a partial run (Ctrl-C) is still saved."""
    sleep = time.sleep
    for _ in range(warmup):
        fn()
        if gap_s:
            sleep(gap_s)
    gc.collect()
    pc = time.perf_counter
    dt, ok, err = out['dt'], out['ok'], out['err']
    for _ in range(n):
        t0 = pc()
        r = fn()
        t1 = pc()
        dt.append(t1 - t0)
        o, e = check(r)
        ok.append(o)
        err.append(e)
        if gap_s:
            sleep(gap_s)


def stats_line(name, res):
    n = len(res['dt'])
    if n == 0:
        return f"{name:<24} n=0"
    us = np.asarray(res['dt']) * 1e6
    p50, p99, p999 = np.percentile(us, [50, 99, 99.9])
    fails = sum(1 for o in res['ok'] if not o)
    if name == 'get_state':
        dirty = 'n/a'
    else:
        dirty = sum(1 for o, e in zip(res['ok'], res['err'])
                    if o and e != CLEAN_ERR)
    return (f"{name:<24} n={n:<5} fail={fails:<4} err!=0x80={dirty!s:<5} "
            f"min={us.min():8.1f}  med={p50:8.1f}  p99={p99:8.1f}  "
            f"p99.9={p999:8.1f}  max={us.max():9.1f}  us")


# --- Output --------------------------------------------------------------------
def _makedirs_tracked(path):
    """makedirs, returning the directories that didn't exist before."""
    created = []
    p = os.path.abspath(path)
    while not os.path.isdir(p):
        created.append(p)
        p = os.path.dirname(p)
    os.makedirs(path, exist_ok=True)
    return created


def _chown_to_invoking_user(paths):
    uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
    if os.geteuid() != 0 or not uid or not gid:
        return
    for p in dict.fromkeys(paths):
        try:
            os.chown(p, int(uid), int(gid))
            print(f"chown {uid}:{gid} {p}")
        except OSError as e:
            print(f"WARNING: chown {p} failed: {e}")


def write_csv(stamp, meta, results, fresh_boot, sched_tag):
    created = _makedirs_tracked(OUT_DIR)
    path = os.path.join(OUT_DIR, f"comm_budget_{stamp}.csv")
    try:
        with open(path, 'w', newline='') as f:
            for line in meta:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(['method', 'i', 'dt_us', 'ok', 'err', 'fresh_boot', 'sched'])
            for name, res in results.items():
                for i, (dt, ok, err) in enumerate(zip(res['dt'], res['ok'], res['err'])):
                    w.writerow([name, i, f"{dt * 1e6:.3f}", int(ok),
                                '' if err is None else err, fresh_boot, sched_tag])
        print(f"\nRaw timings saved to {path}")
    finally:
        _chown_to_invoking_user(created + [OUT_DIR, path])


# --- Main ----------------------------------------------------------------------
def main():
    args = parse_args()
    static_checks()
    gap_s = args.gap_ms / 1000.0

    iface = BearInterface()        # opens the port; sends nothing
    bear = iface.bear
    install_write_guard(bear)
    try:
        return run(args, iface, bear, gap_s)
    finally:
        bear.close_port()


def run(args, iface, bear, gap_s):
    # 6. FIRST hardware action: torque_enable must read 0, or nothing is written.
    (te,), te_err = bear.get_torque_enable(ACTUATOR_ID)[0]
    print(f"\ntorque_enable = {te}  (error byte "
          f"{'none' if te_err is None else hex(te_err)}"
          f"{'' if te_err is None else ': ' + bear.decode_error(te_err)})")
    if te != 0:
        print("torque_enable is not 0 (or the read failed). Exiting with no "
              "writes and no files.")
        return 2

    stamp = time.strftime('%Y%m%d_%H%M%S')
    meta = [f"measure_comm_budget.py  started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
            f"argv: {' '.join(sys.argv)}",
            f"fresh_boot: {args.fresh_boot}",
            f"n: {args.n}  warmup: {args.warmup}  gap_ms: {args.gap_ms}",
            f"torque_enable_start: {te}  error_byte: {te_err}"]

    # 1. pybear version and the bulk_read return-line fix.
    try:
        dist_version = importlib.metadata.version('pybear')
    except importlib.metadata.PackageNotFoundError:
        dist_version = 'not found'
    print(f"\npybear: dist {dist_version}, Manager.__version__ "
          f"{Manager.__version__}, {inspect.getsourcefile(Manager)}")
    meta.append(f"pybear: dist {dist_version}  Manager.__version__ "
                f"{Manager.__version__}  {inspect.getsourcefile(Manager)}")
    fixed = True
    for fn in (Manager.BEAR.bulk_read, Manager.BEAR.bulk_read_write):
        # Code lines only, with file line numbers. The docstring is excluded:
        # its "return: list of status data, ..." line is not code.
        src_lines, first = inspect.getsourcelines(fn)
        body = ast.parse(textwrap.dedent(''.join(src_lines))).body[0].body
        if (isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            body = body[1:]
        code = [(first + i - 1, src_lines[i - 1].strip())
                for stmt in body for i in range(stmt.lineno, stmt.end_lineno + 1)]
        good = [(n, l) for n, l in code if 'for idx in range(len(bear_list))' in l]
        bad = [(n, l) for n, l in code if 'for idx in bear_list' in l]
        ok = bool(good) and not bad
        fixed &= ok
        print(f"  {fn.__name__}: "
              f"{'range(len(bear_list)) fix present' if ok else 'FIX MISSING'}")
        for n, l in good + bad:
            print(f"    line {n}: {l}")
        if not good and not bad:
            print("    neither 'for idx in range(len(bear_list))' nor "
                  "'for idx in bear_list' found in code")
        meta.append(f"{fn.__name__} fix_present: {ok}  lines: "
                    + ("; ".join(f"{n}: {l}" for n, l in good + bad) or 'none'))
    if not fixed:
        print("bulk return fix missing; the bulk methods would mis-index. "
              "Exiting with no writes.")
        return 3

    # 1. Ping scan.
    print(f"\nPing scan IDs {SCAN_IDS.start}..{SCAN_IDS.stop - 1} "
          f"(pybear warnings for silent IDs are expected):")
    found = {}
    for i in SCAN_IDS:
        (data,), err = bear.ping(i)[0]
        if err is not None:
            found[i] = (data, err)
    for i in SCAN_IDS:
        if i in found:
            data, err = found[i]
            print(f"  ID {i:2d}: answers  data={data}  error_byte={hex(err)}")
        else:
            print(f"  ID {i:2d}: -")
    meta.append("ping_scan_found: " + (", ".join(
        f"{i}(data={d},err={e})" for i, (d, e) in found.items()) or 'none'))
    if ACTUATOR_ID not in found:
        print(f"ACTUATOR_ID {ACTUATOR_ID} did not answer. Exiting with no writes.")
        return 4

    # 2. Read back configuration registers (READ ONLY).
    print("\nConfiguration registers (read back, not written):")
    for name in CONFIG_REGS:
        (val,), err = bear.single_read(ACTUATOR_ID, [name])
        if val is None:
            shown = 'READ FAILED'
        elif name == 'mode':
            shown = f"{val} ({MODE_NAMES.get(val, '?')})"
        elif isinstance(val, float):
            shown = f"{val:.6g}"
        else:
            shown = str(val)
        print(f"  {name:<20} {shown}")
        meta.append(f"cfg {name}: {shown}")

    # 7. present_position vs. the expected top stop; goal_iq before any write.
    (pos,), pos_err = bear.get_present_position(ACTUATOR_ID)[0]
    if pos is None:
        pos_line = (f"present_position: READ FAILED  expected_top_stop_rad: "
                    f"{EXPECTED_TOP_STOP_RAD}")
    else:
        d = pos - EXPECTED_TOP_STOP_RAD
        pos_line = (f"present_position_rad: {pos:.4f} ({math.degrees(pos):.2f} deg)  "
                    f"expected_top_stop_rad: {EXPECTED_TOP_STOP_RAD:.4f} "
                    f"({math.degrees(EXPECTED_TOP_STOP_RAD):.2f} deg)  "
                    f"diff_rad: {d:+.4f} ({math.degrees(d):+.2f} deg)")
    (goal_iq_start,), _ = bear.get_goal_iq(ACTUATOR_ID)[0]
    print(f"\n{pos_line}\ngoal_iq before any write: {goal_iq_start}")
    meta.append(pos_line)
    meta.append(f"goal_iq_start: {goal_iq_start}")

    # Environment.
    pol, prio = _sched()
    sched_tag = f"{pol}:{prio}"
    affinity = sorted(os.sched_getaffinity(0))
    xhci_start = _xhci_irq_counts()
    tty = os.path.basename(os.path.realpath(PORT))
    status = _git('status', '--porcelain')
    git_dirty = 'unknown' if status == 'unavailable' else ('yes' if status else 'no')
    env = [
        f"sched: {sched_tag}  affinity: {affinity}",
        f"euid: {os.geteuid()}  SUDO_USER: {os.environ.get('SUDO_USER', '')}",
        f"python: {sys.executable} {sys.version.split()[0]}",
        f"kernel: {os.uname().release} {os.uname().version}",
        f"sched_rt_runtime_us: {_read_text('/proc/sys/kernel/sched_rt_runtime_us')}",
        f"port: {PORT} ({tty})  baudrate: {BAUDRATE}  latency_timer_ms: "
        f"{_read_text(f'/sys/bus/usb-serial/devices/{tty}/latency_timer')}",
        f"pybear timeouts: single_timeout={bear.single_timeout} "
        f"single_try_num={bear.single_try_num} bulk_timeout={bear.bulk_timeout} "
        f"bulk_try_num={bear.bulk_try_num} (bulk retries are capped by "
        f"single_try_num in pybear 0.1.3)",
        f"xhci_irq_counts_start: {xhci_start}",
        f"git: {_git('rev-parse', '--short', 'HEAD')}  dirty: {git_dirty}",
    ]
    print("\n" + "\n".join(env))
    meta.extend(env)
    if pol in ('SCHED_FIFO', 'SCHED_RR'):
        irq_cpus = {cpu for counts in xhci_start.values()
                    for cpu, c in enumerate(counts) if c > 0}
        overlap = sorted(set(affinity) & irq_cpus)
        if overlap:
            warn = (f"WARNING: {pol} on CPU(s) {overlap}, which have serviced xhci "
                    f"interrupts. pybear busy-waits for replies; at RT priority "
                    f"that can starve USB completion on the same CPU. Pin with "
                    f"taskset (see USAGE).")
            print(warn)
            meta.append(warn)

    methods = [
        ('ping', lambda: bear.ping(ACTUATOR_ID), _check),
        ('get_present_position', lambda: bear.get_present_position(ACTUATOR_ID), _check),
        ('get_state', iface.get_state, _check_state),
        ('bulk_read_3', lambda: bear.bulk_read([ACTUATOR_ID], BULK_READ_REGS), _check),
        ('bulk_rw_3_goal_iq0', lambda: bear.bulk_read_write(
            [ACTUATOR_ID], BULK_READ_REGS, WRITE_REGS, WRITE_DATA), _check),
        ('bulk_rw_state4_goal_iq0', lambda: bear.bulk_read_write(
            [ACTUATOR_ID], GET_STATE_REGS, WRITE_REGS, WRITE_DATA), _check),
    ]
    meta.append(f"bulk_read_3 regs: {BULK_READ_REGS}")
    meta.append(f"bulk_rw_state4 regs (= get_state()): {GET_STATE_REGS}  "
                f"write: {WRITE_REGS}={WRITE_DATA}")
    meta.append("stats: microseconds, over ALL calls incl. failures; "
                "err!=0x80 counts successful calls with a fault bit set")

    results = {}
    completed = 'no'
    try:
        print(f"\nTiming {args.n} calls per method "
              f"(+{args.warmup} warmup, {args.gap_ms} ms gap) ...")
        for name, fn, check in methods:
            print(f"  {name} ...", flush=True)
            results[name] = {'dt': [], 'ok': [], 'err': []}
            time_method(fn, check, args.n, args.warmup, gap_s, results[name])
        completed = 'yes'
    finally:
        try:
            (te_end,), _ = bear.get_torque_enable(ACTUATOR_ID)[0]
            (goal_iq_end,), _ = bear.get_goal_iq(ACTUATOR_ID)[0]
        except Exception as e:        # don't mask the original exception
            te_end = goal_iq_end = f"read failed ({e})"
        lines = [stats_line(name, res) for name, res in results.items()]
        print("\n" + "\n".join(lines))
        print(f"torque_enable_end: {te_end}  goal_iq_end: {goal_iq_end}")
        meta.append(f"completed: {completed}")
        meta.append(f"torque_enable_end: {te_end}  goal_iq_end: {goal_iq_end}")
        meta.append(f"xhci_irq_counts_end: {_xhci_irq_counts()}")
        meta.extend(f"stats {l}" for l in lines)
        if results:
            write_csv(stamp, meta, results, args.fresh_boot, sched_tag)
    return 0


if __name__ == '__main__':
    sys.exit(main())
