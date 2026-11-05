"""
Comms soak test (STATE 8 #30). Talk to the actuator for a long time exactly the
way the real scripts do, and write down every hiccup instead of stopping at the
first one.

WHY
---
Four comm failures (Oct 28-29): the last good row came 463.8 / 466.6 / 472.2 /
475.7 s after each run's traffic started (from GO), in 5 Hz read-only idle logs
and 500 Hz holds alike; measured from the port opening they spread wider
(472-510 s). No other run in raw-data has lasted longer than 353 s [verified,
raw-data CSVs, Oct 29]. Every other script aborts on the first failed
exchange, so we have only ever seen the START of a failure. This one keeps
trying every 2 ms through it and times it.

MODES (one per invocation)
--------------------------
  --mode read      motor disabled the whole time; ZERO WRITES. Every 2 ms one
                   bulk read of the 5 loop registers (the idle log's packet).
  --mode enabled0  motor enabled in torque mode, command held at exactly 0 A:
                   every 2 ms BearInterface.exchange(0.0) (the hold loop's own
                   packet). The firmware watchdog (10 ms) is the witness: after
                   an outage, torque_enable / watchdog_timeout say whether the
                   actuator kept hearing us (no trip: only replies were lost),
                   heard nothing for > 10 ms (trip: torque_enable 3), or
                   REBOOTED (torque_enable 0 and watchdog 0 = flash values).
                   After a trip or reboot: disable (confirmed), arm still on the
                   stop?, re-arm, re-enable at 0 A, carry on (<= 5 per run).

THE 470 s QUESTION (read mode only)
  --wait-s S          quiet wait between opening the port and starting the
                      traffic (default 60): separates "since port open" from
                      "since traffic start".
  --reopen-every S    close and reopen the port every S s.
  --pause-every S --pause-s P   no traffic for P s every S s, port left open.

RECORDS
  every exchange: start time, duration, ok, PyBEAR retries/warnings (captured,
  not printed), error byte (bit 0 = the actuator got a garbled packet from us),
  state, and wall clock minus monotonic clock (PyBEAR times its 2 ms timeouts
  with time.time(), the WALL clock: a clock step would fake or hide timeouts).
  Every outage (consecutive failed exchanges): when (since port open, since
  traffic start, wall clock), how long nothing came back, how many exchanges
  failed, the warnings, input voltage after, other processes holding the port
  (read mode right away; enabled0 after the disable), in enabled0 the
  actuator's own account (torque_enable, watchdog_timeout), and THE RAW BYTES:
  a passive tap under PyBEAR counts every packet sent, every byte received and
  every byte PyBEAR threw away unread at its next send (= a reply that came
  late). So each outage reads as one of: silence (nothing came back), partial
  or corrupted replies (bytes came, no valid reply), or late replies.
  Once a second (host side): CPU temperature and CPU 3 clock speed.
  Files (logs/phase2/comms/): comms_soak_<mode>_<ts>.csv = one row per second
  + '#' header/footer with every event and the summary;
  comms_soak_<mode>_<ts>_detail.csv = every exchange within 2 s of anything
  unusual, then the raw bytes around each outage. Written after the run (also
  on a stop), chowned to SUDO_UID.
  --no-tap turns the byte tap off (only to check the tap itself changes nothing).

STOPS EARLY ONLY FOR
  both: link down > 5 s; Ctrl-C.
  read: |iq| > 0.15 A; torque_enable 1, 2 or 3 (re-read every 10 s; a value
  that isn't 0-3 is a crossed reply, logged and re-read 0.5 s later).
  enabled0: |iq| >= 0.3 A for 3 cycles; arm below 105 deg; any error bit
  other than the watchdog's; 65 C; a disable not confirmed; a 6th recovery;
  a refused re-enable.

REFUSES BEFORE OPENING THE PORT: bad arguments; --expect not matching the
actual scheduler (an RT run that isn't, or the reverse); --reopen/--pause
with enabled0. REFUSES BEFORE ANY WRITE: torque_enable != 0 or a fault
(this script never clears one: run test_connection.py / power-cycle);
|iq| >= 0.15 A; and for enabled0: arm not on the top stop (>= 110 deg),
limit_i_max > 5.5, 65 C, SAFETY_CHECKS_CONFIRMED not True, no GO typed.

EVERY WRITE
  read: none (not even a disable).
  enabled0:
    1. watchdog_timeout = 10000 us, read back         (configure_watchdog)
    2. mode 0 -> goal_iq 0 -> torque_enable 1, each read back   (enable)
    3. goal_iq = 0.0 every cycle inside the one bulk packet     (exchange)
    4. after a trip / reboot: goal_iq 0, torque_enable 0 confirmed (disable),
       then 1 and 2 again
    5. at the end: disable(); then watchdog_timeout = 0, read back, ONLY if
       the disable was confirmed (otherwise left armed: "cut power").
  Never a nonzero goal_iq; never limit_i_max, gains or save_config.

RUN (repo root; nothing else may have the port open):
  RT:     sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\
              useful_tools/comms_soak_test.py --mode read --expect rt --minutes 30
  non-RT: ~/projects/bear_controller/venv/bin/python3 useful_tools/comms_soak_test.py \\
              --mode read --expect normal --minutes 20
  Offline (byte-level fake actuator under the real PyBEAR, no port):
          useful_tools/test_comms_soak_offline.py
"""
import sys
sys.dont_write_bytecode = True   # under sudo, leave no root-owned __pycache__

import argparse
import collections
import csv
import math
import os
import struct
import subprocess
import time

import numpy as np
from pybear import Packet

from main_controller.bear_interface import BearInterface
from main_controller.config import (LOG_DIR, PORT, COMM_TIMEOUT, COMM_TRY_NUM, TEMP_WARN,
                                    WATCHDOG_TIMEOUT_MS, SAFETY_CHECKS_CONFIRMED)

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'comms')

PERIOD_S = 0.002                    # 500 Hz, like every Mode 0 loop
READ_REGS = BearInterface.LOOP_READ_REGS
STATE_NAMES = ('position', 'velocity', 'iq', 'temp', 'temp_powerstage')

MINUTES_DEFAULT, MINUTES_RANGE = 30.0, (0.5, 60.0)
WAIT_S_DEFAULT, WAIT_S_RANGE = 60.0, (0.0, 600.0)
EVERY_RANGE = (10.0, 1800.0)        # --reopen-every / --pause-every
PAUSE_S_RANGE = (1.0, 300.0)
REOPEN_GAP_S = 0.5                  # port closed this long on each reopen
GIVEUP_S = 5.0                      # nothing back for this long -> stop
TE_CHECK_EVERY_S = 10.0             # read mode: re-read torque_enable this often
HOST_GAP_S = 0.010                  # loop gap (= watchdog timeout) with no failed exchange
SLOW_TX_US = 4000.0                 # an exchange this slow is "unusual" (detail window)
SLOW_DT_MS = 4.0
DETAIL_WINDOW_S = 2.0
IQ_READ_STOP_A = 0.15               # read mode: motor is off, so this means something drives it
IQ_EN_ABORT_A, IQ_EN_ABORT_N = 0.3, 3
REST_MIN_DEG = 110.0                # enabled0: start (and re-enable) only on the top stop
EN_MIN_DEG = 105.0                  # enabled0: abort if the arm leaves the stop by this much
MAX_RECOVERIES = 5
MAX_SAME_EVENTS = 200               # host-gap events listed one by one; the rest only counted
WATCHDOG_BIT = BearInterface.WATCHDOG_BIT      # 0x08
ERROR_BITS = BearInterface.ERROR_BITS          # 0x7C
COMM_WARN_BIT = 0x01                # the actuator received a corrupted packet
WARN_KINDS = ('single_timeout', 'bulk_timeout', 'max_try_num', 'bad_data')
VALID_TE = (0, 1, 2, 3)             # torque_enable values the SDK defines; anything else = crossed reply
TE_RECHECK_S = 0.5                  # read mode: re-read this soon after an implausible value
CLOCK_STEP_S = 0.001                # wall-minus-monotonic jump that counts as a clock step
TAP_RING = 400                      # raw-byte entries kept (~100 exchanges)
TAP_PRE_S = 0.020                   # bytes shown before an outage's first failure
TAP_POST_N = 40                     # ring entries shown at the outage's end
MAX_TAP_DUMPS = 20                  # outages whose raw bytes are written out
HEX_MAX = 32                        # bytes shown per entry
# What PyBEAR itself can RAISE on malformed bytes: a valid-checksum reply of the
# wrong length (a late reply crossing into another request) is decoded anyway ->
# struct.error / IndexError; a vanished port -> serial.SerialException (an OSError).
BUS_EXC = (struct.error, IndexError, ValueError, TypeError, OSError)
CPU_TEMP_PATH = '/sys/class/thermal/thermal_zone0/temp'
CPU3_FREQ_PATH = '/sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq'


class Stop(Exception):
    """End the run now (the finally: block disables if armed)."""


# ---- raw bytes on the wire, recorded without changing what is sent -------------------
class SerialTap:
    """Sits between PyBEAR and its serial port (PyBEAR's own `self.ser`).
    It passes every call straight through and only notes what happened:
      tx    - each packet PyBEAR sends (its length and instruction byte)
      rx    - each read PyBEAR does (the bytes it got)
      stale - bytes that were waiting, unread, when PyBEAR cleared the input
              buffer before its next send: a reply that arrived after PyBEAR
              had given up on it (PyBEAR would have thrown these away silently;
              the tap reads them out first, then clears as PyBEAR asked).
    Counters run for the whole run; a ring keeps the last TAP_RING entries for
    the dumps around outages. Cost: a few list appends per exchange."""

    def __init__(self, ser, ring, counts):
        self._ser = ser
        self._ring = ring
        self._c = counts

    def reset_input_buffer(self):
        n = self._ser.in_waiting
        if n:
            stale = bytes(self._ser.read(n))
            self._ring.append((time.monotonic(), 'stale', stale))
            self._c['stale_bytes'] += len(stale)
            self._c['stale_events'] += 1
        self._ser.reset_input_buffer()

    def write(self, data):
        b = bytes(data)
        self._ring.append((time.monotonic(), 'tx', b))
        self._c['tx'] += 1
        return self._ser.write(data)

    def read(self, n):
        b = bytes(self._ser.read(n))
        self._ring.append((time.monotonic(), 'rx', b))
        self._c['rx_bytes'] += len(b)
        self._c['rx_reads'] += 1
        return b

    @property
    def in_waiting(self):
        return self._ser.in_waiting

    def __getattr__(self, name):                 # close, is_open, ... unchanged
        return getattr(self._ser, name)


def tap_line(t_ref, entry):
    t, kind, b = entry
    dt = (t - t_ref) * 1e3
    if kind == 'tx':
        instr = f"instr 0x{b[4]:02x}" if len(b) > 4 else 'short'
        return f"{dt:+9.3f} ms  TX    {len(b):3d} B  {instr}"
    shown = b[:HEX_MAX].hex(' ')
    more = f" ... (+{len(b) - HEX_MAX})" if len(b) > HEX_MAX else ''
    return f"{dt:+9.3f} ms  {kind.upper():5s} {len(b):3d} B  {shown}{more}"


def host_sample():
    """CPU temperature (C) and CPU 3 clock (MHz) from sysfs; NaN where unavailable.
    One small file read each, once a second."""
    out = []
    for path, scale in ((CPU_TEMP_PATH, 1e-3), (CPU3_FREQ_PATH, 1e-3)):
        try:
            with open(path) as f:
                out.append(float(f.read().strip()) * scale)
        except (OSError, ValueError):
            out.append(float('nan'))
    return out


# ---- PyBEAR's warnings, recorded instead of printed ---------------------------------
class PyBearWarnings:
    """PyBEAR reports every retry ('... timed out. Now try again'), every give-up
    ('Maximum try number reached') and every garbled reply ('Data corrupted')
    only by printing it. This swaps Packet.PKT.print_warning for a recorder for
    the length of the run: no bus traffic changes, nothing is printed inside the
    2 ms loop (a print to the terminal can stall it), every warning gets a time."""

    def __init__(self):
        self.events = []                       # (monotonic t, kind)
        self._orig = None

    def install(self):
        self._orig = Packet.PKT.__dict__.get('print_warning')
        events = self.events

        def _record(warning=None):
            events.append((time.monotonic(), warning))
        Packet.PKT.print_warning = staticmethod(_record)

    def uninstall(self):
        if self._orig is not None:
            Packet.PKT.print_warning = self._orig
            self._orig = None


# ---- helpers -------------------------------------------------------------------------
def _git(*argv):
    try:
        r = subprocess.run(['git', '-c', f'safe.directory={REPO_DIR}', '-C', REPO_DIR, *argv],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else 'unavailable'
    except (OSError, subprocess.SubprocessError):
        return 'unavailable'


def _read_text(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return 'unavailable'


def port_holders():
    """Other processes with the serial port open (scans /proc; outside the loop only).
    A non-root run can only see its own user's processes; the result says which."""
    target = os.path.realpath(PORT)
    me, found = os.getpid(), []
    try:
        pids = [p for p in os.listdir('/proc') if p.isdigit()]
    except OSError:
        return 'unavailable'
    for pid in pids:
        if int(pid) == me:
            continue
        fd_dir = f'/proc/{pid}/fd'
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.path.realpath(os.path.join(fd_dir, fd)) == target:
                        found.append(f"{pid}:{_read_text(f'/proc/{pid}/comm')}")
                        break
                except OSError:
                    pass
        except OSError:
            pass
    who = 'root' if os.geteuid() == 0 else f'uid {os.geteuid()} (own processes only)'
    return (', '.join(found) if found else 'none') + f' [scan as {who}]'


def fmt_err(err):
    return 'None' if err is None else hex(err)


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', required=True, choices=('read', 'enabled0'))
    ap.add_argument('--expect', required=True, choices=('rt', 'normal'),
                    help='scheduling you meant to run with; refused if it is not what you got')
    ap.add_argument('--minutes', type=float, default=MINUTES_DEFAULT)
    ap.add_argument('--wait-s', type=float, default=WAIT_S_DEFAULT,
                    help='quiet wait between opening the port and starting the traffic')
    ap.add_argument('--reopen-every', type=float, default=0.0,
                    help='read mode: close and reopen the port every S s (0 = never)')
    ap.add_argument('--pause-every', type=float, default=0.0,
                    help='read mode: stop all traffic every S s (0 = never) ...')
    ap.add_argument('--pause-s', type=float, default=0.0, help='... for this long')
    ap.add_argument('--label', default='', help='free text into the header')
    ap.add_argument('--no-tap', action='store_true',
                    help='do not record raw bytes (only to check the tap changes nothing)')
    a = ap.parse_args(argv)
    if not MINUTES_RANGE[0] <= a.minutes <= MINUTES_RANGE[1]:          # NaN fails too
        ap.error(f'--minutes must be {MINUTES_RANGE[0]:g}-{MINUTES_RANGE[1]:g}')
    if not WAIT_S_RANGE[0] <= a.wait_s <= WAIT_S_RANGE[1]:
        ap.error(f'--wait-s must be {WAIT_S_RANGE[0]:g}-{WAIT_S_RANGE[1]:g}')
    if a.reopen_every and not EVERY_RANGE[0] <= a.reopen_every <= EVERY_RANGE[1]:
        ap.error(f'--reopen-every must be 0 or {EVERY_RANGE[0]:g}-{EVERY_RANGE[1]:g} s')
    if a.pause_every and not EVERY_RANGE[0] <= a.pause_every <= EVERY_RANGE[1]:
        ap.error(f'--pause-every must be 0 or {EVERY_RANGE[0]:g}-{EVERY_RANGE[1]:g} s')
    if (a.pause_every > 0) != (a.pause_s > 0):
        ap.error('give both --pause-every and --pause-s, or neither')
    if a.pause_s and not PAUSE_S_RANGE[0] <= a.pause_s <= PAUSE_S_RANGE[1]:
        ap.error(f'--pause-s must be {PAUSE_S_RANGE[0]:g}-{PAUSE_S_RANGE[1]:g} s')
    if a.pause_every and a.pause_s >= a.pause_every:
        ap.error('--pause-s must be shorter than --pause-every')
    if a.reopen_every and a.pause_every:
        ap.error('one discriminator per run: --reopen-every OR --pause-every')
    if a.mode == 'enabled0' and (a.reopen_every or a.pause_every):
        ap.error('--reopen-every / --pause-every are read mode only (silence would trip '
                 'the watchdog of an enabled motor)')
    sched = os.sched_getscheduler(0)
    if a.expect == 'rt' and sched != os.SCHED_FIFO:
        ap.error("--expect rt but not at real-time priority. From the repo root run:\n"
                 "  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 "
                 "useful_tools/comms_soak_test.py ...")
    if a.expect == 'normal' and sched == os.SCHED_FIFO:
        ap.error('--expect normal but running at SCHED_FIFO (drop sudo chrt)')
    return a


class Soak:
    def __init__(self, a):
        self.a = a
        self.warn = PyBearWarnings()
        self.header, self.events, self.outages = [], [], []
        self.end_reason = 'not started'
        self.armed = False                 # enabled0: set before configure_watchdog
        self.enabled = False
        self.recoveries = 0
        self.host_gaps = 0
        self.last_err = 0x80
        self.disable_failed = False
        self.outage = None
        self.t_open = self.t_traffic = None
        self.wall_open = None
        self.paused_s = 0.0
        n = int(a.minutes * 60.0 / PERIOD_S * 1.02) + 20000
        self.n = 0
        self.T = np.zeros(n)                               # exchange start, s since port open
        self.DUR = np.zeros(n, dtype=np.float32)           # us
        self.OK = np.zeros(n, dtype=np.int8)
        self.NW = np.zeros(n, dtype=np.int16)              # PyBEAR warnings during it
        self.ERR = np.full(n, -1, dtype=np.int16)          # -1 = None
        self.DT = np.zeros(n, dtype=np.float32)            # ms since the previous start
        self.ST = np.full((n, 5), np.nan, dtype=np.float32)
        self.WM = np.zeros(n)                              # wall clock - monotonic, s
        self.marks = []                                    # (t since open, label): detail windows
        # raw-byte tap
        self.ring = collections.deque(maxlen=TAP_RING)
        self.tap_counts = dict(tx=0, rx_bytes=0, rx_reads=0, stale_bytes=0, stale_events=0)
        self.tap_dumps = []                                # (title, [lines])
        # host witnesses
        self.host = []                                     # (t since open, cpu C, cpu3 MHz)
        self.clock_steps = 0
        self.te_implausible = 0
        self.raised = collections.Counter()               # exceptions PyBEAR raised, by type

    def call(self, fn, *args):
        """One PyBEAR call. Malformed bytes can make PyBEAR RAISE instead of returning
        None (see BUS_EXC); that is logged, counted, and treated as a failed call --
        it is exactly the kind of event this test is for."""
        try:
            return fn(*args)
        except BUS_EXC as e:
            mod = type(e).__module__
            name = type(e).__name__ if mod == 'builtins' else f"{mod}.{type(e).__name__}"
            self.raised[name] += 1
            if sum(self.raised.values()) <= MAX_SAME_EVENTS:
                self.event(time.monotonic(), f"PyBEAR RAISED {name}: {e} (malformed or crossed "
                                             f"reply); counted as a failed call")
            return None

    def read_one(self, getter, i):
        """A single-register read through call(): the value, or None."""
        r = self.call(getter, i)
        try:
            return r[0][0][0]
        except (TypeError, IndexError):
            return None

    def install_tap(self, iface):
        if not self.a.no_tap:
            iface.bear.ser = SerialTap(iface.bear.ser, self.ring, self.tap_counts)

    def tap_snapshot(self, t_from, t_to=math.inf, last_n=None):
        entries = [e for e in self.ring if t_from <= e[0] <= t_to]
        return entries[-last_n:] if last_n else entries

    # -- times --------------------------------------------------------------------------
    def since_open(self, t):
        return t - self.t_open

    def wall(self, t):
        return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.wall_open + (t - self.t_open)))

    def stamp(self, t):
        tr = f"{t - self.t_traffic:8.3f} s since traffic start, " if self.t_traffic else ''
        return f"{tr}{self.since_open(t):8.3f} s since port open, {self.wall(t)}"

    def event(self, t, text, mark=True):
        line = f"{self.stamp(t)} | {text}"
        self.events.append(line)
        if mark:
            self.marks.append((self.since_open(t), text[:40]))
        return line

    # -- preflight (reads only) ---------------------------------------------------------------
    def preflight(self, iface):
        a, bear, i = self.a, iface.bear, iface.id
        if a.mode == 'enabled0' and SAFETY_CHECKS_CONFIRMED is not True:
            raise SystemExit("REFUSED: SAFETY_CHECKS_CONFIRMED is not True in config.py")
        (te,), err = bear.get_torque_enable(i)[0]
        if te != 0 or err != 0x80:
            raise SystemExit(f"REFUSED: torque_enable={te}, error byte {fmt_err(err)}; need 0 and "
                             f"0x80. This script never clears a fault: run test_connection.py, "
                             f"or power-cycle the actuator.")
        values, err = bear.bulk_read([i], READ_REGS)[0]
        if err is None or any(v is None for v in values):
            raise SystemExit("REFUSED: could not read state.")
        pos, vel, iq, tw, tp = values
        if abs(iq) >= IQ_READ_STOP_A:
            raise SystemExit(f"REFUSED: |iq| {abs(iq):.3f} A with the motor disabled.")
        (lim,), _ = bear.get_limit_i_max(i)[0]
        (wd,), _ = bear.get_watchdog_timeout(i)[0]
        (mode,), _ = bear.get_mode(i)[0]
        (volt,), _ = bear.get_input_voltage(i)[0]
        if a.mode == 'enabled0':
            if math.degrees(pos) < REST_MIN_DEG:
                raise SystemExit(f"REFUSED: arm at {math.degrees(pos):.1f} deg; rest it on the top "
                                 f"stop (>= {REST_MIN_DEG:g} deg) first.")
            if lim is None or lim > 5.5:
                raise SystemExit(f"REFUSED: limit_i_max reads {lim}; expected <= 5.5 A.")
            if max(tw, tp) >= TEMP_WARN:
                raise SystemExit(f"REFUSED: temperature {max(tw, tp):.1f} C >= {TEMP_WARN} C.")
        try:
            aff = sorted(os.sched_getaffinity(0))
        except (AttributeError, OSError):
            aff = 'unavailable'
        try:
            prio = os.sched_getparam(0).sched_priority
        except (AttributeError, OSError):
            prio = 'unavailable'
        dirty = _git('status', '--porcelain', '--', 'useful_tools', 'main_controller')
        self.header += [
            f"comms_soak_test.py  git {_git('rev-parse', '--short', 'HEAD')} "
            f"dirty={'yes' if dirty else 'no'}",
            f"started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}  label={a.label!r}",
            f"mode={a.mode} minutes={a.minutes:g} wait_s={a.wait_s:g} reopen_every={a.reopen_every:g} "
            f"pause_every={a.pause_every:g} pause_s={a.pause_s:g}",
            f"expect={a.expect} sched={os.sched_getscheduler(0)} priority={prio} affinity={aff} "
            f"pid={os.getpid()}",
            f"COMM_TRY_NUM={COMM_TRY_NUM} COMM_TIMEOUT={COMM_TIMEOUT} period={PERIOD_S} "
            f"pybear={os.path.dirname(os.path.abspath(Packet.__file__))} "
            f"tap={'off' if a.no_tap else 'on'}",
            f"host at start: cpu {host_sample()[0]:.1f} C, cpu3 {host_sample()[1]:.0f} MHz; "
            f"wall-monotonic {time.time() - time.monotonic():.6f} s",
            f"port={PORT} -> {os.path.realpath(PORT)}  kernel {os.uname().release}  "
            f"uptime_at_open_s={_read_text('/proc/uptime').split(' ')[0]}",
            f"preflight: torque_enable {te} err {fmt_err(err)} pos {math.degrees(pos):.2f} deg "
            f"iq {iq:+.3f} A winding {tw:.2f} C powerstage {tp:.2f} C input_voltage {volt} V "
            f"limit_i_max {lim} watchdog_timeout {wd} mode {mode}",
            f"port holders at start: {port_holders()}",
        ]
        for line in self.header:
            print(line)

    # -- recording -----------------------------------------------------------------------------
    def record(self, t_a, t_b, ok, nwarn, err, values, t_prev, wm):
        k = self.n
        if k >= len(self.T):
            raise Stop("record buffer full")
        self.T[k] = t_a - self.t_open
        self.WM[k] = wm
        self.DUR[k] = (t_b - t_a) * 1e6
        self.OK[k] = 1 if ok else 0
        self.NW[k] = nwarn
        self.ERR[k] = -1 if err is None else int(err)
        self.DT[k] = 0.0 if t_prev is None else (t_a - t_prev) * 1e3
        if ok:
            self.ST[k] = values
        self.n += 1

    # -- the run ------------------------------------------------------------------------------
    def run(self, iface):
        a, bear, i = self.a, iface.bear, iface.id
        if a.wait_s > 0:
            print(f"Quiet wait {a.wait_s:g} s (port open, no traffic)...")
            time.sleep(a.wait_s)
        self.t_traffic = time.monotonic()
        self.events.append(f"{self.stamp(self.t_traffic)} | traffic starts")
        if a.mode == 'enabled0':
            self.armed = True              # from here on, exit must disable (and disarm if confirmed)
            iface.configure_watchdog(WATCHDOG_TIMEOUT_MS)
            iface.enable()
            self.enabled = True
            self.events.append(f"{self.stamp(time.monotonic())} | armed and enabled at 0 A")
        print(f"Soaking {a.minutes:g} min ({a.mode}); Ctrl-C ends it early (data is kept).")
        t_end = self.t_traffic + a.minutes * 60.0
        next_reopen = self.t_traffic + a.reopen_every if a.reopen_every else math.inf
        next_pause = self.t_traffic + a.pause_every if a.pause_every else math.inf
        next_te = self.t_traffic + TE_CHECK_EVERY_S
        t_prev = None
        last_ok_end = self.t_traffic
        over_iq = 0
        wm_prev = None
        next_host = self.t_traffic
        nxt = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= next_host:                     # host witnesses, once a second
                self.host.append((self.since_open(now), *host_sample()))
                next_host += 1.0
            if now >= t_end:
                if self.outage is not None:
                    self.event(now, f"run time up DURING an outage ({self.outage['n']} failed so far, "
                                    f"started {self.stamp(self.outage['t_first'])})")
                    self.end_reason = 'completed (time up during an outage)'
                else:
                    self.end_reason = 'completed'
                break
            if self.outage is None and now >= next_reopen:
                self.reopen(iface)
                next_reopen += a.reopen_every
                t_prev, nxt, last_ok_end = None, time.monotonic(), time.monotonic()
                continue
            if self.outage is None and now >= next_pause:
                self.event(now, f"pause {a.pause_s:g} s (no traffic, port open)")
                time.sleep(a.pause_s)
                self.paused_s += a.pause_s
                self.event(time.monotonic(), "pause ends, traffic resumes")
                next_pause += a.pause_every
                t_prev, nxt, last_ok_end = None, time.monotonic(), time.monotonic()
                continue

            w0 = len(self.warn.events)
            c0 = dict(self.tap_counts)
            t_a = time.monotonic()
            wm = time.time() - t_a
            if a.mode == 'read':
                r = self.call(bear.bulk_read, [i], READ_REGS)
                values, err = r[0] if r else (None, None)
                ok = err is not None and not any(v is None for v in values)
            else:
                r = self.call(iface.exchange, 0.0)
                state, err = r if r else (None, None)
                ok = state is not None
                values = [state[n] for n in STATE_NAMES] if ok else None
            t_b = time.monotonic()
            nw = len(self.warn.events) - w0
            gap = None if t_prev is None else t_a - t_prev
            self.record(t_a, t_b, ok, nw, err, values, t_prev, wm)
            t_prev = t_a
            if wm_prev is not None and abs(wm - wm_prev) > CLOCK_STEP_S:
                self.clock_steps += 1
                if self.clock_steps <= MAX_SAME_EVENTS:
                    self.event(t_a, f"WALL CLOCK STEP {(wm - wm_prev) * 1e3:+.3f} ms against the "
                                    f"monotonic clock (PyBEAR's timeouts use the wall clock)")
            wm_prev = wm

            if not ok:
                if self.outage is None:
                    self.outage = dict(t_first=t_a, last_ok_end=last_ok_end, n=0, w0=w0, c0=c0,
                                       pre=self.tap_snapshot(t_a - TAP_PRE_S))
                self.outage['n'] += 1
                if t_b - self.outage['last_ok_end'] > GIVEUP_S:
                    self.event(t_b, f"nothing back for > {GIVEUP_S:g} s")
                    self.dump_outage_bytes(self.outage, t_b, 'link down (run stopped)')
                    raise Stop(f"link down > {GIVEUP_S:g} s")
            else:
                if self.outage is not None:
                    recovered = self.close_outage(iface, t_a, t_b, err, w0, c0)
                    # the bookkeeping above took time on purpose; don't count it as a host gap
                    t_prev, nxt = None, time.monotonic()
                    last_ok_end = nxt
                    if recovered:                                       # enabled0: re-armed
                        over_iq, self.last_err = 0, 0x80
                        continue
                    if a.mode == 'read':
                        next_te = min(next_te, nxt + TE_RECHECK_S)      # look again soon
                elif gap is not None and gap > HOST_GAP_S:
                    self.host_gaps += 1
                    if self.host_gaps <= MAX_SAME_EVENTS:
                        self.event(t_a, f"HOST GAP {gap * 1e3:.1f} ms between exchanges, none failed")
                if err != self.last_err:                 # an event when the byte CHANGES only
                    if err != 0x80:
                        self.note_err(t_a, err)
                    self.last_err = err
                if a.mode == 'read':
                    if abs(values[2]) > IQ_READ_STOP_A:
                        raise Stop(f"|iq| {abs(values[2]):.3f} A > {IQ_READ_STOP_A} A with the motor "
                                   f"disabled: something is driving it")
                else:
                    if err & ERROR_BITS & ~WATCHDOG_BIT:
                        raise Stop(f"error byte {hex(err)}")
                    if err & WATCHDOG_BIT:       # tripped with NO failed exchange before it
                        why = ("watchdog bit set after a host gap" if gap is not None and gap > HOST_GAP_S
                               else "watchdog bit set with no failed exchange and no host gap")
                        self.recover(iface, time.monotonic(), why)
                        t_prev, nxt = None, time.monotonic()
                        last_ok_end, over_iq, self.last_err = nxt, 0, 0x80
                        continue
                    over_iq = over_iq + 1 if abs(values[2]) >= IQ_EN_ABORT_A else 0
                    if over_iq >= IQ_EN_ABORT_N:
                        raise Stop(f"|iq| >= {IQ_EN_ABORT_A} A for {IQ_EN_ABORT_N} cycles at a 0 A "
                                   f"command ({values[2]:+.3f} A)")
                    if math.degrees(values[0]) < EN_MIN_DEG:
                        raise Stop(f"arm at {math.degrees(values[0]):.1f} deg (< {EN_MIN_DEG:g}): "
                                   f"it left the top stop")
                    if max(values[3], values[4]) >= TEMP_WARN:
                        raise Stop(f"temperature {max(values[3], values[4]):.1f} C")
                last_ok_end = max(t_b, last_ok_end)      # after an outage: its checks' end, set above

            if a.mode == 'read' and self.outage is None and time.monotonic() >= next_te:
                te = self.read_one(bear.get_torque_enable, i)
                t_c = time.monotonic()
                if te is None:
                    self.event(t_c, "torque_enable check: read failed; re-reading soon")
                    next_te = t_c + TE_RECHECK_S
                    t_prev = None                # its retries took slots: not a host gap
                elif te not in VALID_TE:
                    # not a value the SDK defines: the bytes were another request's
                    # reply (a late reply crossing into this read). Not evidence of
                    # anything enabling the motor; look again soon.
                    self.te_implausible += 1
                    self.event(t_c, f"torque_enable check: implausible value {te} (a crossed / "
                                    f"late reply); re-reading in {TE_RECHECK_S:g} s")
                    next_te = t_c + TE_RECHECK_S
                    t_prev = None
                elif te != 0:
                    raise Stop(f"torque_enable reads {te}: something enabled the motor")
                else:
                    next_te = t_c + TE_CHECK_EVERY_S

            nxt += PERIOD_S
            s = nxt - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                nxt = time.monotonic()

    def note_err(self, t, err):
        bits = [n for b, n in ((0x01, 'comm-warning'), (0x02, 'overheat'), (0x04, 'abs-position'),
                               (0x08, 'watchdog/ESTOP'), (0x10, 'joint-limit'),
                               (0x20, 'hardware-fault'), (0x40, 'init')) if err & b]
        self.event(t, f"error byte {hex(err)} ({', '.join(bits) or 'bit 7 clear'})")

    def close_outage(self, iface, t_a, t_b, err, w_end, c_end):
        """The first good exchange after failed ones. 'down' = from the end of the
        last good exchange to the start of this one: an upper bound on the outage,
        resolution ~2 ms. 'wire' = what crossed the wire during the failed
        exchanges only (c_end = the tap counters when this good one started).
        Then the actuator's side of the story."""
        o = self.outage
        self.outage = None
        kinds = [k for _, k in self.warn.events[o['w0']:w_end]]
        counts = ', '.join(f"{k} x{kinds.count(k)}" for k in WARN_KINDS if kinds.count(k)) or 'none'
        down_ms = (t_a - o['last_ok_end']) * 1e3
        wire = self.wire_account(o, c_end)
        self.dump_outage_bytes(o, t_b, f"outage #{len(self.outages) + 1}")
        bear, i = iface.bear, iface.id
        volt = self.read_one(bear.get_input_voltage, i)
        rec = dict(t=o['t_first'], down_ms=down_ms, n_fail=o['n'], warnings=counts,
                   err_after=fmt_err(err), volt=volt, wire=wire)
        text = (f"OUTAGE #{len(self.outages) + 1}: nothing back for {down_ms:.1f} ms, "
                f"{o['n']} exchange(s) failed; PyBEAR: {counts}; wire: {wire}; first reply "
                f"after: error {fmt_err(err)}; input_voltage {volt} V")
        if self.a.mode == 'read':
            rec['holders'] = port_holders()
            text += f"; port holders: {rec['holders']}"
            rec['actuator'] = 'n/a (read mode)'
        else:
            te = self.read_one(bear.get_torque_enable, i)
            wd = self.read_one(bear.get_watchdog_timeout, i)
            if te == 1 and wd and not (err & WATCHDOG_BIT):
                rec['actuator'] = (f"no trip: torque_enable {te}, watchdog {wd} us -> the actuator kept "
                                   f"hearing us; only replies were lost")
            elif te == 3 or (err & WATCHDOG_BIT):
                rec['actuator'] = (f"WATCHDOG TRIPPED (torque_enable {te}, watchdog {wd} us) -> the "
                                   f"actuator heard nothing valid for > {WATCHDOG_TIMEOUT_MS} ms")
            elif te == 0 and wd == 0:
                rec['actuator'] = ("ACTUATOR RESET: torque_enable 0, watchdog 0 (flash values) -> it "
                                   "rebooted")
            else:
                rec['actuator'] = f"unclear: torque_enable {te}, watchdog {wd} us"
            text += f"; actuator: {rec['actuator']}"
        self.outages.append(rec)
        line = self.event(o['t_first'], text)
        if self.a.mode == 'read':
            print(f"\n{line}")      # never inside an enabled loop: a slow terminal could trip it
        if self.a.mode == 'enabled0' and not rec['actuator'].startswith('no trip'):
            self.recover(iface, time.monotonic(), f"after outage #{len(self.outages)}")
            return True
        return False

    def wire_account(self, o, c1):
        """What crossed the wire from the outage's first failed exchange to the
        start of the good one that ended it (counters c1)."""
        if self.a.no_tap:
            return 'tap off'
        c0 = o['c0']
        d = {k: c1[k] - c0[k] for k in c1}
        return (f"{d['tx']} packets sent, {d['rx_bytes']} bytes read back in {d['rx_reads']} reads, "
                f"{d['stale_bytes']} late bytes thrown away unread ({d['stale_events']} times)")

    def dump_outage_bytes(self, o, t_end, title):
        """Raw bytes around an outage, times relative to its first failed exchange."""
        if self.a.no_tap or len(self.tap_dumps) >= MAX_TAP_DUMPS:
            return
        t_ref = o['t_first']
        pre = [e for e in o['pre'] if e[0] < t_ref]
        during = self.tap_snapshot(t_ref, t_end)
        lines = [f"before the first failure (last {TAP_PRE_S * 1e3:.0f} ms):"]
        lines += ['  ' + tap_line(t_ref, e) for e in pre] or ['  (nothing)']
        lines.append(f"from the first failure to the end (last {TAP_POST_N} entries at most):")
        if len(self.ring) == TAP_RING and self.ring[0][0] > t_ref:
            lines.append(f"  (a long outage: only the last {TAP_RING} entries were kept)")
        lines += ['  ' + tap_line(t_ref, e) for e in during[-TAP_POST_N:]] or ['  (nothing)']
        self.tap_dumps.append((f"{title} at {self.stamp(t_ref)}", lines))

    def recover(self, iface, t, why):
        """enabled0 only: the actuator tripped or rebooted. Disable (confirmed),
        then, if allowed, re-arm and re-enable at 0 A."""
        self.event(t, f"RECOVERY ({why}): disable, re-arm, re-enable")
        self.enabled = False
        if not iface.disable():
            self.disable_failed = True
            raise Stop("disable NOT confirmed during recovery")
        self.events.append(f"{self.stamp(time.monotonic())} | disable confirmed (attempt "
                           f"{iface.last_disable_attempts}, {iface.last_disable_ms:.1f} ms); "
                           f"port holders: {port_holders()}")
        if self.recoveries >= MAX_RECOVERIES:
            raise Stop(f"{MAX_RECOVERIES} recoveries already: stopping")
        r = self.call(iface.bear.bulk_read, [iface.id], READ_REGS)
        values, err = r[0] if r else (None, None)
        if err is None or any(v is None for v in values):
            raise Stop("recovery: state read failed after the disable")
        if math.degrees(values[0]) < REST_MIN_DEG:
            raise Stop(f"recovery: arm at {math.degrees(values[0]):.1f} deg, not on the top stop")
        self.recoveries += 1
        iface.configure_watchdog(WATCHDOG_TIMEOUT_MS)
        iface.enable()
        self.enabled = True
        self.events.append(f"{self.stamp(time.monotonic())} | re-armed and re-enabled at 0 A "
                           f"(recovery {self.recoveries}/{MAX_RECOVERIES})")

    def reopen(self, iface):
        t = time.monotonic()
        self.event(t, "REOPEN: closing the port")
        iface.bear.close_port()
        time.sleep(REOPEN_GAP_S)
        iface.bear.open_port()                 # PyBEAR makes a new serial object here
        self.install_tap(iface)
        self.t_last_open = time.monotonic()
        self.event(self.t_last_open, "REOPEN: port open again")

    # -- summary and files --------------------------------------------------------------------------
    def summary(self):
        n = self.n
        out = []
        if n == 0:
            return ["no exchanges"]
        ok = self.OK[:n] == 1
        dur = self.DUR[:n][ok]
        dt = self.DT[:n][1:]
        kinds = [k for t, k in self.warn.events]
        span = self.T[n - 1] - self.T[0]
        out.append(f"exchanges {n} over {span:.1f} s ({n / max(span, 1e-9):.0f}/s), failed "
                   f"{int((~ok).sum())}, outages {len(self.outages)}, recoveries {self.recoveries}, "
                   f"paused {self.paused_s:g} s")
        out.append("PyBEAR warnings: " + (', '.join(f"{k} x{kinds.count(k)}" for k in WARN_KINDS
                                                  if kinds.count(k)) or 'none'))
        if len(dur):
            q = np.percentile(dur, [50, 99, 99.9])
            out.append(f"good exchange duration us: median {q[0]:.0f}, p99 {q[1]:.0f}, p99.9 {q[2]:.0f}, "
                       f"max {dur.max():.0f}")
        if len(dt):
            out.append(f"start-to-start ms: max {dt.max():.2f}; > 4 ms: {int((dt > 4).sum())}; "
                       f"> 10 ms: {int((dt > 10).sum())}; host gaps (> 10 ms, nothing failed): "
                       f"{self.host_gaps}")
        errs = self.ERR[:n][ok]
        if len(errs):
            out.append("error bytes on good replies: " + ', '.join(
                f"{hex(int(v))} x{int((errs == v).sum())}" for v in np.unique(errs)))
        wm = (self.WM[:n] - self.WM[0]) * 1e3
        out.append(f"wall clock vs monotonic: drift {wm[-1]:+.3f} ms over the run, range "
                   f"{wm.min():+.3f} .. {wm.max():+.3f} ms, steps > {CLOCK_STEP_S * 1e3:g} ms: "
                   f"{self.clock_steps}")
        if self.host:
            h = np.array(self.host, dtype=float)
            out.append(f"host: cpu {np.nanmin(h[:, 1]):.1f}-{np.nanmax(h[:, 1]):.1f} C, cpu3 "
                       f"{np.nanmin(h[:, 2]):.0f}-{np.nanmax(h[:, 2]):.0f} MHz")
        if not self.a.no_tap:
            c = self.tap_counts
            out.append(f"wire (tap): {c['tx']} packets sent, {c['rx_bytes']} bytes read back, "
                       f"{c['stale_bytes']} late bytes thrown away unread ({c['stale_events']} times)")
        if self.te_implausible:
            out.append(f"torque_enable checks that read an impossible value (crossed reply): "
                       f"{self.te_implausible}")
        if self.raised:
            out.append("PyBEAR raised (malformed / crossed replies): " + ', '.join(
                f"{k} x{v}" for k, v in sorted(self.raised.items())))
        for k, o in enumerate(self.outages, 1):
            out.append(f"outage {k}: {self.stamp(o['t'])}; down {o['down_ms']:.1f} ms; "
                       f"{o['n_fail']} failed; {o['warnings']}; wire: {o['wire']}; {o['actuator']}")
        if self.outages:
            first = self.outages[0]['t']
            out.append(f"first outage at {first - self.t_traffic:.1f} s since traffic start / "
                       f"{self.since_open(first):.1f} s since port open")
            out.append("look around each wall time:  journalctl --since '<time - 10 s>' --until "
                       "'<time + 10 s>'  and  sudo dmesg -T | tail -50")
        return out

    def save(self):
        made, p = [], os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        stem = f"comms_soak_{self.a.mode}_{time.strftime('%Y%m%d_%H%M%S')}"
        path = os.path.join(LOG_SUBDIR, stem + '.csv')
        dpath = os.path.join(LOG_SUBDIR, stem + '_detail.csv')
        n = self.n
        T, OK, NW, DUR, DT, ERR, ST = (self.T[:n], self.OK[:n], self.NW[:n], self.DUR[:n],
                                      self.DT[:n], self.ERR[:n], self.ST[:n])
        tr0 = (self.t_traffic - self.t_open) if self.t_traffic else 0.0
        with open(path, 'w', newline='') as f:
            for line in self.header:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(['sec', 't_open_s', 'n_tx', 'n_fail', 'n_warn', 'dur_p50_us', 'dur_max_us',
                        'dt_max_ms', 'err_or', 'n_comm_warn', 'pos_deg', 'iq_max_abs', 'w_temp', 'p_temp',
                        'wall_minus_mono_ms', 'cpu_temp_C', 'cpu3_MHz'])
            host = {}
            for th, ct, mhz in self.host:                    # one sample per traffic second
                host.setdefault(int(math.floor(th - tr0)), (ct, mhz))
            WM0 = self.WM[0] if n else 0.0
            if n:
                sec = np.floor(T - tr0).astype(int)          # T only increases: sec is sorted
                s_lo, s_hi = int(sec[0]), int(sec[-1])
                edges = np.searchsorted(sec, np.arange(s_lo, s_hi + 2), side='left')
                for j, s in enumerate(range(s_lo, s_hi + 1)):
                    a_, b_ = edges[j], edges[j + 1]
                    ct, mhz = host.get(s, (float('nan'), float('nan')))
                    hcols = ['' if math.isnan(ct) else f"{ct:.1f}", '' if math.isnan(mhz) else f"{mhz:.0f}"]
                    if b_ == a_:
                        w.writerow([s, f"{tr0 + s:.3f}", 0, 0, 0, '', '', '', '', 0, '', '', '', '', '']
                                   + hcols)
                        continue
                    sl = slice(a_, b_)
                    mo = OK[sl] == 1
                    e = ERR[sl][mo]
                    eor = int(np.bitwise_or.reduce(e)) if len(e) else None
                    st = ST[sl][mo]
                    w.writerow([s, f"{tr0 + s:.3f}", b_ - a_, int((~mo).sum()), int(NW[sl].sum()),
                                f"{np.median(DUR[sl][mo]):.0f}" if mo.any() else '',
                                f"{DUR[sl].max():.0f}",
                                f"{DT[sl].max():.2f}",
                                hex(eor) if eor is not None else '',
                                int(((e & COMM_WARN_BIT) > 0).sum()) if len(e) else 0,
                                f"{np.degrees(np.nanmean(st[:, 0])):.3f}" if len(st) else '',
                                f"{np.nanmax(np.abs(st[:, 2])):.3f}" if len(st) else '',
                                f"{np.nanmean(st[:, 3]):.2f}" if len(st) else '',
                                f"{np.nanmean(st[:, 4]):.2f}" if len(st) else '',
                                f"{(self.WM[a_:b_].max() - WM0) * 1e3:+.3f}"] + hcols)
            f.write("# --- events ---\n")
            for line in self.events:
                f.write(f"# {line}\n")
            f.write("# --- summary ---\n")
            for line in self.summary():
                f.write(f"# {line}\n")
            f.write(f"# END {self.end_reason}\n")
        # detail: every exchange within DETAIL_WINDOW_S of anything unusual
        unusual = T[(OK == 0) | (NW > 0) | (DUR > SLOW_TX_US) | (DT > SLOW_DT_MS)
                    | ((ERR != 0x80) & (ERR != -1))]
        times = np.sort(np.concatenate([unusual, np.array([m[0] for m in self.marks], dtype=float)]))
        keep = np.zeros(n, dtype=bool)
        if len(times) and n:
            lo = np.searchsorted(times, T - DETAIL_WINDOW_S, side='left')
            hi = np.searchsorted(times, T + DETAIL_WINDOW_S, side='right')
            keep = hi > lo
        with open(dpath, 'w', newline='') as f:
            f.write(f"# comms_soak_test.py detail for {os.path.basename(path)}: every exchange within "
                    f"{DETAIL_WINDOW_S:g} s of a failure, a PyBEAR warning, an exchange > "
                    f"{SLOW_TX_US:.0f} us, a gap > {SLOW_DT_MS:g} ms, an error byte != 0x80, or an event\n")
            w = csv.writer(f)
            w.writerow(['t_open_s', 't_traffic_s', 'ok', 'n_warn', 'dur_us', 'dt_ms', 'err',
                        'pos_deg', 'vel', 'iq', 'w_temp', 'p_temp'])
            for k in np.nonzero(keep)[0]:
                st = ST[k]
                w.writerow([f"{T[k]:.6f}", f"{T[k] - tr0:.6f}", int(OK[k]), int(NW[k]),
                            f"{DUR[k]:.0f}", f"{DT[k]:.3f}", 'None' if ERR[k] < 0 else hex(int(ERR[k])),
                            '' if np.isnan(st[0]) else f"{math.degrees(st[0]):.4f}",
                            '' if np.isnan(st[1]) else f"{st[1]:.4f}",
                            '' if np.isnan(st[2]) else f"{st[2]:.4f}",
                            '' if np.isnan(st[3]) else f"{st[3]:.2f}",
                            '' if np.isnan(st[4]) else f"{st[4]:.2f}"])
            f.write(f"# rows {int(keep.sum())} of {n}\n")
            if self.a.no_tap:
                f.write("# raw bytes: tap off (--no-tap)\n")
            for title, lines in self.tap_dumps:
                f.write(f"# --- raw bytes, {title} (times from its first failed exchange; TX = we "
                        f"sent, RX = PyBEAR read, STALE = arrived late, thrown away unread) ---\n")
                for line in lines:
                    f.write(f"# {line}\n")
        uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
        if os.geteuid() == 0 and uid and gid:
            for q in made + [LOG_SUBDIR, path, dpath]:
                try:
                    os.chown(q, int(uid), int(gid))
                except OSError as e:
                    print(f"WARNING: chown {q} failed: {e}")
        print(f"\nLog: {path}\n     {dpath}")
        return path, dpath


def main(argv=None):
    a = parse_args(argv)                       # SystemExit before any port is opened
    soak = Soak(a)
    soak.warn.install()
    iface = None
    started = False
    try:
        iface = BearInterface()
        soak.install_tap(iface)
        soak.t_open = time.monotonic()
        soak.wall_open = time.time()
        soak.preflight(iface)
        if a.mode == 'enabled0':
            if input("\nArm on the top stop, HANDS CLEAR. Type GO to start: ").strip() != 'GO':
                soak.end_reason = 'not started (no GO)'
                print("Not started.")
                return 1
        started = True
        soak.run(iface)
    except Stop as e:
        soak.end_reason = f"STOPPED: {e}"
        print(f"\n{soak.end_reason}")
    except RuntimeError as e:                   # configure_watchdog / enable refused
        soak.end_reason = f"REFUSED/FAILED: {e}"
        print(f"\n{soak.end_reason}")
    except BUS_EXC as e:                        # outside the guarded loop (preflight, enable)
        mod = type(e).__module__
        name = type(e).__name__ if mod == 'builtins' else f"{mod}.{type(e).__name__}"
        soak.end_reason = f"FAILED: PyBEAR raised {name}: {e}"
        print(f"\n{soak.end_reason}")
    except KeyboardInterrupt:
        soak.end_reason = 'interrupted (Ctrl-C)'
        print("\nInterrupted.")
    finally:
        try:
            if soak.armed and iface is not None:
                ok = iface.disable()
                if ok:
                    iface.bear.set_watchdog_timeout((iface.id, 0))
                    wd = soak.read_one(iface.bear.get_watchdog_timeout, iface.id)
                    soak.events.append(f"end: disable confirmed (attempt {iface.last_disable_attempts}, "
                                       f"{iface.last_disable_ms:.1f} ms); watchdog reset to {wd} us")
                else:
                    msg = ("DISABLE NOT CONFIRMED -- watchdog LEFT ARMED (trips in ~10 ms with no "
                           "traffic). CUT ACTUATOR POWER and check it before anything else.")
                    print(f"\n{msg}")
                    soak.events.append(f"end: {msg}")
        finally:
            soak.warn.uninstall()
            if started or soak.n:
                soak.events.append(f"port holders at end: {port_holders()}")
                print()
                for line in soak.summary():
                    print(line)
                soak.save()
            if iface is not None:
                try:
                    iface.bear.close_port()
                except Exception as e:             # noqa: BLE001
                    print(f"WARNING: close_port: {e!r}")
    print(f"END {soak.end_reason}")
    return 0 if soak.end_reason == 'completed' else 1


if __name__ == '__main__':
    sys.exit(main())