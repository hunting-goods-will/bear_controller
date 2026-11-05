"""
Offline test for comms_soak_test.py. NO SERIAL PORT.

The fake sits BELOW PyBEAR: serial.Serial inside pybear.Packet is replaced by
FakeSerial, which speaks the BEAR byte protocol (instruction packets in, status
packets out, checksums, 0xFE bulk), so PyBEAR's real code runs unmodified:
its timeouts, resends, give-ups, checksum checks, and the input-buffer reset
before each send. Time is simulated (time.time / monotonic / sleep patched; a
poll of in_waiting costs POLL_S, a reply arrives REPLY_S after the request).

The fake actuator (on the top stop, nothing moves):
  * registers by address (configuration and status separately), flash values
    at power-up: mode 0, watchdog 0, limit_i_max 5.5, goal_iq 0, torque_enable 0
  * writes are fire-and-forget (no reply), as on the real bus
  * torque_enable changes become visible 1 ms later (bench, Oct 13)
  * watchdog: enabled, Mode 0, timeout set, and no VALID packet received for
    longer than it -> error |= 0x08, torque_enable 3, current 0; reads count as
    traffic; torque_enable = 0 clears it
  * goal_iq ignored outside modes 0/3 and above limit_i_max (SDK 1.2.0)
  * present_iq follows goal_iq when enabled in Mode 0 (else 0)

Faults injected by time window (seconds of SIMULATED time after the first port open):
  reply_loss  - the actuator gets our packets (and acts on them) but no reply comes back
  cmd_loss    - nothing reaches the actuator (no action, no reply)
  garble      - the reply arrives with a bad checksum
  garble_cmd  - our packet arrives corrupted: no action, no reply, and the next
                good reply carries error bit 0 (Communication warning)
  late        - the reply comes 3 ms late (lands while PyBEAR waits on a LATER request)
  late8       - the reply comes 8 ms late (after PyBEAR has given up on it)
  reboot      - the actuator resets to flash values and is silent for 300 ms
  host_stall  - the Pi freezes for 15 ms inside one sleep (no exchange fails)
One-shot faults (act.once): the next N packets arriving after a time get the fault,
so a single lost reply always lands on an exchange (a sub-ms window may not).
Wall-clock step (clock.wall_steps): time.time() jumps; time.monotonic() doesn't.

Run from the repo root:
  ~/projects/bear_controller/venv/bin/python3 useful_tools/test_comms_soak_offline.py
"""
import sys
sys.dont_write_bytecode = True

import builtins
import contextlib
import csv
import io
import math
import os
import random
import shutil
import struct
import tempfile
import time
from unittest import mock

from pybear import Packet
from pybear.CONTROL_TABLE import REGISTER, INSTRUCTION

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import comms_soak_test as cs                                           # noqa: E402

POLL_S = 25e-6          # one in_waiting poll (PyBEAR busy-waits on it)
TX_S = 15e-6            # one write call
REPLY_S = 0.00049       # request -> reply (the measured ~0.49 ms at a 0.5 ms gap)
LATE_S = {'late': 0.003, 'late8': 0.008}
TE_LAG_S = 0.001
REBOOT_SILENT_S = 0.3
STALL_S = 0.015
PORT_SCAN_S = 0.030     # one /proc scan for other port holders (real time on the Pi)
T0 = 1000.0
TOP_RAD = math.radians(116.3)

CFG = {n: a for n, a in REGISTER['ADDRESS'].items() if REGISTER['REG_TYPE'][n] == 'cfg'}
STAT = {n: a for n, a in REGISTER['ADDRESS'].items() if REGISTER['REG_TYPE'][n] == 'stat'}
CFG_BY_ADDR = {a: n for n, a in CFG.items()}
STAT_BY_ADDR = {a: n for n, a in STAT.items()}
FLASH = {'id': 1, 'mode': 0, 'baudrate': 8, 'homing_offset': -20.3786, 'p_gain_iq': 0.02,
         'i_gain_iq': 0.02, 'limit_i_max': 5.5, 'limit_velocity_max': 1.0, 'limit_acc_max': 5.0,
         'limit_position_min': -0.0873, 'limit_position_max': 2.1817, 'min_voltage': 6.0,
         'max_voltage': 60.0, 'watchdog_timeout': 0, 'temp_limit_low': 80.0, 'temp_limit_high': 100.0}


def csum(p):
    return 255 - sum(p[2:-1]) % 256


def enc(name, v):
    if REGISTER['DATA_TYPE'][name] == 'f32':
        return list(struct.pack('<f', float(v)))
    return list(struct.pack('<i', int(v)))


def dec(name, b):
    if REGISTER['DATA_TYPE'][name] == 'f32':
        return struct.unpack('<f', bytes(b))[0]
    return struct.unpack('<i', bytes(b))[0]


class Clock:
    def __init__(self):
        self.t = T0
        self.act = None
        self.stall_at = []              # simulated times (s after T0) for a host stall
        self.wall_steps = []            # (s after T0, jump s): time.time() only

    def time(self):
        return self.t + sum(j for at, j in self.wall_steps if self.t - T0 >= at)

    def monotonic(self):
        return self.t

    def advance(self, d):
        self.t += d
        if self.act:
            self.act.step()

    def sleep(self, d):
        if self.stall_at and self.t - T0 >= self.stall_at[0]:
            self.stall_at.pop(0)
            d += STALL_S
        self.advance(max(0.0, d))


class FakeActuator:
    def __init__(self, clock, start_rad=TOP_RAD):
        self.clock = clock
        clock.act = self
        self.cfg = dict(FLASH)
        self.stat = {n: 0.0 for n in STAT}
        self.stat['torque_enable'] = 0
        self.te = 0
        self.te_visible = 0
        self.te_change_at = None
        self.err = 0x80
        self.comm_warn = False
        self.pos = start_rad
        self.temp_w, self.temp_p = 26.0, 28.0
        self.last_rx = clock.t
        self.silent_until = -1.0
        self.faults = []                # (t0, t1, kind), seconds after T0
        self.once = []                  # [t0, kind, n]: the next n packets after t0
        self.rebooted = set()
        self.writes = []                # (t, name, value, via)
        self.rx = []                    # (t, instr, bulk_has_write) of every packet that ARRIVED intact
        self.opens = 0
        self.closes = 0
        self.saves = 0
        self.drop_write = None          # fn(name, value) -> True: lose the write
        self.iq_override = None         # fn(act) -> A or None
        self.err_override = None
        self.rng = random.Random(5)

    # -- time ------------------------------------------------------------------------
    def fault(self, kind):
        s = self.clock.t - T0
        return any(t0 <= s < t1 for t0, t1, k in self.faults if k == kind)

    def take_once(self, instr):
        """The one-shot fault (if any) for the packet arriving now. 'crossed_long'
        waits for a single READ_STAT (e.g. the torque_enable check)."""
        s = self.clock.t - T0
        for e in self.once:
            if s >= e[0] and e[2] > 0 and (e[1] != 'crossed_long' or instr == INSTRUCTION['READ_STAT']):
                e[2] -= 1
                return e[1]
        return None

    def step(self):
        t = self.clock.t
        if self.te_change_at is not None and t >= self.te_change_at:
            self.te_visible, self.te_change_at = self.te, None
        for t0, t1, k in self.faults:
            if k == 'reboot' and t - T0 >= t0 and (t0, k) not in self.rebooted:
                self.rebooted.add((t0, k))
                self.cfg = dict(FLASH)
                self.stat = {n: 0.0 for n in STAT}
                self.te = self.te_visible = 0
                self.te_change_at = None
                self.err = 0x80
                self.silent_until = t + REBOOT_SILENT_S
        wd = self.cfg['watchdog_timeout']
        if self.te == 1 and self.cfg['mode'] == 0 and wd > 0 and t - self.last_rx > wd * 1e-6:
            self.err |= 0x08
            self.te = self.te_visible = 3
            self.te_change_at = None

    # -- registers ----------------------------------------------------------------------
    def read_stat(self, name):
        if name == 'torque_enable':
            return self.te_visible
        if name == 'present_position':
            return self.pos + self.rng.gauss(0.0, 1e-4)
        if name == 'present_velocity':
            return self.rng.gauss(0.0, 0.02)
        if name == 'present_iq':
            o = self.iq_override(self) if self.iq_override else None
            if o is not None:
                return o
            return self.stat['goal_iq'] if (self.te == 1 and self.cfg['mode'] == 0) else 0.0
        if name == 'winding_temperature':
            return self.temp_w
        if name == 'powerstage_temperature':
            return self.temp_p
        if name == 'input_voltage':
            return 19.5
        return self.stat.get(name, 0.0)

    def write(self, name, value, via):
        self.writes.append((self.clock.t, name, value, via))
        if self.drop_write and self.drop_write(name, value):
            return
        if name == 'torque_enable':
            if value == 0:
                self.err = 0x80
            self.te = value
            self.te_change_at = self.clock.t + TE_LAG_S
        elif name == 'goal_iq':
            if self.cfg['mode'] not in (0, 3) or abs(value) > self.cfg['limit_i_max'] + 1e-6:
                return
            self.stat['goal_iq'] = value
        elif name in CFG:
            self.cfg[name] = value
        else:
            self.stat[name] = value

    def err_byte(self):
        o = self.err_override(self) if self.err_override else None
        e = self.err if o is None else o
        if self.comm_warn:
            e |= 0x01
            self.comm_warn = False
        return e

    # -- the wire ------------------------------------------------------------------------------
    def receive(self, pkt):
        """One instruction packet from the Pi. Returns (reply bytes or None, extra delay)."""
        t = self.clock.t
        once = self.take_once(pkt[4] if len(pkt) > 4 else None)
        if t < self.silent_until or self.fault('cmd_loss') or once == 'cmd_loss':
            return None, 0.0
        if self.fault('garble_cmd') or once == 'garble_cmd':
            self.comm_warn = True
            return None, 0.0
        if len(pkt) < 6 or pkt[0] != 0xFF or pkt[1] != 0xFF or csum(pkt) != pkt[-1]:
            self.comm_warn = True
            return None, 0.0
        self.last_rx = t
        instr = pkt[4]
        bulk_w = instr == INSTRUCTION['BULK_STAT'] and (pkt[6] & 0x0F) > 0
        self.rx.append((t, instr, bulk_w))
        reply = self.handle(pkt)
        if reply is None or self.fault('reply_loss') or once == 'reply_loss':
            return None, 0.0
        if self.fault('garble') or once == 'garble':
            reply = list(reply)
            reply[-1] = (reply[-1] + 1) % 256
        if once == 'crossed_short':
            # a stale 1-register reply (valid checksum) lands just before the real one
            reply = self.status(pkt[2] if pkt[2] != 0xFE else self.cfg['id'], enc('torque_enable', 0)) \
                + list(reply)
        if once == 'crossed_long':
            # a stale 5-register bulk reply (valid checksum) lands before a single read's reply
            stale = []
            for name in ('present_position', 'present_velocity', 'present_iq',
                         'winding_temperature', 'powerstage_temperature'):
                stale += enc(name, self.read_stat(name))
            reply = self.status(self.cfg['id'], stale) + list(reply)
        late = next((LATE_S[k] for k in LATE_S if self.fault(k) or once == k), 0.0)
        return reply, late

    def status(self, mid, data):
        p = [0xFF, 0xFF, mid, len(data) + 2, self.err_byte()] + data + [0]
        p[-1] = csum(p)
        return p

    def handle(self, pkt):
        mid, instr = pkt[2], pkt[4]
        if instr == INSTRUCTION['PING']:
            return self.status(mid, [1, 0, 0, 0])
        if instr in (INSTRUCTION['READ_STAT'], INSTRUCTION['READ_CFG']):
            stat = instr == INSTRUCTION['READ_STAT']
            data = []
            for a in pkt[5:-1]:
                name = (STAT_BY_ADDR if stat else CFG_BY_ADDR)[a]
                data += enc(name, self.read_stat(name) if stat else self.cfg[name])
            return self.status(mid, data)
        if instr in (INSTRUCTION['WRITE_STAT'], INSTRUCTION['WRITE_CFG']):
            stat = instr == INSTRUCTION['WRITE_STAT']
            body = pkt[5:-1]
            for k in range(0, len(body), 5):
                name = (STAT_BY_ADDR if stat else CFG_BY_ADDR)[body[k]]
                self.write(name, dec(name, body[k + 1:k + 5]), 'single')
            return None
        if instr == INSTRUCTION['SAVE_CFG']:
            self.saves += 1
            return None
        if instr == INSTRUCTION['BULK_STAT']:
            m, rw = pkt[5], pkt[6]
            r, w = rw >> 4, rw & 0x0F
            raddr = pkt[7:7 + r]
            waddr = pkt[7 + r:7 + r + w]
            p = 7 + r + w
            out = []
            for _ in range(m):
                bid = pkt[p]
                p += 1
                for a in waddr:
                    name = STAT_BY_ADDR[a]
                    self.write(name, dec(name, pkt[p:p + 4]), 'bulk')
                    p += 4
                if r and bid == self.cfg['id']:
                    data = []
                    for a in raddr:
                        name = STAT_BY_ADDR[a]
                        data += enc(name, self.read_stat(name))
                    out += self.status(bid, data)
            return out or None
        return None


class FakeSerial:
    """Stands in for serial.Serial inside pybear.Packet."""
    ACT = None

    def __init__(self, port, baudrate, **kw):
        self.act = FakeSerial.ACT
        self.clock = self.act.clock
        self.act.opens += 1
        self.buf = bytearray()
        self.pending = []               # (ready time, bytes)
        self.is_open = True

    def _deliver(self):
        now = self.clock.t
        keep = []
        for ready, b in self.pending:
            if ready <= now:
                self.buf.extend(b)
            else:
                keep.append((ready, b))
        self.pending = keep

    def reset_input_buffer(self):
        self._deliver()
        self.buf.clear()                # bytes still on the wire stay pending (late replies)

    def write(self, data):
        if not self.is_open:
            raise RuntimeError("write on a closed port")
        self.clock.advance(TX_S)
        reply, extra = self.act.receive(list(data))
        if reply:
            self.pending.append((self.clock.t + REPLY_S + extra, bytes(reply)))
        return len(data)

    @property
    def in_waiting(self):
        self.clock.advance(POLL_S)
        self._deliver()
        return len(self.buf)

    def read(self, n):
        self._deliver()
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def close(self):
        self.is_open = False
        self.act.closes += 1


# ---------------------------------------------------------------------------------------------
RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(f"    [{'PASS' if cond else 'FAIL'}] {name}{(' -- ' + detail) if detail else ''}")


def invoke(argv, setup=None, typed='GO', rt=True, start_rad=TOP_RAD, interrupt_at=None):
    clock = Clock()
    act = FakeActuator(clock, start_rad=start_rad)
    if setup:
        setup(act, clock)
    FakeSerial.ACT = act
    logdir = tempfile.mkdtemp(prefix='comms_soak_test_')
    out = io.StringIO()
    sched = os.SCHED_FIFO if rt else os.SCHED_OTHER
    real_sleep = clock.sleep

    def sleep(d):
        if interrupt_at is not None and clock.t - T0 >= interrupt_at:
            raise KeyboardInterrupt
        real_sleep(d)
    # _git is stubbed: subprocess waits with time.sleep (patched = simulated) while
    # timing out on the REAL clock, so a slow git under CPU load would jump the
    # simulated clock by seconds (seen: 235 s) and skip past injected faults.
    # port_holders() really scans /proc; on the Pi that takes real time, so here it
    # costs PORT_SCAN_S of simulated time (the loop must not count it as a host gap).
    real_holders = cs.port_holders

    def holders():
        clock.advance(PORT_SCAN_S)
        return real_holders()
    with mock.patch.object(Packet.serial, 'Serial', FakeSerial), \
            mock.patch.object(cs, '_git', lambda *argv: 'test'), \
            mock.patch.object(cs, 'port_holders', holders), \
            mock.patch.object(cs, 'LOG_SUBDIR', logdir), \
            mock.patch.object(os, 'sched_getscheduler', return_value=sched), \
            mock.patch.object(time, 'time', clock.time), \
            mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', sleep), \
            mock.patch.object(builtins, 'input', lambda prompt='': typed), \
            contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        try:
            rc = cs.main(argv)
        except SystemExit as e:
            rc = f"SystemExit({e.code!s:.300})"
        except KeyboardInterrupt:
            rc = 'KeyboardInterrupt (escaped main)'
    files = sorted(os.listdir(logdir))
    res = dict(rc=rc, out=out.getvalue(), act=act, clock=clock, files=files, main=None, detail=None)
    for fn in files:
        with open(os.path.join(logdir, fn)) as f:
            lines = f.read().splitlines()
        d = dict(comments=[l for l in lines if l.startswith('#')],
                 rows=list(csv.DictReader([l for l in lines if not l.startswith('#')])), name=fn)
        res['detail' if fn.endswith('_detail.csv') else 'main'] = d
    shutil.rmtree(logdir)
    return res


def end_line(res):
    if not res['main']:
        return ''
    return next((l for l in res['main']['comments'] if l.startswith('# END')), '')


def events(res, word):
    return [l for l in (res['main']['comments'] if res['main'] else []) if word in l]


def wire(ev):
    """Parse 'wire: N packets sent, M bytes read back in K reads, S late bytes ...'."""
    try:
        w = ev.split('wire: ')[1].split(';')[0]
        return dict(tx=int(w.split(' packets')[0]), rx=int(w.split('sent, ')[1].split(' bytes')[0]),
                    stale=int(w.split('reads, ')[1].split(' late')[0]))
    except (IndexError, ValueError):
        return None


def tap_dump(res, k):
    """(lines before the first failure, lines from it on) of outage #k's raw-byte dump."""
    lines = res['detail']['comments'] if res['detail'] else []
    start = next((j for j, l in enumerate(lines) if f'raw bytes, outage #{k} ' in l), None)
    if start is None:
        return [], []
    block = []
    for l in lines[start + 1:]:
        if l.startswith('# --- raw bytes'):
            break
        block.append(l)
    cut = next((j for j, l in enumerate(block) if 'from the first failure' in l), len(block))
    return block[:cut], block[cut:]


def write_seq(res):
    return [(n, round(v, 6) if isinstance(v, float) else v, via) for _, n, v, via in res['act'].writes]


def summarize_writes(res):
    out = []
    for n, v, via in write_seq(res):
        key = (n, v, via)
        if out and out[-1][0] == key:
            out[-1][1] += 1
        else:
            out.append([key, 1])
    return [f"{k[0]} = {k[1]} [{k[2]}]" + (f" x{c}" if c > 1 else '') for k, c in out]


def enabled_writes_ok(res, label):
    """enabled0: only the declared registers; goal_iq always exactly 0; no save_config."""
    names = {n for n, v, via in write_seq(res)}
    check(f"{label}: only watchdog_timeout / mode / goal_iq / torque_enable written",
          names <= {'watchdog_timeout', 'mode', 'goal_iq', 'torque_enable'}, str(sorted(names)))
    nz = [v for n, v, via in write_seq(res) if n == 'goal_iq' and v != 0.0]
    check(f"{label}: every goal_iq written is exactly 0.0", not nz, f"{len(nz)} nonzero")
    check(f"{label}: no save_config", res['act'].saves == 0)


PB = "[PyBEAR | WARNING]"


def main():
    R, E = ['--mode', 'read', '--expect', 'rt'], ['--mode', 'enabled0', '--expect', 'rt']

    print("1. Refusals before the port is opened")
    for argv, why in [
            (R + ['--minutes', '0.1'], 'minutes below 0.5'),
            (R + ['--minutes', '90'], 'minutes above 60'),
            (R + ['--minutes', 'nan'], 'minutes NaN'),
            (R + ['--wait-s', '-1'], 'negative wait'),
            (R + ['--reopen-every', '5'], 'reopen every 5 s'),
            (R + ['--pause-every', '60'], 'pause without --pause-s'),
            (R + ['--pause-every', '60', '--pause-s', '60'], 'pause as long as its period'),
            (R + ['--reopen-every', '60', '--pause-every', '60', '--pause-s', '5'], 'both discriminators'),
            (E + ['--reopen-every', '60'], 'reopen with the motor enabled'),
            (E + ['--pause-every', '60', '--pause-s', '5'], 'pause with the motor enabled'),
            (['--mode', 'read', '--expect', 'rt'] + ['--mode', 'spin'], 'unknown mode'),
            (['--mode', 'read'], 'no --expect')]:
        res = invoke(argv)
        check(f"refused: {why}", str(res['rc']).startswith('SystemExit') and res['act'].opens == 0,
              str(res['rc'])[:80])
    res = invoke(R, rt=False)
    check("refused: --expect rt but not at RT (no port opened)",
          str(res['rc']).startswith('SystemExit') and 'real-time' in res['out']
          and res['act'].opens == 0, str(res['rc']))
    res = invoke(['--mode', 'read', '--expect', 'normal'], rt=True)
    check("refused: --expect normal but at SCHED_FIFO (no port opened)",
          str(res['rc']).startswith('SystemExit') and 'SCHED_FIFO' in res['out']
          and res['act'].opens == 0, str(res['rc']))

    print("\n2. Refusals after opening the port: reads only, ZERO writes")
    for argv, setup, why in [
            (R, lambda a, c: setattr(a, 'te_visible', 1), 'read: actuator enabled'),
            (R, lambda a, c: setattr(a, 'err', 0x90), 'read: fault latched (never cleared by this script)'),
            (R, lambda a, c: setattr(a, 'iq_override', lambda x: 0.4), 'read: current with motor off'),
            (E, lambda a, c: setattr(a, 'te_visible', 1), 'enabled0: actuator enabled'),
            (E, lambda a, c: a.cfg.__setitem__('limit_i_max', 6.0), 'enabled0: limit_i_max 6 A'),
            (E, lambda a, c: setattr(a, 'temp_w', 66.0), 'enabled0: too hot')]:
        res = invoke(argv + ['--minutes', '0.5', '--wait-s', '0'], setup=setup)
        check(f"refused: {why}", 'REFUSED' in str(res['rc']) and not res['act'].writes,
              f"{str(res['rc'])[:70]}; writes {len(res['act'].writes)}")
    res = invoke(E + ['--minutes', '0.5', '--wait-s', '0'], start_rad=math.radians(90))
    check("refused: enabled0 with the arm off the top stop", 'REFUSED' in str(res['rc'])
          and not res['act'].writes)
    res = invoke(E + ['--minutes', '0.5', '--wait-s', '0'], typed='go')
    check("enabled0 not started without GO: ZERO writes, no files",
          res['rc'] == 1 and not res['act'].writes and not res['files'])

    print("\n3. Read mode, clean, 1 min after a 20 s quiet wait")
    res = invoke(R + ['--minutes', '1', '--wait-s', '20', '--label', 'clean'])
    check("completed", res['rc'] == 0 and 'END completed' in end_line(res), end_line(res))
    check("ZERO writes to the actuator (no single writes, no bulk write fields)",
          not res['act'].writes and res['act'].saves == 0, str(res['act'].writes[:3]))
    rx = [t - T0 for t, i, *_ in res['act'].rx]
    quiet = [t for t in rx if 0.1 < t < 20.0]
    check("no traffic during the quiet wait (only the preflight reads at the open, then >= 20 s)",
          not quiet and rx and rx[0] < 0.1 and any(t >= 20.0 for t in rx),
          f"{len(quiet)} packets in 0.1-20 s; preflight {sum(t < 0.1 for t in rx)} packets")
    rows = res['main']['rows']
    nt = sum(int(r['n_tx']) for r in rows)
    check("~500 exchanges/s for 60 s", 29000 <= nt <= 30200, f"{nt} exchanges")
    check("one row per second (60-61)", 60 <= len(rows) <= 61, str(len(rows)))
    check("no outage, no warnings", all(int(r['n_fail']) == 0 and int(r['n_warn']) == 0 for r in rows))
    check("header records mode, wait, COMM_TRY_NUM, scheduler, port holders",
          any('mode=read' in l and 'wait_s=20' in l for l in res['main']['comments'])
          and any('COMM_TRY_NUM=' in l for l in res['main']['comments'])
          and any('port holders at start' in l for l in res['main']['comments']))
    check("detail file exists and is small (nothing unusual: event rows only)",
          res['detail'] is not None and len(res['detail']['rows']) < 3000,
          f"{len(res['detail']['rows']) if res['detail'] else 'missing'} rows")
    check("PyBEAR's print_warning restored after the run",
          Packet.PKT.__dict__['print_warning'].__func__.__name__ == 'print_warning')
    cols = list(rows[0].keys()) if rows else []
    check("per-second table has the clock and host columns",
          {'wall_minus_mono_ms', 'cpu_temp_C', 'cpu3_MHz'} <= set(cols), str(cols[-4:]))
    summ = '\n'.join(res['main']['comments'])
    check("tap on: header says so; summary counts >= 1 packet sent per exchange",
          'tap=on' in summ and 'wire (tap):' in summ
          and int(summ.split('wire (tap): ')[1].split(' packets')[0]) >= nt,
          summ.split('wire (tap): ')[1][:80] if 'wire (tap): ' in summ else 'no tap line')
    check("clean run: no clock steps, no host gaps", 'steps > 1 ms: 0' in summ
          and not events(res, 'HOST GAP'))

    print("\n4. Read mode: faults are timed and labelled, and the run rides through them")
    # every fault window starts 40 s after the open = 20 s after the traffic starts
    W = 40.0
    # garbled BULK replies: PyBEAR (e23014a) checks the checksum, returns None at
    # once, prints NO warning and does not retry -- so 'none' is the right count,
    # and only the wire account shows bytes came back.
    for kind, dur, expect_warn, wire_ok, label in [
            ('reply_loss', 0.030, 'max_try_num', lambda w: w['rx'] == 0 and w['stale'] == 0,
             'replies lost for 30 ms'),
            ('cmd_loss', 0.030, 'max_try_num', lambda w: w['rx'] == 0 and w['stale'] == 0,
             'our packets lost for 30 ms'),
            ('garble', 0.010, 'PyBEAR: none', lambda w: w['rx'] > 0, 'garbled replies for 10 ms')]:
        res = invoke(R + ['--minutes', '1', '--wait-s', '20'],
                     setup=lambda a, c, k=kind, d=dur: a.faults.append((W, W + d, k)))
        out_ev = events(res, 'OUTAGE #1')
        check(f"{label}: completed (rode through)", res['rc'] == 0, end_line(res))
        check(f"{label}: exactly one outage logged", len(events(res, 'OUTAGE #')) == 1
              and len(events(res, 'OUTAGE #2')) == 0, out_ev[0][:150] if out_ev else 'none')
        if out_ev:
            ev = out_ev[0]
            down = float(ev.split('nothing back for ')[1].split(' ms')[0])
            tr = float(ev.split('#')[1].split('s since traffic start')[0].strip())
            check(f"{label}: outage length close to the fault ({dur * 1e3:.0f} ms; resolution ~8 ms)",
                  dur * 1e3 - 2 <= down <= dur * 1e3 + 10, f"{down:.1f} ms")
            check(f"{label}: timed ~20 s since traffic start / ~40 s since port open",
                  19.9 <= tr <= 20.1 and ' 40.0' in ev.split('since traffic start,')[1][:12],
                  ev.split('|')[0].strip()[:90])
            check(f"{label}: PyBEAR's warnings as PyBEAR reports them ({expect_warn})",
                  expect_warn in ev, ev.split('PyBEAR: ')[1].split(';')[0])
            w = wire(ev)
            check(f"{label}: wire account right ({'silence' if kind != 'garble' else 'bytes came back'})",
                  w is not None and wire_ok(w), ev.split('wire: ')[1].split(';')[0])
        pre, during = tap_dump(res, 1)
        n_rx = sum(' RX ' in l for l in during)
        # silence: the only RX in 'during' are the 2 reads (4 B + rest) of the reply that ended it
        check(f"{label}: raw bytes dumped: TX before and during; RX during = "
              f"{'more than the closing reply' if kind == 'garble' else 'only the closing reply (2 reads)'}",
              any(' TX ' in l for l in pre) and any(' TX ' in l for l in during)
              and (n_rx > 2 if kind == 'garble' else n_rx == 2), f"RX lines during: {n_rx}")
        check(f"{label}: no HOST GAP from the outage bookkeeping", not events(res, 'HOST GAP'))
        check(f"{label}: PyBEAR printed nothing (captured)", PB not in res['out'])
        if res['detail']:
            ts = [float(r['t_traffic_s']) for r in res['detail']['rows']]
            check(f"{label}: detail rows cover +-2 s around it, not the whole run",
                  ts and min(ts) >= 17.9 and max(ts) <= 22.2 and len(ts) < 3000,
                  f"{len(ts)} rows, {min(ts):.2f}-{max(ts):.2f} s")
        check(f"{label}: still zero writes", not res['act'].writes)

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.once.append([30.0, 'reply_loss', 1]))
    rows = res['main']['rows']
    nw = sum(int(r['n_warn']) for r in rows)
    nf = sum(int(r['n_fail']) for r in rows)
    check("one lost reply: PyBEAR retry recovers it -> no outage, but the retry IS logged",
          nf == 0 and nw == 1 and not events(res, 'OUTAGE'), f"fails {nf}, warnings {nw}")
    check("...and the summary names it", any('bulk_timeout x1' in l for l in res['main']['comments']))
    check("...and the detail file has the +-2 s around it",
          res['detail'] and 500 < len(res['detail']['rows']) < 3000,
          f"{len(res['detail']['rows']) if res['detail'] else 0} rows")

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.once.append([30.0, 'garble_cmd', 1]))
    check("garbled command: next reply's error bit 0 logged as a comm-warning event",
          any('0x81' in l and 'comm-warning' in l for l in events(res, 'error byte')),
          str(events(res, 'error byte')[:1])[:120])
    rows = res['main']['rows']
    check("...and counted in the per-second table", sum(int(r['n_comm_warn']) for r in rows) >= 1)

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.faults.append((30.0, 30.02, 'late')))
    summ = '\n'.join(res['main']['comments'])
    check("late replies (3 ms): run completes, retries logged", res['rc'] == 0
          and sum(int(r['n_warn']) for r in res['main']['rows']) >= 1, end_line(res))
    check("...late bytes thrown away unread are counted",
          'late bytes thrown away' in summ and ' 0 late bytes' not in summ.split('wire (tap):')[1][:120],
          summ.split('wire (tap):')[1][:110] if 'wire (tap):' in summ else '')

    for mode_argv, label in ((R, 'read'), (E, 'enabled0')):
        res = invoke(mode_argv + ['--minutes', '1', '--wait-s', '0'],
                     setup=lambda a, c: a.once.append([30.0, 'crossed_short', 1]))
        ev = events(res, 'RAISED')
        summ = '\n'.join(res['main']['comments'])
        check(f"{label}: a crossed short reply makes PyBEAR raise struct.error -> logged, counted "
              f"as a failed call, run completes", res['rc'] == 0 and ev and 'struct.error' in ev[0]
              and 'PyBEAR raised (malformed / crossed replies): struct.error x1' in summ
              and events(res, 'OUTAGE #1'), (ev or [end_line(res)])[0][-110:])
        if label == 'enabled0':
            enabled_writes_ok(res, "  [crossed reply]")

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.faults.append((30.0, 30.03, 'late8')))
    ev = events(res, 'OUTAGE #1')
    w = wire(ev[0]) if ev else None
    summ = '\n'.join(res['main']['comments'])
    stale = summ.split('wire (tap): ')[1].split(' late bytes')[0].split()[-1] if 'wire (tap): ' in summ else '0'
    check("replies 8 ms late: the outage itself is silence on the wire (they come after PyBEAR "
          "gave up)", res['rc'] == 0 and w is not None and w['rx'] == 0,
          ev[0].split('wire: ')[1].split(';')[0] if ev else 'no outage')
    check("...and arrive afterwards: late bytes thrown away unread are counted, the run rides through",
          int(stale) > 0 and res['rc'] == 0, f"late bytes {stale}; {end_line(res)}")

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.once.append([25.0, 'crossed_long', 1]))
    summ = '\n'.join(res['main']['comments'])
    imp = events(res, 'implausible value')
    check("a stale bulk reply crossing into the torque_enable check is NOT a stop: logged as "
          "implausible, re-read 0.5 s later, run completes",
          res['rc'] == 0 and len(imp) == 1 and 'crossed reply): 1' in summ,
          (imp or [end_line(res)])[0][-110:])
    rx_te = [t - T0 for t, i, bw in res['act'].rx if i == INSTRUCTION['READ_STAT']]
    after = [t for t in rx_te if t > 25.0]
    check("...the re-read comes ~0.5 s after the bad one (not 10 s)",
          len(after) >= 2 and 0.4 < after[1] - after[0] < 0.7,
          f"te reads after 25 s at {[round(t, 2) for t in after[:3]]}")

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: c.wall_steps.append((30.0, 0.005)))
    st = events(res, 'WALL CLOCK STEP')
    check("a +5 ms wall-clock step is caught (PyBEAR times out on the wall clock)",
          len(st) == 1 and '+5.000 ms' in st[0] and res['rc'] == 0, (st or ['none'])[0][-100:])
    check("...and counted in the summary",
          any('steps > 1 ms: 1' in l for l in res['main']['comments']))

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: c.stall_at.append(30.0))
    check("a 15 ms Pi stall with no failed exchange: logged as a HOST GAP, not an outage",
          len(events(res, 'HOST GAP')) == 1 and not events(res, 'OUTAGE'),
          str(events(res, 'HOST GAP'))[:120])

    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: a.faults.append((30.0, 37.0, 'reply_loss')))
    check("link down for 7 s: stops at 5 s, saved, zero writes",
          'link down' in end_line(res) and res['rc'] == 1 and not res['act'].writes, end_line(res))

    # Two outages close together (review Nov 4): outage #2's clock starts after outage
    # #1's checks (input_voltage read, /proc scan; they end ~32.063 s), not at the reply
    # that closed outage #1. (32 s, not 30: the 10 s torque_enable check falls at ~30.03.)
    def two_outages(end2=None):
        def setup(a, c):
            a.faults.append((32.000, 32.030, 'reply_loss'))
            if end2 is None:          # one bulk exchange's tries (PyBEAR gives up a bulk read
                # after single_try_num = COMM_TRY_NUM sends): exactly ONE failed exchange
                a.once.append([32.050, 'reply_loss', cs.COMM_TRY_NUM])
            else:
                a.faults.append((32.050, end2, 'reply_loss'))
        return invoke(R + ['--minutes', '1', '--wait-s', '0'], setup=setup)

    def down_ms(ev):
        return float(ev.split('nothing back for ')[1].split(' ms')[0])
    res = two_outages()
    ev = events(res, 'OUTAGE #2')
    check("two outages close together: #2 (one failed exchange, sent right after #1's checks) "
          "reads 'nothing back' < 10 ms -- #1's checks not counted as silence",
          res['rc'] == 0 and ev and ' 1 exchange(s) failed' in ev[0] and down_ms(ev[0]) < 10.0,
          ev[0].split('| ')[1][:70] if ev else f"no outage #2; {end_line(res)}")
    # silence from the first exchange after #1's checks (~32.065) to 37.050: ~4.985 s
    res = two_outages(37.050)
    ev = events(res, 'OUTAGE #2')
    check("...and the 5 s link-down clock starts after #1's checks: ~4.985 s of silence from "
          "then rides through (the old clock, started at #1's closing reply, stopped the run)",
          res['rc'] == 0 and ev and 4970.0 < down_ms(ev[0]) < 5000.0,
          ev[0].split('| ')[1][:70] if ev else end_line(res))

    def enable_mid(a, c):
        a.faults.append((25.0, 25.0, 'none'))
        orig = a.read_stat
        a.read_stat = lambda n: 1 if (n == 'torque_enable' and c.t - T0 > 25.0) else orig(n)
    res = invoke(R + ['--minutes', '1', '--wait-s', '0'], setup=enable_mid)
    check("read: something enables the motor -> stopped at the next 10 s check",
          'torque_enable reads 1' in end_line(res), end_line(res))
    res = invoke(R + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: setattr(a, 'iq_override', lambda x: 0.3 if x.clock.t - T0 > 25 else None))
    check("read: current appears -> stopped", 'something is driving it' in end_line(res), end_line(res))
    res = invoke(R + ['--minutes', '1', '--wait-s', '0'], interrupt_at=30.0)
    check("read: Ctrl-C -> interrupted, both files saved, zero writes",
          'interrupted' in end_line(res) and res['detail'] is not None and not res['act'].writes,
          end_line(res))

    print("\n5. Read mode discriminators")
    res = invoke(R + ['--minutes', '1', '--wait-s', '0', '--reopen-every', '20'])
    check("--reopen-every 20 over 60 s: 2 reopens (3 opens, 3 closes incl. the end)",
          res['act'].opens == 3 and res['act'].closes == 3, f"opens {res['act'].opens}, closes "
          f"{res['act'].closes}")
    check("...events logged, run completes, zero writes",
          len(events(res, 'REOPEN: port open again')) == 2 and res['rc'] == 0 and not res['act'].writes)
    summ = '\n'.join(res['main']['comments'])
    nt = sum(int(r['n_tx']) for r in res['main']['rows'])
    check("...the tap is re-installed on each new port (packets counted >= exchanges)",
          'wire (tap): ' in summ and int(summ.split('wire (tap): ')[1].split(' packets')[0]) >= nt,
          f"exchanges {nt}; {summ.split('wire (tap): ')[1][:40] if 'wire (tap): ' in summ else ''}")
    res = invoke(R + ['--minutes', '1', '--wait-s', '0', '--pause-every', '20', '--pause-s', '5'])
    rx = [t - T0 for t, i, *_ in res['act'].rx if i == INSTRUCTION['BULK_STAT']]
    gaps = [b - a for a, b in zip(rx, rx[1:]) if b - a > 1.0]
    check("--pause-every 20 --pause-s 5: two 5 s silences on the wire, nothing failed",
          len(gaps) == 2 and all(4.99 <= g <= 5.05 for g in gaps) and not events(res, 'OUTAGE'),
          f"gaps {[round(g, 3) for g in gaps]}")
    check("...not counted as host gaps", not events(res, 'HOST GAP'))

    print("\n6. Enabled at 0 A: writes, and the watchdog as witness")
    res = invoke(E + ['--minutes', '1', '--wait-s', '10'])
    check("completed", res['rc'] == 0 and 'END completed' in end_line(res), end_line(res))
    print("    writes, in order:")
    for line in summarize_writes(res):
        print(f"      {line}")
    seq = [(n, v) for n, v, via in write_seq(res) if not (n == 'goal_iq' and via == 'bulk')]
    check("write order: watchdog 10000, mode 0, goal_iq 0, torque_enable 1, <bulk goal_iq 0 ...>, "
          "goal_iq 0, torque_enable 0, watchdog 0",
          seq == [('watchdog_timeout', 10000), ('mode', 0), ('goal_iq', 0.0), ('torque_enable', 1),
                  ('goal_iq', 0.0), ('torque_enable', 0), ('watchdog_timeout', 0)], str(seq))
    enabled_writes_ok(res, 'clean enabled0')
    first_w = min(t for t, *_ in res['act'].writes) - T0
    check("nothing written during the 10 s quiet wait", first_w >= 10.0, f"first write {first_w:.3f} s")
    loop = [t for t, i, bw in res['act'].rx if i == INSTRUCTION['BULK_STAT'] and bw]
    other = [i for t, i, bw in res['act'].rx if loop[0] < t < loop[-1] and not bw]
    check("enabled loop is bulk_read_write only (one packet per cycle, nothing between)",
          not other, str(sorted(set(other))))
    gaps = [b - a for a, b in zip(loop, loop[1:])]
    check("...one every ~2 ms (max gap < 3 ms)", gaps and max(gaps) < 0.003, f"max {max(gaps) * 1e3:.2f} ms")
    check("the actuator ended disabled with the watchdog off",
          res['act'].te == 0 and res['act'].cfg['watchdog_timeout'] == 0)

    for kind, dur, expect, recov, label in [
            ('reply_loss', 0.030, 'only replies were lost', False, 'replies lost 30 ms'),
            ('cmd_loss', 0.030, 'WATCHDOG TRIPPED', True, 'our packets lost 30 ms'),
            ('reboot', 0.0, 'ACTUATOR RESET', True, 'actuator reboots')]:
        res = invoke(E + ['--minutes', '1', '--wait-s', '0'],
                     setup=lambda a, c, k=kind, d=dur: a.faults.append((30.0, 30.0 + d, k)))
        ev = events(res, 'OUTAGE #1')
        check(f"{label}: completed", res['rc'] == 0, end_line(res))
        check(f"{label}: labelled '{expect}'", bool(ev) and expect in ev[0], ev[0][-160:] if ev else 'none')
        rec = events(res, 'RECOVERY')
        check(f"{label}: {'one recovery (disable, re-arm, re-enable)' if recov else 'no recovery'}",
              len(rec) == (1 if recov else 0) and (not recov or events(res, 're-armed and re-enabled')),
              f"{len(rec)} recoveries")
        enabled_writes_ok(res, f"  [{label}]")
        if recov:
            seq = [(n, v) for n, v, via in write_seq(res) if not (n == 'goal_iq' and via == 'bulk')]
            check(f"{label}: recovery writes = disable, then watchdog, mode, goal_iq, enable again",
                  seq[4:12] == [('goal_iq', 0.0), ('torque_enable', 0), ('watchdog_timeout', 10000),
                                ('mode', 0), ('goal_iq', 0.0), ('torque_enable', 1),
                                ('goal_iq', 0.0), ('torque_enable', 0)], str(seq[4:12]))
        check(f"{label}: ends disabled, watchdog off", res['act'].te == 0
              and res['act'].cfg['watchdog_timeout'] == 0)

    res = invoke(E + ['--minutes', '1', '--wait-s', '0'], setup=lambda a, c: c.stall_at.append(30.0))
    rec = events(res, 'RECOVERY')
    check("Pi stall 15 ms (no exchange failed): watchdog trips, recovery says 'after a host gap'",
          len(rec) == 1 and 'after a host gap' in rec[0] and res['rc'] == 0, rec[0][-90:] if rec else 'none')

    res = invoke(E + ['--minutes', '1', '--wait-s', '0'],
                 setup=lambda a, c: [a.faults.append((10.0 + 5 * k, 10.03 + 5 * k, 'cmd_loss'))
                                     for k in range(6)])
    check("6 trips: stops after 5 recoveries, disabled",
          '5 recoveries already' in end_line(res) and res['act'].te == 0, end_line(res))
    enabled_writes_ok(res, "  [6 trips]")

    def drop_te0(a, c):
        a.drop_write = lambda n, v: n == 'torque_enable' and v == 0 and c.t - T0 > 20.0
    res = invoke(E + ['--minutes', '0.5', '--wait-s', '0'], setup=drop_te0)
    wds = [v for n, v, via in write_seq(res) if n == 'watchdog_timeout']
    check("disable not confirmed at the end: watchdog NOT reset, says cut power",
          wds == [10000] and 'DISABLE NOT CONFIRMED' in res['out'], str(wds))

    for setup, word, label in [
            (lambda a, c: setattr(a, 'iq_override', lambda x: 0.5 if x.clock.t - T0 > 20 else None),
             'at a 0 A command', 'current at a 0 A command'),
            (lambda a, c: setattr(a, 'err_override', lambda x: 0x90 if x.clock.t - T0 > 20 else None),
             'error byte 0x90', 'error 0x90'),
            (lambda a, c: c.__setattr__('_drop', None) or a.faults.append((20.0, 20.0, 'none'))
             or setattr(a, 'pos', TOP_RAD) or setattr(a, 'read_stat', (lambda orig: (lambda n: math.radians(100)
                                                    if n == 'present_position' and c.t - T0 > 20
                                                    else orig(n)))(a.read_stat)),
             'left the top stop', 'arm leaves the stop')]:
        res = invoke(E + ['--minutes', '0.5', '--wait-s', '0'], setup=setup)
        check(f"enabled0 stop: {label}", word in end_line(res) and res['act'].te == 0
              and res['act'].cfg['watchdog_timeout'] == 0, end_line(res))

    def trip_then_moved(a, c):
        a.faults.append((20.0, 20.03, 'cmd_loss'))
        orig = a.read_stat
        a.read_stat = lambda n: (math.radians(100) if n == 'present_position' and c.t - T0 > 20.03
                                 else orig(n))
    res = invoke(E + ['--minutes', '0.5', '--wait-s', '0'], setup=trip_then_moved)
    te1 = [v for n, v, via in write_seq(res) if n == 'torque_enable' and v == 1]
    check("trip, then the arm is off the stop: no re-enable, stopped, disabled",
          'not on the top stop' in end_line(res) and len(te1) == 1 and res['act'].te == 0, end_line(res))

    res = invoke(E + ['--minutes', '1', '--wait-s', '0'], interrupt_at=30.0)
    check("enabled0 Ctrl-C: interrupted, disabled, watchdog off, saved",
          'interrupted' in end_line(res) and res['act'].te == 0
          and res['act'].cfg['watchdog_timeout'] == 0, end_line(res))
    enabled_writes_ok(res, "  [Ctrl-C]")

    print("\n7. --no-tap: same run, nothing recorded below PyBEAR")
    res = invoke(R + ['--minutes', '1', '--wait-s', '0', '--no-tap'],
                 setup=lambda a, c: a.faults.append((30.0, 30.03, 'reply_loss')))
    summ = '\n'.join(res['main']['comments'])
    ev = events(res, 'OUTAGE #1')
    check("--no-tap: completes, header tap=off, outage still logged with 'wire: tap off'",
          res['rc'] == 0 and 'tap=off' in summ and ev and 'wire: tap off' in ev[0], end_line(res))
    check("--no-tap: no raw-byte dump, detail says tap off",
          not tap_dump(res, 1)[1] and any('tap off' in l for l in res['detail']['comments']))
    check("--no-tap: PyBEAR's serial object is not wrapped",
          'wire (tap):' not in summ and res['rc'] == 0)

    n_ok = sum(ok for _, ok in RESULTS)
    print(f"\n{n_ok}/{len(RESULTS)} checks passed")
    if n_ok != len(RESULTS):
        print("FAILED:")
        for n, ok in RESULTS:
            if not ok:
                print(f"  {n}")
        sys.exit(1)


if __name__ == '__main__':
    main()