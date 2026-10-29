"""
Offline test for observer_excitation.py --mode sweep (the H0 bare sweep) and
BearInterface.enable_position_mode(). Replaces the coverage that went with
test_j_sine_offline.py / test_j_release_offline.py (deleted Oct 27).

NO SERIAL PORT: Manager.BEAR is replaced with a function that raises before
bear_interface is imported, and BearInterface is replaced by a factory around
PosFake. Time is simulated (time.monotonic / time.sleep patched; every bus
transaction costs BUS_S). The real script file is run in-process (runpy),
unmodified.

The fake actuator (Position Mode, mode 2):
  * goal_position is accepted only in modes 2/3 and only inside the WRITTEN
    limit_position_min/max (out-of-range goals are ignored, not clamped);
  * the setpoint follows the goal through a 2nd-order profile clamped to the
    written limit_acc_max / limit_velocity_max, and the goal_position
    READ-BACK is that setpoint (Oct 14 probe), even while disabled;
  * enabled: arm follows the setpoint with the measured ~1.4 deg lag; motor
    torque = J*acc - y(pos) + friction, where y (spring - rig gravity) and f
    (friction) are interpolated from a MEASURED sweep table (Oct 14 22:03,
    tonight's arguments; of the four Oct 14-15 runs it drew the most current),
    held flat outside it. Friction keeps its last direction while the arm is
    stopped (the real dwell draws ~4 A, not -y/KT ~2.8 A). Current noise
    0.19 A (measured). Section 4 checks the fake's down-leg and dwell current
    against that run, so the abort margins it reports are real ones;
  * disabled: spring-driven, friction-limited (bare arm rises to the top stop);
  * torque_enable read-back lags the write by 1 ms; a torque_enable write sent
    straight after another write (no read between) is LOST (Oct 13: the
    disable()'s first torque_enable = 0 was dropped after goal_iq; worst case
    here: always);
  * injectable faults: lost writes, current spikes, a jammed arm, error byte,
    temperature, Ctrl-C (one or two).
Every write and every non-get/set call (e.g. save_config) is recorded.

Checks:
  1. argument guards refuse before the port is opened;
  2. pre-enable refusals after the port opens: never write torque_enable 1;
  3. enable_position_mode(): write order mode -> goal -> enable, each confirmed;
     every refusal path leaves torque_enable 1 unwritten (except a lost enable);
  4. tonight's H0 command: completes, settings and limits, phases, park HIGH,
     disable last (confirmed on its first torque_enable = 0), fake current
     matches the measured run, CSV parses with sweep_tables and recovers the
     fake's y / f, t < t_pos < t_vel < t_iq on every row;
  5. runtime aborts (current, trip, jam, error byte, temperature, run cap,
     Ctrl-C) and every park release branch (park current, park stall, park
     deadline, Ctrl-C in the park): each ends with a confirmed disable and
     nothing written after it, and the CSV names the reason.
A section that crashes is reported as a failed check; the others still run.
Prints the write sequence of every path at the end.

Run from the repo root:
  venv/bin/python3 useful_tools/test_observer_excitation_offline.py
"""
import sys
sys.dont_write_bytecode = True

import builtins
import contextlib
import io
import math
import os
import random
import runpy
import shutil
import tempfile
import time
from unittest import mock

from pybear import Manager


def _no_real_bear(*a, **k):
    raise AssertionError("test tried to construct a real Manager.BEAR")


Manager.BEAR = _no_real_bear      # before bear_interface is imported

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from main_controller import bear_interface as bi                     # noqa: E402
from main_controller import config as cfg                            # noqa: E402
from main_controller.controller import KT, PHI_DEG                   # noqa: E402
from sweep_tables import sweep_tables, read_csv                       # noqa: E402

REAL_BI = bi.BearInterface   # captured before any patching

SCRIPT = os.path.join(HERE, 'observer_excitation.py')
H0_ARGS = ['--mode', 'sweep', '--arm-mass-kg', '0', '--arm-com-m', '0',
           '--velocity', '0.10', '--soft-start-s', '0.3']
SETTING_NAMES = ['p_gain_iq', 'i_gain_iq', 'd_gain_iq', 'p_gain_id', 'i_gain_id',
                 'd_gain_id', 'p_gain_velocity', 'i_gain_velocity',
                 'd_gain_velocity', 'p_gain_position', 'i_gain_position',
                 'd_gain_position', 'limit_i_max', 'limit_velocity_max',
                 'limit_acc_max', 'limit_position_min', 'limit_position_max']

BUS_S = 0.001           # s per bus transaction
SUB_S = 0.0002          # physics substep
TE_LAG_S = 0.001        # torque_enable read-back lag
SP_WN = 60.0            # rad/s, setpoint profile natural frequency
POS_KP = 4.6            # 1/s: gives the measured ~1.4 deg lag at 0.10 rad/s
VEL_TAU = 0.02          # s
J = 0.015               # kg m^2
F_STATIC = 0.85         # N m, motor-off stiction (Karnopp)
POS_NOISE = math.radians(0.004)
VEL_NOISE = 0.026
IQ_NOISE = 0.19         # A, measured: down leg 32-36 deg, detrended, Oct 14-15 (0.18-0.19)

# sweep_tables() of raw-data:logs/phase2/observer_csvs/
# observer_sweep_bare_20261014_220311.csv (args identical to H0):
# (vest deg, f N m, y N m). Friction is ~flat, NOT rising with angle.
MEASURED = [(105.2, 0.930, 1.950), (110.1, 0.980, 1.800), (115.0, 0.963, 1.714),
            (120.0, 0.948, 1.666), (125.0, 0.982, 1.634), (130.0, 1.007, 1.598),
            (135.0, 0.932, 1.416), (140.0, 0.946, 1.347), (145.0, 0.972, 1.308),
            (150.0, 0.902, 1.211), (155.0, 0.925, 1.187), (160.0, 0.947, 1.189),
            (165.0, 0.984, 1.196), (169.6, 0.971, 1.223)]
# Same run, measured (|iq|, A): down leg 32-36 deg mean; max over the run; max
# of the lowest of 3 consecutive samples (what the 3x abort sees); dwell mean.
REAL = dict(down_mean=4.26, peak=4.67, run3=4.50, dwell_mean=4.09)
# The other three Oct 14-15 H0-argument runs: down_mean 4.14-4.17, peak
# 4.59-4.64, run3 4.37-4.50, dwell_mean 3.98-4.09.


def _interp(act, col):
    vest = math.degrees(act) + PHI_DEG
    if vest <= MEASURED[0][0]:
        return MEASURED[0][col]
    if vest >= MEASURED[-1][0]:
        return MEASURED[-1][col]
    for a, b in zip(MEASURED, MEASURED[1:]):
        if a[0] <= vest <= b[0]:
            return a[col] + (vest - a[0]) / (b[0] - a[0]) * (b[col] - a[col])


def y_nm(act):
    return _interp(act, 2)


def fric_nm(act):
    return _interp(act, 1)


class Clock:
    def __init__(self):
        self.t = 5000.0
        self.sim = None
        self.kbi_after = []      # Ctrl-C at these times after enable
        self.stall = None        # (after_s, extra_s): one sleep overruns (host freeze)

    def monotonic(self):
        return self.t

    def sleep(self, d):
        self.advance(max(0.0, d))
        te = self.sim.t_enable
        if self.stall and te is not None and self.t - te >= self.stall[0]:
            extra, self.stall = self.stall[1], None
            self.advance(extra)
        if self.kbi_after and te is not None and self.t - te >= self.kbi_after[0]:
            self.kbi_after.pop(0)
            raise KeyboardInterrupt

    def advance(self, d):
        end = self.t + d
        while self.t < end - 1e-12:
            h = min(SUB_S, end - self.t)
            self.sim.step(h)
            self.t += h


class PosFake:
    def __init__(self, clock, start_deg=116.5, top_stop_deg=116.5, seed=27,
                 drop=(), err=0x80, temp=30.0, bounce_deg=3.0):
        self.clock = clock
        clock.sim = self
        self.top = math.radians(top_stop_deg)
        # A free arm hitting the top stop bounces past it (Oct 15: 120.18 deg from a
        # 116.3 deg rest). If the bounce passes the WRITTEN limit_position_max the
        # firmware latches Joint Limit: error 0x90, torque_enable reads 3 (Oct 28).
        self.bounce = math.radians(bounce_deg)
        self.pos, self.vel, self.iq = math.radians(start_deg), 0.0, 0.0
        self.sp, self.sp_v = 0.0, 0.0          # setpoint profile starts at 0
        self.te = self.te_visible = 0
        self.te_change_at = None
        self.err = err
        self.temp = temp
        self.drop = set(drop)                  # register writes that are lost
        self.rng = random.Random(seed)
        self.regs = {'mode': 0, 'goal_position': 0.0, 'goal_iq': 0.0,
                     'limit_position_min': -0.0872665, 'limit_position_max': 2.18166,
                     'limit_velocity_max': 1.0, 'limit_acc_max': 5.0}
        self.writes = []                       # (t, name, value)
        self.lost = []                         # torque_enable writes lost back-to-back
        self.prev_bus_was_write = False
        self.calls = []                        # non-get/set calls
        self.max_iq = 0.0
        self.iq_log = []                       # (t, |iq| as read)
        self.t_enable = None
        # fault injection, times in s after enable
        self.spike = None                      # (after_s, n_reads, value)
        self.jam_after = None
        self.slow_after = None                 # (after_s, max speed rad/s)
        self.err_after = None                  # (after_s, err)
        self.fric_dir = 0.0                    # friction keeps its last direction
        self.last_was_write = False            # for the back-to-back write loss
        self.temp_after = None                 # (after_s, temp)

    def since_enable(self):
        return None if self.t_enable is None else self.clock.t - self.t_enable

    def step(self, h):
        r = self.regs
        a = SP_WN ** 2 * (r['goal_position'] - self.sp) - 2 * SP_WN * self.sp_v
        amax, vmax = r['limit_acc_max'], r['limit_velocity_max']
        a = max(-amax, min(amax, a))
        self.sp_v = max(-vmax, min(vmax, self.sp_v + a * h))
        self.sp += self.sp_v * h
        if self.te_change_at is not None and self.clock.t >= self.te_change_at:
            self.te_visible, self.te_change_at = self.te, None
        s = self.since_enable()
        if self.err_after and s is not None and s >= self.err_after[0]:
            self.err, self.err_after = self.err_after[1], None
        if self.temp_after and s is not None and s >= self.temp_after[0]:
            self.temp, self.temp_after = self.temp_after[1], None
        jammed = self.jam_after is not None and s is not None and s >= self.jam_after
        if jammed:
            self.vel = 0.0
            self.iq = 0.0 if self.te == 0 else max(-5.5, min(5.5, 10 * (self.sp - self.pos) / KT))
        elif self.te == 1 and r['mode'] == 2:
            # no setpoint-velocity feedforward: the real arm lags ~1.4 deg at 0.1 rad/s
            v_cmd = max(-vmax, min(vmax, POS_KP * (self.sp - self.pos)))
            if self.slow_after and s is not None and s >= self.slow_after[0]:
                v_cmd = max(-self.slow_after[1], min(self.slow_after[1], v_cmd))
            acc = (v_cmd - self.vel) / VEL_TAU
            self.vel += acc * h
            self.pos += self.vel * h
            if abs(self.vel) > 0.002:
                self.fric_dir = math.copysign(1.0, self.vel)
            fr = fric_nm(self.pos) * self.fric_dir
            tau = J * acc - y_nm(self.pos) + fr
            self.iq = tau / KT
        else:
            self.iq = 0.0
            self.fric_dir = 0.0
            tau = y_nm(self.pos)
            if self.vel == 0.0:
                if abs(tau) > F_STATIC:
                    self.vel = (tau - math.copysign(F_STATIC, tau)) / J * h
            else:
                new = self.vel + (tau - math.copysign(F_STATIC, self.vel)) / J * h
                self.vel = 0.0 if new * self.vel < 0 else new
            self.pos += self.vel * h
        if self.pos >= self.top:
            if (self.te == 0 and self.vel > 0.2
                    and self.top + self.bounce > self.regs['limit_position_max']):
                self.err = 0x90
                self.te = self.te_visible = 3
                self.te_change_at = None
            self.pos, self.vel = self.top, min(self.vel, 0.0)

    def _read(self, name):
        if name == 'torque_enable':
            return self.te_visible
        if name == 'present_position':
            return self.pos + self.rng.gauss(0.0, POS_NOISE)
        if name == 'present_velocity':
            return self.vel + self.rng.gauss(0.0, VEL_NOISE)
        if name == 'present_iq':
            v = self.iq + self.rng.gauss(0.0, IQ_NOISE)
            self.max_iq = max(self.max_iq, abs(self.iq))
            self.iq_log.append((self.clock.t, abs(v)))
            s = self.since_enable()
            if self.spike and s is not None and s >= self.spike[0] and self.spike[1] > 0:
                self.spike = (self.spike[0], self.spike[1] - 1, self.spike[2])
                return self.spike[2]
            return v
        if name == 'goal_position':
            return self.sp
        if name in ('winding_temperature', 'powerstage_temperature'):
            return self.temp
        return self.regs.get(name)

    def _write(self, name, value):
        self.writes.append((self.clock.t, name, value))
        if name in self.drop:
            return
        if name == 'torque_enable' and self.prev_bus_was_write:
            self.lost.append((self.clock.t, name, value))
            return
        if name == 'goal_iq' and self.regs['mode'] not in (0, 3):
            return             # SDK 1.2.0: goal_iq writable only in modes 0 and 3
        if name == 'goal_position':
            if self.regs['mode'] in (2, 3) and \
                    self.regs['limit_position_min'] <= value <= self.regs['limit_position_max']:
                self.regs[name] = value
            return
        if name == 'torque_enable':
            if value == 1 and self.t_enable is None:
                self.t_enable = self.clock.t
            if value == 0:
                self.err = 0x80
            self.te = value
            self.te_change_at = self.clock.t + TE_LAG_S
            return
        self.regs[name] = value

    def __getattr__(self, attr):
        if attr.startswith('get_'):
            name = attr[4:]

            def getter(*ids):
                self.clock.advance(BUS_S)
                self.last_was_write = False
                return [([self._read(name)], self.err) for _ in ids]
            return getter
        if attr.startswith('set_'):
            name = attr[4:]

            def setter(*pairs):
                self.clock.advance(BUS_S)
                self.prev_bus_was_write = self.last_was_write
                for _, v in pairs:
                    self._write(name, v)
                self.last_was_write = True
            return setter
        if attr.startswith('_'):
            raise AttributeError(attr)

        def other(*a, **k):
            self.calls.append(attr)
        return other


def make_iface(fake):
    iface = REAL_BI.__new__(REAL_BI)
    iface.bear = fake
    iface.id = 1
    iface.last_disable_attempts = iface.last_disable_ms = None
    return iface


def invoke(argv, fake=None, clock=None, on_input=None, safety=True):
    """Run the real observer_excitation.py in-process. Returns dict."""
    made = []

    def factory():
        made.append(1)
        if fake is None:
            raise AssertionError("script opened the port")
        return make_iface(fake)

    def fake_input(p=''):
        print(p)
        if on_input:
            on_input()
        return ''

    logdir = tempfile.mkdtemp(prefix='obsx_')
    out = io.StringIO()
    patches = [mock.patch.object(bi, 'BearInterface', factory),
               mock.patch.object(cfg, 'LOG_DIR', logdir),
               mock.patch.object(cfg, 'SAFETY_CHECKS_CONFIRMED', safety),
               mock.patch.object(builtins, 'input', fake_input),
               mock.patch.object(sys, 'argv', ['observer_excitation.py'] + argv)]
    if clock is not None:
        patches += [mock.patch.object(time, 'monotonic', clock.monotonic),
                    mock.patch.object(time, 'sleep', clock.sleep)]
    rc = 0
    with contextlib.ExitStack() as st:
        for p in patches:
            st.enter_context(p)
        st.enter_context(contextlib.redirect_stdout(out))
        st.enter_context(contextlib.redirect_stderr(out))
        try:
            runpy.run_path(SCRIPT, run_name='__main__')
        except SystemExit as e:
            rc = e.code
        except KeyboardInterrupt:
            rc = 'KeyboardInterrupt (uncaught)'
        except Exception as e:
            rc = f"exception: {e!r}"
    csvs = []
    for root, _, files in os.walk(logdir):
        csvs += [os.path.join(root, f) for f in files if f.endswith('.csv')]
    keep = None
    if csvs:
        keep = os.path.join(tempfile.gettempdir(), 'obsx_keep_' + os.path.basename(csvs[0]))
        n = 0
        while os.path.exists(keep):
            n += 1
            keep = keep.replace('.csv', f'_{n}.csv')
        shutil.copy(csvs[0], keep)
    shutil.rmtree(logdir)
    return dict(rc=rc, out=out.getvalue(), ports=len(made), csv=keep, fake=fake)


FAILS = []
NCHECK = [0]
WRITE_LOG = {}


def check(name, cond, detail=''):
    NCHECK[0] += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ''))
    if not cond:
        FAILS.append(name)


def compress(writes, calls=()):
    """Write sequence with runs of the same register collapsed: 'name=v' or
    'name xN'. The 17 settings collapse to one entry."""
    seq, i = [], 0
    names = [n for _, n, _ in writes]
    if names[:17] == SETTING_NAMES:
        seq.append('17 settings (gains, limits)')
        i = 17
    while i < len(writes):
        n, v = writes[i][1], writes[i][2]
        j = i
        while j < len(writes) and writes[j][1] == n:
            j += 1
        if j - i == 1:
            seq.append(f"{n}={v:g}" if isinstance(v, (int, float)) else n)
        else:
            seq.append(f"{n} x{j - i}")
        i = j
    seq += [f"CALL {c}" for c in calls]
    return ' -> '.join(seq) if seq else '(none)'


def names(fake):
    return [n for _, n, _ in fake.writes]


def enable1_written(fake):
    return any(n == 'torque_enable' and v == 1 for _, n, v in fake.writes)


def ends_disabled(fake, label):
    """Last torque_enable write is 0, confirmed (fake state 0), and nothing at
    all is written after it; the final disable() needed ONE torque_enable = 0
    write (its spacing read beat the back-to-back loss)."""
    te_idx = [i for i, (_, n, _) in enumerate(fake.writes) if n == 'torque_enable']
    last = te_idx[-1] if te_idx else None
    ok = (last is not None and fake.writes[last][2] == 0 and fake.te == 0
          and last == len(fake.writes) - 1)
    check(f"{label}: ends with torque_enable 0, confirmed, nothing written after", ok,
          compress(fake.writes[-3:]))
    gi = [i for i, (_, n, _) in enumerate(fake.writes) if n == 'goal_iq']
    te0 = [i for i in te_idx if gi and i > gi[-1]]
    check(f"{label}: final disable confirmed on its first torque_enable 0 (none lost)",
          len(te0) == 1 and not fake.lost, f"{len(te0)} writes, {len(fake.lost)} lost")


def csv_end(path):
    comments, _ = read_csv(path)
    return ' | '.join(c for c in comments if c.startswith('# END') or c.startswith('# PARK'))


def phases(path):
    _, rows = read_csv(path)
    seen = []
    for r in rows:
        if not seen or seen[-1] != r['phase']:
            seen.append(r['phase'])
    return seen, rows


# ---------------------------------------------------------------------------
def section1():
    print("1. Argument guards refuse before the port is opened")
    bad = {
        'no --arm-mass-kg': ['--mode', 'sweep', '--arm-com-m', '0'],
        'no --mode': ['--arm-mass-kg', '0', '--arm-com-m', '0'],
        'NaN mass': ['--mode', 'sweep', '--arm-mass-kg', 'nan', '--arm-com-m', '0'],
        'negative CoM': ['--mode', 'sweep', '--arm-mass-kg', '0', '--arm-com-m', '-0.1'],
        'velocity 0': H0_ARGS[:6] + ['0'] + H0_ARGS[7:],
        'velocity 0.9 (> 0.8 cap)': H0_ARGS[:6] + ['0.9'] + H0_ARGS[7:],
        'soft-start 0.2 (< 0.3)': H0_ARGS[:8] + ['0.2'],
        'const span < 20 deg (v 0.8, soft 1.0)': H0_ARGS[:6] + ['0.8', '--soft-start-s', '1.0'],
        'run cap (v 0.02 -> >120 s)': H0_ARGS[:6] + ['0.02'] + H0_ARGS[7:],
        '--freq-hz with sweep': H0_ARGS + ['--freq-hz', '1'],
        '--j-sine with sweep': H0_ARGS + ['--j-sine'],
    }
    for name, argv in bad.items():
        r = invoke(argv)
        check(f"refused: {name}", r['rc'] not in (0, None) and r['ports'] == 0,
              f"rc {str(r['rc'])[:40]}, ports {r['ports']}")
    r = invoke(H0_ARGS, safety=False)
    check("refused: SAFETY_CHECKS_CONFIRMED False", r['rc'] not in (0, None) and r['ports'] == 0,
          f"ports {r['ports']}")
    WRITE_LOG['1. any argument refusal'] = '(no port opened, no writes)'


def section2():
    print("2. Pre-enable refusals after the port opens (never torque_enable 1)")
    cases = {
        'error byte 0x81 at start': dict(fake=dict(err=0x81)),
        'too hot (66 C >= TEMP_WARN)': dict(fake=dict(temp=66.0)),
        'start below MIN_ANGLE (4 deg)': dict(fake=dict(start_deg=4.0)),
        'start above hard stop (120 deg)': dict(fake=dict(start_deg=120.0, top_stop_deg=121.0)),
        'readback mismatch (p_gain_position lost)': dict(fake=dict(drop={'p_gain_position'})),
        'error byte appears at the Enter prompt': dict(at_input='err'),
        'arm moved below MIN_ANGLE at the Enter prompt': dict(at_input='move'),
    }
    for name, c in cases.items():
        clock = Clock()
        fake = PosFake(clock, **c.get('fake', {}))
        hook = None
        if c.get('at_input') == 'err':
            hook = lambda f=fake: setattr(f, 'err', 0x81)
        elif c.get('at_input') == 'move':
            hook = lambda f=fake: setattr(f, 'pos', math.radians(4.0))
        r = invoke(H0_ARGS, fake, clock, on_input=hook)
        ok = (r['rc'] not in (0, None) and not enable1_written(fake)
              and 'mode' not in names(fake) and fake.te == 0 and r['csv'] is None)
        check(f"refused, no mode/enable write, no CSV: {name}", ok,
              f"rc {str(r['rc'])[:50]}")
        WRITE_LOG[f"2. {name}"] = compress(fake.writes, fake.calls)
        if r['csv']:
            os.remove(r['csv'])


def section3():
    print("3. enable_position_mode() called directly")
    p_lim = (math.radians(5.0), math.radians(118.0))

    def fresh(**kw):
        # bare arm resting on the top stop, motor off (as before every run)
        clock = Clock()
        fake = PosFake(clock, start_deg=116.5, top_stop_deg=116.5, **kw)
        fake.regs['limit_position_min'], fake.regs['limit_position_max'] = p_lim
        return clock, fake, make_iface(fake)

    def call(iface, target):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                iface.enable_position_mode(target)
                return None
            except RuntimeError as e:
                return str(e)

    # normal
    clock, fake, iface = fresh()
    with mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', clock.sleep):
        err = call(iface, fake.pos)
        clock.advance(0.5)                 # let it hold
    order = [n for n in names(fake)]
    check("normal: enables", err is None and fake.te == 1, str(err))
    check("normal: writes exactly mode -> goal_position -> torque_enable",
          order == ['mode', 'goal_position', 'torque_enable'], ' -> '.join(order))
    check("normal: goal written = the given position, mode 2",
          fake.writes[0][2] == 2 and abs(fake.writes[1][2] - fake.pos) < math.radians(0.2))
    check("normal: arm holds within 0.5 deg after enable (no lurch)",
          abs(fake.pos - fake.writes[1][2]) < math.radians(0.5),
          f"{math.degrees(fake.pos - fake.writes[1][2]):+.3f} deg")
    check("normal: no save_config or other call", fake.calls == [], str(fake.calls))
    WRITE_LOG['3. enable_position_mode normal'] = compress(fake.writes)

    # disable() on the enabled fake (torque_enable lost if sent straight after a write)
    n0 = len(fake.writes)
    with mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', clock.sleep), \
            contextlib.redirect_stdout(io.StringIO()):
        ok = iface.disable()
    dw = [n for _, n, _ in fake.writes[n0:]]
    check("disable(): goal_iq 0, a read, then ONE torque_enable 0, confirmed on attempt 1",
          ok is True and dw == ['goal_iq', 'torque_enable'] and iface.last_disable_attempts == 1
          and fake.te == 0 and not fake.lost,
          f"returned {ok}, writes {dw}, attempts {iface.last_disable_attempts}, lost {len(fake.lost)}")
    WRITE_LOG['3. disable() after enable'] = compress(fake.writes[n0:])

    cases = [
        ('already enabled', dict(pre=lambda f: (setattr(f, 'te', 1), setattr(f, 'te_visible', 1))),
         'torque_enable 1', []),
        ('error byte latched', dict(pre=lambda f: setattr(f, 'err', 0x81)), 'error byte', []),
        ('mode write lost', dict(kw=dict(drop={'mode'})), 'mode reads', ['mode']),
        ('goal outside limits (119 deg, ignored)', dict(target=math.radians(119.0)),
         'not settled', ['mode', 'goal_position']),
        ('arm 2 deg away from the goal', dict(offset=math.radians(-2.0)),
         'present_position', ['mode', 'goal_position']),
    ]
    for name, c, msg, expect in cases:
        clock, fake, iface = fresh(**c.get('kw', {}))
        if 'pre' in c:
            c['pre'](fake)
        target = c.get('target', fake.pos + c.get('offset', 0.0))
        with mock.patch.object(time, 'monotonic', clock.monotonic), \
                mock.patch.object(time, 'sleep', clock.sleep):
            err = call(iface, target)
        ok = err is not None and msg in err and names(fake) == expect and not enable1_written(fake)
        check(f"refuses: {name}; writes {expect or 'none'}, never torque_enable 1", ok,
              f"{err}; wrote {names(fake)}")
        WRITE_LOG[f"3. enable_position_mode refuses: {name}"] = compress(fake.writes)

    clock, fake, iface = fresh(drop={'torque_enable'})
    with mock.patch.object(time, 'monotonic', clock.monotonic), \
            mock.patch.object(time, 'sleep', clock.sleep):
        err = call(iface, fake.pos)
    check("enable write lost: raises 'enable failed' after the 20 ms poll",
          err is not None and 'enable failed' in err and fake.te == 0, str(err))
    WRITE_LOG['3. enable_position_mode: enable write lost'] = compress(fake.writes)


def run_h0(label, setup=None, kbi=(), start_deg=116.5, top_stop_deg=116.5):
    clock = Clock()
    clock.kbi_after = list(kbi)
    fake = PosFake(clock, start_deg=start_deg, top_stop_deg=top_stop_deg)
    if setup:
        setup(fake)
    r = invoke(H0_ARGS, fake, clock)
    WRITE_LOG[label] = compress(fake.writes, fake.calls)
    return r


def section4():
    print("4. Tonight's H0 command, normal run: " + ' '.join(H0_ARGS))
    r = run_h0('4. H0 sweep, normal')
    fake = r['fake']
    check("completes, exit 0, CSV written", r['rc'] == 0 and r['csv'] is not None,
          f"rc {r['rc']}")
    if not r['csv']:
        print(r['out'][-3000:])
        return
    w = fake.writes
    nm = names(fake)
    check("first 17 writes are the settings, in order", nm[:17] == SETTING_NAMES)
    vals = {n: v for _, n, v in w[:17]}
    check("limits: velocity 1.0, acc 5.0, i_max 5.5",
          (vals['limit_velocity_max'], vals['limit_acc_max'], vals['limit_i_max']) == (1.0, 5.0, 5.5),
          f"{vals['limit_velocity_max']}, {vals['limit_acc_max']}, {vals['limit_i_max']}")
    check("firmware position limits: min = MIN_ANGLE (5), max = 125 deg (flash value)",
          abs(vals['limit_position_min'] - cfg.MIN_ANGLE) < 1e-9
          and abs(vals['limit_position_max'] - math.radians(125.0)) < 1e-9,
          f"max {math.degrees(vals['limit_position_max']):.2f} deg")
    gmax = max(v for _, n, v in w[20:] if n == 'goal_position')
    check("every goal sent after enable is still clamped to MAX_ANGLE (118 deg)",
          gmax <= cfg.MAX_ANGLE + 1e-12, f"max goal {math.degrees(gmax):.3f} deg")
    check("then mode 2 -> goal_position -> torque_enable 1",
          nm[17:20] == ['mode', 'goal_position', 'torque_enable'] and w[17][2] == 2 and w[19][2] == 1,
          ' -> '.join(nm[17:20]))
    check("first goal = start position (no lurch)",
          abs(w[18][2] - math.radians(116.5)) < math.radians(0.1), f"{math.degrees(w[18][2]):.2f} deg")
    mid = set(nm[20:-2])
    check("while enabled, only goal_position is written", mid == {'goal_position'}, str(mid))
    check("never save_config (or any non-register call)", fake.calls == [], str(fake.calls))
    check("disable at the end writes goal_iq 0 then torque_enable 0",
          nm[-2:] == ['goal_iq', 'torque_enable'] and w[-2][2] == 0.0 and w[-1][2] == 0)
    ends_disabled(fake, "normal")

    seen, rows = phases(r['csv'])
    check("phases approach -> down -> dwell -> up -> park", seen == ['approach', 'down', 'dwell', 'up', 'park'],
          ' -> '.join(seen))
    if not rows or 'park' not in seen:
        check("CSV has data rows and a park (needed for the rest of section 4)", False,
              f"{len(rows)} rows")
        return
    goal = [float(x['goal_pos']) for x in rows]
    pos = [float(x['pos']) for x in rows]
    lo = min(math.degrees(p) for p in pos)
    hi_up = max(math.degrees(float(x['pos'])) for x in rows if x['phase'] == 'up')
    check("sweep reaches ~30 deg and comes back to ~100 deg (real: bottom 31.2)",
          29.5 < lo < 32.0 and 98.0 < hi_up < 100.5, f"min {lo:.2f}, up-leg max {hi_up:.2f}")
    dn = [x for x in rows if x['phase'] == 'down' and 50 < math.degrees(float(x['pos'])) < 90]
    lag = sum(math.degrees(float(x['pos']) - float(x['goal_pos'])) for x in dn) / len(dn)
    check("fake lags the goal like the real arm (real 1.3-1.5 deg on the down leg)",
          1.1 < lag < 1.8, f"{lag:.2f} deg")
    park = [x for x in rows if x['phase'] == 'park']
    check("parks HIGH: last park goal 110 deg (bare: spring wins)",
          abs(math.degrees(float(park[-1]['goal_pos'])) - 110.0) < 0.01 and 'Park: HIGH' in r['out'])
    at_release = math.degrees(fake.pos)
    fake.clock.advance(2.0)                     # motor off: let the spring act
    check("released at ~108-110 deg (real: 108.1-108.2), arm then rises to the top stop",
          107.0 < at_release < 110.5 and abs(math.degrees(fake.pos) - 116.5) < 0.1,
          f"{at_release:.1f} -> {math.degrees(fake.pos):.1f} deg")
    check("after the release bounce (3 deg past the stop) no Joint Limit latch: "
          "error 0x80, torque_enable 0 (ready for hold_bench_test)",
          fake.err == 0x80 and fake.te_visible == 0, f"error {hex(fake.err)}, te {fake.te_visible}")
    # the fake reproduces Oct 28: same release with the old 118 deg limit latches
    for lim, want in ((118.0, 0x90), (125.0, 0x80)):
        c2 = Clock()
        f2 = PosFake(c2, start_deg=108.0, top_stop_deg=117.2)
        f2.regs['limit_position_max'] = math.radians(lim)
        c2.advance(2.0)
        check(f"fake model: release from 108 deg with limit {lim:.0f} deg -> error {hex(want)}",
              f2.err == want and abs(math.degrees(f2.pos) - 117.2) < 0.1,
              f"error {hex(f2.err)}, te {f2.te_visible}")
    err_max = max(abs(g - p) for g, p in zip(goal, pos))
    check("tracking error stays well inside the 15 deg abort", math.degrees(err_max) < 5.0,
          f"max {math.degrees(err_max):.2f} deg")
    # calibrated current: compare with the measured run, then report the margins
    a = [abs(float(x['present_iq'])) for x in rows]
    ph = [x['phase'] for x in rows]
    pd = [math.degrees(float(x['pos'])) for x in rows]
    dm = [v for v, q, d in zip(a, ph, pd) if q == 'down' and 32 < d < 36]
    dwm = [v for v, q in zip(a, ph) if q == 'dwell']
    lead = [v for v, q in zip(a, ph) if q != 'park']
    run3 = max(min(lead[i:i + 3]) for i in range(len(lead) - 2))
    down_mean, dwell_mean = sum(dm) / len(dm), sum(dwm) / len(dwm)
    check("fake down-leg current (32-36 deg) within 0.15 A of the measured run",
          abs(down_mean - REAL['down_mean']) < 0.15, f"fake {down_mean:.2f}, real {REAL['down_mean']}")
    check("fake dwell current within 0.3 A of the measured run (friction stays loaded)",
          abs(dwell_mean - REAL['dwell_mean']) < 0.3, f"fake {dwell_mean:.2f}, real {REAL['dwell_mean']}")
    check("fake peaks within 0.25 A of the measured run (max, lowest-of-3)",
          abs(max(lead) - REAL['peak']) < 0.25 and abs(run3 - REAL['run3']) < 0.25,
          f"max {max(lead):.2f} vs {REAL['peak']}, lowest-of-3 {run3:.2f} vs {REAL['run3']}")
    check("completes under the aborts: margin to 5.0 A x3 and to the 5.3 A trip",
          run3 < 5.0 and max(lead) < 5.3,
          f"margins {5.0 - run3:.2f} A (x3) and {5.3 - max(lead):.2f} A (trip); "
          f"real {5.0 - REAL['run3']:.2f} / {5.3 - REAL['peak']:.2f}")
    check("CSV END: complete, PARK: parked", '# END: complete' in csv_end(r['csv'])
          and '# PARK: parked' in csv_end(r['csv']), csv_end(r['csv']))

    # timestamps
    t = [float(x['t']) for x in rows]
    tp = [float(x['t_pos']) for x in rows]
    tv = [float(x['t_vel']) for x in rows]
    ti = [float(x['t_iq']) for x in rows]
    order_ok = all(a < b < c < d for a, b, c, d in zip(t, tp, tv, ti))
    mono = all(b > a for a, b in zip(ti, ti[1:]))
    gap = sorted(b - a for a, b in zip(tp, ti))[len(tp) // 2]
    check("every row: t < t_pos < t_vel < t_iq; t_iq increases row to row", order_ok and mono)
    check("pos -> iq reads ~2 ms apart (median)", 0.0015 < gap < 0.0025, f"{gap * 1000:.2f} ms")
    dts = sorted(float(x['loop_dt']) for x in rows if x['phase'] != 'park')
    check("loop period ~5 ms (median loop_dt)", 0.0045 < dts[len(dts) // 2] < 0.0065,
          f"{dts[len(dts) // 2] * 1000:.2f} ms")
    hdr = open(r['csv']).readline()
    real_cols = ('t,loop_dt,goal_pos,pos,vel,present_iq,w_temp,p_temp,mode,phase,freq_hz,amp_deg,'
                 'center_deg,velocity,soft_start_s,arm_mass_kg,arm_com_m,t_pos,t_vel,t_iq')
    body_hdr = [l for l in open(r['csv']).read().splitlines() if not l.startswith('#')][0]
    check("columns identical to the Oct 15 hardware J-sine CSVs", body_hdr == real_cols, body_hdr[:60])

    # the analysis the hold work depends on
    try:
        tab = sweep_tables(r['csv'])
    except SystemExit as e:
        check("sweep_tables parses it", False, str(e))
        return
    check("sweep_tables parses it (>= 10 bins)", len(tab) >= 10, f"{len(tab)} bins, "
          f"vest {tab[0, 0]:.0f}-{tab[-1, 0]:.0f} deg")
    dy, df = [], []
    for vest, f, y, *_ in tab:
        act = math.radians(vest - PHI_DEG)
        dy.append(y - y_nm(act))
        df.append(f - fric_nm(act))
    check("recovers the fake's y = spring - rig gravity (sign right), within 0.05 N m",
          max(map(abs, dy)) < 0.05, f"max |dy| {max(map(abs, dy)):.3f}")
    check("recovers the fake's friction f, within 0.05 N m",
          max(map(abs, df)) < 0.05, f"max |df| {max(map(abs, df)):.3f}")
    os.remove(r['csv'])

    # start on the real Sep 30 resting angle
    r = run_h0('4. H0 sweep, start 117.2 deg', start_deg=117.175, top_stop_deg=117.175)
    check("start at 117.2 deg (Sep 30 rest angle): completes", r['rc'] == 0, f"rc {r['rc']}")
    if r['csv']:
        os.remove(r['csv'])


def section5():
    print("5. Runtime aborts: each parks or releases, then disables")
    cases = [
        ('sustained current 5.2 A x3', dict(setup=lambda f: setattr(f, 'spike', (15.0, 3, -5.2))),
         1, '|present_iq| > 5.0 A for 3', 'parked'),
        ('one sample 5.4 A (immediate trip)', dict(setup=lambda f: setattr(f, 'spike', (15.0, 1, -5.4))),
         1, 'immediate trip', 'parked'),
        ('arm jams (tracking > 15 deg), park then stalls', dict(setup=lambda f: setattr(f, 'jam_after', 8.0)),
         1, 'diverged', 'released early -- no 1 deg of progress'),
        ('error byte 0x81 mid-run', dict(setup=lambda f: setattr(f, 'err_after', (12.0, 0x81))),
         1, 'Error byte', 'parked'),
        ('winding 76 C mid-run', dict(setup=lambda f: setattr(f, 'temp_after', (12.0, 76.0))),
         1, 'TEMP_MAX', 'parked'),
        ('Ctrl-C mid-run', dict(kbi=[12.0]), 130, 'Ctrl-C', 'parked'),
    ]
    for name, kw, rc_want, end_want, park_want in cases:
        r = run_h0(f"5. abort: {name}", **kw)
        fake = r['fake']
        end = csv_end(r['csv']) if r['csv'] else ''
        check(f"{name}: exit {rc_want}, END names it, PARK {park_want}",
              r['rc'] == rc_want and end_want in end and park_want in end,
              f"rc {r['rc']}; {end[:110]}")
        ends_disabled(fake, name)
        if r['csv']:
            os.remove(r['csv'])

    # the park's own release branches (park starts at ~51 deg after a Ctrl-C at 12 s)
    park_cases = [
        ('park current 5.2 A x3', dict(setup=lambda f: setattr(f, 'spike', (12.5, 3, -5.2))),
         'released early -- |present_iq| > 5.0 A for 3 consecutive samples during park'),
        ('park stall (arm jams in the park)', dict(setup=lambda f: setattr(f, 'jam_after', 12.5)),
         'released early -- no 1 deg of progress'),
    ]
    for name, kw, want in park_cases:
        r = run_h0(f"5. park release: {name}", kbi=[12.0], **kw)
        fake = r['fake']
        end = csv_end(r['csv']) if r['csv'] else ''
        check(f"{name}: Ctrl-C, then the park releases for that reason", r['rc'] == 130 and want in end,
              f"rc {r['rc']}; {end[:130]}")
        ends_disabled(fake, name)
        if r['csv']:
            os.remove(r['csv'])

    # Park deadline: UNREACHABLE. The park loop ends when the GOAL reaches 110 deg
    # (<= ~12 s of goal travel at 0.15 rad/s), not the arm, so the 20 s deadline
    # never fires. An arm that crawls (> 1 deg / 2 s, so no stall) is released
    # far from the target and the CSV says "parked". Documented, not endorsed:
    r = run_h0("5. NOTE park with a crawling arm (0.8 deg/s)",
               setup=lambda f: setattr(f, 'slow_after', (12.0, math.radians(0.8))), kbi=[12.0])
    fake = r['fake']
    end = csv_end(r['csv']) if r['csv'] else ''
    rel = math.degrees(fake.pos)
    check("NOTE: crawling arm -> park reports 'parked' with the arm far below 110 deg "
          "(park deadline is unreachable)", 'PARK: parked' in end and rel < 80.0,
          f"arm at release {rel:.1f} deg; {end[-40:]}")
    ends_disabled(fake, "crawling-arm park")
    if r['csv']:
        os.remove(r['csv'])

    r = run_h0("5. run cap (host freezes 105 s in the dwell)",
               setup=lambda f: setattr(f.clock, 'stall', (17.0, 105.0)))
    fake = r['fake']
    end = csv_end(r['csv']) if r['csv'] else ''
    check("run cap: a freeze past 120 s aborts on the cap, then parks",
          r['rc'] == 1 and 'Run cap 120s exceeded' in end and 'PARK: parked' in end,
          f"rc {r['rc']}; {end[:110]}")
    ends_disabled(fake, "run cap")
    if r['csv']:
        os.remove(r['csv'])

    r = run_h0("5. two short spikes 5.2 A x2 (no abort)",
               setup=lambda f: setattr(f, 'spike', (15.0, 2, -5.2)))
    check("2 consecutive samples over 5.0 A do NOT abort (needs 3)", r['rc'] == 0, f"rc {r['rc']}")
    if r['csv']:
        os.remove(r['csv'])

    r = run_h0("5. Ctrl-C mid-run, then Ctrl-C during the park", kbi=[12.0, 12.3])
    fake = r['fake']
    end = csv_end(r['csv']) if r['csv'] else ''
    check("second Ctrl-C during park: release announced, then exits on the Ctrl-C",
          'RELEASING EARLY: park interrupted' in r['out'] and 'uncaught' in str(r['rc']),
          f"rc {r['rc']}; released at {math.degrees(fake.pos):.1f} deg")
    ends_disabled(fake, "double Ctrl-C")
    if r['csv']:
        os.remove(r['csv'])

    # torque_enable = 1 write lost (the 0 writes still work)
    clock = Clock()
    fake = PosFake(clock)
    orig = fake._write

    def lose_enable(name, value, _o=orig, _f=fake):
        if name == 'torque_enable' and value == 1:
            _f.writes.append((_f.clock.t, name, value))
            return
        _o(name, value)
    fake._write = lose_enable
    r = invoke(H0_ARGS, fake, clock)
    WRITE_LOG["5. enable write lost inside the script"] = compress(fake.writes, fake.calls)
    end = csv_end(r['csv']) if r['csv'] else ''
    check("enable not confirmed: script stops, disables, motor never on",
          r['rc'] != 0 and fake.te == 0 and fake.t_enable is None,
          f"rc {str(r['rc'])[:60]}; {end[:80]}")
    ends_disabled(fake, "enable lost")
    check("NOTE: this exits as an uncaught RuntimeError (traceback), CSV END "
          "'unhandled exception'", 'unhandled exception' in end, end[:80])
    if r['csv']:
        os.remove(r['csv'])

    r = run_h0("5. start at 118.6 deg (above MAX_ANGLE, below hard stop)", start_deg=118.6,
               top_stop_deg=118.6)
    fake = r['fake']
    gl = [v for _, n, v in fake.writes if n == 'goal_position']
    check("start 118.6 deg: accepted (limit 125), enable goal = 118.6, every later goal "
          "<= 118 (a 0.6 deg step down), completes",
          r['rc'] == 0 and abs(math.degrees(gl[0]) - 118.6) < 0.1
          and max(gl[1:]) <= cfg.MAX_ANGLE + 1e-12,
          f"rc {str(r['rc'])[:40]}; enable goal {math.degrees(gl[0]):.2f}, later max "
          f"{math.degrees(max(gl[1:])):.2f} deg")
    check("start 118.6 deg: prints the 'will grab here ... clamped to 118' note",
          'will grab here' in r['out'] and 'clamped to 118' in r['out'])
    ends_disabled(fake, "start 118.6")
    if r['csv']:
        os.remove(r['csv'])


def main():
    import traceback
    for sec in (section1, section2, section3, section4, section5):
        try:
            sec()
        except Exception:
            tb = traceback.format_exc()
            print(tb)
            check(f"{sec.__name__} ran without crashing", False, tb.strip().splitlines()[-1])
    print("\nWRITE SEQUENCES (consecutive writes to one register collapsed)")
    for k, v in WRITE_LOG.items():
        print(f"  {k}:\n      {v}")
    print()
    if FAILS:
        print(f"{len(FAILS)} of {NCHECK[0]} CHECK(S) FAILED: " + '; '.join(FAILS))
        return 1
    print(f"ALL CHECKS PASSED ({NCHECK[0]}/{NCHECK[0]})")
    return 0


if __name__ == '__main__':
    sys.exit(main())