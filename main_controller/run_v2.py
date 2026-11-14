"""
Controller v2 run script (V2_ARCHITECTURE v1.4). THIS SCRIPT ENABLES TORQUE (Mode 0).

The plumbing around the v2 controller: bus, sensing, safety trips, shaping chain,
logging. The control logic (state machine, hold law, reading, learner, assist law)
lives in v2_control.Controller; every number lives in v2_params.py.

MODES
  worn   (default)  Sam wearing the vest, a second person at the e-stop.
                    --alpha 0 = hold-only session (no LIFT torque at all);
                    --alpha 0.10 / 0.20 / 0.35 / 0.50 = assist (one step per session).
                    Run cap 60 s.
  bench  (--bench)  No wearer. A bench driver lifts a HEAVY payload to each --bench-angles
                    angle (ascending only), then hands the hold to the controller
                    (Controller.bench_begin_hold) for --bench-hold-s, then takes it back and
                    moves up to the next angle; after the last hold it fades there. The
                    controller never sees the payload or the level (hidden). Upward-only
                    like v2, so the payload must need MORE upward torque than friction or
                    the creep cannot slide down and every reading is rejected: net
                    (payload + rig - spring) about 1.2-3.2 N m at 8 A across the angles
                    (L1 + the plate alone is 0.3-0.6: too light; V2_FABLE_REVIEW A2).
                    The driver never lowers the payload; after the fade it sinks onto the
                    padded catch.

EVERY CYCLE (2 ms, one bulk exchange()):
  position-derived velocity and acceleration (10 ms first-order filters)
  -> controller.step() (or the bench driver) -> requested torque tau (N m, >= 0)
  -> shaping: rate limit (falls <= 20, rises <= 4 N m/s) -> joint-limit taper -> thermal
     derate -> command cap -> and the final output may not rise > 4 N m/s (review #14)
     -> goal_iq.

TRIPS -> FAULT (V2 §8; every trip ends the run: goal_iq 0, confirmed disable):
  failed exchange; error byte with an error bit or the watchdog bit; loop gap > 10 ms;
  upward speed > 1 rad/s in any state but LIFT while the output is > 0;
  upward acceleration > 120 rad/s^2 for 3 cycles in LIFT / SETTLE / LOWER while > 0;
  |iq| >= cap + 0.2 A for 3 cycles or >= cap + 0.5 A once; max temperature >= 75 C;
  a controller exception or an invalid output (NaN, negative); bench: the driven arm
  > 5 deg from its target. Ctrl-C: zero + confirmed disable.
NORMAL END: run cap (60 s worn) or the bench plan done -> fade at 20 N m/s -> disable.

REFUSES BEFORE OPENING THE PORT: bad arguments; not at RT priority.
REFUSES BEFORE ENABLING (reads only): SAFETY_CHECKS_CONFIRMED not True; actuator
enabled or faulted; limit_i_max > 5.5 (not a fresh power-up); max temperature
>= 65 C; worn: arm above vest 120 deg (start with the arm down); bench: arm not
at least 5 deg below the first bench angle; anything but GO at the prompt.

EVERY WRITE (nothing else; no gains, no save_config):
  1. watchdog_timeout = 10000 us, read back              (configure_watchdog)
  2. limit_i_max = cap + 0.7 A, read back                (set_current_limit)
  3. mode 0, goal_iq 0, torque_enable 1, each read back  (enable)
  4. goal_iq every cycle inside the one bulk packet      (exchange)
  5. exit: goal_iq 0, torque_enable 0 confirmed <= 3 tries (disable); ONLY if confirmed:
     watchdog_timeout = 0 and limit_i_max = 5.5, each read back. Not confirmed: the
     watchdog is left armed (trips in ~10 ms) and the script says to cut power.

RUN (from the repo root):
  sudo chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 \\
      -m main_controller.run_v2 --level L3 --cap-a 5.5 --alpha 0 --wearer sam [--run-s 60]
  bench: ... --level L1 --cap-a 5.5 --bench --bench-angles 80,88,94 \\
      --payload-kg 1.137 --payload-r-m 0.213 --payload-angle-deg -29.7
LEARNER CARRY-OVER (worn): identification needs two accepted ~24 s readings, more than
one 60 s run holds, so the learner's window is kept between runs in
logs/phase2/v2/learner_<wearer>_<YYYYmmdd>.json: read at the start (if it exists),
written after every run (normal end or fault). --fresh-learner starts empty. Bench runs
never read or write it. A dial change between runs is handled by the learner itself
(load-change reset; never moves to a stronger level without a new top pair).
Output: logs/phase2/v2/v2_<worn|bench>_<level>_<YYYYmmdd_HHMMSS>.csv, written after
the final disable (also on a fault), chowned to SUDO_UID. '#' header, one row per
cycle, '#' footer with every controller event and the end reason.
"""
import sys
sys.dont_write_bytecode = True

import argparse
import csv
import hashlib
import json
import math
import os
import struct
import subprocess
import time

import numpy as np

from main_controller.bear_interface import BearInterface
from main_controller.config import (LOG_DIR, TEMP_WARN, TEMP_MAX, WATCHDOG_TIMEOUT_MS,
                                    SAFETY_CHECKS_CONFIRMED)
from main_controller.controller import KT, PHI_DEG
from main_controller import v2_params as P
from main_controller.v2_control import Controller

LOG_SUBDIR = os.path.join(LOG_DIR, 'phase2', 'v2')
WORN_START_MAX_VEST_DEG = 120.0
BENCH_START_BELOW_DEG = 5.0
BENCH_ANGLE_RANGE_ACT = (78.0, 98.0)          # vest 150-170: the learning window
BENCH_HOLD_S_RANGE = (30.0, 90.0)
BENCH_RAMP_V, BENCH_RAMP_TR = 0.10, 0.5       # rad/s, s
BENCH_SETTLE_S = 3.0
BENCH_POS_ERR_DEG = 5.0
BENCH_RUN_CAP_S = 300.0
BENCH_KP, BENCH_KI, BENCH_KD = 40.0, 60.0, 1.5
READ_EXC = (struct.error, IndexError, ValueError, TypeError)   # PyBEAR on malformed bytes

COLUMNS = ['t', 'loop_dt', 'act_deg', 'vest_deg', 'vel_reg', 'vel', 'acc', 'iq', 'iq_sent',
           'u_req', 'u_out', 'derate', 'temp_w', 'temp_p', 'err', 'state', 'bench_phase']
BENCH_PHASES = ('-', 'ramp', 'settle', 'ctrl', 'fade')


class Trip(Exception):
    """A V2 §8 trip -> FAULT: zero, confirmed disable, the run ends."""


def _git(*argv):
    try:
        return subprocess.run(['git', *argv], capture_output=True, text=True,
                              timeout=5).stdout.strip()
    except Exception:                                   # noqa: BLE001
        return '?'


def move_profile(a0, a1, v=BENCH_RAMP_V, tr=BENCH_RAMP_TR):
    """Trapezoidal-speed move a0 -> a1 (rad): (duration, fn(tau)). As hold_bench_test."""
    d = abs(a1 - a0)
    if d < 1e-9:
        return 0.0, (lambda tau: a1)
    s = 1.0 if a1 > a0 else -1.0
    tr_e = min(tr, math.sqrt(d * tr / v))
    vpk = v * tr_e / tr
    T = d / vpk + tr_e

    def fn(tau):
        tau = min(max(tau, 0.0), T)
        if tau < tr_e:
            x = vpk * tau * tau / (2 * tr_e)
        elif tau < T - tr_e:
            x = vpk * (tr_e / 2 + tau - tr_e)
        else:
            x = d - vpk * (T - tau) ** 2 / (2 * tr_e)
        return a0 + s * x
    return T, fn


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--level', required=True, choices=('L1', 'L2', 'L3', 'L4'),
                    help='spring dial as set (header only; the controller never reads it)')
    ap.add_argument('--cap-a', type=float, required=True,
                    help=f'command cap, one of {P.CAP_CHOICES_A}')
    ap.add_argument('--alpha', type=float, default=0.0,
                    help=f'assist gain in LIFT, one of {P.ALPHA_STEPS} (0 = hold-only)')
    ap.add_argument('--run-s', type=float, default=P.RUN_CAP_WORN_S,
                    help=f'worn run length, <= {P.RUN_CAP_WORN_S:g} s')
    ap.add_argument('--wearer', default='', help='worn: who (header; names the learner file)')
    ap.add_argument('--fresh-learner', action='store_true',
                    help='worn: ignore today\'s learner file and start empty')
    ap.add_argument('--note', default='')
    ap.add_argument('--bench', action='store_true')
    ap.add_argument('--bench-angles', default='', help='bench: actuator deg, 78-98')
    ap.add_argument('--bench-hold-s', type=float, default=32.0)
    ap.add_argument('--payload-kg', type=float, default=0.0, help='bench: header only')
    ap.add_argument('--payload-r-m', type=float, default=0.0)
    ap.add_argument('--payload-angle-deg', type=float, default=0.0)
    a = ap.parse_args(argv)
    if a.cap_a not in P.CAP_CHOICES_A:
        ap.error(f"--cap-a must be one of {P.CAP_CHOICES_A}")
    if a.alpha not in P.ALPHA_STEPS:
        ap.error(f"--alpha must be one of {P.ALPHA_STEPS}")
    if a.bench:
        if a.alpha != 0.0:
            ap.error("--bench runs hold-only (--alpha 0)")
        try:
            a.angles = [float(x) for x in a.bench_angles.split(',') if x.strip()]
        except ValueError:
            ap.error("--bench-angles: comma-separated numbers")
        if not a.angles:
            ap.error("--bench needs --bench-angles")
        lo, hi = BENCH_ANGLE_RANGE_ACT
        if any(not lo <= x <= hi for x in a.angles):
            ap.error(f"--bench-angles must be actuator {lo:g}-{hi:g} deg (vest 150-170)")
        if any(b <= x for x, b in zip(a.angles, a.angles[1:])):
            ap.error("--bench-angles must be strictly ascending (the driver only lifts)")
        if not BENCH_HOLD_S_RANGE[0] <= a.bench_hold_s <= BENCH_HOLD_S_RANGE[1]:
            ap.error(f"--bench-hold-s must be {BENCH_HOLD_S_RANGE[0]:g}-{BENCH_HOLD_S_RANGE[1]:g}")
        if a.payload_kg <= 0 or a.payload_r_m <= 0:
            ap.error("--bench needs --payload-kg and --payload-r-m (header record)")
    else:
        if a.bench_angles:
            ap.error("--bench-angles needs --bench")
        if not 1.0 <= a.run_s <= P.RUN_CAP_WORN_S:
            ap.error(f"--run-s must be 1-{P.RUN_CAP_WORN_S:g}")
        if not a.wearer or not a.wearer.replace('_', '').isalnum():
            ap.error("worn runs need --wearer (letters, digits, _): it names the learner file")
    a.i_limit = round(a.cap_a + P.LIMIT_ABOVE_CAP_A, 3)
    a.trip3 = a.cap_a + P.TRIP_3X_ABOVE_CAP_A
    a.trip1 = a.cap_a + P.TRIP_1X_ABOVE_CAP_A
    return a


class Run:
    def __init__(self, iface, a):
        self.iface, self.a = iface, a
        self.learner_path = None if a.bench else os.path.join(
            LOG_SUBDIR, f"learner_{a.wearer}_{time.strftime('%Y%m%d')}.json")
        resume = None
        bad = ''
        if self.learner_path and not a.fresh_learner and os.path.exists(self.learner_path):
            try:
                with open(self.learner_path) as f:
                    resume = json.load(f)
                if not isinstance(resume, dict):
                    raise ValueError('not a dict')
            except (OSError, ValueError) as e:
                resume, bad = None, f" (learner file unreadable: {e!r}; started fresh)"
                print(f"WARNING: {self.learner_path} unreadable ({e!r}); starting fresh")
        self.resume_note = ('fresh' if resume is None else
                            f"resumed from {os.path.basename(self.learner_path)}") + bad
        self.ctrl = Controller(cap_a=a.cap_a, alpha=a.alpha, resume=resume)
        self.fields = tuple(getattr(self.ctrl, 'LOG_FIELDS', ()))
        cap_s = BENCH_RUN_CAP_S if a.bench else a.run_s
        self.buf = np.full((int((cap_s + 5.0) / P.PERIOD_S) + 2000,
                            len(COLUMNS) + len(self.fields)), np.nan)
        self.n = 0
        self.header, self.footer, self.events = [], [], []
        self.end_reason = 'not started'
        self.armed = False
        self.limit_written = False
        self.limit_restored = False
        self.t0 = time.monotonic()

    def log(self, row):
        if self.n < len(self.buf):
            self.buf[self.n] = row
            self.n += 1

    # -- reads only ---------------------------------------------------------------
    def preflight(self):
        a, bear, i = self.a, self.iface.bear, self.iface.id
        if SAFETY_CHECKS_CONFIRMED is not True:
            raise SystemExit("REFUSED: SAFETY_CHECKS_CONFIRMED is not True in config.py")
        (te,), err = bear.get_torque_enable(i)[0]
        if te != 0 or err != 0x80:
            raise SystemExit(f"REFUSED: torque_enable={te}, error byte "
                             f"{err if err is None else hex(err)}; need 0 and 0x80.")
        (lim,), _ = bear.get_limit_i_max(i)[0]
        if lim is None or lim > P.I_MAX_RESTORE_A:
            raise SystemExit(f"REFUSED: limit_i_max reads {lim}; expected <= "
                             f"{P.I_MAX_RESTORE_A} A (power-cycle the actuator).")
        values, err = bear.bulk_read([i], self.iface.LOOP_READ_REGS)[0]
        if err is None or any(v is None for v in values):
            raise SystemExit("REFUSED: could not read state.")
        pos, vel, iq, tw, tp = values
        act = math.degrees(pos)
        if max(tw, tp) >= TEMP_WARN:
            raise SystemExit(f"REFUSED: temperature {max(tw, tp):.1f} C >= {TEMP_WARN} C.")
        if a.bench:
            if act > min(a.angles) - BENCH_START_BELOW_DEG:
                raise SystemExit(f"REFUSED: arm at actuator {act:.1f} deg; it must rest at "
                                 f"least {BENCH_START_BELOW_DEG:g} deg below the lowest bench "
                                 f"angle ({min(a.angles):g}).")
        elif act + PHI_DEG > WORN_START_MAX_VEST_DEG:
            raise SystemExit(f"REFUSED: arm at vest {act + PHI_DEG:.1f} deg; start with the arm "
                             f"down (vest <= {WORN_START_MAX_VEST_DEG:g}).")
        self.start_pos = pos
        with open(P.__file__, 'rb') as f:
            ph = hashlib.sha256(f.read()).hexdigest()[:12]
        ch = '?'
        try:
            import main_controller.v2_control as vc
            with open(vc.__file__, 'rb') as f:
                ch = hashlib.sha256(f.read()).hexdigest()[:12]
        except Exception:                               # noqa: BLE001
            pass
        self.header += [
            f"run_v2.py  git {_git('rev-parse', '--short', 'HEAD')} dirty="
            f"{'yes' if _git('status', '--porcelain', '--', 'main_controller') else 'no'}",
            f"started {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
            f"argv {' '.join(sys.argv[1:])}",
            f"level={a.level}",
            f"mode={'bench' if a.bench else 'worn'} cap_a={a.cap_a:g} alpha={a.alpha:g} "
            f"limit_i_max_run={a.i_limit:g} trips={a.trip3:g}Ax3,{a.trip1:g}Ax1 "
            f"run_s={'plan' if a.bench else a.run_s}",
            f"v2_params sha256 {ph}  v2_control sha256 {ch}  KT={KT}",
            f"wearer={a.wearer!r} note={a.note!r}",
            f"readback: limit_i_max={lim}",
            f"start: act {act:.2f} deg (vest {act + PHI_DEG:.2f}) vel {vel:.3f} iq {iq:.3f} "
            f"winding {tw:.1f} C powerstage {tp:.1f} C",
            f"sched {os.sched_getscheduler(0)}",
            f"log_fields {','.join(self.fields)}",
            f"learner {self.resume_note}",
        ]
        if a.bench:
            self.header.append(f"bench angles={a.angles} hold_s={a.bench_hold_s:g} payload_kg="
                               f"{a.payload_kg} payload_r_m={a.payload_r_m} "
                               f"payload_angle_deg={a.payload_angle_deg} (hidden from the controller)")
            self.plan = self.bench_plan(pos)
            planned = sum(seg[2] for seg in self.plan) + 1.0
            self.header.append(f"bench planned {planned:.1f} s (cap {BENCH_RUN_CAP_S:g})")
            if planned > BENCH_RUN_CAP_S - 10.0:
                raise SystemExit(f"REFUSED: bench plan {planned:.0f} s > "
                                 f"{BENCH_RUN_CAP_S - 10.0:g} s; use fewer angles or shorter holds.")
        for line in self.header:
            print(line)

    def recheck(self):
        """Reads only, right after GO: the state may have changed while the prompt waited."""
        a, bear, i = self.a, self.iface.bear, self.iface.id
        (te,), err = bear.get_torque_enable(i)[0]
        if te != 0 or err != 0x80:
            raise SystemExit(f"REFUSED after GO: torque_enable={te}, error byte "
                             f"{err if err is None else hex(err)}.")
        values, err = bear.bulk_read([i], self.iface.LOOP_READ_REGS)[0]
        if err is None or any(v is None for v in values):
            raise SystemExit("REFUSED after GO: could not read state.")
        pos, _, _, tw, tp = values
        act = math.degrees(pos)
        if max(tw, tp) >= TEMP_WARN:
            raise SystemExit(f"REFUSED after GO: temperature {max(tw, tp):.1f} C.")
        if a.bench:
            if abs(pos - self.start_pos) > math.radians(1.0):
                raise SystemExit(f"REFUSED after GO: the arm moved to actuator {act:.1f} deg "
                                 f"since the check (the bench plan starts from the old angle).")
        elif act + PHI_DEG > WORN_START_MAX_VEST_DEG:
            raise SystemExit(f"REFUSED after GO: arm at vest {act + PHI_DEG:.1f} deg; start "
                             f"with the arm down.")
        self.header.append(f"recheck after GO: act {act:.2f} deg, max temp {max(tw, tp):.1f} C")

    def bench_plan(self, start):
        """[(phase, target_act_deg, duration, fn(tau)->rad)] ; 'ctrl' = controller holds."""
        segs, here = [], start
        for deg in self.a.angles:
            tgt = math.radians(deg)
            T, fn = move_profile(here, tgt)
            segs.append(('ramp', deg, T, fn))
            segs.append(('settle', deg, BENCH_SETTLE_S, (lambda tau, x=tgt: x)))
            segs.append(('ctrl', deg, self.a.bench_hold_s, None))
            here = tgt
        return segs                      # then fade at the last angle (see the docstring)

    # -- the loop -----------------------------------------------------------------
    def run(self):
        a, iface = self.a, self.iface
        self.armed = True                     # from here on, exit must disable + disarm
        wd = iface.configure_watchdog(WATCHDOG_TIMEOUT_MS)
        self.limit_written = True
        rb = iface.set_current_limit(a.i_limit)
        iface.enable()
        self.header.append(f"watchdog {wd} us; limit_i_max {rb} A; enabled")
        state, err = iface.exchange(0.0)
        if state is None:
            raise Trip("first exchange failed")
        if iface.watchdog_tripped(err) or iface.has_error(err):
            raise Trip(f"error byte {hex(err)} on the first exchange")
        t_en = time.monotonic()
        self.t_enable = t_en
        pos_prev, vel, acc = state['position'], 0.0, 0.0
        u_out = 0.0                        # final output last cycle, N m
        u_rl = 0.0                         # rate-limited request, N m
        over3 = 0
        acc_n = 0
        t_prev = t_en
        loop_dt = P.PERIOD_S
        nxt = t_en
        cstate = P.IDLE
        fading = False
        fade_reason = None
        # bench driver state
        seg_i, seg_t0 = 0, t_en
        integ = 0.0
        bphase = '-'
        while True:
            now = time.monotonic()
            t = now - t_en
            dtc = min(max(loop_dt, P.PERIOD_S), P.TRIP_LOOP_DT_S)   # last full period
            derate = iface.thermal_scale(1.0, state)
            act_deg = math.degrees(state['position'])
            # ---- requested torque ----------------------------------------------------
            if not fading and not a.bench and t >= a.run_s:
                fading, fade_reason = True, f"run cap {a.run_s:g} s"
            if not fading and a.bench:
                while seg_i < len(self.plan) and now - seg_t0 >= self.plan[seg_i][2]:
                    if self.plan[seg_i][0] == 'ctrl':
                        self.ctrl.bench_end_hold()
                        integ = u_rl           # bumpless hand-back to the driver
                    seg_t0 += self.plan[seg_i][2]
                    seg_i += 1
                    if seg_i < len(self.plan) and self.plan[seg_i][0] == 'ctrl':
                        self.ctrl.bench_begin_hold(self.meas(t, dtc, act_deg, vel, acc, state,
                                                             u_out, derate))
                if seg_i >= len(self.plan):
                    fading, fade_reason = True, 'bench plan done'
            if fading:
                u_req = 0.0
                bphase = 'fade'
                cstate_eff = 'FADE'
            elif a.bench and self.plan[seg_i][0] != 'ctrl':
                bphase, tgt_deg, _, fn = self.plan[seg_i]
                sp = fn(now - seg_t0)
                e = sp - state['position']
                if abs(e) > math.radians(BENCH_POS_ERR_DEG):
                    raise Trip(f"bench: arm {math.degrees(-e):+.1f} deg from its target")
                integ_new = integ + BENCH_KI * e * dtc
                u_req = BENCH_KP * e + integ_new - BENCH_KD * vel
                if not ((u_req < 0.0 and e < 0) or (u_req > a.cap_a * KT and e > 0)):
                    integ = integ_new          # conditional integration
                u_req = max(0.0, u_req)
                cstate_eff = P.IDLE
            else:
                bphase = 'ctrl' if a.bench else '-'
                m = self.meas(t, dtc, act_deg, vel, acc, state, u_out, derate)
                try:
                    out = self.ctrl.step(m)
                except Exception as ex:          # noqa: BLE001
                    raise Trip(f"controller raised {ex!r}")
                u_req, cstate = out.tau, out.state
                if not (isinstance(u_req, (int, float)) and math.isfinite(u_req) and u_req >= 0.0):
                    raise Trip(f"controller output invalid: tau={u_req!r}")
                if cstate not in (P.IDLE, P.LIFT, P.SETTLE, P.HOLD, P.LOWER):
                    raise Trip(f"controller state invalid: {cstate!r}")
                for ev in out.events:
                    self.events.append((round(t, 4),) + tuple(ev))
                cstate_eff = cstate
                clog = out.log
            # ---- shaping (V2 §7 order) + a rise-only limit on the final output -----------
            # r: the spec's asymmetric rate limit, in request space (falls 20, rises 4 N m/s).
            # Taper and derate then act at once (they only ever cut). The final output may
            # not RISE faster than 4 N m/s either (review #14: leaving the taper band or a
            # derate ending must not step the torque up); falls pass through immediately.
            u_rl = max(u_rl - P.RATE_DOWN_NM_S * dtc, min(u_rl + P.RATE_UP_NM_S * dtc, u_req))
            iq_cmd = iface.joint_limit_scale(u_rl / KT, state)
            iq_cmd = iface.thermal_scale(iq_cmd, state)
            iq_cmd = max(0.0, min(a.cap_a, iq_cmd, (u_out + P.RATE_UP_NM_S * dtc) / KT))
            u_out = iq_cmd * KT
            # ---- bus ---------------------------------------------------------------------
            try:
                new, err = iface.exchange(iq_cmd)
            except READ_EXC as ex:
                raise Trip(f"exchange raised {ex!r}")
            t_now = time.monotonic()
            loop_dt = t_now - t_prev
            t_prev = t_now
            if new is None:
                raise Trip("exchange failed (all retries used)")
            state = new
            # ---- sensing: position-derived velocity and acceleration ------------------
            h = max(loop_dt, 1e-4)
            v_raw = (state['position'] - pos_prev) / h
            pos_prev = state['position']
            v_new = vel + (v_raw - vel) * h / (P.VEL_FILTER_S + h)
            a_raw = (v_new - vel) / h
            acc = acc + (a_raw - acc) * h / (P.ACC_FILTER_S + h)
            vel = v_new
            row = [t_now - t_en, loop_dt, math.degrees(state['position']),
                   math.degrees(state['position']) + PHI_DEG, state['velocity'], vel, acc,
                   state['iq'], state['iq_sent'], u_req, u_out, derate, state['temp'],
                   state['temp_powerstage'], err, P.STATE_CODE.get(cstate_eff, -1),
                   BENCH_PHASES.index(bphase)]
            if self.fields:
                row += list(clog) if (not fading and bphase in ('-', 'ctrl')) \
                    else [np.nan] * len(self.fields)
            self.log(row)
            # ---- trips -----------------------------------------------------------------
            if iface.watchdog_tripped(err):
                raise Trip(f"watchdog / e-stop bit (error {hex(err)})")
            if iface.has_error(err):
                raise Trip(f"error byte {hex(err)}")
            if loop_dt > P.TRIP_LOOP_DT_S:
                raise Trip(f"loop gap {loop_dt * 1000:.1f} ms > {P.TRIP_LOOP_DT_S * 1000:.0f} ms")
            iqm = abs(state['iq'])
            if iqm >= a.trip1:
                raise Trip(f"|iq| {iqm:.2f} A >= {a.trip1:g} A")
            over3 = over3 + 1 if iqm >= a.trip3 else 0
            if over3 >= 3:
                raise Trip(f"|iq| >= {a.trip3:g} A for 3 cycles ({iqm:.2f} A)")
            tmax = max(state['temp'], state['temp_powerstage'])
            if tmax >= TEMP_MAX:
                raise Trip(f"temperature {tmax:.1f} C >= {TEMP_MAX} C")
            if u_out > 0.0 and cstate_eff != P.LIFT and vel > P.TRIP_SPEED_UP_RAD_S:
                raise Trip(f"cuff lost? upward speed {vel:.2f} rad/s in {cstate_eff}")
            acc_n = acc_n + 1 if (u_out > 0.0 and cstate_eff in (P.LIFT, P.SETTLE, P.LOWER)
                                  and acc > P.TRIP_ACC_UP_RAD_S2) else 0
            if acc_n >= P.TRIP_ACC_COUNT:
                raise Trip(f"cuff lost? upward acceleration {acc:.0f} rad/s^2 in {cstate_eff}")
            if t_now - t_en > (BENCH_RUN_CAP_S if a.bench else a.run_s + 2.0):
                raise Trip("time cap overrun")
            if fading and u_out <= 0.0:
                self.end_reason = f"completed ({fade_reason})"
                return
            # ---- pace ------------------------------------------------------------------
            nxt += P.PERIOD_S
            s = nxt - time.monotonic()
            if s > 0:
                time.sleep(s)
            else:
                nxt = time.monotonic()

    def meas(self, t, dt, act_deg, vel, acc, state, u_out, derate):
        return dict(t=t, dt=dt, act_deg=act_deg, vest_deg=act_deg + PHI_DEG, vel=vel, acc=acc,
                    u_meas=KT * state['iq'], u_prev=u_out, derate=derate, cap_a=self.a.cap_a)

    def save(self):
        made, p = [], os.path.abspath(LOG_SUBDIR)
        while not os.path.isdir(p):
            made.append(p)
            p = os.path.dirname(p)
        os.makedirs(LOG_SUBDIR, exist_ok=True)
        path = os.path.join(LOG_SUBDIR, f"v2_{'bench' if self.a.bench else 'worn'}_"
                                        f"{self.a.level}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(path, 'w', newline='') as f:
            for line in self.header:
                f.write(f"# {line}\n")
            w = csv.writer(f)
            w.writerow(COLUMNS + list(self.fields))
            for r in self.buf[:self.n]:
                row = [f"{x:.9g}" for x in r]
                row[14] = hex(int(r[14])) if r[14] == r[14] else ''
                code = int(r[15]) if r[15] == r[15] else -1
                row[15] = P.STATES[code] if 0 <= code < len(P.STATES) else 'FADE'
                row[16] = BENCH_PHASES[int(r[16])]
                w.writerow(row)
            for ev in self.events:
                f.write(f"# event {json.dumps(ev, default=str)}\n")
            for line in self.footer:
                f.write(f"# {line}\n")
        uid, gid = os.environ.get('SUDO_UID'), os.environ.get('SUDO_GID')
        if os.geteuid() == 0 and uid and gid:
            for q in made + [LOG_SUBDIR, path]:
                try:
                    os.chown(q, int(uid), int(gid))
                except OSError as e:
                    print(f"WARNING: chown {q} failed: {e}")
        if self.learner_path:
            try:
                with open(self.learner_path, 'w') as f:
                    json.dump(self.ctrl.learner_state(), f)
                if os.geteuid() == 0 and uid and gid:
                    os.chown(self.learner_path, int(uid), int(gid))
                print(f"Learner state: {self.learner_path}")
            except Exception as e:                     # noqa: BLE001
                print(f"WARNING: learner state not saved ({e!r}); the next run starts fresh")
        print(f"\nLog: {path}")
        return path


def shutdown(run):
    """Confirmed disable; disarm and restore only if confirmed (hold_bench_test pattern)."""
    iface = run.iface
    if not run.armed:
        return                                     # a refusal before arming writes nothing
    ok = iface.disable()
    if ok:
        iface.bear.set_watchdog_timeout((iface.id, 0))
        (wd,), _ = iface.bear.get_watchdog_timeout(iface.id)[0]
        run.footer.append(f"disable confirmed attempts={iface.last_disable_attempts} "
                          f"ms={iface.last_disable_ms}; watchdog reset to {wd} us")
        if wd != 0:
            msg = (f"WATCHDOG RESET NOT CONFIRMED (reads {wd}); the actuator is disabled, but "
                   f"power-cycle it before any other script")
            print(f"\n{msg}")
            run.footer.append(msg)
            run.end_reason = f"EXIT NOT CLEAN (watchdog reads {wd}) after: {run.end_reason}" 
        if run.limit_written:
            try:
                rb = iface.set_current_limit(P.I_MAX_RESTORE_A)
                run.limit_restored = True
                run.footer.append(f"limit_i_max restored to {rb} A")
            except Exception as e:                 # noqa: BLE001
                msg = (f"limit_i_max NOT restored ({e}); power-cycle the actuator before any "
                       f"other script")
                print(f"\n{msg}")
                run.footer.append(msg)
    else:
        msg = ("DISABLE NOT CONFIRMED -- watchdog LEFT ARMED (trips in ~10 ms with no traffic). "
               "CUT ACTUATOR POWER and check it before anything else."
               + (f" limit_i_max left at {run.a.i_limit:g} A (RAM; a power cycle restores "
                  f"{P.I_MAX_RESTORE_A})." if run.limit_written else ""))
        print(f"\n{msg}")
        run.footer.append(msg)
        run.end_reason = f"DISABLE NOT CONFIRMED after: {run.end_reason}"


def main(argv=None):
    a = parse_args(argv)
    if os.sched_getscheduler(0) != os.SCHED_FIFO:
        raise SystemExit("REFUSED: not at real-time priority. From the repo root run:\n  sudo "
                         "chrt -f 80 taskset -c 3 ~/projects/bear_controller/venv/bin/python3 "
                         "-m main_controller.run_v2 ...")
    iface = BearInterface()
    run = Run(iface, a)
    started = False
    try:
        run.preflight()
        prompt = ("\nPayload on, padded catch below it, HANDS CLEAR. Type GO to start: "
                  if a.bench else
                  "\nWearer ready, arm DOWN, second person AT THE E-STOP. Type GO to start: ")
        if input(prompt).strip() != 'GO':
            run.end_reason = 'not started (no GO)'
            print("Not started.")
            return 1
        run.recheck()
        started = True
        run.run()
    except Trip as e:
        run.end_reason = f"FAULT: {e}"
        print(f"\nFAULT: {e}")
    except RuntimeError as e:                          # configure / limit / enable refused
        run.end_reason = f"REFUSED/FAILED: {e}"
        print(f"\n{run.end_reason}")
    except KeyboardInterrupt:
        run.end_reason = 'interrupted (Ctrl-C)'
        print("\nInterrupted.")
    except SystemExit as e:                            # a refusal: keep its message
        run.end_reason = f"REFUSED: {e}"
        raise
    except Exception as e:                             # noqa: BLE001  (serial errors, bugs)
        run.end_reason = f"ERROR: {e!r}" if (started or run.armed) else f"not started: {e!r}"
        print(f"\n{run.end_reason}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            shutdown(run)
        finally:
            if run.limit_written and not run.limit_restored \
                    and not any('limit_i_max' in l for l in run.footer):
                msg = (f"limit_i_max left at {a.i_limit:g} A (exit interrupted before a confirmed "
                       f"disable; a power cycle restores {P.I_MAX_RESTORE_A} A)")
                print(f"\n{msg}")
                run.footer.append(msg)
            run.footer.append(f"END {run.end_reason}")
            if started or run.n:
                run.save()
            iface.bear.close_port()
    print("\n" + "\n".join(run.footer))
    return 0 if run.end_reason.startswith('completed') else 1


if __name__ == '__main__':
    sys.exit(main())