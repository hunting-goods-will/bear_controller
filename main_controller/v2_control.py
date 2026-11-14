"""Controller v2 control logic (V2_ARCHITECTURE v1.4 §4-§7). PURE: no bus, no I/O, no clock.

INTERFACE CONTRACT (fixed by run_v2.py; written Nov 11 by Opus, implemented by Fable; V2 v1.4)
--------------------------------------------------------------------------------------------
run_v2.py owns: the bus, sensing (position-derived velocity/acceleration), every V2 §8
trip, the shaping chain (rate limit on the final output, taper, thermal derate, command
cap), FAULT (= the run ends), logging. This module owns everything else: the state
machine (IDLE, LIFT, SETTLE, HOLD, LOWER; never FAULT), the hold law with anchor, creep
and UNLOAD, the reading and its six checks, the learner (window, envelope, level
identification, load-change reset, used A with the C1 margin) and the assist law.

    Controller(cap_a: float, alpha: float, resume: dict | None = None)
        cap_a: the run's command cap (5.5 or 8.0 A); alpha in v2_params.ALPHA_STEPS.
        resume: what learner_state() returned at the end of an earlier run today (same
        wearer), or None. Only the learner carries over (window, level, A_id, latest
        reading); the state machine always starts in IDLE. Must tolerate a malformed or
        empty dict by starting fresh (never raise).
        alpha == 0 is the hold-only session: LIFT requests zero torque (no k_f help either);
        the learner still runs and reports events (shadow mode).

    LOG_FIELDS: tuple of str   # names of the floats in Out.log, fixed for the run; must
                               # include 'level' (0 = unidentified), 'A_id', 'A_used',
                               # 'anchor_deg', 'sp_deg', 'integ' (the tests read them)

    step(m: dict) -> Out       # called once per 2 ms cycle (worn, and bench 'ctrl' phases)
        m['t']        s since enable (monotonic)          m['dt']      s, this cycle (2-10 ms)
        m['act_deg']  actuator angle, deg                 m['vest_deg'] = act_deg + 72
        m['vel']      rad/s, position-derived, filtered 10 ms, + = up
        m['acc']      rad/s^2, same filter on the derivative
        m['u_meas']   KT * measured iq, N m
        m['u_prev']   the torque ACTUALLY output last cycle after shaping, N m (use it for
                      the bumpless SETTLE freeze and the HOLD start)
        (readings use m['u_meas']: the tables and the Nov 10 bias were measured from
         present_iq, so the reading must be too; V2 §6.1)
        m['derate']   thermal derate factor 0..1 (for the effective ceiling)
        m['cap_a']    = cap_a

    Out = namedtuple('Out', 'tau state log events')
        tau     requested torque, N m, finite and >= 0 (run_v2 trips on anything else)
        state   one of v2_params.IDLE/LIFT/SETTLE/HOLD/LOWER
        log     tuple of floats, len(LOG_FIELDS)
        events  list of tuples (kind: str, data: dict) for this cycle, e.g.
                ('transition', {'from':..,'to':..,'why':..}),
                ('reading', {'vest_deg':.., 'u':.., 'accepted':bool, 'failed':[check numbers]}),
                ('identified', {'level':.., 'A_id':..}), ('reset', {...}), ('unload', {...})

    learner_state() -> dict             # JSON-serialisable; run_v2 saves it after every run

    bench_begin_hold(m: dict) -> None   # bench only: enter HOLD now, anchor = m['vest_deg'],
                                        # bumpless from m['u_prev'] (no up-stroke on the bench)
    bench_end_hold() -> None            # bench only: leave HOLD -> IDLE, discard a reading
                                        # in progress; the learner keeps its window

Rules the run loop relies on: step() never raises on finite inputs; never returns a
negative or non-finite tau; never returns a state outside the five above; does not
implement any V2 §8 trip; reads every number from v2_params (no literals).

IMPLEMENTATION NOTES (Fable, Nov 11) -- where the spec left a choice:
  * One reading per hold (cycles 4-6 of the creep). The creep keeps running for the rest
    of the hold (the regime the bench proved and the UNLOAD thresholds were set on).
  * Creep = anchor + env * amp * sin(2 pi f tau); feedforward = env * gain * f_ref *
    sign(cos), slewed at 4 N m/s both ways: the bench instrument's waveform
    (hold_bench_test.py), upward-only here.
  * Travel (check 2) per averaged cycle: up = max(theta) - anchor, down = anchor - min(theta).
  * Headroom (check 5) is judged on every sample of cycles 4-6, on the larger of the
    request and the torque actually output.
  * SETTLE -> LIFT when the latch fires again (not in the V2 §4 table; without it a wearer
    who resumes a lift within 1 s would be in SETTLE with a frozen positive command, where
    the 1 rad/s cuff-lost trip applies). [proposal]
  * w(theta_dot) is the Phase 1 blend (controller.blend) over LATCH_OFF..BLEND_HI.
"""
import math
from collections import deque, namedtuple

from main_controller import v2_params as P
from main_controller.controller import blend

Out = namedtuple('Out', 'tau state log events')

NAN = float('nan')
D2R = math.pi / 180.0


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


# =============================================================================================
# Learner (V2 §6.2-6.5). Same logic as sim/nov8/learner_final.py + the C1 margin.
# =============================================================================================
class Learner:
    def __init__(self, resume=None):
        self.window = []          # [(vest_deg, u)] accepted readings, newest last, <= WIN
        self.level = 0            # 0 = unidentified
        self.A_id = None
        self.last = None          # (vest_deg, u) of the latest accepted reading
        self.resumed = False
        if resume is not None:
            self.resumed = self._load(resume)

    # -- persistence ---------------------------------------------------------------------
    def state(self):
        return {'version': 1,
                'window': [[float(t), float(u)] for t, u in self.window],
                'level': int(self.level),
                'A_id': None if self.A_id is None else float(self.A_id),
                'last': None if self.last is None else [float(self.last[0]), float(self.last[1])]}

    @staticmethod
    def _pair(x):
        if not (isinstance(x, (list, tuple)) and len(x) == 2 and _num(x[0]) and _num(x[1])):
            return None
        lo, hi = P.WINDOW_VEST_DEG
        if not lo <= x[0] <= hi:
            return None
        return (float(x[0]), float(x[1]))

    def _load(self, d):
        """True if d was a valid learner state; anything else leaves the learner fresh."""
        try:
            if not isinstance(d, dict) or d.get('version') != 1:
                return False
            win = d.get('window')
            if not isinstance(win, list) or len(win) > P.WIN:
                return False
            pairs = [self._pair(x) for x in win]
            if any(p is None for p in pairs):
                return False
            level, A_id, last = d.get('level'), d.get('A_id'), d.get('last')
            if isinstance(level, bool) or level not in (0,) + tuple(P.LEVELS):
                return False
            if level == 0:
                if A_id is not None:
                    return False
                last_p = None if last is None else self._pair(last)
                if last is not None and last_p is None:
                    return False
            else:
                last_p = self._pair(last)
                if not (_num(A_id) and 0.0 < A_id <= 2.0 * P.A_CAP_NM) or last_p is None:
                    return False
            self.window, self.level = pairs, int(level)
            self.A_id = None if level == 0 else float(A_id)
            self.last = last_p
            return True
        except Exception:                                    # noqa: BLE001
            self.window, self.level, self.A_id, self.last = [], 0, None, None
            return False

    # -- the update: one accepted reading ----------------------------------------------------
    def add(self, vest_deg, u):
        """Returns a list of events."""
        ev = []
        self.window.append((float(vest_deg), float(u)))
        self.window = self.window[-P.WIN:]
        self.last = (float(vest_deg), float(u))
        # load-change reset: the two newest agree and both sit > DROP below A_id
        if self.level and len(self.window) >= 2:
            a = [P.arm_value(uu, self.level, th) for th, uu in self.window[-2:]]
            if abs(a[0] - a[1]) <= P.TOL_NM and all(x < self.A_id - P.DROP_NM for x in a):
                ev.append(('reset', {'level_was': self.level, 'A_id_was': round(self.A_id, 3),
                                     'A_new': [round(x, 3) for x in a]}))
                self.window = self.window[-2:]
                self.level, self.A_id = 0, None
        # relaxed envelope: per BIN_DEG band keep the highest reading
        bins = {}
        lo = P.WINDOW_VEST_DEG[0]
        for th, uu in self.window:
            b = int((th - lo) // P.BIN_DEG)
            if b not in bins or uu > bins[b][1]:
                bins[b] = (th, uu)
        env = [bins[b] for b in sorted(bins)]
        # weakest level first: a top pair
        for c in P.LEVELS:
            Ac = [P.arm_value(uu, c, th) for th, uu in env]
            top = max(Ac)
            best = None
            for i in range(len(env)):
                for j in range(i + 1, len(env)):
                    if abs(env[i][0] - env[j][0]) >= P.SEP_DEG and abs(Ac[i] - Ac[j]) <= P.TOL_NM:
                        low = min(Ac[i], Ac[j])
                        if low >= top - P.TOL_NM and (best is None or low > best):
                            best = low
            if best is not None:
                if c != self.level or self.A_id is None or abs(best - self.A_id) > 1e-9:
                    ev.append(('identified', {'level': c, 'A_id': round(best, 3),
                                              'level_was': self.level}))
                self.level, self.A_id = c, best
                break
        return ev

    # -- what the assist and the hold ceiling use --------------------------------------------
    def A_used(self):
        """min(A_id - OFFSET, A_last from (u - margin)), capped; None while unidentified."""
        if not self.level or self.A_id is None or self.last is None:
            return None
        a_last = P.arm_value(self.last[1] - P.FAST_MARGIN_READING_NM, self.level, self.last[0])
        return min(self.A_id - P.OFFSET_NM, a_last, P.A_CAP_NM)

    def R_hat(self, vest_deg):
        a = self.A_used()
        if a is None:
            return P.residual_model(P.A_LOW_NM, P.LEVELS[-1], vest_deg)
        return P.residual_model(a, self.level, vest_deg)


# =============================================================================================
# One hold's creep reading (V2 §6.1)
# =============================================================================================
class _Cycle:
    __slots__ = ('su', 'nu', 'sd', 'nd', 'hi', 'lo', 'headroom_bad')

    def __init__(self):
        self.su = self.sd = 0.0
        self.nu = self.nd = 0
        self.hi, self.lo = -1e9, 1e9
        self.headroom_bad = False


class Controller:
    LOG_FIELDS = ('level', 'A_id', 'A_used', 'anchor_deg', 'sp_deg', 'integ', 'e_deg',
                  'ceiling', 'creep_cycle', 'R_hat')

    def __init__(self, cap_a, alpha, resume=None):
        self.cap_a = float(cap_a)
        self.alpha = float(alpha)
        self.learner = Learner(resume)
        self._pending = []
        if resume is not None:
            self._pending.append(('resume', {'ok': self.learner.resumed,
                                             'level': self.learner.level,
                                             'n_window': len(self.learner.window)}))
        self.state = P.IDLE
        self.t_state = 0.0
        # latch
        self.latch_on_t = 0.0
        self.block_until = -1.0
        # timers for the speed rules (s of continuous violation)
        self.t_down = 0.0
        self.t_up = 0.0
        # SETTLE
        self.frozen = 0.0
        self.still = deque()
        # HOLD
        self._clear_hold()

    def learner_state(self):
        return self.learner.state()

    # -- helpers -----------------------------------------------------------------------------
    def _clear_hold(self):
        self.anchor = None            # vest deg
        self.t_hold = 0.0
        self.amp = 0.0                # creep amplitude, deg
        self.f_ref = 0.0
        self.integ = 0.0
        self.ff = 0.0
        self.cycles = {}
        self.read_done = False
        self.sp = NAN
        self.e_deg = NAN
        self.ceiling = NAN
        self.cyc = NAN

    def _go(self, new, why, ev, m):
        ev.append(('transition', {'from': self.state, 'to': new, 'why': why,
                                  'vest_deg': round(m['vest_deg'], 2)}))
        if self.state == P.HOLD:
            self._clear_hold()
        self.state = new
        self.t_state = m['t']
        self.t_down = self.t_up = 0.0

    def _begin_hold(self, m, start_u):
        level = self.learner.level
        self.anchor = m['vest_deg']
        self.t_hold = m['t']
        self.f_ref = float(P.friction(level, self.anchor))
        lo, hi = P.CREEP_DEG_RANGE
        self.amp = min(hi, max(lo, P.CREEP_DEG_PER_NM * self.f_ref))
        self.integ = max(0.0, float(start_u))
        self.ff = 0.0
        self.cycles = {}
        self.read_done = False

    def _ceiling(self, m):
        """Active hold ceiling, N m (V2 §5 / §6.1 check 5)."""
        eff = P.effective_ceiling(self.anchor + self.amp, self.cap_a, m['derate'])
        if self.learner.level:
            idc = (self.learner.R_hat(self.anchor)
                   + float(P.friction(self.learner.level, self.anchor))
                   + P.HOLD_CEILING_MARGIN_NM)
            return max(0.0, min(eff, idc))
        return max(0.0, eff)

    # -- bench hooks -------------------------------------------------------------------------
    def bench_begin_hold(self, m):
        if self.state == P.HOLD:
            self._clear_hold()
        self._pending.append(('transition', {'from': self.state, 'to': P.HOLD,
                                             'why': 'bench_begin_hold',
                                             'vest_deg': round(m['vest_deg'], 2)}))
        self.state = P.HOLD
        self.t_state = m['t']
        self.t_down = self.t_up = 0.0
        self._begin_hold(m, m['u_prev'])

    def bench_end_hold(self):
        if self.state == P.HOLD:
            self._clear_hold()
        self._pending.append(('transition', {'from': self.state, 'to': P.IDLE,
                                             'why': 'bench_end_hold'}))
        self.state = P.IDLE
        self.t_down = self.t_up = 0.0

    # -- one cycle -----------------------------------------------------------------------------
    def step(self, m):
        ev, self._pending = self._pending, []
        t, dt, vest, vel = m['t'], m['dt'], m['vest_deg'], m['vel']
        # timers shared by the speed rules
        self.t_down = self.t_down + dt if vel < -P.LOWER_SPEED_RAD_S else 0.0
        self.t_up = self.t_up + dt if vel > P.UNLOAD_SPEED_RAD_S else 0.0
        lowering = self.t_down >= P.LOWER_SPEED_S

        # ---- transitions. Priority: LOWER -> UNLOAD (HOLD) -> latch (trips are run_v2's) ----
        s = self.state
        if s == P.HOLD:
            if vest < self.anchor - P.LOWER_BELOW_ANCHOR_DEG:
                self._go(P.LOWER, 'sag > 2 deg below the anchor', ev, m)
            elif lowering:
                self._go(P.LOWER, 'downward speed', ev, m)
            elif (vest > self.anchor + self.amp + P.UNLOAD_ABOVE_CREEP_DEG
                  or self.t_up >= P.UNLOAD_SPEED_S):
                why = ('position' if vest > self.anchor + self.amp + P.UNLOAD_ABOVE_CREEP_DEG
                       else 'upward speed')
                ev.append(('unload', {'why': why, 'vest_deg': round(vest, 2),
                                      'anchor_deg': round(self.anchor, 2),
                                      'u_prev': round(m['u_prev'], 3)}))
                self._go(P.IDLE, 'UNLOAD: ' + why, ev, m)
                self.block_until = t + P.UNLOAD_LIFT_BLOCK_S
        elif s == P.LIFT:
            if lowering:
                self._go(P.LOWER, 'downward speed', ev, m)
            elif vel < P.LATCH_OFF_RAD_S and t - self.latch_on_t >= P.LATCH_MIN_ON_S:
                self.frozen = max(0.0, m['u_prev'])
                self.still.clear()
                self._go(P.SETTLE, 'latch off', ev, m)
        elif s == P.SETTLE:
            self.still.append((t, vest))
            while self.still and t - self.still[0][0] > P.STILL_WINDOW_S:
                self.still.popleft()
            full = t - self.t_state >= P.STILL_WINDOW_S
            span = (max(x[1] for x in self.still) - min(x[1] for x in self.still))
            if lowering:
                self._go(P.LOWER, 'downward speed', ev, m)
            elif full and span < P.STILL_SPAN_DEG and vest > P.HOLD_MIN_VEST_DEG:
                self._go(P.HOLD, 'still after an up-stroke', ev, m)
                self._begin_hold(m, self.frozen)
            elif vel > P.LATCH_ON_RAD_S and t >= self.block_until:
                self.latch_on_t = t
                self._go(P.LIFT, 'latch on again', ev, m)
            elif t - self.t_state >= P.SETTLE_TIMEOUT_S:
                self._go(P.IDLE, 'settle timeout', ev, m)
        elif s == P.LOWER:
            if m['u_prev'] <= 0.0:
                self._go(P.IDLE, 'faded to zero', ev, m)
        else:                                                   # IDLE
            if vel > P.LATCH_ON_RAD_S and t >= self.block_until:
                self.latch_on_t = t
                self._go(P.LIFT, 'latch on', ev, m)

        # ---- command ---------------------------------------------------------------------
        s = self.state
        if s == P.HOLD:
            tau = self._hold(m, ev)
        elif s == P.LIFT:
            tau = self._lift(m)
        elif s == P.SETTLE:
            tau = self.frozen
        else:                                                   # IDLE, LOWER: fade to zero
            tau = 0.0
        if not math.isfinite(tau) or tau < 0.0:
            tau = 0.0
        L = self.learner
        a_used = L.A_used()
        log = (float(L.level), NAN if L.A_id is None else L.A_id,
               NAN if a_used is None else a_used,
               NAN if self.anchor is None else self.anchor, self.sp, self.integ, self.e_deg,
               self.ceiling, self.cyc, L.R_hat(vest))
        return Out(tau, s, log, ev)

    # -- LIFT (V2 §7) ----------------------------------------------------------------------
    def _lift(self, m):
        if self.alpha <= 0.0:
            return 0.0
        vest, L = m['vest_deg'], self.learner
        w = blend(m['vel'], P.LATCH_OFF_RAD_S, P.BLEND_HI_RAD_S)
        lo, hi = P.G_LOW_VEST_DEG
        g_low = min(1.0, max(0.0, (vest - lo) / (hi - lo)))
        f = float(P.friction(L.level, vest)) if L.level else P.friction_min(vest)
        return w * (self.alpha * max(0.0, L.R_hat(vest)) * g_low + P.KF * f)

    # -- HOLD (V2 §5, §6.1) ------------------------------------------------------------------
    def _hold(self, m, ev):
        dt, vest = m['dt'], m['vest_deg']
        tau_c = m['t'] - self.t_hold                    # s since HOLD entry
        ph = 2.0 * math.pi * P.CREEP_HZ * tau_c
        x = min(1.0, max(0.0, tau_c * P.CREEP_HZ / P.CREEP_EASE_CYCLES))   # ease in
        env = x * x * (3.0 - 2.0 * x)
        up_leg = math.cos(ph) > 0.0
        sp = self.anchor + env * self.amp * math.sin(ph)
        # feedforward: a square wave slewed at the rise limit in BOTH directions (the bench
        # instrument's symmetric 4 N m/s limit made it this trapezoid; with v2's faster fall
        # the two reversals would be unequal and the creep lopsided)
        ff_t = env * P.CREEP_FF_GAIN * self.f_ref * (1.0 if up_leg else -1.0)
        step = P.RATE_UP_NM_S * dt
        self.ff = min(self.ff + step, max(self.ff - step, ff_t))
        ff = self.ff
        e = (sp - vest) * D2R                           # rad, + = arm below the setpoint
        ceiling = self._ceiling(m)

        # integral: its input saturates INTEG_SAT_BELOW_DEG below the setpoint (bounds what a
        # slow lowering can wind up before the 2 deg release; v1.3's freeze beyond 0.3 deg
        # could not pick up a relaxing wearer). Anti-windup = clamp to [0, ceiling] (V2 §5).
        # No extra rule for the rate limit: pausing integration while the 4 N m/s rise limit
        # held the output back left the creep 0.3 deg low and asymmetric on the fake.
        self.integ += P.KI * min(e, P.INTEG_SAT_BELOW_DEG * D2R) * dt
        self.integ = min(max(self.integ, 0.0), ceiling)
        raw = P.KP * e + self.integ - P.KD * m['vel'] + ff
        tau = min(max(raw, 0.0), ceiling)

        self.sp, self.e_deg, self.ceiling = sp, e / D2R, ceiling
        k = int(tau_c * P.CREEP_HZ)                     # 0-based creep cycle
        self.cyc = float(k + 1)
        first, n = P.CREEP_CYCLES_DISCARD, P.CREEP_CYCLES_READ
        if not self.read_done:
            if first <= k < first + n:
                c = self.cycles.get(k)
                if c is None:
                    c = self.cycles[k] = _Cycle()
                if up_leg:
                    c.su += m['u_meas']
                    c.nu += 1
                else:
                    c.sd += m['u_meas']
                    c.nd += 1
                c.hi, c.lo = max(c.hi, vest), min(c.lo, vest)
                if max(tau, m['u_prev']) > ceiling - P.HEADROOM_NM:
                    c.headroom_bad = True
            elif k >= first + n:
                self.read_done = True
                self._finish_reading(ev)
                # the learner may have just lowered R_hat: apply the new ceiling this cycle
                ceiling = self._ceiling(m)
                self.integ = min(self.integ, ceiling)
                tau = min(tau, ceiling)
                self.ceiling = ceiling
        return tau

    def _finish_reading(self, ev):
        first, n = P.CREEP_CYCLES_DISCARD, P.CREEP_CYCLES_READ
        cyc = [self.cycles.get(k) for k in range(first, first + n)]
        data = {'vest_deg': round(self.anchor, 3), 'accepted': False, 'failed': [], 'u': None}
        if any(c is None or c.nu == 0 or c.nd == 0 for c in cyc):
            data['failed'] = [6]                         # cycles missing: not a finished creep
            ev.append(('reading', data))
            return
        ups = [c.su / c.nu for c in cyc]
        dns = [c.sd / c.nd for c in cyc]
        per = [(a + b) / 2.0 for a, b in zip(ups, dns)]
        u = sum(per) / n
        split = sum((a - b) / 2.0 for a, b in zip(ups, dns)) / n
        t_up = [c.hi - self.anchor for c in cyc]
        t_dn = [self.anchor - c.lo for c in cyc]
        failed = []
        lo, hi = P.WINDOW_VEST_DEG
        if not lo <= self.anchor <= hi:
            failed.append(1)
        if any(a < P.TRAVEL_MIN_DEG or b < P.TRAVEL_MIN_DEG
               or abs(a - b) > P.TRAVEL_ASYM_MAX_DEG for a, b in zip(t_up, t_dn)):
            failed.append(2)
        if not P.SPLIT_BAND[0] * self.f_ref <= split <= P.SPLIT_BAND[1] * self.f_ref:
            failed.append(3)
        if max(per) - min(per) > P.CYCLE_AGREE_NM:
            failed.append(4)
        if any(c.headroom_bad for c in cyc):
            failed.append(5)
        data.update(u=round(u, 4), failed=failed, accepted=not failed,
                    split=round(split, 3), f_ref=round(self.f_ref, 3),
                    per_cycle=[round(x, 3) for x in per],
                    travel_up=[round(x, 2) for x in t_up],
                    travel_down=[round(x, 2) for x in t_dn])
        ev.append(('reading', data))
        if not failed:
            ev.extend(self.learner.add(self.anchor, u))