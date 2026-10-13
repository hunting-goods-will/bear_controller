from pybear import Manager
from main_controller.config import *
import time


class BearInterface:
    def __init__(self):
        self.bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
        self.id = ACTUATOR_ID
        self.bear.single_timeout = COMM_TIMEOUT
        self.bear.single_try_num = COMM_TRY_NUM

    def enable(self):
        """Switch on torque mode (Mode 0) safely.

        Refuses unless the watchdog is armed ON THE ACTUATOR, and confirms
        every write by reading it back (PyBEAR writes are fire-and-forget).
        Call configure_watchdog() first.
        """
        # CHECK 1: watchdog armed on the actuator itself.
        (wd_us,), err = self.bear.get_watchdog_timeout(self.id)[0]
        if wd_us is None:
            raise RuntimeError("enable refused: could not read the watchdog")
        if not (WATCHDOG_MIN_MS * 1000 <= wd_us <= WATCHDOG_MAX_MS * 1000):
            raise RuntimeError(f"enable refused: watchdog is {wd_us} µs "
                               f"(call configure_watchdog() first)")

        # CHECK 2: no fault already latched (0x80 = no error).
        if err != 0x80:
            raise RuntimeError(f"enable refused: error byte {hex(err)} "
                               f"(call disable() to clear it)")

        # Zero the command first, and confirm it.
        self.bear.set_goal_iq((self.id, 0.0))
        (goal,), _ = self.bear.get_goal_iq(self.id)[0]
        if goal is None or goal != 0.0:
            raise RuntimeError(f"enable refused: goal_iq reads {goal}, not 0")

        # Torque mode, and confirm it (a lost write could leave Position Mode).
        self.bear.set_mode((self.id, 0))
        (mode,), _ = self.bear.get_mode(self.id)[0]
        if mode != 0:
            raise RuntimeError(f"enable refused: mode reads {mode}, not 0")

        # Enable, and confirm it. On the bench (Oct 13) one read-back taken
        # right after the write still showed the old value, so poll for up to
        # 20 ms and record how long the change took to show.
        self.bear.set_torque_enable((self.id, 1))
        t0 = time.monotonic()
        reads = 0
        while True:
            (te,), _ = self.bear.get_torque_enable(self.id)[0]
            reads += 1
            if te == 1:
                break
            if time.monotonic() - t0 > 0.020:
                raise RuntimeError(f"enable failed: torque_enable reads {te}, "
                                   f"not 1, after {reads} reads / 20 ms")
        ms = (time.monotonic() - t0) * 1000

        print(f"Actuator enabled in torque mode (watchdog {wd_us} µs; "
              f"confirmed on read {reads}, {ms:.1f} ms).")
        return wd_us
    
    def enable_position_mode(self, initial_position):
        """Set mode to Position (2) and enable, with goal_position pre-set
        to initial_position first so nothing lurches — a mode change while
        enabled acts on the new mode's goal register immediately."""
        self.bear.set_goal_position((self.id, initial_position))
        self.bear.set_mode((self.id, 2))
        self.bear.set_torque_enable((self.id, 1))
        print("Actuator enabled in Position Mode.")

    def disable(self):
        """Zero torque and disable. Always call this on shutdown.

        Never raises (it runs inside finally: blocks). On the bench (Oct 13)
        the torque_enable = 0 write was sometimes lost when it followed the
        goal_iq write back-to-back, so: zero goal_iq and read it back (which
        also spaces the writes), then write torque_enable = 0 and confirm it,
        retrying up to 3 times. Returns True only if confirmed.
        """
        try:
            self.bear.set_goal_iq((self.id, 0.0))
            self.bear.get_goal_iq(self.id)
            te = None
            for attempt in range(1, 4):
                self.bear.set_torque_enable((self.id, 0))
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.020:
                    (te,), _ = self.bear.get_torque_enable(self.id)[0]
                    if te == 0:
                        print(f"Actuator disabled (confirmed, attempt {attempt}).")
                        return True
            print(f"WARNING: disable NOT confirmed (torque_enable reads {te}). "
                  f"Cut actuator power.")
            return False
        except Exception as e:
            print(f"WARNING: disable() raised {e!r}. Cut actuator power.")
            return False

    def set_iq(self, iq):
        """Command a torque (current in Amps). Hard-clamped to MAX_IQ.
        Returns the actually-applied (clamped) value — always track this,
        not your own running total, or you'll silently diverge from
        what the hardware is really doing once you hit the clamp."""
        iq = max(-MAX_IQ, min(MAX_IQ, iq))
        self.bear.set_goal_iq((self.id, iq))
        return iq

    def get_state(self):
        """Read position, velocity, IQ, and temperature in one call.

        Read-only. Does not require torque_enable and carries none of the
        risk enable()/set_iq() do — safe to call regardless of ESTOP status.
        """
        pos, _ = self.bear.get_present_position(self.id)[0]
        vel, _ = self.bear.get_present_velocity(self.id)[0]
        iq, _ = self.bear.get_present_iq(self.id)[0]
        temp, _ = self.bear.get_winding_temperature(self.id)[0]
        return {
            'position': pos[0],
            'velocity': vel[0],
            'iq': iq[0],
            'temp': temp[0]
        }
        # The 4 registers read every loop: exactly the set Block 1 timed.
    LOOP_READ_REGS = ['present_position', 'present_velocity',
                      'present_iq', 'winding_temperature']
    WATCHDOG_BIT = 0x08   # bit 3, "Watchdog Timeout & ESTOP" (SDK Table 3)
    ERROR_BITS = 0x7C     # bits 2-6 are errors; bits 0-1 are only warnings

    def exchange(self, iq_command):
        """One loop's bus traffic in ONE packet: send goal_iq, read state back.

        Returns (state, err). On a failed exchange (all retries used, ~6 ms)
        returns (None, None) -- the loop must then stop.
        Every successful call is a write, so it also keeps the watchdog fed.
        """
        iq = max(-MAX_IQ, min(MAX_IQ, iq_command))
        values, err = self.bear.bulk_read_write(
            [self.id], self.LOOP_READ_REGS, ['goal_iq'], [[iq]])[0]
        if err is None or any(v is None for v in values):
            return None, None
        pos, vel, iq_meas, temp = values
        return {'position': pos, 'velocity': vel, 'iq': iq_meas,
                'temp': temp, 'iq_sent': iq}, err

    def watchdog_tripped(self, err):
        """True if the error byte has the watchdog bit set."""
        return err is not None and bool(err & self.WATCHDOG_BIT)

    def has_error(self, err):
        """True for any real error (bits 2-6). Warnings don't count."""
        return err is not None and bool(err & self.ERROR_BITS)

    def joint_limit_scale(self, iq_command):
        """Zero torque at/beyond MIN_ANGLE/MAX_ANGLE. Stays enabled and
        backdrivable rather than disabling — resumes automatically once
        position is back in range. Separate from the stricter abort-and-
        disable behavior the characterization sweep script needs
        (NEXT_STEPS_v2.md §5).
        """
        position = self.get_state()['position']
        if position <= MIN_ANGLE or position >= MAX_ANGLE:
            print(f"WARNING: position {position:.3f} rad at/beyond joint "
                  f"limit. Zeroing torque.")
            return 0.0
        return iq_command

    def thermal_scale(self, iq_command):
        """ Reduce torque as temperature rises. Never hard-cut."""
        temp = self.get_state()['temp']
        if temp < TEMP_WARN:
            return iq_command
        elif temp > TEMP_MAX:
            print(f"WARNING: Over temp ({temp:.1f}C). Zeroing torque.")
            return 0.0
        scale = 1.0 - (temp - TEMP_WARN) / (TEMP_MAX - TEMP_WARN)
        return iq_command * scale

    def configure_watchdog(self, timeout_ms):
        """Arm the firmware watchdog for this run only. Never saved to flash."""
        if timeout_ms < WATCHDOG_MIN_MS or timeout_ms > WATCHDOG_MAX_MS:
            raise ValueError(f"watchdog timeout {timeout_ms} ms is outside "
                             f"{WATCHDOG_MIN_MS}-{WATCHDOG_MAX_MS} ms")
        timeout_us = int(timeout_ms * 1000)

        self.bear.set_watchdog_timeout((self.id, timeout_us))
        (readback,), err = self.bear.get_watchdog_timeout(self.id)[0]

        if readback is None:
            raise RuntimeError("watchdog read-back failed")
        if readback != timeout_us:
            raise RuntimeError(f"watchdog read back {readback} µs, wrote {timeout_us} µs")
        if err != 0x80:
            raise RuntimeError(f"error byte {err} after arming (0x80 = no error)")

        # No save_config here: per run only.
        return readback
