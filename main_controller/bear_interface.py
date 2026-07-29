from pybear import Manager
from main_controller.config import *


class BearInterface:
    def __init__(self):
        self.bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
        self.id = ACTUATOR_ID
        self.bear.single_timeout = COMM_TIMEOUT
        self.bear.single_try_num = COMM_TRY_NUM

    def enable(self):
        # Set torque/IQ mode and enable the actuator.
        self.bear.set_mode((self.id, 0))  # 0 = torque/IQ mode
        self.bear.set_torque_enable((self.id, 1))
        print("Actuator enabled.")

    def disable(self):
        """Zero torque and disable. Always call this on shutdown."""
        self.bear.set_goal_iq((self.id, 0.0))
        self.bear.set_torque_enable((self.id, 0))
        print("Actuator disabled.")

    def set_iq(self, iq):
        """Command a torque (current in Amps). Hard-clamped to MAX_IQ."""
        iq = max(-MAX_IQ, min(MAX_IQ, iq))
        self.bear.set_goal_iq((self.id, iq))

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
        """Arms the firmware watchdog timeout register. """

        raise NotImplementedError(

            "Watchdog register/method not yet confirmed against the SDK "
            "manual — fill in before use, don't guess."
        )
