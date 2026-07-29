from main_controller.bear_interface import BearInterface

# PLACEHOLDER VALUES — do not tune these yourself, they're not a design,
# they're a stand-in so the file has valid syntax. Still open with your PI:
#   1. exact trigger signal for assistive torque (velocity threshold? IQ
#      spike? IMU?)
#   2. target assistive torque magnitude in Nm, not just "easier"
#   3. whether gravity compensation is required or optional
#   4. joint limits for arm pitch on the EksoVest
# Until those are answered, treat this file as structure, not a runnable
# control design.
ASSIST_IQ = 0.5           # Amps — placeholder, unconfirmed
VELOCITY_THRESHOLD = 0.05  # rad/s — placeholder, unconfirmed


class AssistController:
    def __init__(self):
        self.hw = BearInterface()

    def start(self):
        # This calls enable() and begins commanding torque. Gated in
        # main.py behind SAFETY_CHECKS_CONFIRMED — that gate exists so this
        # method being technically callable doesn't mean it should be
        # called yet.
        self.hw.enable()
        print("Controller running. Ctrl+C to stop safely.")
        try:
            while True:
                self.step()
        except KeyboardInterrupt:
            print("\nCtrl+C received.")
        finally:
            self.hw.disable()  # runs even if code crashes

    def step(self):
        state = self.hw.get_state()

        # Intent detection: upward velocity = user wants to raise arm
        # TODO: refine this trigger signal with your PI — see placeholders above
        if state['velocity'] > VELOCITY_THRESHOLD:
            iq_cmd = ASSIST_IQ
        else:
            iq_cmd = 0.0

        iq_cmd = self.hw.thermal_scale(iq_cmd)
        self.hw.set_iq(iq_cmd)
