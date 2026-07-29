from main_controller.controller import AssistController
from main_controller.config import SAFETY_CHECKS_CONFIRMED

if __name__ == "__main__":
    if not SAFETY_CHECKS_CONFIRMED:
        raise SystemExit(
            "\nBLOCKED: config.SAFETY_CHECKS_CONFIRMED is False.\n"
            "Before flipping it to True, confirm all of the following are\n"
            "actually true — not assumed:\n"
            "  1. ESTOP wiring traced against the connector's pin labels\n"
            "     and confirmed with your PI\n"
            "  2. 3D-printed bench fixture built and the actuator is\n"
            "     physically constrained in it (Phase 6, Rule 1)\n"
            "  3. controller.py's ASSIST_IQ / VELOCITY_THRESHOLD are real\n"
            "     values from your PI, not placeholders\n"
        )

    ctrl = AssistController()
    ctrl.start()
