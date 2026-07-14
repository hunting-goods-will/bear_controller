# Actuator settings — Phase 4, corrected against PROJECT_STATE.md
#
# WHAT'S CORRECTED HERE vs. the original bear_project_setup_guide.md:
#   - PORT uses the stable udev symlink /dev/UB0114, not /dev/ttyUSB0
#   - SAFETY_CHECKS_CONFIRMED is new: a deliberate code-level speed bump
#     (see below)

ACTUATOR_ID = 1
PORT        = '/dev/UB0114'   # stable name from the udev rule, confirmed in Phase 3
BAUDRATE    = 8000000

# Safety limits — start conservative, raise only after bench testing with
# the fixture in place. Do not raise MAX_IQ to match the actuator's peak
# rating "on day one" — Phase 6 Rule 4.
MAX_IQ      = 1.0    # Amps — absolute maximum current command
TEMP_WARN   = 65.0   # degrees C — start scaling torque back
TEMP_MAX    = 75.0   # degrees C — zero torque above this

# Firmware watchdog timeout register (see Westwood SDK Manual v1.0.1).
# Currently 0 (off) on the actuator by default. TBD with your PI:
#   - what timeout value makes sense for your control loop rate
#   - whether to arm it before or after ESTOP is confirmed
# Left as None deliberately — do not fill in a guessed number.
WATCHDOG_TIMEOUT_MS = None

# --- SAFETY GATE ---
# This flag exists so that "can this code technically run" and "should this
# code run" are two separate checks, not one. main.py refuses to call
# ctrl.start() while this is False. Flip it to True only after ALL of the
# following are true, not before:
#   1. ESTOP wiring has been traced against the connector's pin labels and
#      confirmed with your PI (not assumed, not "probably fine")
#   2. The 3D-printed bench fixture is built and the actuator is physically
#      constrained in it (Phase 6, Rule 1 — non-negotiable)
#   3. controller.py's ASSIST_IQ / VELOCITY_THRESHOLD are real values from
#      your PI, not the placeholders currently in that file
SAFETY_CHECKS_CONFIRMED = False
