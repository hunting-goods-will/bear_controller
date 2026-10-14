# Actuator settings
import math 
import os 

ACTUATOR_ID = 1
PORT        = '/dev/ttyUSB0'   # udev rule
BAUDRATE    = 8000000
COMM_TIMEOUT  = 0.002
COMM_TRY_NUM  = 3
MIN_ANGLE = math.radians(5)     # 0.0873 rad, real bracket hard stop at 0,
                                # 5 deg margin
MAX_ANGLE = math.radians(118)   # 2.0595 rad. Deliberately ABOVE the ~116.5 deg
                                # top stop so Position-Mode scripts can enable
                                # with the arm resting on it. Consequence: no
                                # top firmware backstop in Position Mode.
# Homing: homing_offset -20.3786 rad (Oct 2, in flash), referenced to the
# bottom bracket stop; top stop reads ~116.5 deg. Record the resting position
# each session; re-home after any bracket disassembly.

# Mode 0 (torque) software angle limit -- firmware limits don't apply in Mode 0.
# Same values as the Phase 1 run script (worked in the Aug 21 human run).
MODE0_MAX_ANGLE = math.radians(105.0)  # upward torque is 0 at/above this
MODE0_TAPER     = math.radians(15.0)   # linear taper over 90-105 deg

# Safety limits 
MAX_IQ      = 5.5    # Amps. Equal to firmware limit_i_max: a goal above it
                     # is IGNORED by firmware (not clamped), so never send one
TEMP_WARN   = 65.0   # degrees C — start scaling torque back
TEMP_MAX    = 75.0   # degrees C — zero torque above this

# Firmware watchdog (SDK Manual 1.2.0 §2.2.3). Armed per run by
# BearInterface.configure_watchdog()
WATCHDOG_TIMEOUT_MS = 10 # Mode 0 only, written each run and never saved
WATCHDOG_MIN_MS = 8 # Just above ~7.65 ms longest normal gap at 500hz
WATCHDOG_MAX_MS = 50 # Software loop already aborts at 50 ms

# --- SAFETY GATE ---
SAFETY_CHECKS_CONFIRMED = True

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")