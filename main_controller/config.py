# Actuator settings
import math 
import os 

ACTUATOR_ID = 1
PORT        = '/dev/ttyUSB0'   # udev rule
BAUDRATE    = 8000000
COMM_TIMEOUT  = 0.002
COMM_TRY_NUM  = 3
MIN_ANGLE = math.radians(-5)     # 0.0873 rad, real bracket hard stop at 0,
                                # 5 deg margin
MAX_ANGLE = math.radians(118)   # 1.8326 rad, measured top hard stop ranged
                                 # 115.0-118.8 deg 
# Last known-good homing_offset: -19.9935, referenced to bottom bracket stop.
# Verify with calibrate_homing_offset.py each session.          

# Safety limits 
MAX_IQ      = 6.0    # Amps — absolute maximum current command 
TEMP_WARN   = 65.0   # degrees C — start scaling torque back
TEMP_MAX    = 75.0   # degrees C — zero torque above this

# Firmware watchdog timeout register (see Westwood SDK Manual v1.0.1).
# Currently 0 (off) on the actuator by default. 
WATCHDOG_TIMEOUT_MS = None

# --- SAFETY GATE ---
SAFETY_CHECKS_CONFIRMED = True

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")