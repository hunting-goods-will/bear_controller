# Actuator settings
import math 
import os 

ACTUATOR_ID = 1
PORT        = '/dev/ttyUSB0'   # udev rule
BAUDRATE    = 8000000
COMM_TIMEOUT  = 0.002
COMM_TRY_NUM  = 3
MIN_ANGLE = math.radians(20)   # 0.3491 rad
MAX_ANGLE = math.radians(100)  # 1.7453 rad

# Safety limits 
MAX_IQ      = 1.0    # Amps — absolute maximum current command 
TEMP_WARN   = 65.0   # degrees C — start scaling torque back
TEMP_MAX    = 75.0   # degrees C — zero torque above this

# Firmware watchdog timeout register (see Westwood SDK Manual v1.0.1).
# Currently 0 (off) on the actuator by default. 
WATCHDOG_TIMEOUT_MS = None

# --- SAFETY GATE ---
SAFETY_CHECKS_CONFIRMED = True

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")