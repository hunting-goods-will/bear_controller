"""
Diagnoses and corrects the homing_offset calibration.

Read-only until you explicitly confirm. Never calls enable() — homing_offset
is a Configuration Register, not a motion command, so nothing here needs
torque armed. Writing it still touches flash (10K write-cycle endurance per
SDK manual §2.2.1) and must NOT happen while the motor is enabled (manual's
own warning) — this script never enables, so that's satisfied by construction.
"""
from main_controller.bear_interface import BearInterface
import time

iface = BearInterface()

# Sanity-check the method actually exists before relying on it — this
# register name is inferred from the manual's table + prior project notes,
# not independently confirmed against PyBEAR's source.
if not hasattr(iface.bear, "get_homing_offset"):
    candidates = [m for m in dir(iface.bear) if "offset" in m.lower()]
    raise SystemExit(
        f"bear.get_homing_offset doesn't exist. Offset-related methods "
        f"found instead: {candidates}. Use whichever this actually is."
    )

# --- Step 1: read only, nothing written yet ---
offset, err_off = iface.bear.get_homing_offset(iface.id)[0]
pos, err_pos = iface.bear.get_present_position(iface.id)[0]

if offset[0] is None or pos[0] is None:
    print("Read timed out — check actuator power/connection first.")
    raise SystemExit(1)

current_offset = offset[0]
current_position = pos[0]
raw_encoder = current_position - current_offset

print(f"Current homing_offset:     {current_offset:.4f} rad")
print(f"Current present_position:  {current_position:.4f} rad")
print(f"Implied raw encoder value: {raw_encoder:.4f} rad")

new_offset = current_offset - current_position
print(f"\nIf the arm is hanging at the user's side RIGHT NOW, the offset "
      f"that makes present_position read 0 there is: {new_offset:.4f} rad")

# --- Step 2: confirm before writing to flash ---
confirm = input(
    "\nIs the arm physically at the user's side at this exact moment, and "
    "do you want to write this now? [yes/N]: "
)
if confirm.strip().lower() != "yes":
    print("Not confirmed — nothing written.")
    raise SystemExit(0)

iface.bear.set_homing_offset((iface.id, new_offset))
iface.bear.save_config(iface.id)

time.sleep(0.5)
pos_check, err_check = iface.bear.get_present_position(iface.id)[0]
if pos_check[0] is None:
    print("Verification read timed out — re-run the script to check manually.")
else:
    print(f"\nAfter write: present_position = {pos_check[0]:.4f} rad (expect ~0)")
print("Now: raise the arm slightly by hand and re-check present_position — "
      "confirm it INCREASES as the arm rises. If it decreases, the sign "
      "convention is inverted and MIN_ANGLE/MAX_ANGLE need to be swapped "
      "before they mean what they're supposed to.")