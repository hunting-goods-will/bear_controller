"""
Read-only connectivity check.

This script never calls enable(), set_torque_enable(), or set_goal_iq() —
it only pings the actuator and reads two status registers. It carries none
of the ESTOP risk that motion commands do, which is exactly why it's safe
to run today regardless of where the ESTOP question stands.

Run this first, always, before anything in controller.py or main.py.
"""
from pybear import Manager
from config import PORT, BAUDRATE, ACTUATOR_ID

print(f"Connecting to port {PORT}...")
bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)

print(f"Pinging actuator ID {ACTUATOR_ID}...")
result = bear.ping(ACTUATOR_ID)
print(f"Ping result: {result}")

temp, err = bear.get_winding_temperature(ACTUATOR_ID,)[0]
pos, err = bear.get_present_position(ACTUATOR_ID,)[0]
if temp[0] is None or pos[0] is None:
    print("Read timed out (got None) — check actuator power/LED before debugging further.")
else:
    print(f"Temperature: {temp[0]:.1f} C")
    print(f"Position:    {pos[0]:.4f} rad")
print("Connection OK. No torque was commanded — this script never enables the actuator.")


