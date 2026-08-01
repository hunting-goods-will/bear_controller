"""
Read-only connectivity check.
"""
from pybear import Manager
from main_controller.bear_interface import BearInterface
from main_controller.config import PORT, BAUDRATE, ACTUATOR_ID, COMM_TIMEOUT, COMM_TRY_NUM

print(f"Connecting to port {PORT}...")
bear = Manager.BEAR(port=PORT, baudrate=BAUDRATE)
bear.single_timeout = COMM_TIMEOUT
bear.single_try_num = COMM_TRY_NUM

print(f"Pinging actuator ID {ACTUATOR_ID}...")
result = bear.ping(ACTUATOR_ID)
print(f"Ping result: {result}")

temp, err = bear.get_winding_temperature(ACTUATOR_ID,)[0]
pos, err = bear.get_present_position(ACTUATOR_ID,)[0]
if temp[0] is None or pos[0] is None:
    print("Read timed out (got None), check actuator power/LED before debugging further.")
else:
    print(f"Temperature: {temp[0]:.1f} C")
    print(f"Position:    {pos[0]:.4f} rad")
print("Connection OK. No torque was commanded — this script never enables the actuator.")

iface = BearInterface()
limit, err = iface.bear.get_limit_i_max(iface.id)[0]
print(f"limit_i_max: {limit[0]} A")

