# First motion test for Panda BEAR Westwood Actuator
from bear_interface import BearInterface
import time

iface = BearInterface()   # reuses your tested connection code

# set_p_gain_iq:
iface.bear.set_p_gain_iq((iface.id, 0.02))
iface.bear.set_i_gain_iq((iface.id, 0.02))
iface.bear.set_d_gain_iq((iface.id, 0))

print(iface.bear.get_p_gain_iq(iface.id))

# For The ID's
iface.bear.set_p_gain_id((iface.id, 0.02))
iface.bear.set_i_gain_id((iface.id, 0.02))
iface.bear.set_d_gain_id((iface.id, 0))

# Limit Max I
iface.bear.set_limit_i_max((iface.id, 0.5))

try:
    # 1. Enable the actuator
    iface.enable()

    # 2. Command a small torque
    iface.set_iq(0.45)

    # 3. monitoring loop
    for i in range(10):
        state = iface.get_state()
        print(f"t={i*0.2:.1f}s  pos={state['position']:.3f}  vel={state['velocity']:.3f}  "
              f"iq={state['iq']:.3f}  temp={state['temp']:.1f}")
        time.sleep(0.2)

finally:
    # 4. zero IQ and disable — iface.disable() already does both of these
    # internally (set_goal_iq(0.0) then set_torque_enable(0)), so no need
    # to write those two raw calls separately like you had before
    iface.disable()
