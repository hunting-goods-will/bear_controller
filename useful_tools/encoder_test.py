from main_controller.bear_interface import BearInterface
iface = BearInterface()

pos, err = iface.bear.get_present_position(iface.id)[0]
homing_complete, hc_err = iface.bear.single_read(iface.id, ['homing_complete'])

print(f"present_position: {pos[0]:.4f} rad")
print(f"error byte: {err}  (baseline healthy = 128; if different, XOR "
      f"against 128 to see which bit flipped — bit 2 = absolute position error)")
print(f"homing_complete: {homing_complete}  error: {hc_err}")