from main_controller.bear_interface import BearInterface
from datetime import datetime
import os
import time
import csv


def main():
    # Timestamp and log file setup
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    csv_filename = os.path.join(log_dir, f"iq_torque_char_{timestamp}.csv")

    # Sweep definition — reuses the exact range already safely tested live
    current_sweep = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4]
    torque_constant = 0.67

    iface = BearInterface()  # reuses your tested connection code

    # PID gain setup (iq loop)
    iface.bear.set_p_gain_iq((iface.id, 0.02))
    iface.bear.set_i_gain_iq((iface.id, 0.02))
    iface.bear.set_d_gain_iq((iface.id, 0))
    print(iface.bear.get_p_gain_iq(iface.id))

    # PID gain setup (id loop)
    iface.bear.set_p_gain_id((iface.id, 0.02))
    iface.bear.set_i_gain_id((iface.id, 0.02))
    iface.bear.set_d_gain_id((iface.id, 0))

    # Firmware-level current backstop, independent of MAX_IQ in config.py
    iface.bear.set_limit_i_max((iface.id, 0.5))

    with open(csv_filename, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)

        # CSV header row
        writer.writerow([
            "timestamp",
            "commanded_iq",
            "present_iq",
            "torque_nm",
            "position",
            "velocity",
            "temp_c"
        ])

        start_run_time = time.time()

        try:
            iface.enable()

            # Execute the sweep
            for cmd_iq in current_sweep:
                print(f"Sweeping to commanded IQ: {cmd_iq} A")
                iface.set_iq(cmd_iq)

                # Electrical settling window (0.5 seconds)
                time.sleep(0.5)

                # High-resolution sampling: 100 samples over 2 seconds (20ms interval)
                sample_count = 100
                sample_interval = 0.02
                for _ in range(sample_count):
                    loop_start = time.time()

                    # Fetch state telemetry from the hardware registers
                    state = iface.get_state()

                    # Compute torque based on actual measured current
                    present_iq = state.get("iq", 0.0)
                    computed_torque = present_iq * torque_constant

                    # Absolute log timestamp
                    elapsed_time = time.time() - start_run_time

                    # Log individual row data directly to disk buffer
                    writer.writerow([
                        f"{elapsed_time:.4f}",
                        cmd_iq,
                        present_iq,
                        f"{computed_torque:.4f}",
                        state.get("position", 0.0),
                        state.get("velocity", 0.0),
                        state.get("temp", 0.0)
                    ])

                    # Enforce precise 20ms timing cadence
                    elapsed_loop = time.time() - loop_start
                    sleep_time = sample_interval - elapsed_loop
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            print("\nSweep successfully completed!")

        except KeyboardInterrupt:
            print("\nCharacterization interrupted by user (Ctrl+C).")
        except Exception as e:
            print(f"\nAn error occurred during execution: {e}")
        finally:
            # Safety shutdown backstop — runs no matter how the try block exits
            print("Disabling actuator and releasing hardware safely...")
            try:
                iface.set_iq(0.0)
                iface.disable()
            except Exception as shutdown_err:
                print(f"Error during safe disable sequence: {shutdown_err}")

    print(f"Data capture sequence terminated. Log finalized inside {csv_filename}")


if __name__ == "__main__":
    main()
