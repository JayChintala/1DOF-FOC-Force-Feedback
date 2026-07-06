"""
Passive monitoring test script.

Brings up the CAN link, starts the motor, streams decoded telemetry to
the console at ~20 Hz, and stops the motor cleanly on Ctrl+C.

This does NOT send SET_IQ or run any control loop -- it's purely for
validating that START/STOP land and telemetry decodes correctly before
building anything on top.

Run from the project's software/ dir with the venv active:
    python3 monitor_test.py
"""

import sys
import time

from can_interface import MotorCANInterface


def main():
    iface = MotorCANInterface(channel="can0")
    iface.start_listening()

    try:
        print("Sending START...")
        iface.send_start()
        time.sleep(0.2)  # give the drive a moment to enter RUN state

        print("Listening for telemetry. Ctrl+C to stop.\n")
        while True:
            t = iface.get_telemetry()
            print(
                f"Iq={t['iq_readback']!s:>10}  "
                f"angle_deg={t['elec_angle_deg']!s:>10}  "
                f"enc_raw={t['enc_count_raw']!s:>12}  "
                f"enc_unwrapped={t['enc_count_unwrapped']!s:>12}",
                end="\r",
            )
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping motor...")
        iface.send_stop()
    finally:
        iface.stop_listening()
        print("CAN interface closed.")


if __name__ == "__main__":
    sys.exit(main())
