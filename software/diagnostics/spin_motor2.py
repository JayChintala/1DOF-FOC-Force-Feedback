"""
Spin Motor 2 (ESC 2, node_base=0x020) with a small open-loop Iq command.

START -> wait -> SET_IQ -> hold -> zero -> STOP.
Ctrl+C stops immediately and zeros torque.
"""

import sys
import time

from can_interface import MotorCANInterface

NODE_BASE = 0x020   # ESC 2 / Motor 2
SPIN_IQ_A = 0.20    # modest open-loop current
SPIN_TIME_S = 3.0


def main():
    iface = MotorCANInterface(channel="can0", node_base=NODE_BASE)
    iface.start_listening()
    try:
        print("START...")
        iface.send_start()
        time.sleep(0.2)

        print(f"Spinning Motor 2 at Iq = +{SPIN_IQ_A} A for {SPIN_TIME_S}s...")
        iface.send_set_iq(SPIN_IQ_A)

        t_end = time.time() + SPIN_TIME_S
        while time.time() < t_end:
            time.sleep(0.25)
            t = iface.get_telemetry()
            enc = t["enc_count_unwrapped"]
            iq = t["iq_readback"]
            print(f"  enc={enc if enc is None else f'{enc:.0f}'}  iq_rb={iq}")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        iface.send_set_iq(0.0)
        time.sleep(0.05)
        iface.send_stop()
        time.sleep(0.05)
        iface.stop_listening()
        print("Stopped, torque zeroed, CAN closed.")


if __name__ == "__main__":
    sys.exit(main())
