"""
Encoder resolution check.

Verifies the assumed ENC_PULSE_NBR = 4000 (counts per mechanical
revolution) by having you manually rotate the shaft a known number of
full revolutions and comparing against the measured unwrapped delta.
This assumption feeds directly into WrappingCounter's wrap detection
-- if it's wrong, you'd see spurious jumps in enc_count_unwrapped right
at wrap boundaries, which could look like control instability even with
a correct control law.

Procedure:
  1. Put a physical mark/reference on the shaft and a fixed reference
     point on the housing (tape works fine) so you can tell when you've
     completed exactly one full revolution.
  2. Run this script. It captures a baseline position, then waits for
     you to rotate the shaft by hand.
  3. Rotate SLOWLY (to avoid WrappingCounter misdetecting wrap
     direction -- it assumes consecutive samples never differ by more
     than half a revolution) through exactly N_REVOLUTIONS full turns,
     always in the same direction, ending back at your mark.
  4. Press Enter. It reports measured counts/revolution vs. the 4000
     assumption.

Run from the project's software/ dir with the venv active:
    python3 encoder_resolution_check.py
"""

import sys
import time

from can_interface import MotorCANInterface, ENC_PULSE_NBR

N_REVOLUTIONS = 5  # more revolutions = less relative error from imprecise
                    # start/end alignment by eye


def main():
    iface = MotorCANInterface(channel="can0")
    iface.start_listening()

    try:
        print("Sending START...")
        iface.send_start()
        time.sleep(0.2)

        print("Commanding zero torque (motor will NOT move on its own)...")
        iface.send_set_iq(0.0)
        time.sleep(0.3)

        t0 = iface.get_telemetry()
        pos_before = t0["enc_count_unwrapped"]
        if pos_before is None:
            print("No encoder telemetry -- check CAN link.")
            return
        print(f"\nBaseline captured: {pos_before:.0f} counts")
        print(
            f"Now SLOWLY rotate the shaft by hand exactly "
            f"{N_REVOLUTIONS} full revolutions in one direction, "
            f"ending back at your mark.\n"
        )
        input("Press Enter once you've completed the rotation: ")

        t1 = iface.get_telemetry()
        pos_after = t1["enc_count_unwrapped"]
        delta = pos_after - pos_before

        measured_per_rev = delta / N_REVOLUTIONS

        print(f"\npos_before = {pos_before:.0f}")
        print(f"pos_after  = {pos_after:.0f}")
        print(f"delta      = {delta:+.0f} counts over {N_REVOLUTIONS} revolutions")
        print(f"measured counts/revolution = {measured_per_rev:.1f}")
        print(f"assumed ENC_PULSE_NBR      = {ENC_PULSE_NBR}")

        pct_error = abs(measured_per_rev - ENC_PULSE_NBR) / ENC_PULSE_NBR * 100
        print(f"discrepancy = {pct_error:.2f}%")

        if pct_error < 2.0:
            print("\n=> Within expected hand-rotation precision. 4000 assumption looks correct.")
        else:
            print(
                "\n=> Discrepancy larger than expected hand-rotation error. "
                "ENC_PULSE_NBR may be wrong, or a wrap was misdetected during "
                "the manual rotation (rotate slower and retry to rule that out)."
            )

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        iface.send_set_iq(0.0)
        time.sleep(0.05)
        iface.send_stop()
        iface.stop_listening()
        print("CAN interface closed.")


if __name__ == "__main__":
    sys.exit(main())
