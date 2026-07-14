"""
IQ_READBACK sensing probe -- diagnostic for the force-mirror premise.

force_mirror_test.py assumes Motor 1's IQ_READBACK reflects the force a
hand applies to its shaft while Motor 1 is commanded to zero torque. In a
current-controlled FOC that is suspect: with Iq commanded to 0, the current
loop actively regulates measured Iq *to* 0, so back-driving the shaft
produces motion/back-EMF, not q-axis current -- IQ_READBACK should stay
near 0 regardless of applied force.

This probe tests that directly and non-destructively. It:
  - starts Motor 1,
  - commands Iq = 0 the entire time (never anything else),
  - prints IQ_READBACK, its age, and encoder position at ~50 Hz,
  - tracks the peak |Iq| seen so you can twist the shaft and watch.

WHAT TO WATCH FOR:
  - If |Iq| stays near ~0 (only tiny transient blips during fast twists)
    while you push HARD on the shaft, the mirror premise is confirmed
    broken: there is no force signal to mirror.
  - If |Iq| rises meaningfully and proportionally to how hard you push,
    the premise holds and the mirror problem is elsewhere.

SAFETY: only ever commands Iq = 0, so Motor 1 is never actively driven.
On a near-frictionless shaft it may still coast after you spin it by hand.
Ctrl+C zeroes torque and stops.

Run from software/ with the venv active:
    python3 iq_readback_probe.py
"""

import sys
import time

from can_interface import MotorCANInterface, ENC_PULSE_NBR

NODE_BASE_M1 = 0x000
PRINT_HZ = 50.0
PRINT_PERIOD_S = 1.0 / PRINT_HZ
RUN_DURATION_S = 20.0


def main():
    m1 = MotorCANInterface(channel="can0", node_base=NODE_BASE_M1)
    m1.start_listening()

    peak_abs_iq = 0.0

    try:
        print("Starting Motor 1 (commanded to ZERO torque the whole time)...")
        m1.send_start()
        time.sleep(0.2)
        m1.send_set_iq(0.0)
        time.sleep(0.3)

        t0 = m1.get_telemetry()
        if t0["iq_readback"] is None:
            print("No IQ_READBACK telemetry yet -- check CAN link/node_base.")
            return

        print(f"\nProbing for {RUN_DURATION_S:.0f}s at {PRINT_HZ:.0f} Hz.")
        print("Twist / push Motor 1's shaft by hand and watch Iq.\n")

        start = time.time()
        while time.time() - start < RUN_DURATION_S:
            # Re-assert the zero command each loop so nothing else can drive it.
            m1.send_set_iq(0.0)

            t = m1.get_telemetry()
            iq = t["iq_readback"]
            iq_time = t["iq_readback_time"]
            pos = t["enc_count_unwrapped"]

            now = time.time()
            age_ms = (now - iq_time) * 1000.0 if iq_time is not None else float("nan")
            if iq is not None:
                peak_abs_iq = max(peak_abs_iq, abs(iq))

            pos_rev = (pos / ENC_PULSE_NBR) if pos is not None else float("nan")
            iq_str = f"{iq:+.4f}" if iq is not None else "  None "
            print(
                f"t={now - start:5.1f}s  Iq={iq_str} A  "
                f"peak|Iq|={peak_abs_iq:.4f} A  age={age_ms:5.1f} ms  "
                f"pos={pos_rev:+8.2f} rev"
            )

            time.sleep(PRINT_PERIOD_S)

        print(f"\nDone. Peak |Iq| seen while back-driving: {peak_abs_iq:.4f} A")
        print("If that stayed near 0 despite hard pushing, the force-mirror")
        print("sensing premise is confirmed broken (nothing to mirror).")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Stopping Motor 1...")
        try:
            m1.send_set_iq(0.0)
            time.sleep(0.05)
            m1.send_stop()
        except Exception as e:
            print(f"[warn] error during stop: {e}")
        m1.stop_listening()
        print("CAN interface closed.")


if __name__ == "__main__":
    sys.exit(main())
