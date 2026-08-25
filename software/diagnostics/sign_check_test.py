"""
Sign-convention check -- both motors.

Applies a small, fixed, open-loop Iq command (no P, no D, no feedback)
to each motor in turn and reports which direction its encoder count
moves. This exists because position_hold_test.py showed exponential
runaway consistent with the P/D sign convention being backwards
relative to what was previously assumed (positive Iq decreases count)
-- rather than guess again from noisy closed-loop data, this isolates
the one fact we need directly, per motor.

Motors are tested ONE AT A TIME, sequentially. ESC 2 is a newly
bring-up node -- its sign convention should NOT be assumed to match
ESC 1 just because the firmware is shared. Testing them separately
(rather than commanding both simultaneously) also avoids conflating
one motor's movement with the other's on the shared bus.

SAFETY: TEST_IQ_A is intentionally small and each run is short. Ctrl+C
stops immediately and zeros torque on whichever motor is currently
under test (and, in the finally block, attempts to zero/stop both).

Run from the project's software/ dir with the venv active:
    python3 sign_check_test.py
"""

import sys
import time

from can_interface import MotorCANInterface

TEST_IQ_A = 0.15       # small, deliberate, easy to feel/see
TEST_DURATION_S = 1.0  # short -- just enough to see clear directional movement

# node_base values must match can_driver.h's CAN_NODE_BASE for each board:
#   ESC 1: CAN_NODE_ID = 0 -> node_base = 0x000
#   ESC 2: CAN_NODE_ID = 1 -> node_base = 0x020 (CAN_NODE_STRIDE)
MOTORS = [
    {"name": "ESC 1 (Motor 1)", "node_base": 0x000},
    {"name": "ESC 2 (Motor 2)", "node_base": 0x020},
]


def run_sign_check(name: str, iface: MotorCANInterface) -> None:
    print(f"\n=== {name} (node_base=0x{iface.node_base:03X}) ===")

    print("Sending START...")
    iface.send_start()
    time.sleep(0.2)

    print("Commanding zero torque, capturing baseline position...")
    iface.send_set_iq(0.0)
    time.sleep(0.3)

    t0 = iface.get_telemetry()
    pos_before = t0["enc_count_unwrapped"]
    if pos_before is None:
        print(f"No encoder telemetry from {name} -- check CAN link/node_base.")
        iface.send_stop()
        return
    print(f"pos_before = {pos_before:.0f} counts")

    print(f"Commanding constant Iq = +{TEST_IQ_A}A for {TEST_DURATION_S}s...")
    iface.send_set_iq(TEST_IQ_A)
    time.sleep(TEST_DURATION_S)

    iface.send_set_iq(0.0)
    time.sleep(0.1)

    t1 = iface.get_telemetry()
    pos_after = t1["enc_count_unwrapped"]
    print(f"pos_after  = {pos_after:.0f} counts")

    delta = pos_after - pos_before
    print(f"delta = {delta:+.0f} counts")
    if delta > 0:
        print(f"=> {name}: Positive Iq INCREASED encoder count.")
    elif delta < 0:
        print(f"=> {name}: Positive Iq DECREASED encoder count.")
    else:
        print(f"=> {name}: No measurable movement -- rerun, or check for binding/friction.")

    iface.send_stop()
    time.sleep(0.1)


def main():
    interfaces = []
    try:
        for motor in MOTORS:
            iface = MotorCANInterface(channel="can0", node_base=motor["node_base"])
            iface.start_listening()
            interfaces.append((motor["name"], iface))

        # Small settle time so each interface's RX thread has a chance to
        # receive at least one telemetry frame before we read baseline.
        time.sleep(0.3)

        for name, iface in interfaces:
            run_sign_check(name, iface)

        print("\nDone. Compare the two results above -- do not assume they match.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for _, iface in interfaces:
            try:
                iface.send_set_iq(0.0)
                time.sleep(0.05)
                iface.send_stop()
            except Exception:
                pass
        for _, iface in interfaces:
            iface.stop_listening()
        print("CAN interfaces closed.")


if __name__ == "__main__":
    sys.exit(main())
