"""
Force-mirroring test -- the core 1DOF force-feedback loop.

Motor 1 (ESC 1) is kept energized with a small nonzero Iq command so
the firmware produces useful IQ_READBACK telemetry. If Motor 1 is
resisted or back-driven, its sensed current is forwarded to Motor 2
(ESC 2) in real time, at 1:1 gain -- so Motor 2 should push/resist
with roughly the same torque Motor 1 is feeling.

SAFETY:
    - Both motors MUST be mechanically secured before running this.
        Motor 2 will move in response to whatever Motor 1's shaft
        experiences -- an unsecured Motor 2 driven by someone twisting
        Motor 1 by hand is a real pinch/contact hazard (see sign-check
        test results: 0.15A alone spun a free motor ~20 rev/s).
    - M1_IQ_COMMAND_A is the deliberate excitation applied to Motor 1.
        Keep it small until the rig's behavior is understood.
    - IQ_MAX_A below is a hard safety clamp applied to every command
        sent to Motor 2, independent of whatever Motor 1 reports.
  - A telemetry staleness watchdog zeroes Motor 2's torque if Motor
    1's IQ_READBACK hasn't updated recently -- prevents Motor 2
    continuing on a stale/last-known value if the CAN link hiccups.
  - Ctrl+C stops both motors immediately.

Run from the project's software/ dir with the venv active:
    python3 force_mirror_test.py
"""

import sys
import time

from can_interface import MotorCANInterface

LOOP_HZ = 200.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ

MIRROR_GAIN = 1.0          # 1:1 direct mirror, per current test plan
M1_IQ_COMMAND_A = 0.15     # nonzero excitation required for IQ_READBACK
IQ_MAX_A = 0.3             # hard safety clamp on what Motor 2 is ever sent

STALE_TIMEOUT_S = 0.05     # if Motor 1's IQ_READBACK is older than this,
                           # treat it as stale and zero Motor 2's torque
STALE_WARN_EVERY_S = 1.0   # rate-limit stale-telemetry console warnings

NODE_BASE_M1 = 0x000
NODE_BASE_M2 = 0x020


def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def main():
    m1 = MotorCANInterface(channel="can0", node_base=NODE_BASE_M1)
    m2 = MotorCANInterface(channel="can0", node_base=NODE_BASE_M2)
    m1.start_listening()
    m2.start_listening()

    last_stale_warn = 0.0
    stale_count = 0

    try:
        print("Starting Motor 1 (sensing side)...")
        m1.send_start()
        time.sleep(0.2)
        m1.send_set_iq(M1_IQ_COMMAND_A)

        print("Starting Motor 2 (mirroring side)...")
        m2.send_start()
        time.sleep(0.2)
        m2.send_set_iq(0.0)

        print(
            f"Mirroring at {LOOP_HZ:.0f} Hz, M1_IQ={M1_IQ_COMMAND_A}A, "
            f"gain={MIRROR_GAIN}, clamp=+-{IQ_MAX_A}A. Ctrl+C to stop.\n"
        )

        next_tick = time.time()
        while True:
            now = time.time()

            t1 = m1.get_telemetry()
            iq1 = t1["iq_readback"]
            iq1_time = t1["iq_readback_time"]

            if iq1 is None or iq1_time is None or (now - iq1_time) > STALE_TIMEOUT_S:
                # Stale or missing telemetry -- fail safe.
                m2.send_set_iq(0.0)
                stale_count += 1
                if now - last_stale_warn > STALE_WARN_EVERY_S:
                    print(f"[warn] Motor 1 IQ_READBACK stale/missing "
                          f"(stale_count={stale_count}) -- Motor 2 zeroed.")
                    last_stale_warn = now
            else:
                iq2_cmd = clamp(iq1 * MIRROR_GAIN, IQ_MAX_A)
                m2.send_set_iq(iq2_cmd)

            m1.send_set_iq(M1_IQ_COMMAND_A)

            next_tick += LOOP_PERIOD_S
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Loop is running behind -- resync rather than accumulate drift.
                next_tick = time.time()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Stopping both motors...")
        try:
            m1.send_set_iq(0.0)
            m2.send_set_iq(0.0)
            time.sleep(0.05)
            m1.send_stop()
            m2.send_stop()
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        m1.stop_listening()
        m2.stop_listening()
        print("CAN interfaces closed.")


if __name__ == "__main__":
    sys.exit(main())