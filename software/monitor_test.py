"""
Passive monitoring / diagnostic test script.

Brings up the CAN link, starts the motor, samples telemetry at 100 Hz
for a fixed capture window, then prints a diagnostic report:
  - effective update rate of each telemetry signal (IQ, angle, enc)
  - longest run of consecutive stale (duplicate) samples per signal
  - max gap between updates per signal
  - encoder unwrap sanity (raw vs unwrapped) at start/end

Does NOT send SET_IQ or run any control loop -- purely for validating
that START/STOP land and telemetry decodes/updates correctly before
building anything on top.

Run from the project's software/ dir with the venv active:
    python3 monitor_test.py
"""

import sys
import time

from can_interface import MotorCANInterface

POLL_HZ = 100
CAPTURE_SECONDS = 5.0


def summarize(name, samples, key, time_key):
    """samples: list of (poll_time, telemetry_dict)"""
    values = [s[1][key] for s in samples]
    times = [s[1][time_key] for s in samples]

    # longest run of consecutive identical values (staleness)
    longest_run = 1
    current_run = 1
    for i in range(1, len(values)):
        if values[i] == values[i - 1] and values[i] is not None:
            current_run += 1
            longest_run = max(longest_run, current_run)
        else:
            current_run = 1

    # effective update rate: count distinct non-None timestamps
    distinct_times = sorted(set(t for t in times if t is not None))
    duration = samples[-1][0] - samples[0][0]
    n_updates = len(distinct_times)
    rate_hz = n_updates / duration if duration > 0 else 0.0

    # max gap between successive updates
    max_gap = 0.0
    for i in range(1, len(distinct_times)):
        max_gap = max(max_gap, distinct_times[i] - distinct_times[i - 1])

    print(f"--- {name} ---")
    print(f"  effective update rate:      {rate_hz:6.1f} Hz  "
          f"({n_updates} updates / {duration:.2f} s)")
    print(f"  longest stale run (polls):  {longest_run}  "
          f"(~{longest_run / POLL_HZ * 1000:.0f} ms at {POLL_HZ} Hz poll)")
    print(f"  max gap between updates:    {max_gap * 1000:.1f} ms")


def main():
    iface = MotorCANInterface(channel="can0")
    iface.start_listening()

    try:
        print("Sending START...")
        iface.send_start()
        time.sleep(0.2)

        input(
            f"About to capture {CAPTURE_SECONDS:.0f}s of telemetry at "
            f"{POLL_HZ} Hz.\nRotate the shaft by hand partway through "
            f"(to cross the encoder wrap boundary) then press Enter to begin: "
        )

        samples = []
        period = 1.0 / POLL_HZ
        t_end = time.time() + CAPTURE_SECONDS
        next_poll = time.time()

        while time.time() < t_end:
            t = iface.get_telemetry()
            samples.append((time.time(), t))
            next_poll += period
            sleep_for = next_poll - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

        print(f"\nCaptured {len(samples)} polls over "
              f"{samples[-1][0] - samples[0][0]:.2f} s.\n")

        summarize("IQ_READBACK", samples, "iq_readback", "iq_readback_time")
        summarize("ELEC_ANGLE", samples, "elec_angle_deg", "elec_angle_time")
        summarize("ENC_COUNT", samples, "enc_count_raw", "enc_count_time")

        first_enc = samples[0][1]["enc_count_unwrapped"]
        last_enc = samples[-1][1]["enc_count_unwrapped"]
        first_raw = samples[0][1]["enc_count_raw"]
        last_raw = samples[-1][1]["enc_count_raw"]
        print("\n--- Encoder unwrap sanity ---")
        print(f"  raw:        {first_raw} -> {last_raw}")
        print(f"  unwrapped:  {first_enc} -> {last_enc}")
        print(f"  (if you rotated more than ~half a revolution, unwrapped "
              f"delta should reflect that continuously, not jump by "
              f"exactly ±{4000})")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nStopping motor...")
        iface.send_stop()
        iface.stop_listening()
        print("CAN interface closed.")


if __name__ == "__main__":
    sys.exit(main())
