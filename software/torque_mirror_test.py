"""
Torque-mirror test -- bidirectional current-based force feedback.

Both motors are kept energized with a small nonzero Iq bias so the
firmware produces useful IQ_READBACK telemetry on each side (per
iq_readback_probe.py: IQ_READBACK reads ~0 at Iq_cmd=0 regardless of
external torque -- the current loop actively cancels it out, so there is
no back-drive signal to mirror without a bias current).

Each motor's command is its own bias plus a term proportional to the
DIFFERENCE between the two motors' sensed deviation-from-bias (i.e. how
much more/less resisted the other motor is than this one):

    dev_a   = iq1_readback - IQ_BIAS_A
    dev_b   = iq2_readback - IQ_BIAS_A
    iq1_cmd = clamp(IQ_BIAS_A + MIRROR_GAIN * (dev_b - dev_a), -IQ_MAX_A, IQ_MAX_A)
    iq2_cmd = clamp(IQ_BIAS_A + MIRROR_GAIN * (dev_a - dev_b), -IQ_MAX_A, IQ_MAX_A)

(Earlier version fed back the raw dev_Y with no dev_X term -- that made
the common-mode (both motors drifting the same direction together) a
marginally-stable pure integrator at MIRROR_GAIN=1.0, which showed up on
the bench as a slow wander/jitter with no restoring force. Feeding back
the *difference* instead, mirroring the same relative-error structure
position_mirror_test.py uses for position, gives the common mode a real
decay rate regardless of gain.)

Unlike position_mirror_test.py (a virtual spring/damper whose felt force
grows with displacement), this loop couples current directly to current --
the felt force responds immediately to how resisted the other shaft is,
independent of how far either shaft has turned.

SAFETY:
    - Both motors MUST be mechanically secured before running this. Unlike
        force_mirror_test.py, BOTH motors move under their own bias current
        as soon as they start -- there is no "downstream-only" motor here.
    - This is a closed bidirectional loop -- each motor's output feeds the
        other's input. This has real positive-feedback/runaway potential
        that a one-directional mirror does not. Start with the defaults
        below (matching force_mirror_test.py's proven numbers) and only
        raise MIRROR_GAIN after confirming stable behavior in the logged
        plot.
    - IQ_MAX_A below is a hard safety clamp applied to every command sent
        to either motor, independent of what the other motor reports.
    - A telemetry staleness watchdog zeroes BOTH motors' torque if either
        side's IQ_READBACK hasn't updated recently.
    - Ctrl+C stops both motors immediately.

Every run writes a CSV to ./logs/ and, on exit, renders a diagnostic PNG
next to it via plot_run.plot_torque_mirror_log().

Run from the project's software/ dir with the venv active:
    python3 torque_mirror_test.py
"""

import csv
import os
import sys
import time

from can_interface import MotorCANInterface
from plot_run import plot_torque_mirror_log

LOOP_HZ = 750.0
LOOP_PERIOD_S = 1.0 / LOOP_HZ

IQ_BIAS_A = 0.15           # nonzero excitation required for IQ_READBACK, both motors
MIRROR_GAIN = 1.0          # 1:1 mirror of the other motor's deviation from its bias
IQ_MAX_A = 0.3             # hard safety clamp on every command sent, either motor

# Ramp the bias in linearly instead of stepping straight to IQ_BIAS_A --
# a bare step spins a free motor immediately (0.15A alone is enough per
# force_mirror_test.py's sign-check note), which feels like the rig
# lurching the moment the script starts.
STARTUP_RAMP_S = 1.5

# EMA low-pass on each motor's IQ_READBACK before mirroring it: raw readback
# is noisy, and at MIRROR_GAIN=1.0 that noise gets injected straight into the
# other motor's command every tick, which is what makes the unfiltered loop
# feel jittery/buzzy. filt = ALPHA*raw + (1-ALPHA)*filt_prev; 1.0 disables.
IQ_FILTER_ALPHA = 0.15

STALE_TIMEOUT_S = 0.05     # if either motor's IQ_READBACK is older than this,
                           # treat it as stale and zero BOTH motors' torque
STALE_WARN_EVERY_S = 1.0   # rate-limit stale-telemetry console warnings

NODE_BASE_M1 = 0x000
NODE_BASE_M2 = 0x020

LOG_DIR = "logs"


def clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = f"torque_mirror_bias{IQ_BIAS_A:g}_gain{MIRROR_GAIN:g}"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    m1 = MotorCANInterface(channel="can0", node_base=NODE_BASE_M1)
    m2 = MotorCANInterface(channel="can0", node_base=NODE_BASE_M2)
    m1.start_listening()
    m2.start_listening()

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(
        ["t_s", "iq1_readback", "iq2_readback", "iq1_filt", "iq2_filt",
         "iq1_cmd", "iq2_cmd", "dev_a", "dev_b"]
    )

    last_stale_warn = 0.0
    stale_count = 0
    rows_logged = 0

    try:
        print("Starting Motor 1...")
        m1.send_start()
        time.sleep(0.2)
        m1.send_set_iq(0.0)

        print("Starting Motor 2...")
        m2.send_start()
        time.sleep(0.2)
        m2.send_set_iq(0.0)

        print(
            f"Mirroring at {LOOP_HZ:.0f} Hz, IQ_BIAS={IQ_BIAS_A}A "
            f"(ramped over {STARTUP_RAMP_S:g}s), gain={MIRROR_GAIN}, "
            f"clamp=+-{IQ_MAX_A}A. Logging to {log_path}. Ctrl+C to stop.\n"
        )

        start_time = time.time()
        next_tick = start_time
        iq1_filt = 0.0
        iq2_filt = 0.0
        while True:
            now = time.time()
            elapsed = now - start_time
            bias = IQ_BIAS_A if elapsed >= STARTUP_RAMP_S else IQ_BIAS_A * (elapsed / STARTUP_RAMP_S)

            t1 = m1.get_telemetry()
            t2 = m2.get_telemetry()
            iq1 = t1["iq_readback"]
            iq2 = t2["iq_readback"]
            iq1_time = t1["iq_readback_time"]
            iq2_time = t2["iq_readback_time"]

            stale = (
                iq1 is None or iq2 is None
                or iq1_time is None or iq2_time is None
                or (now - iq1_time) > STALE_TIMEOUT_S
                or (now - iq2_time) > STALE_TIMEOUT_S
            )
            if stale:
                m1.send_set_iq(0.0)
                m2.send_set_iq(0.0)
                stale_count += 1
                if now - last_stale_warn > STALE_WARN_EVERY_S:
                    print(f"[warn] IQ_READBACK stale/missing on one or both "
                          f"motors (stale_count={stale_count}) -- both zeroed.")
                    last_stale_warn = now
            else:
                iq1_filt = IQ_FILTER_ALPHA * iq1 + (1 - IQ_FILTER_ALPHA) * iq1_filt
                iq2_filt = IQ_FILTER_ALPHA * iq2 + (1 - IQ_FILTER_ALPHA) * iq2_filt
                dev_a = iq1_filt - bias
                dev_b = iq2_filt - bias
                iq1_cmd = clamp(bias + MIRROR_GAIN * (dev_b - dev_a), IQ_MAX_A)
                iq2_cmd = clamp(bias + MIRROR_GAIN * (dev_a - dev_b), IQ_MAX_A)
                m1.send_set_iq(iq1_cmd)
                m2.send_set_iq(iq2_cmd)

                csv_writer.writerow(
                    [f"{elapsed:.4f}", f"{iq1:.4f}", f"{iq2:.4f}",
                     f"{iq1_filt:.4f}", f"{iq2_filt:.4f}",
                     f"{iq1_cmd:.4f}", f"{iq2_cmd:.4f}",
                     f"{dev_a:.4f}", f"{dev_b:.4f}"]
                )
                rows_logged += 1

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
        csv_file.close()
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
        if rows_logged == 0:
            os.remove(log_path)
            print("No data logged -- log discarded.")
        else:
            print(f"Log saved: {log_path}")
            png_path = plot_torque_mirror_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
