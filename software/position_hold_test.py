"""
Position-hold demo.

Closes a simple PD position loop on top of the existing torque (SET_IQ)
interface: captures the shaft's starting position as a target, then
continuously commands a current proportional to position error (plus a
damping term on velocity) to push it back toward that target.

Every run writes a timestamped CSV to ./logs/ in addition to printing
to the console, so you can plot it afterward with plot_run.py instead
of copy-pasting terminal output.

This is a functional demonstration, not a tuned controller -- if you
push the shaft, you should feel it resist and spring back toward the
starting position. That's the torque loop actively responding to
disturbance, not just holding a static current.

SAFETY: Iq is clamped to IQ_MAX_A below. Start low. If the shaft
oscillates/buzzes uncontrollably instead of settling, KP is too high
relative to KD -- lower KP or raise KD before increasing either further.
Conversely, if Iq_cmd is slamming between +IQ_MAX_A and -IQ_MAX_A almost
every sample (visible clearly in the plot), KD is too high relative to
sensor noise/loop delay -- that's derivative kick, not underdamping.

This version adds two things the earlier iteration didn't have:
  1. A runaway watchdog: if |error| or |velocity| exceeds a hard limit,
     torque is zeroed and the run aborts immediately. This exists because
     multiple prior sessions on this exact hardware required a manual
     Ctrl+C during multi-second high-speed runaways.
  2. An EMA low-pass filter on the velocity estimate used for the KD
     term. Raw diff-based velocity was noisy enough to cause inconsistent
     behavior run-to-run at identical gains. Set VEL_FILTER_ALPHA = 1.0
     to disable filtering entirely (i.e. use raw velocity) if you want
     to A/B it against the filtered case.

It assumes a fixed sign convention (SIGN constant below) rather than
probing it with an open-loop pulse, since positive-Iq-direction has
flipped between sessions on this exact hardware for reasons not yet
root-caused, and probing it directly was itself causing problems (see
comment above SIGN).

Run from the project's software/ dir with the venv active:
    python3 position_hold_test.py
"""

import csv
import os
import sys
import time

from can_interface import MotorCANInterface, ENC_PULSE_NBR
from plot_run import plot_log

# ---- Tuning ----
# Reset to the best confirmed-stable reference point from prior sessions.
# (Do not reintroduce untested gains at the same time as new safety code.)
KP = 0.0001
KD = 0.000005
IQ_MAX_A = 0.3

CONTROL_RATE_HZ = 100.0
RUN_DURATION_S = 15.0

# ---- Velocity filter ----
# EMA: vel_filt = ALPHA * vel_raw + (1 - ALPHA) * vel_filt_prev
# ALPHA = 1.0 disables filtering (raw velocity, matches earlier behavior).
# Lower ALPHA = more smoothing, more lag. Start at 0.3 and adjust by feel.
VEL_FILTER_ALPHA = .25

# ---- Watchdog ----
# These are deliberately loose first-pass limits, not tuned to this plant.
# Tighten once you have a feel for normal excursion size during a push test.
WATCHDOG_VEL_LIMIT_CNT_S = 200_000.0   # counts/s
WATCHDOG_ERROR_LIMIT_CNT = 40000.0    # counts (10 rotations @ 4000 cnt/rev)

# ---- Sign convention ----
# Positive-Iq direction has flipped between sessions on this exact hardware
# for reasons not yet root-caused. Rather than probing it with an open-loop
# current pulse (which on this near-frictionless plant leaves the shaft
# actually coasting afterward -- zero current does not mean zero velocity
# here -- and was tripping the watchdog on frame 1), we assume a sign and
# let the watchdog catch it cheaply if wrong: a closed-loop PD with the
# wrong sign diverges fast (error grows monotonically instead of settling),
# so a bad sign shows up as an immediate watchdog trip with error growing
# in one direction, not as banked-up real velocity.
# If that happens: flip this to -1 and rerun.
SIGN = 1

LOG_DIR = "logs"


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = f"position_hold_KP{KP:g}_KD{KD:g}_A{IQ_MAX_A:g}"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    iface = MotorCANInterface(channel="can0")
    iface.start_listening()

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(
        ["t_s", "pos", "target_pos", "error", "vel", "vel_raw",
         "iq_cmd", "iq_actual"]
    )

    no_data = False

    try:
        print("Sending START...")
        iface.send_start()
        time.sleep(0.2)

        print("Commanding zero torque to let alignment settle...")
        iface.send_set_iq(0.0)
        time.sleep(0.3)

        print(f"Using assumed sign={SIGN} (flip SIGN constant if watchdog "
              f"trips immediately with error growing monotonically).")

        t0 = iface.get_telemetry()
        if t0["enc_count_unwrapped"] is None:
            print("No encoder telemetry yet -- check CAN link before running this.")
            no_data = True
            return
        target_pos = t0["enc_count_unwrapped"]
        target_deg = target_pos / ENC_PULSE_NBR * 360.0
        print(
            f"Holding position: {target_pos:.0f} counts ({target_deg:.1f} mech deg)")
        print(f"KP={KP} A/count  KD={KD} A/(count/s)  IQ_MAX={IQ_MAX_A}A")
        print(f"Watchdog: |vel|>{WATCHDOG_VEL_LIMIT_CNT_S:g} cnt/s or "
              f"|error|>{WATCHDOG_ERROR_LIMIT_CNT:g} cnt -> abort")
        print(f"Velocity filter alpha: {VEL_FILTER_ALPHA:g}")
        print(f"Logging to {log_path}")
        print("Push the shaft and watch it resist. Ctrl+C to stop early.\n")

        dt = 1.0 / CONTROL_RATE_HZ
        prev_pos = target_pos
        prev_time = time.monotonic()
        start_time = prev_time
        vel_filt = 0.0
        watchdog_tripped = False

        while time.monotonic() - start_time < RUN_DURATION_S:
            t = iface.get_telemetry()
            pos = t["enc_count_unwrapped"]
            now = time.monotonic()

            if pos is None:
                time.sleep(dt)
                continue

            elapsed = now - prev_time
            # Guard against the "stale velocity baseline" failure mode seen
            # in the relay autotuner: on the very first loop iteration,
            # elapsed only reflects get_telemetry() call overhead (a few
            # hundred microseconds), not a real control period. Any small
            # position noise divided by that near-zero elapsed produces a
            # spuriously huge velocity that can trip the watchdog on frame 1
            # even though nothing actually moved.
            if elapsed < 0.5 * dt:
                vel_raw = 0.0
            else:
                vel_raw = (pos - prev_pos) / elapsed
            vel_filt = VEL_FILTER_ALPHA * vel_raw + \
                (1 - VEL_FILTER_ALPHA) * vel_filt

            error = target_pos - pos

            # Watchdog check -- before computing/sending anything further.
            if abs(vel_filt) > WATCHDOG_VEL_LIMIT_CNT_S or abs(error) > WATCHDOG_ERROR_LIMIT_CNT:
                print(
                    f"\nWATCHDOG TRIPPED: vel_filt={vel_filt:.0f} cnt/s, "
                    f"error={error:.0f} cnt. Zeroing torque and aborting."
                )
                watchdog_tripped = True
                break

            iq_cmd = SIGN * (KP * error - KD * vel_filt)
            iq_cmd = clamp(iq_cmd, -IQ_MAX_A, IQ_MAX_A)

            iface.send_set_iq(iq_cmd)

            t_rel = now - start_time
            iq_actual = t["iq_readback"]
            csv_writer.writerow(
                [f"{t_rel:.4f}", pos, target_pos, error, f"{vel_filt:.1f}",
                 f"{vel_raw:.1f}", f"{iq_cmd:.4f}",
                 iq_actual if iq_actual is not None else ""]
            )

            print(
                f"pos={pos:>10.0f}  error={error:>8.0f}  "
                f"vel={vel_filt:>10.1f} cnt/s  Iq_cmd={iq_cmd:>7.3f}A  "
                f"Iq_actual={iq_actual!s:>8}"
            )

            prev_pos = pos
            prev_time = now
            time.sleep(dt)

        if watchdog_tripped:
            pass
        elif time.monotonic() - start_time >= RUN_DURATION_S:
            print("\nStopping (run complete)...")

    except KeyboardInterrupt:
        print("\nStopping (interrupted)...")
    finally:
        csv_file.close()
        iface.send_set_iq(0.0)
        time.sleep(0.05)
        iface.send_stop()
        iface.stop_listening()
        print("CAN interface closed.")
        if no_data:
            os.remove(log_path)  # aborted before any rows were logged
            print(
                "No data logged (aborted before run started) -- log discarded, no plot.")
        else:
            print(f"Log saved: {log_path}")
            png_path = plot_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
