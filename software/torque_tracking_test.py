"""
Torque-tracking verification test.

Every closed-loop test in this project (position_hold_test.py,
force_mirror_test.py, bilateral_test.py) is built on one unverified
assumption: that IQ_READBACK is an accurate, promptly-reported measure
of the current the motor is actually producing. position_hold_test.py
demonstrates the torque interface works *by feel* -- push the shaft,
watch it resist. This script checks the same interface *quantitatively*
and in open loop: it commands a staircase of fixed Iq setpoints (no P,
no D, no feedback) and measures, per step:

  - Steady-state tracking error: does IQ_READBACK settle near the
    commanded value, once transients have died out?
  - Telemetry freshness: the worst staleness (time since the last real
    IQ_READBACK frame) observed at any sampled instant during the run --
    catches both brief comms hiccups and a freeze that never recovers
    before the run ends.

IQ_READBACK is run through a 3-tap causal median filter before use (both
for the tracking report and the CSV/plot) to knock out single-sample
noise spikes without adding the lag of an IIR filter.

At the end it prints a PASS/FAIL table against the tolerances below,
so this can be rerun after firmware/ESC changes as a regression check
rather than something you eyeball once and forget.

Motor should be free-spinning / unloaded -- this measures the current
loop's own tracking and reporting, not the mechanical response to an
external load (that's what force_mirror_test.py and bilateral_test.py
are for).

SAFETY:
  - Motor shaft must be clear to spin freely -- no fingers, no
    obstructions, nothing attached that constant torque could sling.
  - Step levels/durations are deliberately small and short. Per
    sign_check_test.py, 0.15A alone spun a free motor ~20 rev/s, so
    higher steps here are held only briefly.
  - Every nonzero step is followed by a 0A dwell (see STEP_LEVELS_A)
    rather than ramping straight to the next nonzero level. On a
    near-frictionless free shaft, consecutive same-sign steps add
    momentum instead of settling, and an earlier version of this test
    reliably spun the shaft fast enough to trip the watchdog by the
    third step. The 0A dwells give friction a chance to bleed off
    speed between commands instead of letting it ratchet up.
  - Before the sequence starts, the script waits for the shaft to be
    near-stationary (see wait_for_still()) rather than assuming it's at
    rest -- back-to-back runs (or a prior watchdog abort) can leave the
    shaft still coasting, which otherwise contaminates the first
    step's "zero-command" baseline with leftover back-EMF current.
  - A velocity watchdog aborts the run (zeroing torque immediately) if
    the shaft accelerates past WATCHDOG_VEL_LIMIT_CNT_S -- constant
    current on a near-frictionless free shaft accelerates indefinitely
    rather than settling to a speed, so this is the thing that actually
    keeps a step from running away.
  - Ctrl+C stops the motor immediately.

If this test reports a large IQ_READBACK gap (stale-gap FAIL) on a run
that otherwise never tripped the watchdog, that's worth checking
independently with `candump can0` running in another terminal while you
rerun this -- a real intermittent CAN stall would also be dropping
SET_IQ commands during that window, not just IQ_READBACK, which is a
firmware/bus issue this script can't diagnose on its own.

Run from the project's software/ dir with the venv active:
    python3 torque_tracking_test.py
"""

import csv
import os
import statistics
import sys
import time
from collections import deque

from can_interface import MotorCANInterface, ENC_PULSE_NBR

# ---- Which motor to test ----
# ESC 1: CAN_NODE_ID = 0 -> node_base = 0x000
# ESC 2: CAN_NODE_ID = 1 -> node_base = 0x020
MOTOR_NODE_BASE = 0x020

CONTROL_RATE_HZ = 200.0

# ---- Step sequence ----
# Small, symmetric steps in both directions, each followed by a 0A dwell
# so the shaft has a chance to shed momentum before the next nonzero
# step -- see SAFETY above for why this replaced a monotonic staircase.
#
# Includes a +-0.8A step (matching position_hold_test.py's IQ_MAX_A, the
# highest current run on this hardware to date) to check tracking at a
# force level actually useful for haptics -- 0.05-0.15A confirmed
# accurate tracking but is too weak to be felt as meaningful resistance.
# 0.8A is >5x the 0.15A level that free-spun a motor ~20 rev/s
# (sign_check_test.py), so this step MUST be run with the shaft actively
# held/resisted by hand -- do not run this step on a free shaft.
STEP_LEVELS_A = [0.0, 0.05, 0.0, 0.10, 0.0, 0.15, 0.0, 0.40, 0.0,
                 -0.05, 0.0, -0.10, 0.0, -0.15, 0.0, -0.60, 0.0]
STEP_DURATION_S = 0.4

# ---- Pre-run stillness check ----
# Wait for the shaft to be near-stationary before starting the sequence,
# so a prior run's residual spin (from momentum or a watchdog abort)
# doesn't contaminate this run's first "zero-command" baseline.
STILL_VEL_LIMIT_CNT_S = 2000.0   # counts/s (~0.5 rev/s @ 4000 cnt/rev)
STILL_WAIT_TIMEOUT_S = 10.0

# Fraction of each step's duration treated as transient and excluded
# from the steady-state error calculation -- e.g. 0.5 means only the
# second half of each step's samples count toward the reported error.
SETTLE_FRACTION = 0.5

# ---- Pass/fail tolerances ----
# Steady-state |actual - cmd| this loose, per step, counts as tracking
# OK. Kept generous for a first-pass check; tighten once you have a
# baseline for this hardware.
TRACKING_TOL_A = 0.03
# Longest allowed gap between consecutive IQ_READBACK samples during
# the whole run. Anything longer means telemetry isn't actually
# streaming back at a rate useful for closed-loop control.
STALE_GAP_TOL_S = 0.05

# ---- Watchdog ----
WATCHDOG_VEL_LIMIT_CNT_S = 150_000.0  # counts/s (~37 rev/s @ 4000 cnt/rev)

LOG_DIR = "logs"


def wait_for_still(iface, vel_limit, timeout_s, dt):
    """
    Poll encoder velocity until it drops below vel_limit, or timeout_s
    elapses. Returns True if it settled, False if it timed out (caller
    proceeds anyway with a warning -- a truly frictionless shaft may
    never read exactly still, and this is a courtesy wait, not a hard
    precondition).
    """
    prev_pos = None
    prev_time = None
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        t = iface.get_telemetry()
        pos = t["enc_count_unwrapped"]
        now = time.monotonic()
        if pos is None:
            time.sleep(dt)
            continue
        if prev_pos is not None:
            elapsed = now - prev_time
            vel = 0.0 if elapsed < 0.5 * dt else (pos - prev_pos) / elapsed
            if abs(vel) < vel_limit:
                return True
        prev_pos = pos
        prev_time = now
        time.sleep(dt)
    return False


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = "torque_tracking"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    iface = MotorCANInterface(channel="can0", node_base=MOTOR_NODE_BASE)
    iface.start_listening()

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(
        ["t_s", "step_idx", "iq_cmd", "iq_actual", "iq_filtered", "iq_err",
         "vel", "iq_readback_age_s"]
    )

    no_data = False
    watchdog_tripped = False
    # Per-step samples collected during the settle window, for the
    # end-of-run tracking report: step_idx -> list of filtered iq_actual
    step_samples = {i: [] for i in range(len(STEP_LEVELS_A))}
    max_stale_gap = 0.0

    try:
        max_level = max(abs(v) for v in STEP_LEVELS_A)
        print(f"Sequence peaks at {max_level:.2f}A. Confirm the shaft is actively "
              f"held/resisted by hand for this run before continuing -- this level "
              f"is well past what's safe to run on a free/unrestrained shaft.")
        input("Press Enter to START and begin the torque-tracking sequence "
              "(Ctrl+C to abort)...")

        print("Sending START...")
        iface.send_start()
        time.sleep(0.2)
        iface.send_set_iq(0.0)
        time.sleep(0.2)

        t0 = iface.get_telemetry()
        if t0["enc_count_unwrapped"] is None:
            print("No encoder telemetry yet -- check CAN link before running this.")
            no_data = True
            return

        dt = 1.0 / CONTROL_RATE_HZ
        print("Waiting for shaft to settle near-stationary before starting...")
        settled = wait_for_still(iface, STILL_VEL_LIMIT_CNT_S, STILL_WAIT_TIMEOUT_S, dt)
        if not settled:
            print(f"[warn] shaft did not settle below {STILL_VEL_LIMIT_CNT_S:g} cnt/s "
                  f"within {STILL_WAIT_TIMEOUT_S}s -- proceeding anyway, but the "
                  f"zero-command baseline below may be contaminated by residual spin.")

        print(f"Steps (A): {STEP_LEVELS_A}")
        print(f"Step duration: {STEP_DURATION_S}s  Settle fraction: {SETTLE_FRACTION}")
        print(f"Tracking tolerance: +-{TRACKING_TOL_A}A  "
              f"Stale-gap tolerance: {STALE_GAP_TOL_S}s")
        print(f"Watchdog: |vel| > {WATCHDOG_VEL_LIMIT_CNT_S:g} cnt/s -> abort")
        print(f"Logging to {log_path}\n")

        prev_pos = iface.get_telemetry()["enc_count_unwrapped"]
        prev_time = time.monotonic()
        start_time = prev_time
        # 3-tap causal median filter on IQ_READBACK -- knocks out single-
        # sample noise spikes without the lag of an IIR filter and without
        # looking at future samples. Persists across step boundaries
        # (rather than resetting per step) since it's just smoothing a
        # continuous measurement stream.
        iq_median_window = deque(maxlen=3)

        for step_idx, iq_cmd in enumerate(STEP_LEVELS_A):
            step_start = time.monotonic()
            print(f"Step {step_idx}: Iq_cmd = {iq_cmd:+.3f}A")
            iface.send_set_iq(iq_cmd)

            while time.monotonic() - step_start < STEP_DURATION_S:
                t = iface.get_telemetry()
                pos = t["enc_count_unwrapped"]
                now = time.monotonic()

                if pos is None:
                    time.sleep(dt)
                    continue

                elapsed = now - prev_time
                vel = 0.0 if elapsed < 0.5 * dt else (pos - prev_pos) / elapsed

                if abs(vel) > WATCHDOG_VEL_LIMIT_CNT_S:
                    print(f"\nWATCHDOG TRIPPED: vel={vel:.0f} cnt/s. "
                          f"Zeroing torque and aborting.")
                    watchdog_tripped = True
                    break

                iq_actual = t["iq_readback"]
                if iq_actual is not None:
                    iq_median_window.append(iq_actual)
                iq_filtered = statistics.median(iq_median_window) if iq_median_window else None
                iq_time = t["iq_readback_time"]
                now_wall = time.time()
                age = (now_wall - iq_time) if iq_time is not None else None

                # Track the worst staleness seen at any sampled instant, not
                # just gaps between two known-different frames -- a freeze
                # that never recovers before the run ends would otherwise
                # never register (no "next" frame to diff against), which is
                # exactly what happened on ESC2: IQ_READBACK stopped mid-run
                # and this metric read a clean 0.0093s where reality was a
                # 2+ second stall, because it was only measuring transitions.
                if age is not None:
                    max_stale_gap = max(max_stale_gap, age)
                # Stale readings shouldn't count as evidence the current loop
                # under/over-shot -- they're not a measurement of this step
                # at all, just a leftover value from before the freeze.
                iq_fresh = age is not None and age <= STALE_GAP_TOL_S

                # Tracking error is measured against the filtered signal --
                # that's the point of filtering it (see iq_median_window
                # above): steady-state error shouldn't be dominated by
                # single-sample noise spikes.
                iq_err = (iq_filtered - iq_cmd) if iq_filtered is not None else None
                t_rel = now - start_time
                csv_writer.writerow(
                    [f"{t_rel:.4f}", step_idx, f"{iq_cmd:.4f}",
                     iq_actual if iq_actual is not None else "",
                     f"{iq_filtered:.4f}" if iq_filtered is not None else "",
                     f"{iq_err:.4f}" if iq_err is not None else "",
                     f"{vel:.1f}",
                     f"{age:.4f}" if age is not None else ""]
                )

                # Only samples in the back end of the step ("settled") and
                # backed by a fresh reading count toward the tracking report.
                if (now - step_start) >= SETTLE_FRACTION * STEP_DURATION_S \
                        and iq_fresh:
                    step_samples[step_idx].append(iq_filtered)

                # Print at ~10Hz, not every 5ms sample -- console I/O on the
                # Pi can occasionally block long enough to stall this poll
                # loop itself, which showed up as a spurious multi-hundred-ms
                # "telemetry gap" that was actually our own print() call
                # being slow, not a real IQ_READBACK dropout.
                if int(t_rel * 10) != int((t_rel - dt) * 10):
                    print(f"  Iq_actual={iq_actual!s:>8}  "
                          f"Iq_filtered={iq_filtered!s:>8}  "
                          f"err={iq_err if iq_err is not None else 'n/a':>8}  "
                          f"vel={vel:>9.0f} cnt/s  age={age if age is not None else 'n/a'}")

                prev_pos = pos
                prev_time = now
                time.sleep(dt)

            if watchdog_tripped:
                break

        if not watchdog_tripped:
            print("\nSequence complete.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        csv_file.close()
        try:
            iface.send_set_iq(0.0)
            time.sleep(0.05)
            iface.send_stop()
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        iface.stop_listening()
        print("CAN interface closed.")

        if no_data:
            os.remove(log_path)
            print("No data logged (aborted before run started) -- log discarded.")
            return

        print(f"Log saved: {log_path}")
        try:
            from plot_run import plot_torque_tracking_log
            png_path = plot_torque_tracking_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")
        except Exception as e:
            print(f"[warn] plotting failed: {e}")

        # ---- Summary report ----
        print("\n=== Torque tracking report ===")
        print(f"{'step':>4}  {'cmd(A)':>8}  {'mean_filtered(A)':>17}  "
              f"{'error(A)':>10}  {'result':>6}")
        overall_pass = True
        for step_idx, iq_cmd in enumerate(STEP_LEVELS_A):
            samples = step_samples[step_idx]
            if not samples:
                print(f"{step_idx:>4}  {iq_cmd:>8.3f}  {'no data':>17}  "
                      f"{'--':>10}  {'FAIL':>6}")
                overall_pass = False
                continue
            mean_filtered = sum(samples) / len(samples)
            err = mean_filtered - iq_cmd
            ok = abs(err) <= TRACKING_TOL_A
            overall_pass &= ok
            print(f"{step_idx:>4}  {iq_cmd:>8.3f}  {mean_filtered:>17.4f}  "
                  f"{err:>10.4f}  {'PASS' if ok else 'FAIL':>6}")

        stale_ok = max_stale_gap <= STALE_GAP_TOL_S
        overall_pass &= stale_ok
        print(f"\nWorst IQ_READBACK staleness observed: {max_stale_gap:.4f}s "
              f"(tolerance {STALE_GAP_TOL_S}s) -> {'PASS' if stale_ok else 'FAIL'}")

        if watchdog_tripped:
            overall_pass = False
            print("Run aborted by watchdog -- treat as FAIL regardless of "
                  "partial step results above.")

        print(f"\nOVERALL: {'PASS' if overall_pass else 'FAIL'}")


if __name__ == "__main__":
    sys.exit(main())
