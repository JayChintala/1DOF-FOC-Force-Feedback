"""
Bilateral (position-coupled) force-feedback test -- the real 1DOF haptic loop.

Two motors are linked by a virtual spring+damper computed on the Pi. Each
is a torque source (FOC SET_IQ); the commanded torque is proportional to
the DIFFERENCE in the two shafts' positions (plus a damping term on the
difference in their velocities):

    err  = (pos_A - pos_A0) - (pos_B - pos_B0)   # relative displacement
    derr = vel_A - vel_B
    coupling  = KP*err + KD*derr
    tau_A = -SIGN * coupling      # A is pulled toward B
    tau_B = +SIGN * coupling      # B is pulled toward A

Turn shaft A and B follows; hold/block B and you feel the resistance in A,
and vice-versa. The force is conveyed by the coupling, so this relies ONLY
on the clean encoder telemetry and the working torque command -- it does
NOT read IQ_READBACK, which the iq_readback_probe.py run showed is just
noise around zero on a zero-commanded (free) motor. Torque is still 100%
FOC-generated; the force simply emerges from the position coupling rather
than from an explicit current measurement.

Baselines are captured at startup, so the shafts couple from wherever they
currently sit -- neither jerks to match the other's absolute encoder count.

SAFETY:
  - Both motors are ACTIVELY DRIVEN here (unlike the old force_mirror
    where one was left free). Mechanically secure both before running.
  - IQ_MAX_A hard-clamps every command to each motor.
  - Runaway watchdog: if |err| or either |vel| exceeds a hard limit,
    both torques are zeroed and the run aborts. A WRONG global SIGN turns
    the coupling into a repeller (motors fly apart) -- that shows up as an
    immediate watchdog trip with err growing monotonically. If that
    happens, flip SIGN to -1 and rerun.
  - Telemetry staleness watchdog: if either motor's ENC_COUNT goes stale,
    both torques are zeroed until fresh data returns.
  - Ctrl+C zeros and stops both motors.

Every run writes a timestamped-name CSV to ./logs/ and, on exit, renders a
4-panel PNG next to it via plot_run.plot_bilateral_log(), so you can inspect
the coupling afterward instead of reading the scrolling console.

Run from software/ with the venv active:
    python3 bilateral_test.py

Re-plot an existing log without re-running the motors:
    python3 plot_run.py logs/bilateral_KP0.00026_KD1e-05_A0.8.csv
"""

import csv
import os
import sys
import time

from can_interface import MotorCANInterface, ENC_PULSE_NBR
from plot_run import plot_bilateral_log

# ---- Nodes ----
# A and B are symmetric; label them however your rig is wired.
NODE_BASE_A = 0x000    # e.g. controller CAN address
NODE_BASE_B = 0x020    # e.g. robot CAN address

# ---- Tuning ----
# Starting point carried over from position_hold_test.py's stable gains.
KP = 0.00026           # A/count of relative position error
KD = 0.00001           # A/(count/s) of relative velocity
IQ_MAX_A = 0.8         # hard per-motor clamp (below position_hold's 0.8 for
                       # a first bilateral bring-up -- raise once it feels safe)

CONTROL_RATE_HZ = 300.0
RUN_DURATION_S = 30.0

# ---- Velocity filter (per shaft) ----
# EMA: vel_filt = ALPHA*vel_raw + (1-ALPHA)*vel_filt_prev. 1.0 disables.
VEL_FILTER_ALPHA = 0.25

# ---- Watchdogs ----
WATCHDOG_VEL_LIMIT_CNT_S = 200_000.0   # counts/s, per shaft
WATCHDOG_ERROR_LIMIT_CNT = 40000.0     # counts of relative error
STALE_TIMEOUT_S = 0.05                 # ENC_COUNT older than this -> fail safe

# ---- Sign convention ----
# sign_check_test.py - Increasing Iq = 1, Decreasing Iq = -1
SIGN = 1

LOG_DIR = "logs"


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def main():
    """
    Run the position-coupled bilateral loop end to end.

    Sequence: START both motors -> command zero torque so alignment settles
    -> capture each shaft's baseline position -> run the fixed-rate coupling
    loop (with staleness + runaway watchdogs) for RUN_DURATION_S -> on exit,
    always zero and STOP both motors, close the CAN interfaces, save the CSV,
    and render the diagnostic PNG. Returns None (process exit code 0).
    """
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = f"bilateral_KP{KP:g}_KD{KD:g}_A{IQ_MAX_A:g}"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    a = MotorCANInterface(channel="can0", node_base=NODE_BASE_A)
    b = MotorCANInterface(channel="can0", node_base=NODE_BASE_B)
    a.start_listening()
    b.start_listening()

    csv_file = open(log_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(
        ["t_s", "pos_a", "pos_b", "d_a", "d_b", "err", "vel_a", "vel_b",
         "derr", "iq_a_cmd", "iq_b_cmd", "iq_a_actual", "iq_b_actual"]
    )

    no_data = False
    stale_count = 0

    try:
        print("Confirm BOTH motors are mechanically secured before continuing.")
        input("Press Enter to START both motors and begin coupling (Ctrl+C to abort)...")

        print("Sending START to both...")
        a.send_start()
        b.send_start()
        time.sleep(0.2)
        a.send_set_iq(0.0)
        b.send_set_iq(0.0)
        time.sleep(0.3)

        ta0 = a.get_telemetry()
        tb0 = b.get_telemetry()
        if ta0["enc_count_unwrapped"] is None or tb0["enc_count_unwrapped"] is None:
            print("No encoder telemetry from one/both motors -- check CAN link/node_base.")
            no_data = True
            return

        base_a = ta0["enc_count_unwrapped"]
        base_b = tb0["enc_count_unwrapped"]
        print(f"Baselines: A={base_a:.0f}  B={base_b:.0f} counts")
        print(f"KP={KP} A/count  KD={KD} A/(count/s)  IQ_MAX={IQ_MAX_A}A  SIGN={SIGN}")
        print(f"Watchdog: |err|>{WATCHDOG_ERROR_LIMIT_CNT:g} cnt or "
              f"|vel|>{WATCHDOG_VEL_LIMIT_CNT_S:g} cnt/s -> abort")
        print(f"Logging to {log_path}")
        print("Turn one shaft; the other should follow. Block one; feel it in "
              "the other. Ctrl+C to stop.\n")

        dt = 1.0 / CONTROL_RATE_HZ
        prev_pos_a = base_a
        prev_pos_b = base_b
        prev_time = time.monotonic()
        start_time = prev_time
        vel_a_filt = 0.0
        vel_b_filt = 0.0
        last_stale_warn = 0.0
        watchdog_tripped = False

        while time.monotonic() - start_time < RUN_DURATION_S:
            ta = a.get_telemetry()
            tb = b.get_telemetry()
            now = time.monotonic()
            now_wall = time.time()  # telemetry timestamps are wall-clock

            pos_a = ta["enc_count_unwrapped"]
            pos_b = tb["enc_count_unwrapped"]
            ta_time = ta["enc_count_time"]
            tb_time = tb["enc_count_time"]

            # Fail-safe on missing/stale encoder telemetry from EITHER motor.
            stale = (
                pos_a is None or pos_b is None
                or ta_time is None or tb_time is None
                or (now_wall - ta_time) > STALE_TIMEOUT_S
                or (now_wall - tb_time) > STALE_TIMEOUT_S
            )
            if stale:
                a.send_set_iq(0.0)
                b.send_set_iq(0.0)
                stale_count += 1
                if now - last_stale_warn > 1.0:
                    print(f"[warn] encoder telemetry stale/missing "
                          f"(stale_count={stale_count}) -- both motors zeroed.")
                    last_stale_warn = now
                time.sleep(dt)
                continue

            elapsed = now - prev_time
            # First-frame guard (same rationale as position_hold): a near-zero
            # elapsed turns tiny position noise into a spurious huge velocity.
            if elapsed < 0.5 * dt:
                vel_a_raw = 0.0
                vel_b_raw = 0.0
            else:
                vel_a_raw = (pos_a - prev_pos_a) / elapsed
                vel_b_raw = (pos_b - prev_pos_b) / elapsed
            vel_a_filt = VEL_FILTER_ALPHA * vel_a_raw + (1 - VEL_FILTER_ALPHA) * vel_a_filt
            vel_b_filt = VEL_FILTER_ALPHA * vel_b_raw + (1 - VEL_FILTER_ALPHA) * vel_b_filt

            d_a = pos_a - base_a
            d_b = pos_b - base_b
            err = d_a - d_b
            derr = vel_a_filt - vel_b_filt

            # Watchdog -- before computing/sending torque.
            if (abs(err) > WATCHDOG_ERROR_LIMIT_CNT
                    or abs(vel_a_filt) > WATCHDOG_VEL_LIMIT_CNT_S
                    or abs(vel_b_filt) > WATCHDOG_VEL_LIMIT_CNT_S):
                print(
                    f"\nWATCHDOG TRIPPED: err={err:.0f} cnt, "
                    f"vel_a={vel_a_filt:.0f}, vel_b={vel_b_filt:.0f} cnt/s. "
                    f"Zeroing both and aborting.\n"
                    f"(If err grew monotonically from the start, flip SIGN.)"
                )
                watchdog_tripped = True
                break

            coupling = KP * err + KD * derr
            iq_a = clamp(-SIGN * coupling, -IQ_MAX_A, IQ_MAX_A)
            iq_b = clamp(+SIGN * coupling, -IQ_MAX_A, IQ_MAX_A)

            a.send_set_iq(iq_a)
            b.send_set_iq(iq_b)

            t_rel = now - start_time
            iq_a_act = ta["iq_readback"]
            iq_b_act = tb["iq_readback"]
            csv_writer.writerow(
                [f"{t_rel:.4f}", pos_a, pos_b, d_a, d_b, err,
                 f"{vel_a_filt:.1f}", f"{vel_b_filt:.1f}", f"{derr:.1f}",
                 f"{iq_a:.4f}", f"{iq_b:.4f}",
                 iq_a_act if iq_a_act is not None else "",
                 iq_b_act if iq_b_act is not None else ""]
            )

            print(
                f"t={t_rel:5.1f}s  err={err:>8.0f}  "
                f"vA={vel_a_filt:>9.1f}  vB={vel_b_filt:>9.1f}  "
                f"iqA={iq_a:>7.3f}  iqB={iq_b:>7.3f}"
            )

            prev_pos_a = pos_a
            prev_pos_b = pos_b
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
        try:
            a.send_set_iq(0.0)
            b.send_set_iq(0.0)
            time.sleep(0.05)
            a.send_stop()
            b.send_stop()
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        a.stop_listening()
        b.stop_listening()
        print("CAN interfaces closed.")
        if no_data:
            os.remove(log_path)
            print("No data logged (aborted before run started) -- log discarded.")
        else:
            print(f"Log saved: {log_path}")
            png_path = plot_bilateral_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
