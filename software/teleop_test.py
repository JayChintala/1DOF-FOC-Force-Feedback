"""
Teleop test -- position-driven slave with current-based force reflection.

Motor B (ESC 2) is the CONTROLLER: it's spun by hand and drives nothing
on its own. Motor A (ESC 1) is the ROBOT: it moves ONLY in response to
Motor B's position changing, via a one-directional position PD (same
math as position_mirror_test.py, but only applied to A -- B never gets a
torque command derived from position error).

Channel 1 -- position drives the slave (A tracks B):
    err  = (pos_B - base_B) - (pos_A - base_A)
    derr = vel_B - vel_A
    iq_a_cmd = clamp(KP*err + KD*derr, -IQ1_MAX_A, IQ1_MAX_A)

Channel 2 -- Motor A's actual sensed current is reflected onto Motor B,
so whatever resistance the robot (A) is fighting gets felt on the
controller (B):
    iq_a_filt = EMA(iq_a_readback)
    iq_b_cmd  = clamp(FEEDBACK_GAIN * iq_a_filt, -IQ2_MAX_A, IQ2_MAX_A)

Unlike torque_mirror_test.py's bidirectional current mirror, these two
channels are independent and one-directional -- Motor B never receives
anything derived from its own state, so there's no closed-loop
common-mode drift to worry about. There's also no need for an artificial
bias current on either motor: Motor A's Iq command is never idle-zero by
design (it's whatever the position PD computes), so IQ_READBACK on A is
meaningful without one, and there's no startup lurch.
"""

import csv
import os
import sys
import time

from can_interface import MotorCANInterface
from plot_run import plot_teleop_log

# ---- Nodes ----
NODE_BASE_A = 0x000    # Motor A -- the "robot" (slave, position-tracking)
NODE_BASE_B = 0x020    # Motor B -- the "controller" (master, hand-driven)

# ---- Channel 1: position -> Motor A ----
# Starting point carried over from position_mirror_test.py's stable gains.
KP = 0.000265           # A/count of relative position error
KD = 0.00001            # A/(count/s) of relative velocity
IQ1_MAX_A = 0.8         # hard clamp on Motor A's commanded current

# EMA: vel_filt = ALPHA*vel_raw + (1-ALPHA)*vel_filt_prev. 0.25 matches
# position_hold_test.py rather than position_mirror_test.py's 0.5 -- Motor
# A is the sole actuator here (like position_hold's single free motor),
# not sharing the correction with a second motor, so it's more exposed to
# velocity-estimate noise feeding the KD term and benefits from heavier
# smoothing.
VEL_FILTER_ALPHA = 0.25

WATCHDOG_VEL_LIMIT_CNT_S = 200_000.0   # counts/s, either shaft
WATCHDOG_ERROR_LIMIT_CNT = 40000.0     # counts of relative error

# sign_check_test.py - Increasing Iq = 1, Decreasing Iq = -1
SIGN = 1

# ---- Channel 2: Motor A's current -> Motor B ----
# NEGATIVE by design: iq_a_cmd points in the direction that would reduce
# err (pull A toward B) -- feeding that same sign to B pushes B further
# AWAY from A (positive feedback/runaway, confirmed on the bench: holding
# A and spinning B kept accelerating B in the same direction instead of
# resisting). Negating it makes B get pulled back toward A's actual
# position instead, which is the correct restoring/resistance feel.
FEEDBACK_GAIN = -1.0    # A/A -- how much of A's sensed current is felt on B
IQ2_MAX_A = 0.3         # hard clamp on Motor B's commanded current (hand-held)
IQ_FILTER_ALPHA = 0.15  # EMA low-pass on Motor A's IQ_READBACK before mirroring

# ---- Shared ----
# 200 Hz matches the exact rate position_hold_test.py validated KP/KD at
# for a single free motor. position_mirror_test.py runs its (same) gains
# at 750 Hz, but that works there because both motors share the position
# correction; here Motor A alone must correct 100% of any error, so it's
# more exposed to loop-timing jitter and velocity-estimate noise. Console
# print() every iteration was blowing the achieved rate out to ~240 Hz
# with dt spikes up to 13.7ms (vs. a 1.33ms target at 750 Hz) -- see
# PRINT_EVERY_S below.
CONTROL_RATE_HZ = 200
STALE_TIMEOUT_S = 0.05  # telemetry older than this -> fail safe
STALE_WARN_EVERY_S = 1.0
PRINT_EVERY_S = 0.1     # throttle console output so it can't bottleneck the loop

LOG_DIR = "logs"


def clamp(value, lo, hi=None):
    if hi is None:
        lo, hi = -lo, lo
    return max(lo, min(hi, value))


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = f"teleop_KP{KP:g}_KD{KD:g}_FB{FEEDBACK_GAIN:g}"
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
         "iq_a_cmd", "iq_a_readback", "iq_a_filt", "iq_b_cmd", "iq_b_readback"]
    )

    stale_count = 0
    rows_logged = 0

    try:
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
            return

        base_a = ta0["enc_count_unwrapped"]
        base_b = tb0["enc_count_unwrapped"]
        print(f"Baselines: A={base_a:.0f}  B={base_b:.0f} counts")
        print(f"Channel 1 (position->A): KP={KP} KD={KD} IQ1_MAX={IQ1_MAX_A}A SIGN={SIGN}")
        print(f"Channel 2 (A current->B): FEEDBACK_GAIN={FEEDBACK_GAIN} IQ2_MAX={IQ2_MAX_A}A")
        print(f"Logging to {log_path}")
        print("Spin B; A should follow. Hold A; feel resistance in B. Ctrl+C to stop.\n")

        dt = 1.0 / CONTROL_RATE_HZ
        prev_pos_a = base_a
        prev_pos_b = base_b
        prev_time = time.monotonic()
        start_time = prev_time
        vel_a_filt = 0.0
        vel_b_filt = 0.0
        iq_a_filt = 0.0
        last_stale_warn = 0.0
        last_print = 0.0
        watchdog_tripped = False

        while True:
            ta = a.get_telemetry()
            tb = b.get_telemetry()
            now = time.monotonic()
            now_wall = time.time()  # telemetry timestamps are wall-clock

            pos_a = ta["enc_count_unwrapped"]
            pos_b = tb["enc_count_unwrapped"]
            ta_time = ta["enc_count_time"]
            tb_time = tb["enc_count_time"]
            iq_a = ta["iq_readback"]
            iq_a_time = ta["iq_readback_time"]

            # Fail-safe on missing/stale encoder telemetry from EITHER motor.
            enc_stale = (
                pos_a is None or pos_b is None
                or ta_time is None or tb_time is None
                or (now_wall - ta_time) > STALE_TIMEOUT_S
                or (now_wall - tb_time) > STALE_TIMEOUT_S
            )
            if enc_stale:
                a.send_set_iq(0.0)
                b.send_set_iq(0.0)
                stale_count += 1
                if now - last_stale_warn > STALE_WARN_EVERY_S:
                    print(f"[warn] encoder telemetry stale/missing "
                          f"(stale_count={stale_count}) -- both motors zeroed.")
                    last_stale_warn = now
                time.sleep(dt)
                continue

            elapsed = now - prev_time
            # First-frame guard: a near-zero elapsed turns tiny position
            # noise into a spurious huge velocity.
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
            err = d_b - d_a
            derr = vel_b_filt - vel_a_filt

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

            # Channel 1: position -> Motor A.
            coupling = KP * err + KD * derr
            iq_a_cmd = clamp(SIGN * coupling, IQ1_MAX_A)
            a.send_set_iq(iq_a_cmd)

            # Channel 2: Motor A's sensed current -> Motor B.
            iq_a_stale = (
                iq_a is None or iq_a_time is None
                or (now_wall - iq_a_time) > STALE_TIMEOUT_S
            )
            if iq_a_stale:
                iq_b_cmd = 0.0
            else:
                iq_a_filt = IQ_FILTER_ALPHA * iq_a + (1 - IQ_FILTER_ALPHA) * iq_a_filt
                iq_b_cmd = clamp(FEEDBACK_GAIN * iq_a_filt, IQ2_MAX_A)
            b.send_set_iq(iq_b_cmd)

            t_rel = now - start_time
            iq_a_act = ta["iq_readback"]
            iq_b_act = tb["iq_readback"]
            csv_writer.writerow(
                [f"{t_rel:.4f}", pos_a, pos_b, d_a, d_b, err,
                 f"{vel_a_filt:.1f}", f"{vel_b_filt:.1f}",
                 f"{iq_a_cmd:.4f}",
                 iq_a_act if iq_a_act is not None else "",
                 f"{iq_a_filt:.4f}",
                 f"{iq_b_cmd:.4f}",
                 iq_b_act if iq_b_act is not None else ""]
            )
            rows_logged += 1

            if now - last_print > PRINT_EVERY_S:
                print(
                    f"t={t_rel:5.1f}s  err={err:>8.0f}  "
                    f"iqA_cmd={iq_a_cmd:>7.3f}  iqA_filt={iq_a_filt:>7.3f}  "
                    f"iqB_cmd={iq_b_cmd:>7.3f}"
                )
                last_print = now

            prev_pos_a = pos_a
            prev_pos_b = pos_b
            prev_time = now
            time.sleep(dt)

        if not watchdog_tripped:
            pass

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
        if rows_logged == 0:
            os.remove(log_path)
            print("No data logged -- log discarded.")
        else:
            print(f"Log saved: {log_path}")
            png_path = plot_teleop_log(log_path)
            if png_path:
                print(f"Plot saved: {png_path}")


if __name__ == "__main__":
    sys.exit(main())
