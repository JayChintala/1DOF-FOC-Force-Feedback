"""
Teleop test -- velocity-driven slave with current-based force reflection.

Motor B (ESC 2) is the CONTROLLER: it's spun by hand and drives nothing on its own. 
Motor A (ESC 1) is the ROBOT: it tries to match Motor B's VELOCITY (not position).

Channel 1 -- velocity drives the slave (A tracks B's speed), PD on
velocity error:
    verr      = vel_B - vel_A
    accel_err = d(verr)/dt, EMA-filtered
    iq_a_cmd  = clamp(IQ1_MAX_A*tanh(KV_SLOPE*verr/IQ1_MAX_A) + KD_SLOPE*accel_err,
                       -IQ1_MAX_A, IQ1_MAX_A)

The plant here (current -> torque -> acceleration -> velocity) is
essentially an integrator. A pure-proportional (P-only) controller closed
around an integrator plant WILL ring once gain*loop-delay crosses a
threshold -- that's what every oscillation so far has actually been
(confirmed: vel_a rang 2.4-7x harder than vel_b regardless of B-side
damping, i.e. Motor A oscillating on its own, chasing zero velocity).
Every fix before this one (output filtering, velocity filtering) was
trying to patch that with low-pass filters, which remove high-frequency
content but don't add phase margin the way real derivative action does.
KD_SLOPE adds a term that opposes the RATE of change of the velocity
error specifically -- it engages hard during a fast oscillatory swing and
barely at all during a slow, steady push, which is what lets KV_SLOPE be
raised again without reintroducing the ring.

tanh (rather than a plain linear KV*verr on the P term) still gives a
steep initial ramp for small verr -- most use is at low speed -- while
its slope naturally decreases toward the IQ1_MAX_A ceiling instead of
ramping linearly into a hard clip.

A never corrects any position offset accumulated while blocked -- once free
again it matches B's speed going forward but does not catch back up to
B's absolute position. 

Channel 2 -- Motor A's actual sensed current is reflected onto Motor B,
so whatever resistance the robot (A) is fighting gets felt on the
controller (B), plus a damping term on B's own velocity:
    iq_a_filt = EMA(iq_a_readback)
    iq_b_cmd  = clamp(FEEDBACK_GAIN * iq_a_filt - KB_DAMP * vel_b_filt,
                       -IQ2_MAX_A, IQ2_MAX_A)

CORRECTION: these two channels are NOT actually independent once you
account for the physical hardware, only in the software's data-flow.
verr (which drives Channel 1) depends on vel_b; Channel 1's output drives
Channel 2's input (iq_a_filt); Channel 2's output physically spins Motor
B, which changes vel_b right back -- a genuine closed loop through the
hardware, with no term anywhere damping B's OWN velocity independent of
what A is doing. Confirmed on the bench: holding B tightly (but not with
literally infinite rigidity) still oscillated once slightly disturbed --
a sustained ~5Hz ring with vel_a/vel_b swinging tens of thousands of
counts/s, i.e. feedback howl through the A<->B loop, not local noise/lag
in Channel 1 alone. KB_DAMP adds real damping directly on the part of the
loop that had none, without touching the steady-state resistance you
feel from a sustained push (that still comes entirely from
FEEDBACK_GAIN * iq_a_filt).

There's no need for an artificial bias current on either motor: Motor
A's Iq command is never idle-zero by design (it's whatever the velocity
controller computes), so IQ_READBACK on A is meaningful without one, and
there's no startup lurch.
"""

import csv
import math
import os
import sys
import time

from can_interface import MotorCANInterface
from plot_run import plot_teleop_log

# ---- Nodes ----
NODE_BASE_A = 0x000    # Motor A -- the "robot" (slave, position-tracking)
NODE_BASE_B = 0x020    # Motor B -- the "controller" (master, hand-driven)

# ---- Channel 1: velocity -> Motor A (PD on velocity error) ----
# History: KV_SLOPE=0.0001 (P-only) rang badly. Adding a D term made it
# WORSE (KD_SLOPE=0.000005 + KV_SLOPE=0.00001 hit 198k cnt/s peak, the
# worst yet) -- accel_err_filt is a numerical double-derivative of
# position, which amplifies encoder/timing noise by roughly 1/dt^2;
# confirmed by zeroing KD_SLOPE at the same KV_SLOPE=0.00001, which
# passed the let-go-completely test with NO oscillation (but very weak
# feedback, as expected -- 0.00001 is the original "too weak" baseline).
# D term stays at 0 until it can be redone with a properly (heavily)
# filtered acceleration estimate rather than raw double-differencing.
# Now raising KV_SLOPE in SMALL increments from the confirmed-stable
# 0.00001, re-testing (disturb B, let go of both motors) at each step --
# 0.000018 is the pre-tanh value that was stable before any of this
# tuning started, a reasonable next step rather than jumping further.
KV_SLOPE = 0.000018      # A/(count/s), initial P-term slope near verr=0
IQ1_MAX_A = 1.0          # hard clamp/asymptote on Motor A's commanded
                         # current -- the "door's" max resistance. Raised
                         # from 0.8 -- observed peak usage was only ~0.6A,
                         # so 0.8 wasn't actually the binding ceiling yet.
                         # NOTE: current oscillation is also very likely
                         # why peak commands cap around 0.6A rather than
                         # reaching this ceiling -- the output filter
                         # below never lets a rapidly-reversing raw signal
                         # settle at an extreme. Expect this to resolve on
                         # its own once the ring is actually gone; revisit
                         # separately only if it's still capped after that.

# D term: opposes the RATE OF CHANGE of the velocity error (relative
# acceleration), not the error itself. Raised 10x from 0.0000005 -- that
# value was too small to have any visible braking effect against the
# accelerations actually seen during the ring (tens of thousands of
# counts/s^2). ACCEL_FILTER_ALPHA smooths the (noisy, double-differenced)
# acceleration estimate before use. Both remain unvalidated -- tune
# KD_SLOPE first if oscillation persists (higher damps harder) or if the
# feel goes mushy/laggy (lower).
KD_SLOPE = 0.0             # A/(count/s^2) of relative acceleration -- TEMPORARILY
                            # zeroed as a diagnostic: last run (KD_SLOPE=0.000005)
                            # got WORSE (peak vel_a 198k cnt/s, worst yet) even
                            # though KV_SLOPE was cut to its most conservative
                            # value. accel_err_filt is a numerical double-
                            # derivative of position, which amplifies encoder/
                            # timing noise by roughly 1/dt^2 -- suspect the D
                            # term is injecting noise-driven commands rather
                            # than damping. Zeroing it isolates whether D is the
                            # problem before touching anything else.
ACCEL_FILTER_ALPHA = 0.15

# Low-pass filter on the OUTPUT command (iq_a_cmd), on top of the PD
# terms above -- kept as a secondary smoothing stage, not the primary
# stability mechanism now that KD_SLOPE provides real derivative action.
IQ_A_CMD_FILTER_ALPHA = 0.12

# EMA: vel_filt = ALPHA*vel_raw + (1-ALPHA)*vel_filt_prev.
VEL_FILTER_ALPHA = 0.25

WATCHDOG_VEL_LIMIT_CNT_S = 200_000.0   # counts/s, either shaft -- genuine
                                        # runaway-speed protection. No
                                        # position-error watchdog: with a
                                        # pure velocity controller, a large
                                        # position gap while A is blocked
                                        # is the expected steady state, not
                                        # a fault.

# sign_check_test.py - Increasing Iq = 1, Decreasing Iq = -1
SIGN = 1

# ---- Channel 2: Motor A's current -> Motor B ----
# err (pull A toward B) -- feeding that same sign to B pushes B further
# AWAY from A (positive feedback/runaway, confirmed on the bench: holding
# A and spinning B kept accelerating B in the same direction instead of
# resisting). Negating it makes B get pulled back toward A's actual
# position instead, which is the correct restoring/resistance feel.
FEEDBACK_GAIN = -1.0    # A/A -- how much of A's sensed current is felt on B
IQ2_MAX_A = IQ1_MAX_A   # match Motor A's ceiling -- the point of Channel 2 is
                         # to feel exactly what A feels, so there's no reason
                         # for B's clamp to be lower than A's. (Was capped
                         # lower, 0.3 then 0.7, purely as an extra caution
                         # while validating this on the bench -- not because
                         # the hardware itself differs between the two motors.)
IQ_FILTER_ALPHA = 0.08  # EMA low-pass on Motor A's IQ_READBACK before mirroring --
                         # lowered from 0.15, reported jittery/inconsistent on the
                         # bench; heavier smoothing trades a bit more lag for a
                         # steadier felt resistance.

# Damping on Motor B's OWN velocity, independent of Channel 1 -- the
# actual fix for the ~5Hz, huge-amplitude oscillation, which turned out
# to be a closed loop through the physical hardware (vel_b -> Channel 1
# -> iq_a -> Channel 2 -> iq_b -> physically spins B -> vel_b), not a
# Channel-1-only problem. Nothing upstream of this damped B's own motion.
# Sized so it's small relative to a normal push (a few thousand counts/s)
# but substantial at the oscillation's observed amplitude (tens of
# thousands of counts/s) -- tune this first if oscillation persists or if
# it now feels like B is dragging during a normal push.
KB_DAMP = 0.00002       # A/(count/s) of Motor B's own filtered velocity


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
    base_name = f"teleop_KVs{KV_SLOPE:g}_FB{FEEDBACK_GAIN:g}"
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
        print(f"Channel 1 (velocity->A): KV_SLOPE={KV_SLOPE} KD_SLOPE={KD_SLOPE} "
              f"IQ1_MAX={IQ1_MAX_A}A SIGN={SIGN}")
        print(f"Channel 2 (A current->B): FEEDBACK_GAIN={FEEDBACK_GAIN} "
              f"KB_DAMP={KB_DAMP} IQ2_MAX={IQ2_MAX_A}A")
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
        iq_a_cmd_filt = 0.0
        prev_verr = 0.0
        accel_err_filt = 0.0
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
            err = d_b - d_a       # logged for visibility only -- not used for control
            verr = vel_b_filt - vel_a_filt

            # D term: rate of change of verr (relative acceleration),
            # filtered -- see KD_SLOPE/ACCEL_FILTER_ALPHA comment above.
            if elapsed < 0.5 * dt:
                accel_err_raw = 0.0
            else:
                accel_err_raw = (verr - prev_verr) / elapsed
            accel_err_filt = (ACCEL_FILTER_ALPHA * accel_err_raw
                               + (1 - ACCEL_FILTER_ALPHA) * accel_err_filt)
            prev_verr = verr

            # Watchdog -- before computing/sending torque. Position error
            # (err) is NOT checked here: with a pure velocity controller, a
            # large/growing gap while A is blocked is the expected steady
            # state (the whole point of the door model), not a fault.
            if (abs(vel_a_filt) > WATCHDOG_VEL_LIMIT_CNT_S
                    or abs(vel_b_filt) > WATCHDOG_VEL_LIMIT_CNT_S):
                print(
                    f"\nWATCHDOG TRIPPED: vel_a={vel_a_filt:.0f}, "
                    f"vel_b={vel_b_filt:.0f} cnt/s. Zeroing both and aborting."
                )
                watchdog_tripped = True
                break

            # Channel 1: PD on velocity error -- P (tanh saturation) plus D
            # (opposes the rate of change of verr, see KD_SLOPE comment),
            # then low-pass the combined OUTPUT command as a secondary
            # smoothing stage.
            iq_a_p = IQ1_MAX_A * math.tanh(SIGN * KV_SLOPE * verr / IQ1_MAX_A)
            iq_a_d = SIGN * KD_SLOPE * accel_err_filt
            iq_a_cmd_raw = clamp(iq_a_p + iq_a_d, IQ1_MAX_A)
            iq_a_cmd_filt = (IQ_A_CMD_FILTER_ALPHA * iq_a_cmd_raw
                              + (1 - IQ_A_CMD_FILTER_ALPHA) * iq_a_cmd_filt)
            iq_a_cmd = iq_a_cmd_filt
            a.send_set_iq(iq_a_cmd)

            # Channel 2: Motor A's sensed current -> Motor B.
            iq_a_stale = (
                iq_a is None or iq_a_time is None
                or (now_wall - iq_a_time) > STALE_TIMEOUT_S
            )
            if iq_a_stale:
                iq_b_cmd = clamp(-KB_DAMP * vel_b_filt, IQ2_MAX_A)
            else:
                iq_a_filt = IQ_FILTER_ALPHA * iq_a + (1 - IQ_FILTER_ALPHA) * iq_a_filt
                iq_b_cmd = clamp(FEEDBACK_GAIN * iq_a_filt - KB_DAMP * vel_b_filt, IQ2_MAX_A)
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
