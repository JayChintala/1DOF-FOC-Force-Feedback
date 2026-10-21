"""
Closed-loop constant-speed spin of one motor (host velocity loop).

    python3 spin_test.py [--node B] [--speed 4000] [--time 5] [--iq-max 0.15]

Replaces software/spin_motor2.py for the ESC_based firmware: that script
sends SET_IQ once (the firmware zeroes current after 200 ms without a
resend) and runs open loop, so on a direct-drive link nothing bounds the
speed. Here a P loop on velocity, resent every tick, holds a target speed
with the current clamped at --iq-max. Starting point for the A4 ripple runs.

Velocity is a least-squares slope of position against the ESC's own
microsecond clock over the last VEL_WIN frames (every TELEM frame, via
drain_records()), so dt never comes from a nominal constant.

Safety: refuses to start if the node's idle Iq says it was reset (the next
START would run the 2 s alignment, which drops SET_IQ); aborts on
|v| > max(OVERSPEED_FACTOR * |speed|, OVERSPEED_MIN_CNT_S) or on
stale telemetry.
"""

import argparse
import collections
import csv
import os
import statistics as st
import sys
import time

from can_interface import MotorCANInterface

NODES = {"A": 0x000, "B": 0x020}
CONTROL_RATE_HZ = 500
KV = 1.5e-5             # A per count/s of speed error
KI = 0.0                # A per count of integrated speed error (--ki); 0 = P only
RAMP_S = 0.5            # speed reference ramp at start and end
VEL_WIN = 8             # frames in the LS velocity window (8 ms at 1 kHz)
OVERSPEED_FACTOR = 3.0
OVERSPEED_MIN_CNT_S = 10_000.0  # never abort below this (A2's speed band): at low
                                # targets a stick-slip breakaway burst alone exceeds 3x
STALE_S = 0.05
RESET_IQ_A = 1.0        # |idle Iq| above this = uncalibrated, node was reset


class LsVelocity:
    """Sliding least-squares slope of position vs MCU time."""

    def __init__(self, n=VEL_WIN):
        self.buf = collections.deque(maxlen=n)

    def update(self, t_us, pos):
        self.buf.append((t_us * 1e-6, pos))
        if len(self.buf) < 3:
            return 0.0
        ts = [p[0] for p in self.buf]
        xs = [p[1] for p in self.buf]
        mt, mx = st.mean(ts), st.mean(xs)
        den = sum((t - mt) ** 2 for t in ts)
        return sum((t - mt) * (x - mx) for t, x in zip(ts, xs)) / den if den else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", choices=NODES, default="B")
    ap.add_argument("--speed", type=float, default=4000.0, help="counts/s, signed")
    ap.add_argument("--time", type=float, default=5.0, help="seconds at speed")
    ap.add_argument("--iq-max", type=float, default=0.15)
    ap.add_argument("--kv", type=float, default=KV, help="A/(count/s)")
    ap.add_argument("--ki", type=float, default=KI,
                    help="A/count; integrator clamped to +-iq-max (anti-windup)")
    ap.add_argument("--coast", type=float, default=0.0,
                    help="seconds at Iq = 0 straight after the hold, logged (A4); "
                         "replaces the ramp-down")
    a = ap.parse_args()

    os.makedirs("logs", exist_ok=True)
    base = f"spin_{a.node}_{a.speed:+.0f}_{a.iq_max:g}A_kv{a.kv:g}_ki{a.ki:g}"
    if a.coast:
        base += f"_coast{a.coast:g}"
    log_path = os.path.join("logs", f"{base}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join("logs", f"{base}_{suffix}.csv")
        suffix += 1
    m = MotorCANInterface(channel="can0", node_base=NODES[a.node])
    m.start_listening()
    rows = []
    try:
        time.sleep(0.5)
        idle = [r["iq_meas"] for r in m.drain_records()]
        if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
            print(f"ABORT: idle Iq n={len(idle)} mean="
                  f"{st.mean(idle) if idle else float('nan'):+.2f} A -- node missing "
                  "or reset (run zero_iq_check.py first to re-align)")
            return 1

        m.send_start()
        time.sleep(0.2)
        vel = LsVelocity()
        v = 0.0
        v_ref, iq = 0.0, 0.0  # command in force, logged with each frame
        phase = "ramp"        # ramp -> hold -> ramp_down | coast
        dt = 1.0 / CONTROL_RATE_HZ
        t0 = time.monotonic()
        next_deadline = t0
        prev_now = t0
        iq_i = 0.0  # integral term, A
        t_hold_end = RAMP_S + a.time
        total = t_hold_end + (a.coast if a.coast else RAMP_S)
        reason = "done"
        while True:
            now = time.monotonic()
            tr = now - t0
            if tr >= total:
                break
            for r in m.drain_records():
                v = vel.update(r["mcu_time_us"], r["enc"])
                rows.append((r["bus_time"], r["mcu_time_us"], r["enc"], r["iq_meas"], v,
                             v_ref, iq, phase))
            tel = m.get_telemetry()
            if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > STALE_S:
                reason = "stale telemetry"
                break
            if abs(v) > max(OVERSPEED_FACTOR * abs(a.speed), OVERSPEED_MIN_CNT_S):
                reason = f"overspeed {v:.0f} counts/s"
                break
            if tr < RAMP_S:
                phase = "ramp"
            elif tr < t_hold_end:
                phase = "hold"
            else:
                phase = "coast" if a.coast else "ramp_down"
            if phase == "coast":
                v_ref, iq = float("nan"), 0.0
            else:
                frac = min(1.0, tr / RAMP_S, (total - tr) / RAMP_S)
                v_ref = a.speed * max(0.0, frac)
                # dt for the integrator is the measured loop period, not 1/rate.
                e = v_ref - v
                iq_p = a.kv * e
                iq_i = max(-a.iq_max, min(a.iq_max, iq_i + a.ki * e * (now - prev_now)))
                iq = max(-a.iq_max, min(a.iq_max, iq_p + iq_i))
            prev_now = now
            m.send_set_iq(iq)
            next_deadline += dt
            slack = next_deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_deadline = time.monotonic()
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e} (firmware 200 ms SET_IQ timeout is the backstop)")
        time.sleep(0.05)
        m.stop_listening()

    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bus_time", "mcu_time_us", "enc", "iq_meas", "vel", "v_ref", "iq_cmd",
                    "phase"])
        w.writerows(rows)

    hold = [r for r in rows if r[7] == "hold"]
    print(f"Stopped: {reason}. Log: {os.path.abspath(log_path)}")
    if hold:
        vs = [r[4] for r in hold]
        iqs = [r[6] for r in hold]
        sat = sum(abs(i) >= a.iq_max - 1e-9 for i in iqs) / len(iqs)
        print(f"At speed ({len(hold)} frames): vel mean={st.mean(vs):.0f} "
              f"std={st.pstdev(vs):.0f} counts/s (target {a.speed:.0f}); "
              f"iq_cmd mean={st.mean(iqs):+.3f} A, saturated {100 * sat:.0f}% of frames")
        print(f"Revolutions turned: {(rows[-1][2] - rows[0][2]) / 4000:+.2f}")
    coast = [r for r in rows if r[7] == "coast"]
    if coast:
        print(f"Coast ({len(coast)} frames): speed {coast[0][4]:.0f} -> {coast[-1][4]:.0f} "
              f"counts/s over {(coast[-1][1] - coast[0][1]) * 1e-6:.2f} s")
    return 0 if reason == "done" else 1


if __name__ == "__main__":
    sys.exit(main())
