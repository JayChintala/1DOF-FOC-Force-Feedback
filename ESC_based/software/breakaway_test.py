"""
Breakaway (static friction) of one motor, hands off: slow current ramps.

    python3 breakaway_test.py --node B            # run (about 30 s)
    python3 breakaway_test.py --analyze logs/breakaway_B.csv

Each trial ramps Iq from 0 at RAMP_A_S in one direction until the shaft
breaks away (moved > MOVE_COUNTS from the trial start AND |v| >
MOVE_VEL), records that current, then brakes with a velocity damping
term (no position hold) until still for SETTLE_S. Directions alternate;
the shaft ends each trial at a new angle, so TRIALS trials sample several
angles. Every TELEM frame is logged (mcu_time_us, enc, iq_meas) plus the
commanded current and the trial number; analysis re-detects breakaway
from the frames.

Leader stiction compensation (TEST_LOG 2026-10-26) needs this plus the
running friction (spin_test.py at a few speeds).

Safety: current clamped to IQ_MAX_A; abort if a trial travels more than
MAX_TRAVEL counts, exceeds MAX_VEL, telemetry goes stale, or the node
looks reset (idle-Iq precheck).
"""

import argparse
import csv
import os
import statistics as st
import sys
import time

import numpy as np

from can_interface import MotorCANInterface
from estimators import AlphaBetaVelocity
from spin_test import RESET_IQ_A

NODES = {"A": 0x000, "B": 0x020}
RATE_HZ = 500
RAMP_A_S = 0.1          # A/s
IQ_MAX_A = 0.2
MOVE_COUNTS = 5
MOVE_VEL = 200.0        # counts/s
BRAKE_KD = 3e-5         # A/(count/s)
SETTLE_S = 0.4
TRIALS = 8
MAX_TRAVEL = 600        # counts per trial (54 deg)
MAX_VEL = 8000.0


def run(node):
    os.makedirs("logs", exist_ok=True)
    path = os.path.join("logs", f"breakaway_{node}.csv")
    n = 2
    while os.path.exists(path):
        path = os.path.join("logs", f"breakaway_{node}_{n}.csv")
        n += 1
    m = MotorCANInterface(channel="can0", node_base=NODES[node], pair_dbg=False)
    m.start_listening()
    rows, found, reason = [], [], "done"
    try:
        time.sleep(0.5)
        idle = [r["iq_meas"] for r in m.drain_records()]
        if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
            print("ABORT: node missing or reset (run zero_iq_check.py first)")
            return 1
        m.send_start()
        t0 = time.monotonic()
        while not m.get_debug().get("run"):
            if time.monotonic() - t0 > 5:
                print("ABORT: no RUN")
                return 1
            m.send_set_iq(0.0)
            time.sleep(0.01)
        vel = AlphaBetaVelocity(55.0)
        x, v = None, 0.0
        dt = 1.0 / RATE_HZ
        for trial in range(TRIALS):
            sign = 1 if trial % 2 == 0 else -1
            for r in m.drain_records():
                x, v = r["enc"], vel.update(r["mcu_time_us"], r["enc"])
            if x is None:
                reason = "ABORT: no telemetry frames"
                break
            x0, iq, phase, t_ph = x, 0.0, "ramp", time.monotonic()
            deadline = time.monotonic()
            while True:
                now = time.monotonic()
                for r in m.drain_records():
                    x, v = r["enc"], vel.update(r["mcu_time_us"], r["enc"])
                    rows.append((r["mcu_time_us"], x, r["iq_meas"], iq, trial, sign))
                tel = m.get_telemetry()
                if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
                    reason = "ABORT: stale telemetry"
                elif abs(x - x0) > MAX_TRAVEL or abs(v) > MAX_VEL:
                    reason = f"ABORT: trial {trial} travel {x - x0:+.0f} counts / speed {v:.0f} counts/s"
                if reason != "done":
                    break
                if phase == "ramp":
                    iq = sign * min(IQ_MAX_A, RAMP_A_S * (now - t_ph))
                    if abs(x - x0) > MOVE_COUNTS and abs(v) > MOVE_VEL:
                        found.append((trial, sign, iq))
                        phase, t_ph = "brake", now
                    elif abs(iq) >= IQ_MAX_A and now - t_ph > IQ_MAX_A / RAMP_A_S + 1.0:
                        found.append((trial, sign, None))
                        phase, t_ph = "brake", now
                else:
                    iq = max(-IQ_MAX_A, min(IQ_MAX_A, -BRAKE_KD * v))
                    if abs(v) > MOVE_VEL:
                        t_ph = now
                    elif now - t_ph > SETTLE_S:
                        break
                m.send_set_iq(iq)
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            if reason != "done":
                break
            b = found[-1][2] if found and found[-1][0] == trial else None
            print(f"trial {trial} dir {sign:+d}: " + (f"breakaway {abs(b) * 1e3:.0f} mA" if b is not None
                                                     else f"no breakaway by {IQ_MAX_A} A"))
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e}")
        m.stop_listening()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mcu_time_us", "enc", "iq_meas", "iq_cmd", "trial", "dir"])
        w.writerows(rows)
    print(f"{reason}. {len(rows)} frames. Log: {os.path.abspath(path)}")
    if rows:
        analyze(path)
    return 0 if reason == "done" else 1


def analyze(path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    if d.ndim == 1 or len(d) == 0:
        print("no data")
        return
    t, p, i, ic, tr, dr = d[:, 0] * 1e-6, d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 5]
    print(f"{path}: {len(t)} frames")
    print("  trial dir  angle(cnt mod 4000)  breakaway cmd mA  measured mA (50 ms mean before)")
    br = {1: [], -1: []}
    for k in np.unique(tr):
        s = np.where(tr == k)[0]
        x0 = p[s[0]]
        moved = s[np.abs(p[s] - x0) > MOVE_COUNTS]
        if len(moved) == 0:
            print(f"  {int(k):5d} {int(dr[s[0]]):+3d}  no breakaway")
            continue
        j = moved[0]
        w = s[(t[s] > t[j] - 0.05) & (t[s] <= t[j])]
        sgn = int(dr[s[0]])
        br[sgn].append(abs(i[w].mean()))
        print(f"  {int(k):5d} {sgn:+3d}  {x0 % 4000:8.0f}          {abs(ic[j]) * 1e3:6.0f}           {abs(i[w].mean()) * 1e3:6.0f}")
    for sgn in (1, -1):
        if br[sgn]:
            print(f"  dir {sgn:+d}: breakaway (measured) median {np.median(br[sgn]) * 1e3:.0f} mA, "
                  f"range {min(br[sgn]) * 1e3:.0f}-{max(br[sgn]) * 1e3:.0f} mA (n={len(br[sgn])})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", choices=NODES, default="B")
    ap.add_argument("--analyze")
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze)
        return 0
    return run(a.node)


if __name__ == "__main__":
    sys.exit(main())
