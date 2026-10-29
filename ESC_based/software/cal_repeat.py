"""
Does the leader's 7/rev ripple grow with time in RUN? (TEST_LOG 2026-10-28)

    python3 cal_repeat.py [--wait 60] [--n 2]

One START of both boards (as teleop does), then N leader_cal sweeps (about
30 s each, hands off, follower at 0 A) separated by WAIT seconds of RUN at
0 A on both. Prints each sweep's harmonics. Same START throughout, so a
change between sweeps is time-in-RUN (or temperature), not a new offset
calibration. Each sweep goes both directions, so direction is not
confounded with time the way it was in friction_map.py.

Safety: as leader_cal (lag, speed and staleness aborts; 0.3 A clamp), plus
the idle-Iq reset precheck on both nodes.
"""

import argparse
import json
import math
import os
import statistics as st
import sys
import time

import leader_cal
from can_interface import MotorCANInterface
from spin_test import RESET_IQ_A


def hold_zero(a, b, seconds):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        a.send_set_iq(0.0)
        b.send_set_iq(0.0)
        a.drain_records()
        b.drain_records()
        time.sleep(0.005)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wait", type=float, default=60.0)
    ap.add_argument("--n", type=int, default=2)
    args = ap.parse_args()
    os.makedirs("logs", exist_ok=True)
    stem = time.strftime("logs/cal_repeat_%Y%m%d_%H%M%S")
    a = MotorCANInterface(channel="can0", node_base=0x000, pair_dbg=False)
    b = MotorCANInterface(channel="can0", node_base=0x020, pair_dbg=False)
    a.start_listening()
    b.start_listening()
    results = []
    try:
        time.sleep(0.5)
        for name, m in (("A", a), ("B", b)):
            idle = [r["iq_meas"] for r in m.drain_records()]
            if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
                print(f"ABORT: node {name} missing or reset (run zero_iq_check.py first)")
                return 1
        a.send_start()
        b.send_start()
        time.sleep(0.2)
        hold_zero(a, b, 0.3)
        t0 = time.monotonic()
        for k in range(args.n):
            if k:
                print(f"waiting {args.wait:.0f} s in RUN at 0 A (hands off)...")
                hold_zero(a, b, args.wait)
            t_rel = time.monotonic() - t0
            print(f"sweep {k + 1}/{args.n} at t = {t_rel:.0f} s after START")
            rows = leader_cal.sweep(b, a, log_path=f"{stem}_{k + 1}.csv")
            if rows is None:
                return 1
            mdl = leader_cal.fit(rows)
            mdl["t_after_start_s"] = t_rel
            results.append(mdl)
            print("  " + leader_cal.summary(mdl))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        for m in (a, b):
            try:
                m.send_set_iq(0.0)
            except Exception:
                pass
        time.sleep(0.05)
        for m in (a, b):
            try:
                m.send_stop()
            except Exception as e:
                print(f"[warn] stop: {e}")
            m.stop_listening()
        for name, m in (("A", a), ("B", b)):
            if m.save_diag_history(f"{stem}_diag{name}.csv"):
                print(f"DIAG {name} -> {stem}_diag{name}.csv")
    with open(stem + ".json", "w") as f:
        json.dump(results, f, indent=1)
    print(f"saved {stem}.json")
    for h in ("7", "12", "28"):
        print(f"  {h}/rev: " + "  ".join(
            f"t={r['t_after_start_s']:.0f}s {math.hypot(*r['harmonics_A'][h]) * 1e3:.1f} mA "
            f"@{math.degrees(math.atan2(-r['harmonics_A'][h][1], r['harmonics_A'][h][0])):.0f}°"
            for r in results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
