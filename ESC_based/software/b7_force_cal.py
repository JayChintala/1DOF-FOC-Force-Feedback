"""
Plan B7: static force calibration of the follower (A) against a kitchen scale.

    python3 b7_force_cal.py --sign +1          # run the schedule
    python3 b7_force_cal.py --fit logs/B7_cal.csv --grams 12,25,51,...   # fit

Setup (user at the bench): the follower link rests on a kitchen scale on a
rigid support; tare the scale with the link resting on it at 0 A. --sign is
the current sign that pushes the link INTO the scale. If unsure, try +1: a
wrong sign lifts the link, which aborts on ABORT_TRAVEL at level 1; rerun
with -1 (rename or delete the aborted log first).

Schedule: for each level, hold SET_IQ for HOLD_S (resent every 2 ms), then
GAP_S at 0 A. The user reads the steady scale value near the end of each
hold and writes it down. Every TELEM frame is logged with the level index,
so the fit uses the MEASURED current (mean iq_meas over the last half of
each hold), not the command.

Safety: never more than MAX_A (0.8 A = NOMINAL_CURRENT_A; the firmware's
direct path does not clamp -- see TEST_LOG "Firmware changes"). Aborts and
zeroes if the shaft moves faster than ABORT_VEL counts/s (link slipped off
the scale) or telemetry goes stale.

Fit: F = m * 9.81 / 1000 N vs mean measured Iq; pass (brief B7) R^2 >= 0.99
and slope within 15 % of 3.3 N/A. Keep this data separate from Phase D.
"""

import argparse
import csv
import os
import statistics as st
import sys
import time

import numpy as np

from can_interface import MotorCANInterface
from spin_test import RESET_IQ_A

LEVELS_A = (0.05, 0.10, 0.20, 0.30, 0.45, 0.60, 0.80)
HOLD_S = 5.0
GAP_S = 4.0
LEAD_S = 5.0              # 0 A countdown before the first level
MAX_A = 0.8
ABORT_VEL = 2000.0        # counts/s
ABORT_TRAVEL = 100        # counts moved within one level: not pressing a rigid stop
EXPECT_N_PER_A = 3.3      # 1 A ~ 66 mN m at a 2 cm link
LOG = os.path.join("logs", "B7_cal.csv")


def run(sign, log=LOG):
    os.makedirs("logs", exist_ok=True)
    if os.path.exists(log):
        print(f"{log} exists -- move it first (or pass --log) so calibration data is never overwritten")
        return 1
    print("Schedule (time from START):")
    t = LEAD_S
    for k, lv in enumerate(LEVELS_A):
        print(f"  level {k + 1}: {sign * lv:+.2f} A  from {t:5.1f} s to {t + HOLD_S:5.1f} s "
              f"-> read the scale around {t + HOLD_S - 1:4.1f} s")
        t += HOLD_S + GAP_S
    m = MotorCANInterface(channel="can0", node_base=0x000, pair_dbg=False)
    m.start_listening()
    rows, reason = [], "done"
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
        m.drain_records()
        plan = [(-1, 0.0, LEAD_S)]
        for k, lv in enumerate(LEVELS_A):
            plan += [(k, sign * min(lv, MAX_A), HOLD_S), (-1, 0.0, GAP_S)]
        prev = None
        t_start = time.monotonic()
        for k, amps, dur in plan:
            if k >= 0:
                print(f"  level {k + 1}: {amps:+.2f} A  (t = {time.monotonic() - t_start:5.1f} s)")
            end = time.monotonic() + dur
            enc_level0 = None
            while time.monotonic() < end:
                m.send_set_iq(amps)
                for r in m.drain_records():
                    rows.append((k, amps, r["bus_time"], r["mcu_time_us"], r["enc"], r["iq_meas"]))
                    if prev is not None and r["mcu_time_us"] > prev[0]:
                        v = (r["enc"] - prev[1]) / ((r["mcu_time_us"] - prev[0]) * 1e-6)
                        if abs(v) > ABORT_VEL and abs(r["enc"] - rows[max(0, len(rows) - 20)][4]) > 20:
                            reason = f"ABORT: shaft moving ({v:.0f} counts/s) -- slipped off the scale?"
                            raise RuntimeError(reason)
                    prev = (r["mcu_time_us"], r["enc"])
                    if k >= 0:
                        enc_level0 = r["enc"] if enc_level0 is None else enc_level0
                        if abs(r["enc"] - enc_level0) > ABORT_TRAVEL:
                            reason = (f"ABORT: shaft travelled {r['enc'] - enc_level0:+.0f} counts at "
                                      f"level {k + 1} -- not pressing a rigid stop (wrong --sign?)")
                            raise RuntimeError(reason)
                tel = m.get_telemetry()
                if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
                    reason = "ABORT: stale telemetry"
                    raise RuntimeError(reason)
                time.sleep(0.002)
    except (RuntimeError, KeyboardInterrupt) as e:
        reason = str(e) or "interrupted"
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e}")
        m.stop_listening()
    n_diag = m.save_diag_history(log[:-4] + "_diag.csv")
    if n_diag:
        print(f"DIAG frames: {n_diag} -> {log[:-4]}_diag.csv")
    with open(log, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["level", "iq_cmd", "bus_time", "mcu_time_us", "enc", "iq_meas"])
        w.writerows(rows)
    print(f"{reason}. Log: {os.path.abspath(log)}")
    summarize(log)
    return 0 if reason == "done" else 1


def level_means(path):
    by = {}
    for r in csv.DictReader(open(path)):
        k = int(r["level"])
        if k >= 0:
            by.setdefault(k, []).append((float(r["iq_cmd"]), float(r["iq_meas"]), float(r["enc"])))
    out = []
    for k in sorted(by):
        v = by[k][len(by[k]) // 2:]                  # last half of the hold: settled
        out.append((k, v[0][0], st.mean(x[1] for x in v), st.pstdev(x[1] for x in v),
                    max(x[2] for x in v) - min(x[2] for x in v)))
    return out


def summarize(path):
    print("level  iq_cmd  iq_meas mean (std)   shaft travel in 2nd half")
    for k, c, mi, sd, tr in level_means(path):
        print(f"  {k + 1:3d}  {c:+.2f}   {mi:+.4f} ({sd:.4f})   {tr:.0f} counts")


def fit(path, grams):
    lv = level_means(path)
    if len(grams) != len(lv):
        print(f"{len(grams)} readings for {len(lv)} levels -- give one per completed level")
        return 1
    i = np.array([abs(x[2]) for x in lv])
    F = np.array(grams) * 9.81 / 1000.0
    A = np.vstack([i, np.ones_like(i)]).T
    (k, b), *_ = np.linalg.lstsq(A, F, rcond=None)
    pred = A @ np.array([k, b])
    r2 = 1 - np.sum((F - pred) ** 2) / np.sum((F - F.mean()) ** 2)
    print("level  |iq_meas| A   scale g   force N   fit N")
    for n, (ii, g, f, pf) in enumerate(zip(i, grams, F, pred)):
        print(f"  {n + 1:3d}   {ii:.4f}     {g:7.1f}   {f:.4f}   {pf:.4f}")
    dev = 100 * (k - EXPECT_N_PER_A) / EXPECT_N_PER_A
    ok = r2 >= 0.99 and abs(dev) <= 15
    print(f"[B7] slope {k:.3f} N/A ({dev:+.1f}% vs {EXPECT_N_PER_A} N/A), intercept {b:+.4f} N, "
          f"R^2 {r2:.4f} -> {'PASS' if ok else 'FAIL'} (R^2 >= 0.99, slope within 15 %)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sign", type=int, choices=(1, -1))
    ap.add_argument("--fit")
    ap.add_argument("--grams", type=lambda s: [float(x) for x in s.split(",")])
    ap.add_argument("--log", default=LOG, help="output log for a run (default logs/B7_cal.csv)")
    a = ap.parse_args()
    if a.fit:
        return fit(a.fit, a.grams)
    if a.sign is None:
        ap.error("--sign +1/-1 (the current sign that presses INTO the scale) is required")
    return run(a.sign, a.log)


if __name__ == "__main__":
    sys.exit(main())
