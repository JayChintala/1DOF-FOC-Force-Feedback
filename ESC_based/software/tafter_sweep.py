"""
CHANGE 7 test: stall steps on the follower at several sampling delays
(Tafter), in one START. TEST_LOG 2026-10-28 ("current breakdown").

    python3 tafter_sweep.py [--values 807,1360,2040] [--levels 0.3,0.45,0.6,0.8] [--window 40,70|off]

The user holds the follower arm still at a "bad" rotor angle (electrical
~40-70 deg, where measured Iq broke down above ~0.55 A). Before START a
passive guide prints the live electrical angle and the distance to the
window, and starts only after the arm has stayed inside it for 2 s. For
each Tafter value: SET_PARAM, wait for the DIAG2 readback to confirm it,
then hold each current level HOLD_S (resent every 2 ms) with GAP_S at 0 A
between. Every TELEM frame is logged with (tafter, level); DIAG and DIAG2
are saved alongside. Tafter is set back to can_interface.DEFAULT_TAFTER
before STOP. The firmware lowers MaxModule with larger Tafter (85 % at 1360, 70.5 % at 2040), so the
top level can be voltage-limited at 2040 -- watch the circle count.

Summary per (Tafter, level), over the second half of each hold: measured
Iq mean/std vs command, DIAG2 Vq before limitation, circle-limited and
non-forced period counts (per ~1600-period frame).
Pass (TEST_LOG CHANGE 7): within 2 % of command up to 0.8 A, std <= 25 mA.

Safety as b7_force_cal: |I| <= 0.8 A; abort if the shaft moves faster than
ABORT_VEL or travels more than ABORT_TRAVEL within a level (hand slipped),
telemetry goes stale, or the node looks reset; zero + default Tafter + STOP
on any exit.
"""

import argparse
import csv
import os
import statistics as st
import sys
import time

import numpy as np

from can_interface import DEFAULT_TAFTER, MotorCANInterface
from spin_test import RESET_IQ_A

HOLD_S = 4.0
GAP_S = 2.0
LEAD_S = 2.0
MAX_A = 0.8
ABORT_VEL = 2000.0
ABORT_TRAVEL = 100
RATE_HZ = 500
COUNTS_PER_ELEC = 4000 / 7          # 7 pole pairs, 4000 counts/rev
TIP_MM_PER_COUNT = 2 * np.pi * 75.7 / 4000   # 3D-printed arm, r = 75.7 mm
GUIDE_HOLD_S = 2.0


def elec_deg(enc):
    return (enc % COUNTS_PER_ELEC) / COUNTS_PER_ELEC * 360


def guide(m, lo, hi):
    """Passive (nothing energized): print the follower's electrical angle
    live until the arm has stayed inside [lo, hi] deg for GUIDE_HOLD_S.
    The window repeats 7 times per rev; the hint points to the nearest one."""
    centre = 0.5 * (lo + hi)
    tty = sys.stdout.isatty()
    print(f"Hold the follower arm near the tip so the angle reads {lo:.0f}-{hi:.0f} deg electrical "
          f"(window {(hi - lo) / 360 * COUNTS_PER_ELEC:.0f} counts = "
          f"{(hi - lo) / 360 * COUNTS_PER_ELEC * TIP_MM_PER_COUNT:.0f} mm at the tip; "
          f"repeats every 1/7 rev). Ctrl-C to quit.")
    t_in, t_print = None, 0.0
    while True:
        enc = m.get_telemetry()["enc_count_unwrapped"]
        now = time.monotonic()
        if enc is None:
            time.sleep(0.05)
            continue
        ang = elec_deg(enc)
        inside = lo <= ang <= hi
        t_in = (t_in or now) if inside else None
        if inside:
            msg = f"IN WINDOW   hold still {now - t_in:3.1f}/{GUIDE_HOLD_S:.0f} s"
        else:
            d = (centre - ang + 180) % 360 - 180      # deg elec to the centre
            c = d / 360 * COUNTS_PER_ELEC
            msg = f"out: move {c:+5.0f} counts ({abs(c) * TIP_MM_PER_COUNT:4.1f} mm at tip); watch the number shrink"
        line = f"enc {enc:7d}  elec {ang:5.1f} deg  {msg}"
        if tty:
            print("\r" + line.ljust(100), end="", flush=True)
        elif now - t_print > 0.5:
            print(line, flush=True)
            t_print = now
        if inside and now - t_in >= GUIDE_HOLD_S:
            print(f"\nHeld at {ang:.0f} deg. Keep holding: START in 1 s, then the current "
                  "steps begin (push back, don't let it turn).", flush=True)
            time.sleep(1.0)
            return ang
        time.sleep(0.05)


def run(values, levels, sign, stem, window):
    m = MotorCANInterface(channel="can0", node_base=0x000, pair_dbg=False)
    m.start_listening()
    rows, reason = [], "done"
    try:
        time.sleep(0.5)
        idle = [r["iq_meas"] for r in m.drain_records()]
        if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
            print("ABORT: follower missing or reset (run zero_iq_check.py first)")
            return 1
        if window:
            guide(m, *window)
        m.send_start()
        t0 = time.monotonic()
        while not m.get_debug().get("run"):
            if time.monotonic() - t0 > 5:
                print("ABORT: no RUN")
                return 1
            m.send_set_iq(0.0)
            time.sleep(0.01)
        m.drain_records()
        dt = 1.0 / RATE_HZ
        for tafter in values:
            m.send_tafter(tafter)
            t_set = time.monotonic()
            while True:
                m.send_set_iq(0.0)
                m.drain_records()
                d2 = m.get_diag()["diag2"]
                if d2 and d2["tafter"] == tafter and d2["time"] > time.time() - 0.2:
                    break
                if time.monotonic() - t_set > 1.5:
                    reason = f"ABORT: Tafter {tafter} not confirmed by DIAG2 (got {d2})"
                    break
                time.sleep(dt)
            if reason != "done":
                break
            print(f"Tafter {tafter} counts confirmed")
            plan = [(-1, 0.0, LEAD_S)]
            for k, lv in enumerate(levels):
                plan += [(k, sign * lv, HOLD_S), (-1, 0.0, GAP_S)]
            for k, iq, dur in plan:
                t_seg = deadline = time.monotonic()
                x0 = None
                while time.monotonic() - t_seg < dur:
                    for r in m.drain_records():
                        if x0 is None:
                            x0 = r["enc"]
                        rows.append((tafter, k, iq, r["bus_time"], r["mcu_time_us"], r["enc"], r["iq_meas"]))
                        if k >= 0 and abs(r["enc"] - x0) > ABORT_TRAVEL:
                            reason = f"ABORT: shaft moved {r['enc'] - x0:+.0f} counts (level {k + 1})"
                    tel = m.get_telemetry()
                    if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
                        reason = "ABORT: stale telemetry"
                    if len(rows) > 20:
                        e = [r[5] for r in rows[-20:]]
                        if abs(e[-1] - e[0]) / 0.02 > ABORT_VEL:
                            reason = "ABORT: shaft moving fast (hand slipped?)"
                    if reason != "done":
                        break
                    m.send_set_iq(max(-MAX_A, min(MAX_A, iq)))
                    deadline += dt
                    slack = deadline - time.monotonic()
                    if slack > 0:
                        time.sleep(slack)
                    else:
                        deadline = time.monotonic()
                if reason != "done":
                    break
            if reason != "done":
                break
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.02)
            m.send_tafter(DEFAULT_TAFTER)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e}")
        m.stop_listening()
    m.save_diag_history(stem + "_diag.csv")
    m.save_diag2_history(stem + "_diag2.csv")
    with open(stem + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tafter", "level", "iq_cmd", "bus_time", "mcu_time_us", "enc", "iq_meas"])
        w.writerows(rows)
    print(f"{reason}. {len(rows)} frames. Log: {os.path.abspath(stem)}.csv (+ _diag, _diag2)")
    if rows:
        summarize(stem)
    return 0 if reason == "done" else 1


def summarize(stem):
    rows = list(csv.DictReader(open(stem + ".csv")))
    d2 = list(csv.DictReader(open(stem + "_diag2.csv"))) if os.path.exists(stem + "_diag2.csv") else []
    d2t = np.array([float(r["rx_time"]) for r in d2])
    enc = np.array([float(r["enc"]) for r in rows])
    print(f"electrical angle (encoder-relative): {elec_deg(np.mean(enc)):.0f} deg")
    print("Tafter  cmd     meas mean (std)   err %  elec | DIAG2: Vq pre-limit %full  circle/frame  non-forced/frame")
    keys = sorted({(int(r["tafter"]), int(r["level"])) for r in rows if int(r["level"]) >= 0})
    for ta, k in keys:
        sel = [r for r in rows if int(r["tafter"]) == ta and int(r["level"]) == k]
        bt = np.array([float(r["bus_time"]) for r in sel])
        h = bt >= bt.min() + 0.5 * (bt.max() - bt.min())
        i = np.array([float(r["iq_meas"]) for r in sel])[h]
        cmd = float(sel[0]["iq_cmd"])
        e = np.array([float(r["enc"]) for r in sel])[h]
        line = (f"{ta:6d}  {cmd:+.2f}  {i.mean():+.3f} ({i.std() * 1e3:5.1f} mA)  "
                f"{100 * (abs(i.mean()) / abs(cmd) - 1):+5.1f}  {elec_deg(e.mean()):4.0f}")
        if len(d2t):
            s = (d2t >= bt[h].min()) & (d2t <= bt[h].max())
            if s.any():
                vq = np.array([abs(int(d2[j]["vq_pre"])) for j in np.where(s)[0]])
                nc = np.array([int(d2[j]["n_circle"]) for j in np.where(s)[0]])
                nn = np.array([int(d2[j]["n_nonforced"]) for j in np.where(s)[0]])
                line += f"  | {100 * vq.mean() / 32767:5.1f} %   {nc.mean():7.1f}   {nn.mean():9.1f}"
        print(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--values", default="807,1360,2040")
    ap.add_argument("--levels", default="0.3,0.45,0.6,0.8")
    ap.add_argument("--sign", type=int, choices=(1, -1), default=-1)
    ap.add_argument("--window", default="40,70",
                    help="electrical-angle window (deg) the guide waits for before START; 'off' skips it")
    ap.add_argument("--summarize", help="re-print the summary of a saved run (stem, no .csv)")
    a = ap.parse_args()
    if a.summarize:
        summarize(a.summarize)
        return 0
    values = [int(v) for v in a.values.split(",")]
    levels = [float(v) for v in a.levels.split(",")]
    if any(not 400 <= v <= 2040 for v in values) or any(not 0 < l <= MAX_A for l in levels):
        ap.error("Tafter values 400..2040, levels 0 < I <= 0.8 A")
    os.makedirs("logs", exist_ok=True)
    stem = time.strftime("logs/tafter_sweep_%Y%m%d_%H%M%S")
    window = None if a.window == "off" else tuple(float(v) for v in a.window.split(","))
    return run(values, levels, a.sign, stem, window)


if __name__ == "__main__":
    sys.exit(main())
