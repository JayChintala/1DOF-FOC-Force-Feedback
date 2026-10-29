"""
Quasi-static friction / angle-torque map of one free-spinning motor:
slow position sweep under a stiff PD hold.

    python3 friction_map.py --node B            # run (about 50 s)
    python3 friction_map.py --analyze logs/friction_map_B.csv

The position target walks +REVS revolutions at SWEEP_CNT_S, pauses, then
walks back. A PD loop (current = KP*(x_ref - x) + KD*(v_ref - v)) makes
the shaft follow; at constant slow speed the current it needs is the
load at that angle:

    i_fwd(th) = r(th) + f,   i_back(th) = r(th) - f

so the mean of the two directions is the angle-locked torque r(th) (cogging,
ripple from current-sensor offsets, a rub) and half their difference is
the friction f. Analysis bins by angle and fits harmonics (1, 2, 7, 14,
21, 28 and 84/rev).

Why (TEST_LOG 2026-10-26): spin_test with integral action stick-slipped
on the leader -- crept at 30-110 mA near counts 3970-110 and 600-700,
free-ran at 0-7 mA in between -- and the leader has no unbalanced load.

Gains: KP 1e-3 A/count, KD 1.2e-5 A/(count/s) on the leader's G of about
7.9e6 counts/s^2 per A give about 14 Hz with damping about 0.5 (inferred),
holding 0.1 A of load with about 100 counts of lag.

Safety: current clamped to IQ_MAX_A; abort if |x_ref - x| > MAX_LAG,
|v| > MAX_VEL, telemetry goes stale, or the node looks reset.
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
KP = 1e-3
KD = 1.2e-5
IQ_MAX_A = 0.3
SWEEP_CNT_S = 200.0
REVS = 1.1
ACCEL_S = 0.5          # target speed ramp at each end of a sweep
PAUSE_S = 1.0
MAX_LAG = 300
MAX_VEL = 8000.0       # stick-slip bursts reach ~4000 counts/s on the leader (run 1 aborted at 3000)
ENC = 4000
HARMONICS = (1, 2, 7, 14, 21, 28, 84)


def target(tau, dist):
    """Trapezoidal move of `dist` counts at SWEEP_CNT_S: (x, v) at tau seconds."""
    v, a = SWEEP_CNT_S, SWEEP_CNT_S / ACCEL_S
    T = dist / v + ACCEL_S
    if tau <= 0:
        return 0.0, 0.0
    if tau < ACCEL_S:
        return 0.5 * a * tau ** 2, a * tau
    if tau < T - ACCEL_S:
        return 0.5 * a * ACCEL_S ** 2 + v * (tau - ACCEL_S), v
    if tau < T:
        r = T - tau
        return dist - 0.5 * a * r ** 2, a * r
    return dist, 0.0


def run(node):
    os.makedirs("logs", exist_ok=True)
    path = os.path.join("logs", f"friction_map_{node}.csv")
    n = 2
    while os.path.exists(path):
        path = os.path.join("logs", f"friction_map_{node}_{n}.csv")
        n += 1
    m = MotorCANInterface(channel="can0", node_base=NODES[node], pair_dbg=False)
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
        vel = AlphaBetaVelocity(55.0)
        recs = m.drain_records()
        if not recs:
            print("ABORT: no telemetry frames")
            return 1
        x = x0 = recs[-1]["enc"]
        v = 0.0
        dist = REVS * ENC
        T_move = dist / SWEEP_CNT_S + ACCEL_S
        segs = [("fwd", +1, T_move), ("pause", 0, PAUSE_S), ("back", -1, T_move), ("end", 0, PAUSE_S)]
        dt = 1.0 / RATE_HZ
        base, iq = 0.0, 0.0
        for name, sgn, dur in segs:
            t_seg = time.monotonic()
            deadline = t_seg
            print(f"{name} ({dur:.1f} s)")
            while True:
                now = time.monotonic()
                tau = now - t_seg
                if tau >= dur:
                    break
                if sgn:
                    px, pv = target(tau, dist)
                    x_ref, v_ref = base + sgn * px, sgn * pv
                else:
                    x_ref, v_ref = base, 0.0
                for r in m.drain_records():
                    x = r["enc"]
                    v = vel.update(r["mcu_time_us"], x)
                    rows.append((r["mcu_time_us"], x - x0, r["iq_meas"], iq, x_ref, v_ref, name))
                tel = m.get_telemetry()
                if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
                    reason = "ABORT: stale telemetry"
                elif abs(x_ref - (x - x0)) > MAX_LAG or abs(v) > MAX_VEL:
                    reason = f"ABORT: lag {x_ref - (x - x0):+.0f} counts / speed {v:.0f} counts/s"
                if reason != "done":
                    break
                iq = max(-IQ_MAX_A, min(IQ_MAX_A, KP * (x_ref - (x - x0)) + KD * (v_ref - v)))
                m.send_set_iq(iq)
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            if reason != "done":
                break
            if sgn:
                base += sgn * dist
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
        w.writerow(["mcu_time_us", "rel", "iq_meas", "iq_cmd", "x_ref", "v_ref", "seg"])
        w.writerows(rows)
    print(f"{reason}. {len(rows)} frames. Log: {os.path.abspath(path)}  (rel = counts from start; "
          f"start raw {x0 % ENC:.0f})")
    if rows and reason == "done":
        analyze(path, x0 % ENC)
    return 0 if reason == "done" else 1


def analyze(path, start_raw=None, nbins=80):
    rows = list(csv.DictReader(open(path)))
    if start_raw is None:
        start_raw = 0.0
        print("(angles relative to the run's start position)")
    rel = np.array([float(r["rel"]) for r in rows])
    i = np.array([float(r["iq_meas"]) for r in rows])
    vr = np.array([float(r["v_ref"]) for r in rows])
    seg = np.array([r["seg"] for r in rows])
    th = (rel + start_raw) % ENC
    steady = np.abs(np.abs(vr) - SWEEP_CNT_S) < 1.0
    edges = np.linspace(0, ENC, nbins + 1)
    out = {}
    for name in ("fwd", "back"):
        sel = steady & (seg == name)
        b = np.digitize(th[sel], edges) - 1
        out[name] = np.array([i[sel][b == k].mean() if np.any(b == k) else np.nan for k in range(nbins)])
    r_th = 0.5 * (out["fwd"] + out["back"])
    f_th = 0.5 * (out["fwd"] - out["back"])
    print(f"{path}: {steady.sum()} steady frames")
    print(f"  friction (half fwd-back difference): mean {np.nanmean(f_th) * 1e3:.1f} mA, "
          f"range {np.nanmin(f_th) * 1e3:.1f}..{np.nanmax(f_th) * 1e3:.1f} mA over angle")
    print(f"  angle-locked torque (fwd/back mean): mean {np.nanmean(r_th) * 1e3:+.1f} mA, "
          f"peak-peak {(np.nanmax(r_th) - np.nanmin(r_th)) * 1e3:.1f} mA, rms about mean "
          f"{np.nanstd(r_th) * 1e3:.1f} mA")
    # harmonic fit on the raw steady frames, both directions, friction removed
    sel = steady & np.isin(seg, ("fwd", "back"))
    y = i[sel] - np.sign(vr[sel]) * np.nanmean(f_th)
    ang = 2 * np.pi * th[sel] / ENC
    cols = [np.ones(sel.sum())]
    for h in HARMONICS:
        cols += [np.cos(h * ang), np.sin(h * ang)]
    X = np.vstack(cols).T
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    var = np.var(y)
    print("  harmonic amplitudes (mA) and share of variance:")
    for k, h in enumerate(HARMONICS):
        a = np.hypot(coef[1 + 2 * k], coef[2 + 2 * k])
        print(f"    {h:3d}/rev  {a * 1e3:5.1f} mA   {100 * 0.5 * a * a / var:4.1f} %")
    print("  angle (raw counts)  fwd mA  back mA  locked mA  friction mA")
    for k in range(0, nbins, 4):
        print(f"    {edges[k]:5.0f}          {out['fwd'][k] * 1e3:6.1f}  {out['back'][k] * 1e3:6.1f}  "
              f"{r_th[k] * 1e3:+7.1f}   {f_th[k] * 1e3:6.1f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", choices=NODES, default="B")
    ap.add_argument("--analyze")
    ap.add_argument("--start-raw", type=float, help="raw encoder count at the run's start (printed by the run)")
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze, a.start_raw)
        return 0
    return run(a.node)


if __name__ == "__main__":
    sys.exit(main())
