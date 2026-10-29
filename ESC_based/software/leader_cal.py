"""
In-session leader ripple calibration (TEST_LOG 2026-10-26): a hands-off
position sweep of the leader right after teleop's START, fitted into a
LeaderComp model for that session.

Why in-session: the leader's angle-locked torque has
  - 7/rev (electrical): fixed phase, amplitude 8-41 mA, new every START;
  - 12/rev (and 24, 36/rev) at about 33 mA: locked to the stator, so its
    phase in raw counts depends on which of the 7 electrical zeros the
    power-up alignment landed on (moved about 107 deg between power-ups);
  - 14, 28, 42, 84/rev: repeatable.
A fixed map cannot follow the first two; a fit in the same START can.
Offline on four sweeps: angle-locked rms 28-39 mA -> 10-16 mA.

The sweep: PD hold on the leader (KP 1e-3 A/count, KD 1.2e-5, as in
friction_map.py), target walks +REVS then back at SPEED_CNT_S; the
follower is sent 0 A every tick (its 20 ms host timeout would zero it
anyway). Fit on the constant-speed frames:

    i = c0 + f sign(v) + sum_h [a_h cos(h th) + b_h sin(h th)]

Abort (returns None) on lag > MAX_LAG, |v| > MAX_VEL or stale telemetry.
"""

import csv
import json
import math
import time

import numpy as np

from estimators import AlphaBetaVelocity

ENC = 4000
HARMONICS = (7, 12, 14, 24, 28, 36, 42, 84)
RATE_HZ = 500
KP = 1e-3
KD = 1.2e-5
IQ_MAX_A = 0.3
SPEED_CNT_S = 300.0
REVS = 1.0
ACCEL_S = 0.5
PAUSE_S = 0.5
MAX_LAG = 300
MAX_VEL = 8000.0
STALE_S = 0.05


def _target(tau, dist, v):
    a = v / ACCEL_S
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


def sweep(b, a, log_path=None, print_fn=print):
    """Run the sweep on leader interface b (follower a held at 0 A).
    Returns a list of (mcu_us, enc, iq_meas, v_ref) or None on abort."""
    vel = AlphaBetaVelocity(55.0)
    a.drain_records()
    recs = b.drain_records()
    if not recs:
        print_fn("leader cal: ABORT, no leader frames")
        return None
    x0 = x = recs[-1]["enc"]
    v = 0.0
    dist = REVS * ENC
    T = dist / SPEED_CNT_S + ACCEL_S
    rows, iq = [], 0.0
    base = 0.0
    dt = 1.0 / RATE_HZ
    try:
        for sgn, dur in ((+1, T), (0, PAUSE_S), (-1, T), (0, PAUSE_S)):
            t_seg = deadline = time.monotonic()
            while True:
                now = time.monotonic()
                tau = now - t_seg
                if tau >= dur:
                    break
                if sgn:
                    px, pv = _target(tau, dist, SPEED_CNT_S)
                    x_ref, v_ref = base + sgn * px, sgn * pv
                else:
                    x_ref, v_ref = base, 0.0
                for r in b.drain_records():
                    x = r["enc"]
                    v = vel.update(r["mcu_time_us"], x)
                    rows.append((r["mcu_time_us"], x, r["iq_meas"], v_ref))
                a.drain_records()
                tel = b.get_telemetry()
                if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > STALE_S:
                    print_fn("leader cal: ABORT, stale leader telemetry")
                    return None
                lag = x_ref - (x - x0)
                if abs(lag) > MAX_LAG or abs(v) > MAX_VEL:
                    print_fn(f"leader cal: ABORT, lag {lag:+.0f} counts / speed {v:.0f} counts/s")
                    return None
                iq = max(-IQ_MAX_A, min(IQ_MAX_A, KP * lag + KD * (v_ref - v)))
                b.send_set_iq(iq)
                a.send_set_iq(0.0)
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            if sgn:
                base += sgn * dist
    finally:
        b.send_set_iq(0.0)
        a.send_set_iq(0.0)
        if log_path:
            with open(log_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["mcu_time_us", "enc", "iq_meas", "v_ref"])
                w.writerows(rows)
    return rows


def fit(rows, v0=300.0):
    """LeaderComp model dict from sweep rows (constant-speed frames only)."""
    d = np.array(rows, dtype=float)
    m = np.abs(np.abs(d[:, 3]) - SPEED_CNT_S) < 1.0
    th = 2 * np.pi * (d[m, 1] % ENC) / ENC
    s = np.sign(d[m, 3])
    X = np.vstack([np.ones(m.sum()), s] + sum(([np.cos(h * th), np.sin(h * th)] for h in HARMONICS), [])).T
    c, *_ = np.linalg.lstsq(X, d[m, 2], rcond=None)
    harm = {str(h): [float(c[2 + 2 * k]), float(c[3 + 2 * k])] for k, h in enumerate(HARMONICS)}
    resid = d[m, 2] - X @ c
    k25 = np.ones(25) / 25
    sm = lambda y: np.convolve(y, k25, "same")
    locked = d[m, 2] - c[0] - c[1] * s
    return {"harmonics_A": harm, "coulomb_A": float(c[1]), "v0_cnt_s": v0,
            "frames": int(m.sum()),
            "locked_rms_A": float(np.std(sm(locked))), "resid_rms_A": float(np.std(sm(resid))),
            "note": "in-session leader_cal fit; amps to overcome the disturbance; count = leader raw"}


def summary(model):
    amp = {h: math.hypot(*ab) for h, ab in model["harmonics_A"].items()}
    top = sorted(amp, key=amp.get, reverse=True)[:4]
    return (f"{model['frames']} frames; " + ", ".join(f"{h}/rev {amp[h] * 1e3:.1f} mA" for h in top)
            + f"; Coulomb {model['coulomb_A'] * 1e3:.1f} mA; locked rms {model['locked_rms_A'] * 1e3:.1f} -> "
              f"{model['resid_rms_A'] * 1e3:.1f} mA after fit")


if __name__ == "__main__":
    # offline: fit a saved cal CSV (or a friction_map log via --fm log:start_raw)
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="a *_cal.csv written by teleop's leader cal")
    ap.add_argument("--out")
    a_ = ap.parse_args()
    rows = [(float(r["mcu_time_us"]), float(r["enc"]), float(r["iq_meas"]), float(r["v_ref"]))
            for r in csv.DictReader(open(a_.csv))]
    m = fit(rows)
    print(summary(m))
    if a_.out:
        with open(a_.out, "w") as f:
            json.dump(m, f, indent=1)
