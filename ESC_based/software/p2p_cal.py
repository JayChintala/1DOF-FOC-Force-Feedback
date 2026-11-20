"""
In-session ripple (cogging) calibration of BOTH motors for P2P mode
(firmware CHANGE 10, couple_ctrl.h "Ripple").

Why: in P2P each motor's disturbance observer reports its angle-locked
ripple (12/24/36-rev slot cogging, 7/rev current-offset ripple, 14..84/rev)
as external force, so the force servo hands both motors' ripple to the
operator. A fixed map cannot follow it (TEST_LOG 2026-10-26: 12/rev phase
moves per power-up, 7/rev amplitude per START), so it is measured right
after START, the same way leader_cal.py does it for teleop, but for both
motors at once. 1/rev is included: on arm A it is mostly gravity, so the
arm's weight stops being reported as force too.

The sweep: each motor independently, PD position hold from the Pi in direct
SET_IQ mode, target walks +REVS then back at SPEED_CNT_S (leader_cal's
trajectory and gains). Both motors turn ~1.25 rev each way, hands off.

Fit per motor (constant-speed frames only):
    i = c0 + Fc*sign(v) + sum_h [a_h cos(h th) + b_h sin(h th)],  th = raw count

Upload: both motors' tables to both ESCs (SET_PARAM 32..50 node 0, 64..82
node 1, microamps), then ripple % (SET_PARAM 8).

Offline re-fit of a saved sweep:  python3 p2p_cal.py logs/<run>_cal.csv
"""

import csv
import json
import math
import sys
import time

import numpy as np

from estimators import AlphaBetaVelocity
from leader_cal import (ACCEL_S, IQ_MAX_A, KD, KP, MAX_LAG, MAX_VEL, PAUSE_S,
                        RATE_HZ, SPEED_CNT_S, STALE_S, _target)

ENC = 4000
# Must match COUPLE_RIPPLE_HARMONICS in couple_ctrl.h, in order.
HARMONICS = (1, 7, 12, 14, 24, 28, 36, 42, 84)
REVS = 1.25                       # > 1 rev so every angle is covered at speed
NODE_OF = {"A": 0, "B": 1}
PARAM_RIPPLE_PCT = 8
PARAM_TABLE_BASE = {0: 32, 1: 64}
COEF_MAX_A = 0.3
FC_MAX_A = 0.2


def sweep_both(ifaces, print_fn=print):
    """Hands-off PD sweep of every motor in ifaces at once (direct mode).
    Returns {name: [(mcu_us, enc, iq_meas, v_ref), ...]} or None on abort."""
    vel = {n: AlphaBetaVelocity(55.0) for n in ifaces}
    x0, x, v, rows = {}, {}, {}, {n: [] for n in ifaces}
    for n, i in ifaces.items():
        recs = i.drain_records()
        if not recs:
            print_fn(f"cal: ABORT, no frames from {n}")
            return None
        x0[n] = x[n] = recs[-1]["enc"]
        v[n] = 0.0
    dist = REVS * ENC
    T = dist / SPEED_CNT_S + ACCEL_S
    dt = 1.0 / RATE_HZ
    base = 0.0
    ok = False
    try:
        for sgn, dur in ((+1, T), (0, PAUSE_S), (-1, T), (0, PAUSE_S)):
            t_seg = deadline = time.monotonic()
            while True:
                tau = time.monotonic() - t_seg
                if tau >= dur:
                    break
                if sgn:
                    px, pv = _target(tau, dist, SPEED_CNT_S)
                    x_ref, v_ref = base + sgn * px, sgn * pv
                else:
                    x_ref, v_ref = base, 0.0
                for n, i in ifaces.items():
                    for r in i.drain_records():
                        x[n] = r["enc"]
                        v[n] = vel[n].update(r["mcu_time_us"], x[n])
                        rows[n].append((r["mcu_time_us"], x[n], r["iq_meas"], v_ref))
                    tel = i.get_telemetry()
                    if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > STALE_S:
                        print_fn(f"cal: ABORT, stale telemetry from {n}")
                        return None
                    lag = x_ref - (x[n] - x0[n])
                    if abs(lag) > MAX_LAG or abs(v[n]) > MAX_VEL:
                        print_fn(f"cal: ABORT, {n} lag {lag:+.0f} counts / speed {v[n]:.0f} "
                                 f"counts/s (shaft blocked or touched?)")
                        return None
                    i.send_set_iq(max(-IQ_MAX_A, min(IQ_MAX_A, KP * lag + KD * (v_ref - v[n]))))
                deadline += dt
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            if sgn:
                base += sgn * dist
        ok = True
    finally:
        for i in ifaces.values():
            i.send_set_iq(0.0)
    return rows if ok else None


def fit(rows):
    """Ripple model for one motor from its sweep rows."""
    d = np.array(rows, dtype=float)
    m = np.abs(np.abs(d[:, 3]) - SPEED_CNT_S) < 1.0
    th = 2 * np.pi * (d[m, 1] % ENC) / ENC
    s = np.sign(d[m, 3])
    X = np.vstack([np.ones(m.sum()), s] +
                  sum(([np.cos(h * th), np.sin(h * th)] for h in HARMONICS), [])).T
    c, *_ = np.linalg.lstsq(X, d[m, 2], rcond=None)
    k25 = np.ones(25) / 25
    sm = lambda y: np.convolve(y, k25, "same")
    locked = d[m, 2] - c[0] - c[1] * s
    resid = d[m, 2] - X @ c
    return {"harmonics": list(HARMONICS),
            "ab_A": [float(x) for x in c[2:]],
            "coulomb_A": float(c[1]), "offset_A": float(c[0]),
            "frames": int(m.sum()),
            "locked_rms_A": float(np.std(sm(locked))),
            "resid_rms_A": float(np.std(sm(resid))),
            "note": "amps to overcome; th = raw count; P2P CHANGE 10"}


def summary(name, model):
    amp = {h: math.hypot(model["ab_A"][2 * k], model["ab_A"][2 * k + 1])
           for k, h in enumerate(model["harmonics"])}
    top = sorted(amp, key=amp.get, reverse=True)[:4]
    return (f"{name}: " + ", ".join(f"{h}/rev {amp[h] * 1e3:.0f} mA" for h in top)
            + f"; Coulomb {model['coulomb_A'] * 1e3:.0f} mA; angle-locked rms "
              f"{model['locked_rms_A'] * 1e3:.0f} -> {model['resid_rms_A'] * 1e3:.0f} mA after fit")


def upload(ifaces, models, ripple_pct):
    """Both motors' tables to every ESC, then the ripple %. Coefficients
    are clipped to the firmware's accepted range (it would reject them)."""
    for i in ifaces.values():
        for name, model in models.items():
            base = PARAM_TABLE_BASE[NODE_OF[name]]
            for k, c in enumerate(model["ab_A"]):
                c = max(-COEF_MAX_A, min(COEF_MAX_A, c))
                i.send_set_param(base + k, int(round(c * 1e6)))
            fc = max(0.0, min(FC_MAX_A, model["coulomb_A"]))
            i.send_set_param(base + 2 * len(HARMONICS), int(round(fc * 1e6)))
        i.send_set_param(PARAM_RIPPLE_PCT, int(ripple_pct))


def save(path_base, rows, models):
    with open(path_base + "_cal.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["motor", "mcu_time_us", "enc", "iq_meas", "v_ref"])
        for n, rs in rows.items():
            w.writerows((n,) + r for r in rs)
    with open(path_base + "_cal.json", "w") as f:
        json.dump(models, f, indent=1)


if __name__ == "__main__":
    rows = {}
    for r in csv.DictReader(open(sys.argv[1])):
        rows.setdefault(r["motor"], []).append(
            (float(r["mcu_time_us"]), float(r["enc"]), float(r["iq_meas"]), float(r["v_ref"])))
    for n, rs in sorted(rows.items()):
        print(summary(n, fit(rs)))
