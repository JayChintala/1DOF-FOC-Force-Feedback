"""
Fit the follower model used by teleop's shadow external-torque estimate (C2)
from a vigorous free-motion every-frame log, and save it as JSON.

    python3 fit_follower_model.py logs/C2_free_arm3d_frames.csv [--out logs/follower_model.json]

Two steps, both validated on 2026-10-25 data (TEST_LOG, "New 3D-printed
follower arm"):
  1. Plant gain G: least squares i = a/G + friction(v) + const on moving
     frames (|v| > MOVING), v and a from a centred 15-frame LS fit of
     position on the ESC clock.
  2. Friction + gravity on the ESTIMATOR OUTPUT: run the same estimator
     teleop runs (ExternalTorqueObserver at 20 Hz with G, AB 55 Hz velocity,
     current delayed ALIGN frames) and regress its motor-side output on
     [1, sign v, sign v * exp(-|v|/VS), v/1e4, cos th, sin th]. Teleop
     subtracts exactly these terms (model_terms()).

Refit whenever the arm or mount changes. Reports a held-out check (fit on
the first half, test on the second).
"""

import argparse
import csv
import json
import math
import sys

import numpy as np

from estimators import AlphaBetaVelocity, ExternalTorqueObserver

ENC = 4000
MOVING = 1500.0
VS = 1500.0
ALIGN = 3
FC = 20.0
HALF = 7


def model_terms(coeffs, v, count):
    """Friction + gravity, motor-side amps, for one sample or arrays."""
    th = 2 * math.pi * (count % ENC) / ENC if np.isscalar(count) else 2 * np.pi * (np.asarray(count) % ENC) / ENC
    s = np.sign(v)
    ex = np.exp(-np.abs(v) / VS)
    c = coeffs
    return (c["offset"] + c["coulomb"] * s + c["lowspeed"] * s * ex + c["viscous_per_10k"] * v / 1e4
            + c["grav_cos"] * np.cos(th) + c["grav_sin"] * np.sin(th))


def load_frames(path, motor="A"):
    t, p, i = [], [], []
    for r in csv.DictReader(open(path)):
        if r["motor"] == motor and int(r["loop_iter"]) >= 0:
            t.append(float(r["mcu_time_us"]) * 1e-6)
            p.append(float(r["enc"]))
            i.append(float(r["iq_meas"]))
    return np.array(t), np.array(p), np.array(i)


def run_estimator(t, p, i, G):
    ob, ve = ExternalTorqueObserver(G, FC), AlphaBetaVelocity(55.0)
    i_d = np.concatenate([np.full(ALIGN, i[0]), i[:-ALIGN]])
    m, v = np.empty(len(t)), np.empty(len(t))
    for k in range(len(t)):
        v[k] = ve.update(t[k] * 1e6, p[k])
        m[k] = ob.update(v[k], i_d[k], (t[k] - t[k - 1]) if k else 1e-3)
    return m, v


def design(v, p):
    th = 2 * np.pi * (p % ENC) / ENC
    s = np.sign(v)
    return np.vstack([np.ones_like(v), s, s * np.exp(-np.abs(v) / VS), v / 1e4, np.cos(th), np.sin(th)]).T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames")
    ap.add_argument("--out", default="logs/follower_model.json")
    a = ap.parse_args()
    t, p, i = load_frames(a.frames)
    k = np.arange(-HALF, HALF + 1)
    P = np.linalg.pinv(np.vstack([np.ones_like(k), k, k * k]).T.astype(float))
    dt = np.median(np.diff(t))
    v = np.convolve(p, P[1][::-1], "same") / dt
    acc = np.convolve(p, 2 * P[2][::-1], "same") / dt ** 2
    s = slice(600, len(t) - 600)
    mv = np.abs(v[s]) > MOVING
    X = np.vstack([acc[s], np.sign(v[s]), v[s] / 1e4, np.ones(len(acc[s]))]).T[mv]
    sol, *_ = np.linalg.lstsq(X, i[s][mv], rcond=None)
    G = 1.0 / sol[0]
    r2 = 1 - np.var(i[s][mv] - X @ sol) / np.var(i[s][mv])
    print(f"plant: G {G:,.0f} counts/s^2 per A (R^2 {r2:.3f}, moving {100 * mv.mean():.0f}% of frames)")

    m, ve = run_estimator(t, p, i, G)
    m, ve, pp = m[500:], ve[500:], p[500:]
    D = design(ve, pp)
    h = len(m) // 2
    c_half, *_ = np.linalg.lstsq(D[:h], m[:h], rcond=None)
    held = m[h:] - D[h:] @ c_half
    c_all, *_ = np.linalg.lstsq(D, m, rcond=None)
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    print(f"estimator output rms {rms(m) * 1e3:.1f} mA; held-out half {rms(m[h:]) * 1e3:.1f} -> "
          f"{rms(held) * 1e3:.1f} mA after friction+gravity (p99 {np.percentile(np.abs(held), 99) * 1e3:.0f} mA)")
    names = ["offset", "coulomb", "lowspeed", "viscous_per_10k", "grav_cos", "grav_sin"]
    coeffs = {n: float(x) for n, x in zip(names, c_all)}
    model = {"G_cnt_s2_per_A": G, "fc_hz": FC, "align_frames": ALIGN, "vs_cnt_s": VS,
             "coeffs_A": coeffs, "source": a.frames, "plant_r2": r2,
             "heldout_rms_A": rms(held), "note": "motor-side amps; tau_env = -(observer - terms)"}
    with open(a.out, "w") as f:
        json.dump(model, f, indent=1)
    print("coeffs (mA): " + ", ".join(f"{n} {v * 1e3:+.1f}" for n, v in coeffs.items()))
    print(f"saved {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
