"""
Plan B6: fit the follower ripple model and test it on held-out logs,
estimate side only (the motor command is never touched).

    python3 b6_ripple.py            # fit, save logs/ripple_model_A.json, evaluate

Fit (sample level, every TELEM frame, moving frames only):
    r = iq_meas - a/G_A  regressed on  [1, sign(v), v]           (nuisance:
                                        constant + Coulomb + viscous friction)
                                      + RippleModel columns       (the model)
  a from a centred 15-frame quadratic LS kernel on mcu_time_us
  (Savitzky-Golay 2nd derivative), v from the 1st-derivative kernel.

Test: estimators.ExternalTorqueObserver (20 Hz, G_A from A2) run frame by
frame with AB 55 Hz velocity, with and without the ripple model, on runs
NOT used in the fit. The ripple left in its output is measured by
regressing the output on cos/sin(n theta) per harmonic.

Pass (brief B6): >= 60 % lower 14/rev amplitude on held-out free-space logs.
"""

import csv

import numpy as np

from estimators import AlphaBetaVelocity, ExternalTorqueObserver, RippleModel

G_A = 9_025_000
MOVING_MIN = 1500.0       # counts/s; below this the A4 data is stick-slip
HALF = 7                  # 15-frame kernel
MODEL_PATH = "logs/ripple_model_A.json"
REPORT_H = (12, 14, 24, 28, 36, 42, 84)

TRAIN = [("unl +4k", "logs/spin_A_+4000_0.3A_kv1.5e-05_ki0.0002_coast3.csv", None),
         ("unl -4k", "logs/spin_A_-4000_0.3A_kv1.5e-05_ki0.0002_coast3.csv", None),
         ("load +4k", "logs/spin_A_+4000_0.45A_kv1.5e-05_ki0.0002.csv", (0, 12)),
         ("load -4k", "logs/spin_A_-4000_0.45A_kv1.5e-05_ki0.0002.csv", (0, 4))]
HELDOUT = [("unl +10k", "logs/spin_A_+10000_0.3A_kv1.5e-05_ki0.0002_coast3.csv", None),
           ("unl -10k", "logs/spin_A_-10000_0.3A_kv1.5e-05_ki0.0002_coast3.csv", None),
           ("unl +4k lowgain", "logs/spin_A_+4000_0.3A_kv7.5e-06_ki0.0001.csv", None)]


def load(path, window):
    t, p, i = [], [], []
    for r in csv.DictReader(open(path)):
        if r["phase"] == "hold":
            t.append(float(r["mcu_time_us"]) * 1e-6)
            p.append(float(r["enc"]))
            i.append(float(r["iq_meas"]))
    t, p, i = map(np.array, (t, p, i))
    if window:
        tt = t - t[0]
        s = (tt >= window[0]) & (tt < window[1])
        t, p, i = t[s], p[s], i[s]
    return t, p, i


def derivs(t, p):
    """Centred quadratic LS fit over 2*HALF+1 frames: velocity and acceleration."""
    k = np.arange(-HALF, HALF + 1)
    V = np.vstack([np.ones_like(k), k, k * k]).T.astype(float)
    P = np.linalg.pinv(V)                      # rows: c0, c1, c2 per sample offset
    dt = np.median(np.diff(t))
    v = np.convolve(p, P[1][::-1], mode="same") / dt
    a = np.convolve(p, 2 * P[2][::-1], mode="same") / dt ** 2
    v[:HALF] = v[-HALF:] = np.nan
    a[:HALF] = a[-HALF:] = np.nan
    return v, a


def harm_amp(y, count, n):
    th = 2 * np.pi * (count % 4000) / 4000
    X = np.vstack([np.ones_like(th), np.cos(n * th), np.sin(n * th)]).T
    s, *_ = np.linalg.lstsq(X, y - y.mean(), rcond=None)
    return float(np.hypot(s[1], s[2]))


def main():
    model = RippleModel()
    Xs, ys = [], []
    for name, path, win in TRAIN:
        t, p, i = load(path, win)
        v, a = derivs(t, p)
        ok = np.isfinite(a) & (np.abs(v) > MOVING_MIN)
        r = i[ok] - a[ok] / G_A
        nuis = np.vstack([np.ones(ok.sum()), np.sign(v[ok]), v[ok] / 1e4]).T
        Xs.append(np.hstack([nuis, model.columns(p[ok], i[ok])]))
        ys.append(r)
        print(f"train {name:9s}: {ok.sum():6d} moving frames, mean i {i[ok].mean():+.3f} A")
    X, y = np.vstack(Xs), np.concatenate(ys)
    sol, *_ = np.linalg.lstsq(X, y, rcond=None)
    model.coeffs = list(sol[3:])
    resid = y - X @ sol
    r2_model = 1 - np.var(y - X[:, :3] @ sol[:3] - X[:, 3:] @ sol[3:]) / np.var(y - X[:, :3] @ sol[:3])
    print(f"nuisance: const {sol[0]*1e3:+.1f} mA, Coulomb {sol[1]*1e3:+.1f} mA, "
          f"viscous {sol[2]*1e3:+.2f} mA per 10k counts/s")
    print(f"model share of r variance after nuisance (sample level, incl. noise): {100*r2_model:.1f}%")
    print("fitted amplitudes (mA, or mA/A for g):",
          ", ".join(f"{k} {v*1e3:.1f}" for k, v in model.amplitudes().items()))
    model.save(MODEL_PATH, meta={"G_A": G_A, "train": [x[1] for x in TRAIN],
                                 "moving_min": MOVING_MIN, "kernel_frames": 2 * HALF + 1})
    print(f"saved {MODEL_PATH}")

    print("\nObserver output ripple (mA), without -> with ripple model; reduction:")
    print(f"  {'run':16s} " + " ".join(f"{n:>17d}" for n in REPORT_H))
    verdict = []
    for tag, runs in (("held-out", HELDOUT), ("train", TRAIN)):
        for name, path, win in runs:
            t, p, i = load(path, win)
            outs = {}
            for use in (False, True):
                ob = ExternalTorqueObserver(G_A, 20.0, model if use else None)
                ve = AlphaBetaVelocity(55.0)
                tau = np.empty(len(t))
                for k in range(len(t)):
                    vv = ve.update(t[k] * 1e6, p[k])
                    dt = t[k] - t[k - 1] if k else 0.001
                    tau[k] = ob.update(vv, i[k], dt, p[k])
                outs[use] = tau[200:]
            cnt = p[200:]
            cells = []
            for n in REPORT_H:
                a0, a1 = harm_amp(outs[False], cnt, n), harm_amp(outs[True], cnt, n)
                red = 1 - a1 / a0 if a0 else float("nan")
                cells.append(f"{a0*1e3:5.1f}->{a1*1e3:5.1f} {100*red:4.0f}%")
                if n == 14 and tag == "held-out":
                    verdict.append((name, red))
            rms0, rms1 = np.std(outs[False]), np.std(outs[True])
            print(f"  {tag[:4]} {name:11s} " + " ".join(f"{c:>17s}" for c in cells)
                  + f"   rms {rms0*1e3:.1f}->{rms1*1e3:.1f}")
    worst = min(r for _, r in verdict)
    print(f"\n[B6] 14/rev reduction on held-out runs: "
          + ", ".join(f"{n} {100*r:.0f}%" for n, r in verdict)
          + f"  -> {'PASS' if worst >= 0.6 else 'FAIL'} (>= 60 %)")


if __name__ == "__main__":
    main()
