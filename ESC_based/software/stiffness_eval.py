"""
Rigid-vs-soft stiffness from teleop logs (D2 preview, TEST_LOG 2026-10-29).

    python3 stiffness_eval.py RIGID.csv [SOFT.csv] [--side +1|-1]

Per press (tau_env_N on the --side sign above FMIN for > 0.3 s), using
quasi-static frames only (|vel_a| < VMAX counts/s): follower travel between
the median position at low force (< F_LO) and at high force (> F_HI), and
k = dF / travel in N/mm at the tip of the 3D-printed arm (r = 75.7 mm,
0.119 mm/count). Per press, not pooled, so an object that shifts between
presses does not inflate the travel. Travel <= 1 count is encoder-limited
(k is then a lower bound). The ratio uses the per-press medians.
"""

import argparse
import csv

import numpy as np

MM_PER_COUNT = 2 * np.pi * 75.7 / 4000
VMAX = 300.0
FMIN = 0.15
F_LO, F_HI = 0.3, 0.7
MIN_PRESS_S = 0.3


def presses(path, side):
    r = list(csv.DictReader(open(path)))
    t, pa, va, f = (np.array([float(x[k]) for x in r]) for k in ("t_s", "pos_a", "vel_a", "tau_env_N"))
    t = t - t[0]
    f = side * f
    on = f > FMIN
    out, s = [], None
    for i, v in enumerate(np.append(on, False)):
        if v and s is None:
            s = i
        elif not v and s is not None:
            if t[i - 1] - t[s] > MIN_PRESS_S:
                F, P, q = f[s:i], pa[s:i], abs(va[s:i]) < VMAX
                lo, hi = q & (F < F_LO), q & (F > F_HI)
                if lo.sum() >= 10 and hi.sum() >= 10:
                    d = abs(np.median(P[hi]) - np.median(P[lo]))
                    df = np.median(F[hi]) - np.median(F[lo])
                    out.append((t[s], np.median(P[lo]), d, df, df / (max(d, 1) * MM_PER_COUNT)))
            s = None
    return out


def evaluate(path, side):
    ps = presses(path, side)
    print(f"{path} (side {side:+d}): {len(ps)} presses with quasi-static frames below {F_LO} and above {F_HI} N")
    for t0, p0, d, df, k in ps:
        print(f"  {t0:5.1f} s  contact at {p0:6.0f}  travel {d:3.0f} counts ({d * MM_PER_COUNT:.2f} mm) "
              f"for {df:.2f} N -> {k:5.2f} N/mm" + ("  (>=, encoder-limited)" if d <= 1 else ""))
    if not ps:
        return None
    k = float(np.median([p[4] for p in ps]))
    print(f"  median: travel {np.median([p[2] for p in ps]):.0f} counts, k {k:.2f} N/mm")
    return k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--side", type=int, choices=(1, -1), default=1)
    a = ap.parse_args()
    ks = [evaluate(p, a.side) for p in a.logs]
    if len(ks) == 2 and all(ks):
        print(f"ratio first/second = {ks[0] / ks[1]:.1f}x (D2 pass: rigid >= 5x soft)")


if __name__ == "__main__":
    main()
