"""
Leader-side contact metrics from teleop logs (option A, TEST_LOG 2026-10-29).

    python3 leader_travel_eval.py LOG.csv [LOG.csv ...]

Per press (tau_env_N > 0.15 for > 0.3 s, reaching 0.7 N): leader and
follower travel from 0.15 to 0.7 N and the time it took; leader velocity
sign flips above 200 counts/s while in contact (ringing), per press; the
leader spring term (iq_b_spring) if logged. Medians over presses.
"""

import csv
import sys

import numpy as np

F_ON, F_FULL, MIN_S, V_FLIP = 0.15, 0.7, 0.3, 200.0


def evaluate(path):
    r = list(csv.DictReader(open(path)))
    cols = r[0].keys()
    g = lambda k: np.array([float(x[k]) if x[k] != "" else 0.0 for x in r])
    t, pa, pb, vb, f = g("t_s"), g("pos_a"), g("pos_b"), g("vel_b"), g("tau_env_N")
    t = t - t[0]
    spring = g("iq_b_spring") if "iq_b_spring" in cols else None
    on, s, rows = f > F_ON, None, []
    for i, v in enumerate(np.append(on, False)):
        if v and s is None:
            s = i
        elif not v and s is not None:
            if t[i - 1] - t[s] > MIN_S:
                j = np.flatnonzero(f[s:i] > F_FULL)
                if len(j):
                    k = s + j[0]
                    sg = np.sign(vb[s:i][abs(vb[s:i]) > V_FLIP])
                    flips = int(np.sum(sg[1:] != sg[:-1])) if len(sg) > 1 else 0
                    sp = abs(spring[s:i]).max() if spring is not None else float("nan")
                    rows.append((abs(pb[k] - pb[s]), abs(pa[k] - pa[s]), t[k] - t[s], flips, sp))
            s = None
    if not rows:
        print(f"{path}: no presses reaching {F_FULL} N")
        return
    a = np.array(rows)
    print(f"{path}: {len(a)} presses | leader {np.median(a[:, 0]):.0f} counts ({np.median(a[:, 0]) * 0.09:.1f} deg), "
          f"follower {np.median(a[:, 1]):.0f} counts, {np.median(a[:, 2]) * 1e3:.0f} ms | "
          f"leader vel sign flips per press median {np.median(a[:, 3]):.0f} (max {a[:, 3].max():.0f})"
          + (f" | leader spring peak median {np.median(a[:, 4]) * 1e3:.0f} mA" if spring is not None else ""))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        evaluate(p)
