"""
Score teleop logs for sustained holds and contact-onset ringing
(TEST_LOG 2026-10-26, contact spring).

    python3 hold_eval.py logs/REF_2026-10-26.csv logs/HOLD3_....csv [...]

Per log, over contacts (|tau_env| > 80 mA) lasting >= 0.5 s:
  - plateau slope of |tau_env| (A/s), from 0.25 s after onset to 0.1 s
    before the end. Without the spring held presses faded at -0.3..-0.7.
  - onset ringing in the first 0.3 s: leader-velocity sign flips (|v| >
    300 counts/s) and 5-25 Hz leader-velocity energy.
  - onset peak / plateau, follower impact speed, follower current peak.
Plus the free felt rms (>= 0.3 s from any contact) and loop p95.
"""

import argparse
import csv

import numpy as np

CONTACT_A = 0.08


def score(path):
    rows = list(csv.DictReader(open(path)))
    col = lambda k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])
    t, tau, vb, va, ia, dt = (col(k) for k in ("t_s", "tau_env", "vel_b", "vel_a",
                                                "iq_a_readback", "loop_dt"))
    con = np.abs(tau) > CONTACT_A
    free = np.ones(len(t), bool)
    for k in np.where(con)[0]:
        free[max(0, k - 150):k + 150] = False
    felt = np.sign(tau) * np.maximum(0, np.abs(tau) - 0.02)
    edges = np.diff(np.r_[0, con.astype(int), 0])
    out = []
    for s, e in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        if t[e - 1] - t[s] < 0.5:
            continue
        sg = np.sign(tau[s:e].mean())
        idx = np.arange(s, e)
        pl = idx[(t[idx] > t[s] + 0.25) & (t[idx] < t[e - 1] - 0.1)]
        on = (t >= t[s]) & (t < t[s] + 0.3)
        early = (t >= t[s]) & (t < t[s] + 0.15)
        ref = (t >= t[s] + 0.3) & (t < min(t[e - 1], t[s] + 0.6))
        v = vb[on]
        f = np.fft.rfftfreq(on.sum(), 0.002)
        X = np.fft.rfft((v - v.mean()) * np.hanning(on.sum()))
        band = (f >= 5) & (f <= 25)
        out.append(dict(
            t0=t[s], dur=t[e - 1] - t[s], mean=tau[s:e].mean(),
            slope=np.polyfit(t[pl], np.abs(tau[pl]), 1)[0] if len(pl) > 20 else np.nan,
            flips=int(np.sum(np.diff(np.sign(v[np.abs(v) > 300])) != 0)),
            ring=np.sqrt(np.sum(np.abs(X[band]) ** 2)) / on.sum(),
            over=(sg * tau[early]).max() / (sg * tau[ref]).mean() if ref.sum() > 20 else np.nan,
            impact=np.abs(va[(t >= t[s] - 0.05) & (t < t[s])]).mean(),
            ia_max=np.nanmax(np.abs(ia[s:e]))))
    return dict(free=np.sqrt(np.mean(felt[free] ** 2)), p95=np.percentile(dt, 95), c=out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    for p in a.csv:
        r = score(p)
        c = r["c"]
        med = lambda k: np.nanmedian([x[k] for x in c]) if c else np.nan
        long_ = [x for x in c if x["dur"] >= 1.5]
        print(f"{p}\n  free felt {r['free'] * 1e3:.1f} mA, loop p95 {r['p95'] * 1e3:.2f} ms, "
              f"{len(c)} contacts >= 0.5 s ({len(long_)} >= 1.5 s)")
        print(f"  onset (median): sign flips {med('flips'):.0f}, 5-25 Hz energy {med('ring'):.0f}, "
              f"peak/plateau {med('over'):.2f}, impact {med('impact'):.0f} counts/s; "
              f"follower |i| max {max(x['ia_max'] for x in c) if c else np.nan:.2f} A")
        if long_:
            print("  holds >= 1.5 s: " + ", ".join(f"{x['t0']:.1f} s {x['dur']:.1f} s "
                                                  f"{abs(x['mean']) * 1e3:.0f} mA slope {x['slope']:+.2f} A/s"
                                                  for x in long_))


if __name__ == "__main__":
    main()
