"""
Leader (Motor B) disturbance model: angle-locked ripple + Coulomb friction,
in amps of the current needed to overcome it. TEST_LOG 2026-10-26
("leader friction measurements").

    d(th, v) = g_r * sum_h [a_h cos(h th) + b_h sin(h th)] + g_f * f * tanh(v / v0)

with th = 2 pi (count mod 4000) / 4000. Only harmonics that repeated
across STARTs go in the model (28 and 84/rev). The 7/rev part moved
36 -> 24 mA and 13 deg between STARTs (current-offset calibration,
inferred), so it is left out until the firmware averages longer.

Used two ways in teleop (TELEOP_LEADER_COMP):
  feed-forward  iq_b_cmd += d        (the operator stops feeling it)
  observer      i_b -> i_b - d       (the hand-torque estimate stops
                                       seeing it as hand torque, so the
                                       follower is not driven by it)

Fit from friction_map.py sweeps:

    python3 leader_comp.py --fit logs/friction_map_B_2.csv:1257 logs/friction_map_B_3.csv:1279 \\
        --out leader_comp_B.json
"""

import json
import math

ENC = 4000
HARMONICS_REPEATABLE = (28, 84)


class LeaderComp:
    def __init__(self, model, gain_ripple=1.0, gain_fric=0.5):
        self.h = [(int(h), a, b) for h, (a, b) in model["harmonics_A"].items()]
        self.f = model["coulomb_A"]
        self.v0 = model["v0_cnt_s"]
        self.gr, self.gf = gain_ripple, gain_fric

    @classmethod
    def load(cls, path, **kw):
        with open(path) as fh:
            return cls(json.load(fh), **kw)

    def d(self, count, v):
        th = 2 * math.pi * (count % ENC) / ENC
        r = 0.0
        for h, a, b in self.h:
            r += a * math.cos(h * th) + b * math.sin(h * th)
        return self.gr * r + self.gf * self.f * math.tanh(v / self.v0)


def fit(specs, v0=300.0, sweep=200.0):
    import csv

    import numpy as np

    rows_th, rows_i, rows_s = [], [], []
    for spec in specs:
        path, start_raw = spec.rsplit(":", 1)
        for r in csv.DictReader(open(path)):
            if r["seg"] in ("fwd", "back") and abs(abs(float(r["v_ref"])) - sweep) < 1.0:
                rows_th.append((float(r["rel"]) + float(start_raw)) % ENC)
                rows_i.append(float(r["iq_meas"]))
                rows_s.append(1.0 if float(r["v_ref"]) > 0 else -1.0)
    th = 2 * np.pi * np.array(rows_th) / ENC
    i, s = np.array(rows_i), np.array(rows_s)
    # all harmonics in the fit (so 7/rev does not leak into the others), only
    # the repeatable ones kept
    hs = (1, 2, 7, 14, 21, 28, 84)
    X = [np.ones_like(th), s]
    for h in hs:
        X += [np.cos(h * th), np.sin(h * th)]
    c, *_ = np.linalg.lstsq(np.vstack(X).T, i, rcond=None)
    harm = {str(h): [float(c[2 + 2 * k]), float(c[3 + 2 * k])] for k, h in enumerate(hs)
            if h in HARMONICS_REPEATABLE}
    model = {"harmonics_A": harm, "coulomb_A": float(c[1]), "v0_cnt_s": v0, "sources": list(specs),
             "note": "amps to overcome the disturbance; count = leader raw encoder count"}
    for h, (a, b) in harm.items():
        print(f"  {h}/rev {math.hypot(a, b) * 1e3:.1f} mA")
    print(f"  Coulomb {model['coulomb_A'] * 1e3:.1f} mA; offset {c[0] * 1e3:+.1f} mA (not used)")
    return model


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--fit", nargs="+", required=True, help="friction_map log:start_raw")
    ap.add_argument("--out", default="leader_comp_B.json")
    a = ap.parse_args()
    m = fit(a.fit)
    with open(a.out, "w") as fh:
        json.dump(m, fh, indent=1)
    print(f"saved {a.out}")
