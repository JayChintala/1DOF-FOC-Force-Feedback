"""
Compare P2P runs, phase by phase, for felt cogging. Use with runs made with
`p2p_teleop.py --guide` (timed cues: slow_B, slow_A, push), so every run had
the same movements. Older runs without a guide are analysed as one phase.

    python3 p2p_ab.py logs/<runA>.csv logs/<runB>.csv [...]

Per slow-turn phase, for the shaft the operator turns (slow_B -> B, slow_A -> A):

  speed ripple   angle-locked harmonics of that shaft's velocity, % of its
                 mean speed. Model-free: works with or without --no-cal.
                 Steady hand turning + cogging = speed wobble at the cogging
                 harmonics (TEST_LOG 2026-10-26 used the same measure).
  force ripple   (cal runs only) angle-locked force left in the operator's
                 hand: calibrated ripple r(th) minus the angle-locked part of
                 (iq - a/G) on that shaft, in mA. Needs the run's _cal.csv to
                 recover raw angles.

A phase whose turning samples cover less than MIN_COVER of the circle is
reported as "insufficient coverage" instead of a number: a harmonic fit over
a few hundred counts of angle is meaningless (the 2026-11-19 warm-up A/B run
2 had 11,000 counts of travel inside a ~250-count span and blew up).
"""

import csv
import json
import os
import sys

import numpy as np

H = (1, 7, 12, 14, 24, 28, 36, 42, 84)
G = {"A": 4.082e6, "B": 8.877e6}
MIN_COVER = 0.95           # fraction of the 4000-count circle the turning
                           # samples must cover (100 bins); total travel can
                           # be large while the angle barely changes (jitter)
MIN_SPEED = 200.0          # counts/s; slower samples are not "turning"
DRIVER = {"slow_A": "A", "slow_B": "B"}


def load(base):
    cfg = json.load(open(base + ".json"))
    rows = list(csv.DictReader(open(base + ".csv")))
    cal = calrows = None
    if os.path.exists(base + "_cal.json"):
        cal = json.load(open(base + "_cal.json"))
        calrows = list(csv.DictReader(open(base + "_cal.csv")))
    data = {}
    for m in "AB":
        lr = [x for x in rows if x["motor"] == m]
        r = [x for x in lr if x["engaged"] == "1"]
        t_us = np.array([float(x["mcu_time_us"]) for x in r])
        p = np.array([float(x["pos"]) for x in r])
        iq = np.array([float(x["iq_meas"]) for x in r])
        sat = np.array([x["saturated"] == "1" for x in r])
        t = t_us * 1e-6
        k = 5
        v = np.zeros_like(p)
        v[k:-k] = (p[2 * k:] - p[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
        a = np.zeros_like(p)
        a[k:-k] = (v[2 * k:] - v[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
        d = np.convolve(iq, np.ones(2 * k + 1) / (2 * k + 1), "same") - a / G[m]
        off = None
        if calrows:
            cr = [x for x in calrows if x["motor"] == m]
            off = float(cr[-1]["enc"]) - float(lr[0]["pos"])   # same counter
        data[m] = dict(t_us=t_us, p=p, v=v, d=d, sat=sat, off=off)
    return cfg, cal, data


def harm_fit(th, y, extra):
    X = [np.ones(len(th))] + extra
    for h in H:
        X += [np.cos(h * th), np.sin(h * th)]
    X = np.array(X).T
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return c[1 + len(extra):]


def amps(ab):
    return [float(np.hypot(ab[2 * i], ab[2 * i + 1])) for i in range(len(H))]


def rms(ab, skip_first=True):
    a = amps(ab)[1 if skip_first else 0:]
    return float(np.sqrt(sum(x * x for x in a) / 2))


def phases(cfg, data):
    if cfg.get("guide") and cfg.get("engage_mcu_us"):
        return [(g["name"], g["start_s"], g["end_s"]) for g in cfg["guide"]]
    t0 = {m: data[m]["t_us"][0] for m in "AB"}
    return [("all", 0, (data["A"]["t_us"][-1] - t0["A"]) * 1e-6)]


def analyse(base):
    cfg, cal, data = load(base)
    eng = cfg.get("engage_mcu_us") or {m: data[m]["t_us"][0] for m in "AB"}
    out = {"run": os.path.basename(base), "ripple_pct": cfg.get("ripple_pct"),
           "warmup_s": cfg.get("warmup_s", 0), "phases": {}}
    for name, s0, s1 in phases(cfg, data):
        res = {}
        for m in "AB":
            D = data[m]
            el = (D["t_us"] - eng[m]) * 1e-6
            win = (el >= s0) & (el < s1)
            res[m + "_sat_pct"] = 100.0 * float(D["sat"][win].mean()) if win.any() else 0.0
            res[m + "_travel"] = float(np.sum(np.abs(np.diff(D["p"][win])))) if win.sum() > 1 else 0.0
        drv = DRIVER.get(name) or ("B" if name == "all" else None)
        if drv:
            D = data[drv]
            el = (D["t_us"] - eng[drv]) * 1e-6
            mov = (el >= s0) & (el < s1) & (np.abs(D["v"]) > MIN_SPEED)
            res["driver"] = drv
            bins = np.unique(((D["p"][mov] % 4000) // 40).astype(int)) if mov.any() else []
            cover = len(bins) / 100.0
            res["cover"] = cover
            if cover < MIN_COVER or mov.sum() < 500:
                res["note"] = f"insufficient coverage ({100 * cover:.0f}% of the circle while turning)"
            else:
                th = 2 * np.pi * ((D["p"][mov] + (D["off"] or 0.0)) % 4000) / 4000
                sgn = np.sign(D["v"][mov])
                vbar = float(np.mean(np.abs(D["v"][mov])))
                # speed ripple: |v| vs angle (direction-folded), % of mean speed
                sv = harm_fit(th, np.abs(D["v"][mov]), [sgn])
                res["mean_speed"] = vbar
                res["speed_ripple_pct"] = {h: 100 * a / vbar for h, a in zip(H, amps(sv))}
                res["speed_ripple_rms_pct"] = 100 * rms(sv) / vbar
                if cal and D["off"] is not None:
                    locked = harm_fit(th, D["d"][mov], [sgn])
                    felt = np.array(cal[drv]["ab_A"]) - locked
                    res["force_ripple_mA"] = {h: 1e3 * a for h, a in zip(H, amps(felt))}
                    res["force_ripple_rms_mA"] = 1e3 * rms(felt)
                    res["cal_ripple_rms_mA"] = 1e3 * rms(np.array(cal[drv]["ab_A"]))
        out["phases"][name] = res
    return out


def report(results):
    for r in results:
        print(f"\n=== {r['run']}  (ripple {r['ripple_pct']} %, warm-up {r['warmup_s']} s)")
        for name, res in r["phases"].items():
            line = (f"  {name:7s} travel A {res['A_travel']:6.0f} B {res['B_travel']:6.0f} cnt, "
                    f"sat A {res['A_sat_pct']:4.1f}% B {res['B_sat_pct']:4.1f}%")
            print(line)
            if "note" in res:
                print(f"          {res['driver']}: {res['note']}")
            elif "speed_ripple_pct" in res:
                sp = res["speed_ripple_pct"]
                print(f"          {res['driver']} speed ripple rms {res['speed_ripple_rms_pct']:.1f}% of "
                      f"{res['mean_speed']:.0f} cnt/s: " + " ".join(f"{h}:{sp[h]:.1f}" for h in H[1:]))
                if "force_ripple_mA" in res:
                    fr = res["force_ripple_mA"]
                    print(f"          {res['driver']} force ripple in hand rms {res['force_ripple_rms_mA']:.0f} mA "
                          f"(motor's own {res['cal_ripple_rms_mA']:.0f}): "
                          + " ".join(f"{h}:{fr[h]:.0f}" for h in H[1:]))


if __name__ == "__main__":
    res = [analyse(a[:-4] if a.endswith(".csv") else a) for a in sys.argv[1:]]
    report(res)
    json.dump(res, open("p2p_ab_last.json", "w"), indent=1, default=float)
