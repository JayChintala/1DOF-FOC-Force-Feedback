"""
P2P vs the old teleop controller: free-space RMS, taps, holds, rigid vs
sponge (Peer_to_Peer/testing.md, "P2P vs teleop").

    python3 p2p_compare.py logs/p2p_*_FREE_air.csv logs/p2p_*_TAPS_rigid.csv \\
        logs/p2p_*_CONTACT_rigid.csv logs/p2p_*_CONTACT_sponge.csv

Runs come from `p2p_teleop.py --guide free|taps|contact --surface ...`. Roles
match teleop: the operator holds B (teleop's leader), A touches the
environment (teleop's follower). Each metric uses the teleop definition and,
where possible, the same detector code:

  free     RMS of B's commanded current over the 'free' phase -- teleop's A1
           "free-space RMS of iq_b_cmd". Also the hand force estimate on B
           (RFOB), which is the physically felt quantity in P2P: B's command
           also carries its own observer's friction/inertia cancellation.
  taps     analyze_teleop_baseline.impacts() (the detector behind teleop's
           4 ms): A stops sharply while B is still moving that way; 10-90 %
           rise of B's current (and of the hand-force estimate) after impact.
  holds    hold_eval.py's definitions: contacts with |f_A| > 80 mA for
           >= 0.5 s; plateau slope of |f_A| from 0.25 s after onset to 0.1 s
           before the end; leader(B)-velocity sign flips (|v| > 300) in the
           first 0.3 s. Plus static force matching |f_A + f_B| / |f_A| on
           the plateau (action = reaction; teleop's "static <= 5 %" was
           against a scale, so this is the closest P2P analogue, not equal).
  stiffness stiffness_eval.py's definition: per press, A's travel between
           quasi-static (|v_A| < 300) frames at F < 0.3 N and F > 0.7 N ->
           k in N/mm at A's tip. Plus the operator side: B's travel from
           contact (F > 0.15 N) to 0.7 N, teleop's "leader travel" metric.

Forces come from replaying the firmware's observers (couple_ctrl.c DobStep,
RFOB, ripple table) on the logged measured currents and encoders, so they are
what the ESCs computed, up to timing.
"""

import csv
import json
import math
import os
import sys

import numpy as np

import analyze_teleop_baseline as atb

G = {"A": 4.082e6, "B": 8.877e6}
FC = {"A": 0.025, "B": 0.017}
FRIC_V0 = 300.0
VEL_ALPHA = 0.5
N_PER_A = 1.058                          # at A's tip (B7, 2026-11-15)
MM_PER_COUNT_A = 2 * math.pi * 75.7 / 4000
CONTACT_A = 0.08                          # hold_eval.py
VMAX = 300.0                              # stiffness_eval.py
FMIN, F_LO, F_HI = 0.15, 0.3, 0.7         # N, stiffness_eval.py
MIN_PRESS_S = 0.3

# Teleop baselines (TEST_LOG.md; same definitions as above).
TELEOP = {
    "free_iq_b_rms_mA": (18.7, "C3 operating point, 2026-10-25 (felt 8.8 + damping 15.2)"),
    "free_felt_mA": (10.1, "HOLD3, 2026-10-26, hold_eval.py free felt"),
    # run 21's 26.2 mA (2026-11-15) is NOT a free-motion baseline: the handle
    # moved ~100 counts in the whole run (found 2026-11-22).
    "tap_rise_ms": (4.0, "C3 taps-only, 2026-10-25 (p25 4, p75 34); original path 66 ms"),
    "hold_slope_A_s": ("-0.03 / +0.02", "HOLD3 with contact spring; -0.32/-0.52 without"),
    "onset_flips": (2, "HOLD3 median"),
    "k_rigid_N_mm": (1.62, "run 22, 2026-11-15 (lower bound; encoder-limited)"),
    "k_sponge_N_mm": (0.08, "run _16 sponge, 2026-10-29 (ratio 19.9x vs rigid)"),
    "leader_travel_rigid": (382, "run _14, 2026-10-29, contact -> 0.7 N, no leader spring"),
    "leader_travel_rigid_spring": (50, "run _18, 2026-10-29, leader contact spring K_L 2e-3 (current teleop)"),
    "leader_travel_sponge": (473, "run _16, 2026-10-29"),
}


def load(base):
    cfg = json.load(open(base + ".json"))
    rows = list(csv.DictReader(open(base + ".csv")))
    cal = calrows = None
    if os.path.exists(base + "_cal.json"):
        cal = json.load(open(base + "_cal.json"))
        calrows = list(csv.DictReader(open(base + "_cal.csv")))
    M = {}
    for m in "AB":
        lr = [x for x in rows if x["motor"] == m]
        f = lambda k: np.array([float(x[k]) if x[k] != "" else np.nan for x in lr])
        off = 0.0
        if calrows:
            cr = [x for x in calrows if x["motor"] == m]
            off = float(cr[-1]["enc"]) - float(lr[0]["pos"])
        M[m] = dict(t=f("t_s"), us=f("mcu_time_us"), pos=f("pos"), raw=f("pos") + off,
                    iq=f("iq_meas"), cmd=f("iq_cmd"), err=f("err"),
                    eng=np.array([x["engaged"] == "1" for x in lr]),
                    sat=np.array([x["saturated"] == "1" for x in lr]))
    return cfg, cal, M


def ripple(cal, m, raw):
    if not cal:
        return np.zeros_like(raw)
    th = 2 * np.pi * (raw % 4000) / 4000
    ab = cal[m]["ab_A"]
    return sum(ab[2 * k] * np.cos(h * th) + ab[2 * k + 1] * np.sin(h * th)
               for k, h in enumerate(cal[m]["harmonics"]))


def observe(cfg, cal, m, D):
    """Replay the firmware observer for motor m; returns (v, f): velocity
    counts/s and external force in amps (+ toward +counts)."""
    g = cfg["dob_g"]
    rip = (cfg.get("ripple_pct") or 0) / 100.0
    comp = cfg["fric_pct"] / 100.0
    fc = min(0.2, max(0.0, cal[m]["coulomb_A"])) if cal else FC[m]
    k = g / G[m]
    n = len(D["t"])
    v = np.zeros(n)
    f = np.zeros(n)
    iq_eff = D["iq"] - rip * ripple(cal, m, D["raw"])
    z = None
    was_eng = False
    for i in range(n):
        if i:
            dt = (D["us"][i] - D["us"][i - 1]) * 1e-6
            if dt > 0:
                v[i] = VEL_ALPHA * (D["pos"][i] - D["pos"][i - 1]) / dt + (1 - VEL_ALPHA) * v[i - 1]
            else:
                v[i] = v[i - 1]
        else:
            dt = 1e-3
        if D["eng"][i] and not was_eng:
            z = None                                   # firmware resets at engage
        was_eng = D["eng"][i]
        if z is None:
            z = iq_eff[i] + k * v[i]
        z += min(1.0, g * dt) * (iq_eff[i] + k * v[i] - z)
        d = z - k * v[i]
        f[i] = -(d - comp * fc * max(-1.0, min(1.0, v[i] / FRIC_V0)))
    return v, f


def build(base):
    cfg, cal, M = load(base)
    for m in "AB":
        M[m]["v"], M[m]["f"] = observe(cfg, cal, m, M[m])
    A = M["A"]
    tb = M["B"]["t"]
    al = {k: np.interp(A["t"], tb, M["B"][k]) for k in ("pos", "v", "f", "cmd")}
    eng = cfg.get("engage_mcu_us")
    el = (A["us"] - eng["A"]) * 1e-6 if eng else A["t"] - A["t"][A["eng"]][0]
    return cfg, A, al, el


def phase_mask(cfg, el, names):
    m = np.zeros(len(el), bool)
    for g in cfg.get("guide") or []:
        if g["name"] in names:
            m |= (el >= g["start_s"]) & (el < g["end_s"])
    return m


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")


def m_free(cfg, A, B, el):
    w = phase_mask(cfg, el, {"free"}) & A["eng"]
    return {"iq_b_rms_mA": 1e3 * rms(B["cmd"][w]),
            "hand_force_rms_mA": 1e3 * rms(B["f"][w]),
            "f_A_rms_mA": 1e3 * rms(A["f"][w]),
            "v_b_rms": rms(B["v"][w]),
            "gap_rms": rms(A["err"][w]),
            "sat_pct": 100 * float(np.mean(A["sat"][w])) if w.any() else float("nan")}


def m_taps(cfg, A, B, el):
    w = phase_mask(cfg, el, {"taps"}) & A["eng"]
    t = list(A["t"][w])
    out = {}
    for name, y in (("iq_b", B["cmd"][w]), ("hand_force", -B["f"][w])):
        ev = atb.impacts(t, list(A["v"][w]), list(B["v"][w]), list(y))
        r = [e["rise_s"] * 1e3 for e in ev]
        out[name] = {"n": len(ev), "rise_ms_median": float(np.median(r)) if r else float("nan"),
                     "rise_ms_p25_p75": (float(np.percentile(r, 25)), float(np.percentile(r, 75))) if r else None,
                     "peak_mA_median": 1e3 * float(np.median([abs(e["peak_A"]) for e in ev])) if ev else float("nan")}
    return out


def contacts(A, B, w):
    con = (np.abs(A["f"]) > CONTACT_A) & w
    edges = np.diff(np.r_[0, con.astype(int), 0])
    return list(zip(np.where(edges == 1)[0], np.where(edges == -1)[0]))


def m_holds(cfg, A, B, el):
    w = phase_mask(cfg, el, {"hold_gentle", "hold_firm"}) & A["eng"]
    t, fa, fb = A["t"], A["f"], B["f"]
    out = []
    for s, e in contacts(A, B, w):
        if t[e - 1] - t[s] < 0.5:
            continue
        idx = np.arange(s, e)
        pl = idx[(t[idx] > t[s] + 0.25) & (t[idx] < t[e - 1] - 0.1)]
        on = (t >= t[s]) & (t < t[s] + 0.3)
        vb = B["v"][on]
        out.append({"t0": float(t[s]), "dur_s": float(t[e - 1] - t[s]),
                    "force_N": float(np.mean(np.abs(fa[pl]))) * N_PER_A if len(pl) else float("nan"),
                    "slope_A_s": float(np.polyfit(t[pl], np.abs(fa[pl]), 1)[0]) if len(pl) > 20 else float("nan"),
                    "slope_hand_A_s": float(np.polyfit(t[pl], np.abs(fb[pl]), 1)[0]) if len(pl) > 20 else float("nan"),
                    "match_pct": 100 * float(np.median(np.abs(fa[pl] + fb[pl]) / np.abs(fa[pl]))) if len(pl) else float("nan"),
                    "onset_flips": int(np.sum(np.diff(np.sign(vb[np.abs(vb) > 300])) != 0)),
                    "sat_pct": 100 * float(np.mean(A["sat"][s:e]))})
    return out


def m_stiffness(cfg, A, B, el):
    w = phase_mask(cfg, el, {"presses", "hold_gentle", "hold_firm"}) & A["eng"]
    t = A["t"]
    F_all = np.abs(A["f"]) * N_PER_A
    out = []
    for s, e in contacts(A, B, w):
        if t[e - 1] - t[s] < MIN_PRESS_S:
            continue
        F, PA, PB, q = F_all[s:e], A["pos"][s:e], B["pos"][s:e], np.abs(A["v"][s:e]) < VMAX
        lo, hi = q & (F < F_LO), q & (F > F_HI)
        rec = {"t0": float(t[s]), "peak_N": float(F.max())}
        if lo.sum() >= 10 and hi.sum() >= 10:
            d = abs(np.median(PA[hi]) - np.median(PA[lo]))
            dF = float(np.median(F[hi]) - np.median(F[lo]))
            rec.update(travel_A=float(d), k_N_mm=dF / (max(d, 1) * MM_PER_COUNT_A),
                       encoder_limited=bool(d <= 1))
        on = np.where(F > FMIN)[0]
        up = np.where(F > F_HI)[0]
        if len(on) and len(up) and up[0] > on[0]:
            rec["travel_B_to_0.7N"] = float(abs(PB[up[0]] - PB[on[0]]))
            rec["travel_A_to_0.7N"] = float(abs(PA[up[0]] - PA[on[0]]))
        out.append(rec)
    return out


def analyse(path):
    base = path[:-4] if path.endswith(".csv") else path
    cfg, A, B, el = build(base)
    kind = cfg.get("guide_name")
    res = {"run": os.path.basename(base), "guide": kind, "surface": cfg.get("surface"),
           "settings": {k: cfg.get(k) for k in ("wn_hz", "zeta", "kf_pct", "dob_g", "fric_pct",
                                                 "ripple_pct", "iq_max")}}
    if kind == "free":
        res["free"] = m_free(cfg, A, B, el)
    elif kind == "taps":
        res["taps"] = m_taps(cfg, A, B, el)
    elif kind == "contact":
        res["holds"] = m_holds(cfg, A, B, el)
        res["presses"] = m_stiffness(cfg, A, B, el)
    return res


def med(xs):
    xs = [x for x in xs if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.median(xs)) if xs else float("nan")


def report(results):
    print("P2P vs teleop (teleop numbers from TEST_LOG.md; see TELEOP in p2p_compare.py)\n")
    k = {}
    for r in results:
        print(f"=== {r['run']}  [{r['guide']}, {r['surface']}]  {r['settings']}")
        if "free" in r:
            f = r["free"]
            print(f"  free-space RMS of B's current: {f['iq_b_rms_mA']:.1f} mA   "
                  f"(teleop {TELEOP['free_iq_b_rms_mA'][0]} mA; see p2p_feel.py for a like-for-like comparison)")
            print(f"  hand force on B (RFOB) rms:    {f['hand_force_rms_mA']:.1f} mA   "
                  f"(teleop free felt {TELEOP['free_felt_mA'][0]} mA)")
            print(f"  |v_B| rms {f['v_b_rms']:.0f} counts/s, gap rms {f['gap_rms']:.1f} counts, "
                  f"f_A rms {f['f_A_rms_mA']:.1f} mA, saturated {f['sat_pct']:.1f}%")
        if "taps" in r:
            for name, tp in r["taps"].items():
                q = tp["rise_ms_p25_p75"]
                print(f"  taps ({name}): n {tp['n']}, rise median {tp['rise_ms_median']:.1f} ms"
                      + (f" (p25 {q[0]:.1f}, p75 {q[1]:.1f})" if q else "")
                      + f", peak {tp['peak_mA_median']:.0f} mA   (teleop {TELEOP['tap_rise_ms'][0]} ms)")
        if "holds" in r:
            for h in r["holds"]:
                print(f"  hold {h['dur_s']:4.1f} s at {h['force_N']:.2f} N: slope {h['slope_A_s']:+.3f} A/s "
                      f"(hand {h['slope_hand_A_s']:+.3f}), |f_A+f_B|/|f_A| {h['match_pct']:.1f}%, "
                      f"onset flips {h['onset_flips']}, saturated {h['sat_pct']:.0f}%")
            print(f"  (teleop holds: slope {TELEOP['hold_slope_A_s'][0]} A/s, onset flips {TELEOP['onset_flips'][0]})")
        if "presses" in r:
            ks = [p["k_N_mm"] for p in r["presses"] if "k_N_mm" in p]
            tb = [p["travel_B_to_0.7N"] for p in r["presses"] if "travel_B_to_0.7N" in p]
            ta = [p["travel_A_to_0.7N"] for p in r["presses"] if "travel_A_to_0.7N" in p]
            print(f"  presses: {len(r['presses'])}, with k: {len(ks)}; median k {med(ks):.2f} N/mm; "
                  f"travel contact->0.7 N: A {med(ta):.0f}, B {med(tb):.0f} counts")
            k[r["surface"]] = (med(ks), med(tb), med(ta))
        print()
    if "rigid" in k and "sponge" in k:
        (kr, br, ar), (ks_, bs, as_) = k["rigid"], k["sponge"]
        print(f"rigid vs sponge: k {kr:.2f} vs {ks_:.2f} N/mm -> ratio {kr / ks_:.1f}x "
              f"(teleop follower {TELEOP['k_rigid_N_mm'][0]} vs {TELEOP['k_sponge_N_mm'][0]}, 19.9x)")
        print(f"  operator side (B) travel to 0.7 N: rigid {br:.0f} vs sponge {bs:.0f} counts "
              f"(teleop leader {TELEOP['leader_travel_rigid'][0]} vs {TELEOP['leader_travel_sponge'][0]} without the "
              f"leader spring; rigid {TELEOP['leader_travel_rigid_spring'][0]} with it)")


if __name__ == "__main__":
    results = [analyse(p) for p in sys.argv[1:]]
    report(results)
    with open("p2p_compare_last.json", "w") as f:
        json.dump(results, f, indent=1, default=float)
