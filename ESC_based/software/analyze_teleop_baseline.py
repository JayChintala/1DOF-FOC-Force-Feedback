"""
Phase A1/A6 metrics from teleop_test.py logs.

    python3 analyze_teleop_baseline.py --free logs/<free>.csv [--taps logs/<table>.csv ...]

A1  free-space RMS of iq_b_cmd (the felt current), over the whole --free log
    after DROP_HEAD_S; 10-90 % rise time of iq_b_cmd per detected tap.
A6  loop_dt histogram percentiles and fresh-frame fraction, every log given.

Tap detection: |iq_b_cmd - local baseline| crosses TAP_THRESH_A after at
least QUIET_S below it. Baseline = median over the PRE_WIN_S window ending
PRE_GAP_S before the crossing; peak = max deviation within PEAK_WIN_S.
Rise = first 10 % crossing to first 90 % crossing of that peak, linearly
interpolated between samples.
"""

import argparse
import csv
import math
import statistics as st

DROP_HEAD_S = 1.0
TAP_THRESH_A = 0.05
QUIET_S = 0.2
PRE_WIN_S = 0.10
PRE_GAP_S = 0.05
PEAK_WIN_S = 0.40
PEAK_HALF = 5         # samples either side of the max for the peak level
LOOP_DT_PASS_S = 0.0021


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {}
    for k in ("t_s", "iq_b_cmd", "loop_dt", "fresh_a", "fresh_b", "vel_a", "vel_b"):
        cols[k] = [float(r[k]) for r in rows if r.get(k, "") != ""]
    return cols


def pct(xs, p):
    s = sorted(xs)
    i = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(i), math.ceil(i)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def crossing_time(t, dev, i0, i1, level):
    for j in range(i0, i1):
        if dev[j] >= level:
            if j == i0:
                return t[j]
            f = (level - dev[j - 1]) / (dev[j] - dev[j - 1])
            return t[j - 1] + f * (t[j] - t[j - 1])
    return None


def taps(t, y):
    out = []
    n = len(t)
    last_quiet_start = t[0]
    i = 0
    while i < n:
        # Local baseline window ending PRE_GAP_S before sample i.
        lo = next((j for j in range(i, -1, -1) if t[i] - t[j] >= PRE_GAP_S + PRE_WIN_S), None)
        hi = next((j for j in range(i, -1, -1) if t[i] - t[j] >= PRE_GAP_S), None)
        if lo is None or hi is None or hi <= lo:
            i += 1
            continue
        base = st.median(y[lo:hi])
        if abs(y[i] - base) < TAP_THRESH_A:
            i += 1
            continue
        if t[i] - last_quiet_start < QUIET_S:
            i += 1
            continue
        sign = 1.0 if y[i] > base else -1.0
        end = i
        while end < n and t[end] - t[i] <= PEAK_WIN_S:
            end += 1
        start = hi
        # 3-point median against single-sample noise at the 10 % crossing;
        # peak level = median around the max, so the noisiest sample does
        # not set the 90 % level (that biased rise time ~+7 ms on synthetic
        # tau = 57 ms steps).
        raw = [sign * (v - base) for v in y[max(0, start - 1):min(n, end + PEAK_HALF + 1)]]
        off = max(0, start - 1)
        dev = [0.0] * n
        for j in range(start, min(n, end + PEAK_HALF)):
            r = j - off
            dev[j] = st.median(raw[max(0, r - 1):r + 2])
        k_peak = max(range(start, end), key=lambda j: dev[j])
        peak = st.median(dev[max(start, k_peak - PEAK_HALF):k_peak + PEAK_HALF + 1])
        t10 = crossing_time(t, dev, start, k_peak + 1, 0.1 * peak)
        t90 = crossing_time(t, dev, start, k_peak + 1, 0.9 * peak)
        if t10 is not None and t90 is not None:
            out.append({"t": t[i], "peak_A": sign * peak, "rise_s": t90 - t10})
        # Skip to the end of this event: wait until it settles back below.
        j = end
        while j < n and abs(y[j] - base) >= TAP_THRESH_A:
            j += 1
        last_quiet_start = t[j] if j < n else t[-1]
        i = j
    return out


IMPACT_VEL_MIN = 3000.0   # counts/s the follower must be moving at before impact
IMPACT_STOP_FRAC = 0.2    # ... and drop to this fraction of it
IMPACT_WIN_S = 0.02       # ... within this window
IMPACT_LEADER_MIN = 1500.0  # leader still moving the same way at the stop
IMPACT_DEBOUNCE_S = 0.5
IMPACT_PEAK_WIN_S = 0.4   # felt-force peak searched this long after impact


def impacts(t, vel_a, vel_b, y):
    """
    Taps found from the follower, not the felt current: a sharp stop of
    vel_a (>= IMPACT_VEL_MIN to <= IMPACT_STOP_FRAC of it within
    IMPACT_WIN_S) while the leader is still moving the same way at
    >= IMPACT_LEADER_MIN -- in a free reversal the leader turns round too.
    The current-threshold detector in taps() also fires on
    free-motion transients, which slow-press logs showed are most events.
    Rise is 10-90 % of iq_b_cmd from its value at impact to its peak.
    """
    out = []
    last = -1e9
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > IMPACT_WIN_S:
            j += 1
        if t[i] - last < IMPACT_DEBOUNCE_S:
            continue
        v0 = max(vel_a[j:i + 1], key=abs)
        if abs(v0) < IMPACT_VEL_MIN or abs(vel_a[i]) > IMPACT_STOP_FRAC * abs(v0):
            continue
        if vel_b[i] * v0 <= 0 or abs(vel_b[i]) < IMPACT_LEADER_MIN:
            continue
        k_imp = vel_a.index(v0, j, i + 1)
        end = k_imp
        while end < len(t) and t[end] - t[k_imp] <= IMPACT_PEAK_WIN_S:
            end += 1
        base = y[k_imp]
        sign = 1.0 if max(y[k_imp:end], key=lambda v: abs(v - base)) > base else -1.0
        dev = [sign * (v - base) for v in y[k_imp:end]]
        ts = t[k_imp:end]
        k_pk = max(range(len(dev)), key=lambda k: dev[k])
        peak = st.median(dev[max(0, k_pk - PEAK_HALF):k_pk + PEAK_HALF + 1])
        if peak <= 0:
            continue
        t10 = crossing_time(ts, dev, 0, k_pk + 1, 0.1 * peak)
        t90 = crossing_time(ts, dev, 0, k_pk + 1, 0.9 * peak)
        if t10 is not None and t90 is not None:
            out.append({"t": t[k_imp], "v0": v0, "peak_A": sign * peak,
                        "rise_s": t90 - t10, "lag_s": t10 - t[k_imp]})
        last = t[i]
    return out


GAP_TOL_US = 50


def frames_check(path):
    """
    B1 pass checks on a teleop <log>_frames.csv (TELEOP_FRAME_QUEUE=1):
    per motor, mcu_time_us gaps (1000 +- 50 us) and missing frames, counted
    only over the control loop (loop_iter >= 0) so the known 2-frame drop
    at START (loop_iter -1) is excluded, plus how long each frame waited in
    the queue (kernel bus_time -> drained, upper bound one loop period).
    """
    rows = {"A": [], "B": []}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows[r["motor"]].append((int(r["loop_iter"]), float(r["mcu_time_us"]),
                                     float(r["bus_time"])))
    ok = True
    for m, v in rows.items():
        loop = [x for x in v if x[0] >= 0]
        if len(loop) < 2:
            print(f"  [B1] {m}: no loop frames")
            ok = False
            continue
        mt = [x[1] for x in loop]
        gaps = [b - a for a, b in zip(mt, mt[1:])]
        bad = [g for g in gaps if abs(g - 1000) > GAP_TOL_US]
        missing = sum(max(0, round(g / 1000) - 1) for g in gaps)
        dur = (mt[-1] - mt[0]) * 1e-6
        dup = sum(1 for g in gaps if g <= 0)
        ok = ok and not bad and missing == 0
        print(f"  [B1] {m}: {len(loop)} frames over {dur:.1f} s ({len(loop) / dur:.1f}/s); "
              f"gaps min {min(gaps):.0f} max {max(gaps):.0f} us, {len(bad)} outside 1000+-{GAP_TOL_US}; "
              f"missing {missing}; non-increasing {dup}")
        if bad:
            print(f"        first bad gaps (us): {bad[:10]}")
    print(f"  [B1] frames: {'PASS' if ok else 'FAIL'} (all gaps 1000+-{GAP_TOL_US} us, 0 missing)")
    return ok


def timing(path, c):
    dt = c["loop_dt"]
    n = len(dt)
    p50, p95, p99 = (pct(dt, p) for p in (50, 95, 99))
    fa = sum(c["fresh_a"]) / n
    fb = sum(c["fresh_b"]) / n
    verdict = "PASS" if p95 <= LOOP_DT_PASS_S else "FAIL"
    print(f"  [A6/B1] loop_dt p50={p50*1e3:.3f} p95={p95*1e3:.3f} p99={p99*1e3:.3f} "
          f"max={max(dt)*1e3:.3f} ms  (p95<=2.1 ms: {verdict}; also watch max)")
    edges = [0, 1.9, 2.0, 2.1, 2.5, 3.0, 5.0, 1e9]
    hist = [sum(1 for v in dt if a <= v * 1e3 < b) for a, b in zip(edges, edges[1:])]
    labels = ["<1.9", "1.9-2.0", "2.0-2.1", "2.1-2.5", "2.5-3", "3-5", ">5"]
    print("        hist ms: " + "  ".join(f"{l}:{h}" for l, h in zip(labels, hist)))
    print(f"        fresh A={100*fa:.1f}%  B={100*fb:.1f}% of iterations "
          f"(500 Hz loop on 1 kHz TELEM: 100% = no iteration without a new frame)")
    return p95


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--free", required=True)
    ap.add_argument("--taps", nargs="*", default=[])
    ap.add_argument("--frames", nargs="*", default=[],
                    help="teleop <log>_frames.csv files: B1 gap/missing-frame checks")
    ap.add_argument("--impacts", nargs="*", default=[],
                    help="tap logs analysed by follower impact (preferred)")
    a = ap.parse_args()

    c = load(a.free)
    t0 = c["t_s"][0]
    y = [v for tt, v in zip(c["t_s"], c["iq_b_cmd"]) if tt - t0 >= DROP_HEAD_S]
    rms = math.sqrt(sum(v * v for v in y) / len(y))
    print(f"{a.free}: {c['t_s'][-1] - t0:.1f} s")
    print(f"  [A1] free-space RMS iq_b_cmd = {rms:.4f} A  (n={len(y)}, "
          f"mean={st.mean(y):+.4f}, std={st.pstdev(y):.4f})")
    timing(a.free, c)

    for path in a.taps:
        c = load(path)
        ev = taps(c["t_s"], c["iq_b_cmd"])
        print(f"{path}: {c['t_s'][-1] - c['t_s'][0]:.1f} s, {len(ev)} taps "
              f"(threshold {TAP_THRESH_A} A)")
        for e in ev:
            print(f"    t={e['t']:7.2f}s  peak={e['peak_A']:+.3f} A  rise={e['rise_s']*1e3:6.1f} ms")
        if ev:
            r = [e["rise_s"] * 1e3 for e in ev]
            print(f"  [A1] 10-90% rise: median={st.median(r):.1f} ms  "
                  f"min={min(r):.1f}  max={max(r):.1f}")
        timing(path, c)

    for path in a.frames:
        print(path)
        frames_check(path)

    for path in a.impacts:
        c = load(path)
        ev = impacts(c["t_s"], c["vel_a"], c["vel_b"], c["iq_b_cmd"])
        print(f"{path}: {len(ev)} impacts (follower stop from >= {IMPACT_VEL_MIN:.0f} counts/s)")
        for e in ev:
            print(f"    t={e['t']:7.2f}s  v0={e['v0']:+7.0f}  felt peak={e['peak_A']:+.3f} A  "
                  f"lag to 10%={e['lag_s']*1e3:5.1f} ms  rise={e['rise_s']*1e3:6.1f} ms")
        if ev:
            r = [e["rise_s"] * 1e3 for e in ev]
            pk = [abs(e["peak_A"]) for e in ev]
            print(f"  [A1] impact 10-90% rise: median={st.median(r):.1f} ms  "
                  f"min={min(r):.1f}  max={max(r):.1f};  |peak| median={st.median(pk):.3f} A")


if __name__ == "__main__":
    main()
