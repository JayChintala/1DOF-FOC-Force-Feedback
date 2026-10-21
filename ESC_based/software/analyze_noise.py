"""
Phase A3: Iq noise floor from coupling_monitor.py HOLD logs (zero gains).

    python3 analyze_noise.py logs/<hold>.csv [--motor A] [--drift]

Reports, from the measured Iq (TELEM, one raw sample per 1 kHz tick):
  - mean / std, and that the applied command really was 0 A (iq_cmd);
  - histogram: is Iq on a lattice? Iq mixes phase ADC counts through the
    Park transform, so the step depends on rotor angle -- the lattice is
    checked per angle bin, not globally;
  - std per 1/16-rev angle bin (P2 found A's noise angle-dependent);
  - amplitude spectrum (Welch, dt from mcu_time_us) with the top peaks;
  - --drift: 10 s window means across the log; A3 passes when the range of
    those means is < DRIFT_PASS_A (brief: 10 min at rest).
"""

import argparse
import csv

import numpy as np

DRIFT_WIN_S = 10.0
DRIFT_PASS_A = 0.005
ANGLE_BINS = 16
ENC = 4000


def load(path, motor):
    t, pos, iq, cmd = [], [], [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            # Engaged rows only: the log starts before START, so it also holds
            # idle, offset-calibration and possibly alignment frames.
            if r["motor"] != motor or r["iq_meas"] == "" or r["engaged"] != "1":
                continue
            t.append(float(r["mcu_time_us"]) * 1e-6)
            pos.append(float(r["pos"]))
            iq.append(float(r["iq_meas"]))
            cmd.append(float(r["iq_cmd"]) if r["iq_cmd"] not in ("", "None") else np.nan)
    return np.array(t), np.array(pos), np.array(iq), np.array(cmd)


def lattice_step(x):
    """Smallest common spacing of the distinct values (0 if none)."""
    u = np.unique(np.round(x, 6))
    if len(u) < 3:
        return 0.0, len(u)
    d = np.diff(u)
    step = np.median(d[d > 1e-5])
    return float(step), len(u)


def welch(x, dt, nseg=1024):
    x = x - x.mean()
    win = np.hanning(nseg)
    step = nseg // 2
    segs = [x[i:i + nseg] * win for i in range(0, len(x) - nseg + 1, step)]
    if not segs:
        return None, None
    p = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    # Amplitude-ish scale (A RMS per bin) so peaks read in amps.
    amp = np.sqrt(2 * p) / win.sum()
    f = np.fft.rfftfreq(nseg, dt)
    return f, amp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--motor", default="A")
    ap.add_argument("--drift", action="store_true")
    a = ap.parse_args()

    t, pos, iq, cmd = load(a.log, a.motor)
    if len(t) < 2000:
        print(f"{a.log}: only {len(t)} samples for motor {a.motor}")
        return 1
    dts = np.diff(t)
    dt = float(np.median(dts))
    dur = t[-1] - t[0]
    gaps = int(np.sum(np.abs(dts - dt) > 50e-6))
    print(f"{a.log}  motor {a.motor}: {len(t)} samples, {dur:.1f} s, "
          f"dt median {dt * 1e6:.0f} us, {gaps} gaps outside +-50 us")
    nz = np.nanmax(np.abs(cmd)) if np.any(np.isfinite(cmd)) else float("nan")
    print(f"  applied command: max |iq_cmd| = {nz:.4f} A (must be 0)")
    print(f"  Iq: mean {iq.mean():+.5f} A, std {iq.std():.5f} A, "
          f"p1..p99 {np.percentile(iq, 1):+.4f}..{np.percentile(iq, 99):+.4f}")
    span = pos.max() - pos.min()
    print(f"  shaft travel: {span:.0f} counts ({span / ENC:.2f} rev)")

    step, nvals = lattice_step(iq)
    print(f"  distinct Iq values: {nvals}; median spacing {step * 1e3:.2f} mA "
          f"(phase ADC count = 29.4 mA; Iq step varies with angle)")

    ang = (pos % ENC) / ENC
    bins = (ang * ANGLE_BINS).astype(int)
    rows = []
    for b in range(ANGLE_BINS):
        sel = iq[bins == b]
        if len(sel) >= 200:
            s, n = lattice_step(sel)
            rows.append((b, len(sel), sel.mean(), sel.std(), s, n))
    if rows:
        print(f"  per angle bin (1/{ANGLE_BINS} rev, >=200 samples): bin n mean std step(mA) nvals")
        for b, n, m, s, st_, nv in rows:
            print(f"    {b:2d} {n:6d} {m:+.4f} {s:.4f} {st_ * 1e3:6.2f} {nv:4d}")
        stds = [r[3] for r in rows]
        if len(rows) > 1:
            print(f"  std across angle bins: {min(stds):.4f}..{max(stds):.4f} A "
                  f"(ratio {max(stds) / min(stds):.1f})")

    f, amp = welch(iq, dt)
    if f is not None:
        band = (f > 0.5)
        top = np.argsort(amp[band])[::-1][:8]
        fb, ab = f[band][top], amp[band][top]
        floor = float(np.median(amp[band]))
        print(f"  spectrum (Welch 1024, {f[1]:.2f} Hz bins): median floor {floor * 1e3:.2f} mA/bin; "
              f"top peaks: " + ", ".join(f"{x:.1f} Hz {y * 1e3:.2f} mA" for x, y in
                                          sorted(zip(fb, ab))))

    if a.drift:
        edges = np.arange(t[0], t[-1], DRIFT_WIN_S)
        means = [iq[(t >= e) & (t < e + DRIFT_WIN_S)].mean() for e in edges
                 if np.sum((t >= e) & (t < e + DRIFT_WIN_S)) > 0.5 * DRIFT_WIN_S / dt]
        if len(means) >= 2:
            rng = max(means) - min(means)
            slope = np.polyfit(np.arange(len(means)) * DRIFT_WIN_S, means, 1)[0]
            verdict = "PASS" if rng < DRIFT_PASS_A else "FAIL"
            print(f"  [A3] drift: {len(means)} x {DRIFT_WIN_S:.0f} s means range "
                  f"{rng * 1e3:.2f} mA, trend {slope * 600 * 1e3:+.2f} mA per 10 min "
                  f"(< {DRIFT_PASS_A * 1e3:.0f} mA: {verdict})")
    return 0


if __name__ == "__main__":
    main()
