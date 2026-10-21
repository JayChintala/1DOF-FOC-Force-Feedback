"""
Phase A4: ripple / sector characterisation from spin_test.py logs.

    python3 analyze_ripple.py logs/spin_A_*.csv [--g 9025000]

Works in the ANGLE domain, per run, over the "hold" phase (constant speed):

  - Speed vs angle: the encoder position is cut into NB bins per rev; the
    MCU time at each bin crossing (interpolated) gives v(theta) per bin,
    averaged over all revolutions. Time comes from mcu_time_us only.
  - Acceleration vs angle: a(theta) = v * dv/dpos, derivative by FFT over
    the (periodic) revolution.
  - Equivalent ripple current: r(theta) = i_cmd(theta) - a(theta)/G, i.e.
    the commanded current that did NOT show up as acceleration. Includes
    gravity (1/rev), cogging/torque ripple, and angle-dependent friction.
    The brief's point: measured Iq alone hides slow sensing errors because
    the current loop holds it at the reference -- so this uses
    acceleration, and reports iq_meas - iq_cmd separately (sensing view).
  - Harmonics of r(theta) at MODEL_H (1 = gravity, plus 7/14/28/42/84);
    "explained" = share of r's variance (DC removed) in those harmonics.
  - Across runs: r_n = c_n + i_mean * g_n fitted per harmonic (complex
    least squares over runs), needs runs at different mean Iq (loaded runs).
  - Noise vs angle: std of iq_meas - iq_cmd per 1/16-rev bin.

Pass (brief A4): model explains >= 50 % of the harmonic variance. If the
42/rev amplitude is < 10 % of 14/rev there are no meaningful sector spikes.
"""

import argparse
import csv

import numpy as np

ENC = 4000
NB = 500                       # angle bins per rev (8 counts each)
MODEL_H = (1, 7, 14, 28, 42, 84)
NOISE_BINS = 16
G_DEFAULT = 9_025_000          # A2, motor A, counts/s^2 per A


def load(path):
    cols = {k: [] for k in ("t", "pos", "iq", "cmd", "phase")}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("phase") != "hold":
                continue
            cols["t"].append(float(r["mcu_time_us"]) * 1e-6)
            cols["pos"].append(float(r["enc"]))
            cols["iq"].append(float(r["iq_meas"]))
            cols["cmd"].append(float(r["iq_cmd"]))
    return {k: np.array(v) for k, v in cols.items() if k != "phase"}


def angle_profile(d):
    """Per-bin mean speed (from crossing times), i_cmd and i_meas."""
    t, pos = d["t"], d["pos"]
    sgn = 1.0 if pos[-1] > pos[0] else -1.0
    p = sgn * pos
    p = np.maximum.accumulate(p)           # stick-slip / tiny reversals
    width = ENC / NB
    k0 = int(np.ceil(p[0] / width))
    k1 = int(np.floor(p[-1] / width))
    if k1 - k0 < 2 * NB:
        return None
    edges = np.arange(k0, k1 + 1) * width
    # strictly increasing p for interpolation
    keep = np.concatenate(([True], np.diff(p) > 0))
    tc = np.interp(edges, p[keep], t[keep])
    v = width / np.diff(tc)                 # counts/s, magnitude
    mid = (edges[:-1] + edges[1:]) / 2
    b = ((sgn * mid) % ENC / width).astype(int) % NB
    vb = np.bincount(b, weights=v, minlength=NB) / np.maximum(np.bincount(b, minlength=NB), 1)
    # currents per bin from the samples themselves
    bs = ((pos % ENC) / width).astype(int) % NB
    cnt = np.maximum(np.bincount(bs, minlength=NB), 1)
    icmd = np.bincount(bs, weights=d["cmd"], minlength=NB) / cnt
    imeas = np.bincount(bs, weights=d["iq"], minlength=NB) / cnt
    revs = (k1 - k0) / NB
    return sgn, vb, icmd, imeas, revs


def ddpos(y):
    """d/dpos of a periodic per-bin profile, counts^-1, via FFT."""
    Y = np.fft.rfft(y)
    n = np.arange(len(Y))
    return np.fft.irfft(Y * 1j * 2 * np.pi * n / ENC, len(y))


def harmonics(y):
    Y = np.fft.rfft(y - y.mean()) * 2 / len(y)
    return Y                                # complex amplitude per harmonic n


RESOLVE_MAX_H = 100            # alt denominator: harmonics up to this only. At
                               # 10,000 counts/s, 1 kHz telemetry resolves ~200/rev
                               # at best, and bin interpolation smears well below


def explained(Y, model_h=MODEL_H, max_h=None):
    pw = np.abs(Y[1:max_h + 1 if max_h else None]) ** 2
    sel = sum(pw[h - 1] for h in model_h if h - 1 < len(pw))
    return sel / pw.sum() if pw.sum() else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--g", type=float, default=G_DEFAULT)
    ap.add_argument("--harmonics", type=lambda x: tuple(int(v) for v in x.split(",")),
                    default=None, help="extra model set to report alongside MODEL_H, "
                    "e.g. 1,7,12,14,24,28,36,42,84")
    a = ap.parse_args()

    runs = []
    for path in a.logs:
        d = load(path)
        if len(d["t"]) < 1000:
            print(f"{path}: too little hold data ({len(d['t'])})")
            continue
        prof = angle_profile(d)
        if prof is None:
            print(f"{path}: fewer than 2 revs in hold, skipped")
            continue
        sgn, vb, icmd, imeas, revs = prof
        # a = dv/dt = v dv/dx = |v| d|v|/dx: already in the +counts frame for
        # either direction of travel (vb is |v| binned by true angle).
        acc = vb * ddpos(vb)                # counts/s^2, +counts frame
        r = icmd - acc / a.g                # A
        Yr, Ys, Yv = harmonics(r), harmonics(imeas - icmd), harmonics(vb)
        imean = float(d["cmd"].mean())
        runs.append((path, sgn * vb.mean(), imean, Yr, Ys))
        print(f"\n{path}")
        print(f"  {revs:.1f} rev in hold; speed {sgn * vb.mean():+.0f} counts/s "
              f"(angle-profile ripple {vb.std():.0f}); mean i_cmd {imean:+.4f} A, "
              f"mean i_meas {d['iq'].mean():+.4f} A")
        print(f"  {'n/rev':>6s} {'speed (c/s)':>12s} {'r_eq (mA)':>10s} {'meas-cmd (mA)':>14s}")
        for h in MODEL_H:
            print(f"  {h:6d} {abs(Yv[h]):12.0f} {abs(Yr[h]) * 1e3:10.2f} {abs(Ys[h]) * 1e3:14.2f}")
        top = np.argsort(np.abs(Yr[1:]))[::-1][:6] + 1
        print(f"  largest r_eq harmonics: " +
              ", ".join(f"{n}/rev {abs(Yr[n]) * 1e3:.2f} mA" for n in top))
        ex = explained(Yr)
        r42_14 = abs(Yr[42]) / abs(Yr[14]) if abs(Yr[14]) else float("nan")
        print(f"  model harmonics {MODEL_H} explain {100 * ex:.0f}% of r_eq harmonic "
              f"variance ({'PASS' if ex >= 0.5 else 'FAIL'} >= 50%); "
              f"42/14 ratio {r42_14:.2f} ({'< 0.1: no sector spikes' if r42_14 < 0.1 else '>= 0.1'})")
        ex100 = explained(Yr, MODEL_H, RESOLVE_MAX_H)
        print(f"    same, of harmonics <= {RESOLVE_MAX_H}/rev only: {100 * ex100:.0f}%")
        if a.harmonics:
            exa = explained(Yr, a.harmonics)
            exa100 = explained(Yr, a.harmonics, RESOLVE_MAX_H)
            print(f"  alt model {a.harmonics} explains {100 * exa:.0f}% "
                  f"({'PASS' if exa >= 0.5 else 'FAIL'} >= 50%); "
                  f"of harmonics <= {RESOLVE_MAX_H}/rev: {100 * exa100:.0f}%")
        exs = explained(Ys)
        print(f"  sensing view (iq_meas - iq_cmd): rms {np.std(imeas - icmd) * 1e3:.2f} mA per bin, "
              f"model harmonics explain {100 * exs:.0f}%")

        # noise vs angle, from raw samples
        resid = d["iq"] - d["cmd"]
        nb = ((d["pos"] % ENC) / (ENC / NOISE_BINS)).astype(int) % NOISE_BINS
        stds = [resid[nb == k].std() for k in range(NOISE_BINS) if np.sum(nb == k) > 100]
        if stds:
            print(f"  raw iq_meas - iq_cmd std per 1/{NOISE_BINS} rev: "
                  f"{min(stds):.4f}..{max(stds):.4f} A (ratio {max(stds) / min(stds):.1f})")

    if len(runs) >= 2:
        hs = sorted(set(MODEL_H) | set(a.harmonics or ()))
        print("\nPhase of r_eq per harmonic (deg) by run -- an angle-locked ripple "
              "keeps its phase across runs:")
        print("  n/rev " + " ".join(f"{x[1]:>+9.0f}" for x in runs) + "   (mean speed)")
        for h in hs:
            print(f"  {h:5d} " + " ".join(f"{np.degrees(np.angle(x[3][h])):9.0f}" for x in runs)
                  + "   amps(mA) " + " ".join(f"{abs(x[3][h]) * 1e3:5.1f}" for x in runs))

    if len(runs) >= 3:
        im = np.array([x[2] for x in runs])
        print(f"\nAcross {len(runs)} runs: r_n = c_n + i_mean * g_n "
              f"(i_mean spread {im.min():+.3f}..{im.max():+.3f} A)")
        A = np.vstack([np.ones_like(im), im]).T
        cond = np.linalg.cond(A)
        for h in MODEL_H:
            y = np.array([x[3][h] for x in runs])
            sol, *_ = np.linalg.lstsq(A.astype(complex), y, rcond=None)
            print(f"  {h:3d}/rev: |c| {abs(sol[0]) * 1e3:6.2f} mA   |g| {abs(sol[1]) * 1e3:7.2f} mA/A")
        print(f"  (design condition number {cond:.0f}: g is only meaningful with loaded "
              "runs spreading i_mean)")
    return 0


if __name__ == "__main__":
    main()
