"""
Plan B2: offline comparison of velocity estimators on logged TELEM frames.

    python3 b2_eval.py

Datasets (every frame, ESC mcu_time_us):
  - constant speed: spin_A_{+-4000,+-10000} hold phases (A4)
  - rest: A3_rest_10min.csv (HOLD, 0 A, untouched)
  - real motion: teleop_KVs3e-05_FB-1_8_frames.csv (B1, both motors)

Reference: zero-phase velocity = central difference on mcu time, then an
FFT low-pass at REF_CUTOFF_HZ (no scipy on the Pi; frames are uniform at
1 ms so an index-domain FFT is fine). Estimators are run as they would be
live -- new ones on every frame, the legacy one once per 2 ms loop tick
with the latest frame and kernel bus_time -- and scored at the loop ticks:
  lag   = shift (0..15 ms) minimising RMS(est(t) - ref(t - lag))
  noise = that minimum RMS (counts/s), i.e. error with the lag removed
  rest  = fraction of ticks with output exactly 0, and max |v|, on A3 rest

Pass (brief B2): >= 3x lower noise than legacy at constant speed, lag < 3 ms,
exactly 0 at rest.
"""

import csv

import numpy as np

from estimators import AlphaBetaVelocity, LegacyVelocity, LsVelocity

REF_CUTOFF_HZ = 100.0
TICK_FRAMES = 2            # 500 Hz loop on 1 kHz telemetry
MAX_LAG_MS = 15

CANDIDATES = {
    "legacy": None,
    "LS 4 ms": lambda: LsVelocity(0.004),
    "LS 6 ms": lambda: LsVelocity(0.006),
    "LS 8 ms": lambda: LsVelocity(0.008),
    "AB 30 Hz": lambda: AlphaBetaVelocity(30.0),
    "AB 60 Hz": lambda: AlphaBetaVelocity(60.0),
    "AB 100 Hz": lambda: AlphaBetaVelocity(100.0),
}


def load_spin(path):
    t, p, b = [], [], []
    for r in csv.DictReader(open(path)):
        if r["phase"] == "hold":
            t.append(float(r["mcu_time_us"]))
            p.append(float(r["enc"]))
            b.append(float(r["bus_time"]))
    return np.array(t), np.array(p), np.array(b)


def load_coupling(path, motor="A"):
    t, p, b = [], [], []
    for r in csv.DictReader(open(path)):
        if r["motor"] == motor and r["engaged"] == "1":
            t.append(float(r["mcu_time_us"]))
            p.append(float(r["pos"]))
            b.append(float(r["t_s"]))
    return np.array(t), np.array(p), np.array(b)


def load_frames(path, motor):
    t, p, b = [], [], []
    for r in csv.DictReader(open(path)):
        if r["motor"] == motor and int(r["loop_iter"]) >= 0:
            t.append(float(r["mcu_time_us"]))
            p.append(float(r["enc"]))
            b.append(float(r["bus_time"]))
    return np.array(t), np.array(p), np.array(b)


def reference(t_us, pos):
    t = t_us * 1e-6
    v = np.gradient(pos, t)
    n = len(v)
    V = np.fft.rfft(v - v.mean())
    f = np.fft.rfftfreq(n, np.median(np.diff(t)))
    taper = np.clip((REF_CUTOFF_HZ * 1.2 - f) / (REF_CUTOFF_HZ * 0.4), 0, 1)
    return np.fft.irfft(V * taper, n) + v.mean()


def run(t_us, pos, bus):
    """Outputs per estimator, sampled at loop ticks; tick frame indices."""
    ticks = np.arange(TICK_FRAMES - 1, len(t_us), TICK_FRAMES)
    out = {}
    for name, make in CANDIDATES.items():
        if make is None:
            est = LegacyVelocity()
            out[name] = np.array([est.update_tick(bus[k], pos[k]) for k in ticks])
            continue
        est = make()
        v = np.empty(len(t_us))
        for k in range(len(t_us)):
            v[k] = est.update(t_us[k], pos[k])
        out[name] = v[ticks]
    return ticks, out


def score(t_us, ref, ticks, est):
    t = t_us * 1e-6
    tt = t[ticks]
    best = (np.inf, 0.0)
    skip = 50                                     # estimator warm-up
    for lag_ms in np.arange(0, MAX_LAG_MS + 0.25, 0.25):
        r = np.interp(tt - lag_ms * 1e-3, t, ref)
        e = np.sqrt(np.mean((est[skip:] - r[skip:]) ** 2))
        if e < best[0]:
            best = (e, lag_ms)
    return best


def main():
    datasets = []
    for sp in ("+4000", "-4000", "+10000", "-10000"):
        datasets.append((f"spin {sp}", load_spin(f"logs/spin_A_{sp}_0.3A_kv1.5e-05_ki0.0002_coast3.csv")))
    for m in ("A", "B"):
        datasets.append((f"teleop B1 {m}", load_frames("logs/teleop_KVs3e-05_FB-1_8_frames.csv", m)))

    print(f"Noise = RMS error vs zero-phase reference ({REF_CUTOFF_HZ:g} Hz) after removing "
          f"the best lag; counts/s. Lag in ms.")
    header = "  ".join(f"{n:>16s}" for n in CANDIDATES)
    print(f"{'dataset':16s}  {header}")
    summary = {n: [] for n in CANDIDATES}
    for name, (t, p, b) in datasets:
        ref = reference(t, p)
        ticks, out = run(t, p, b)
        cells = []
        for n, est in out.items():
            e, lag = score(t, ref, ticks, est)
            summary[n].append((name, e, lag))
            cells.append(f"{e:8.0f} @{lag:4.1f}ms")
        print(f"{name:16s}  " + "  ".join(f"{c:>16s}" for c in cells))

    print("\nNoise ratio legacy / estimator at constant speed (spin runs), and worst lag:")
    leg = {d: e for d, e, _ in summary["legacy"]}
    for n in CANDIDATES:
        if n == "legacy":
            continue
        rows = [(d, e, lag) for d, e, lag in summary[n] if d.startswith("spin")]
        ratios = [leg[d] / e for d, e, _ in rows]
        lags = [lag for _, _, lag in summary[n]]
        print(f"  {n:10s} ratio {min(ratios):.2f}..{max(ratios):.2f}x   max lag {max(lags):.2f} ms   "
              f"({'meets' if min(ratios) >= 3 and max(lags) < 3 else 'fails'} >=3x and <3 ms)")

    t, p, b = load_coupling("logs/A3_rest_10min.csv")
    print(f"\nRest (A3_rest_10min, {len(t)} frames, {np.sum(np.diff(p) != 0)} position changes):")
    ticks, out = run(t, p, b)
    for n, est in out.items():
        e = est[50:]
        print(f"  {n:10s} exactly-0 ticks {100 * np.mean(e == 0):6.2f}%   max |v| {np.max(np.abs(e)):7.1f} counts/s   "
              f"rms {np.sqrt(np.mean(e ** 2)):6.1f}")


if __name__ == "__main__":
    main()
