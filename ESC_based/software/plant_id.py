"""
Limited-range plant identification of one motor + arm (A4b, adapted for an
arm that cannot rotate freely): G(f) = acceleration / current.

    python3 plant_id.py --node A                 # run (holds, injects chirp, logs)
    python3 plant_id.py --analyze logs/plant_id_A.csv

The shaft is held at its starting angle by a host PD loop (moderately stiff,
so the injected current moves it only tens of counts), and a current chirp
r(t) = AMP * sin(2 pi * phase(t)), log-swept F0 -> F1 over SWEEP_S, is added
to the PD output. Every TELEM frame is logged (mcu_time_us, enc, iq_meas)
together with the chirp value in force.

Analysis: first a time-domain fit a = G*(i - c*sign(v)) - b*v (friction
would otherwise bias G hugely at small motion), then on the friction-
corrected current the indirect closed-loop estimate G(f) = S_ra/S_ri: the
chirp r is the instrument, a is acceleration from a centred 15-frame LS fit
of position on the ESC clock, i is the measured Iq. Using r as reference
keeps the feedback loop (and the hold's response to friction/noise) from
biasing the estimate. A rigid single inertia gives a flat |G| and ~0 phase
(beyond the pure delay); compliance shows up as a resonance/anti-resonance.

LIMITATION (validated in simulation, 2026-10-25): with Coulomb friction
above about a third of AMP the held shaft spends much of the sweep stuck
in stiction, and the estimate collapses (20 mA -> G -18 %; 40 mA -> G off
by 16x) even with the friction term. The follower has 30-50 mA of
friction, so this test needs AMP of about 0.2 A, which the limited travel
does not allow at low frequency. Identify G from vigorous free-motion
teleop data instead (see TEST_LOG, C2 rerun).

Safety: total current clamped to IQ_MAX_A; abort if the shaft leaves
+-MAX_TRAVEL counts of the start, exceeds MAX_VEL, telemetry goes stale, or
the node looks reset (idle-Iq precheck).
"""

import argparse
import collections
import csv
import math
import os
import statistics as st
import sys
import time

import numpy as np

from can_interface import MotorCANInterface
from estimators import AlphaBetaVelocity
from spin_test import RESET_IQ_A

NODES = {"A": 0x000, "B": 0x020}
RATE_HZ = 500
KP = 1.25e-3          # A/count   (0.05 A chirp -> ~40 counts static)
KD = 1.5e-5           # A/(count/s)
AMP = 0.05            # A
F0, F1 = 1.0, 50.0    # Hz
SWEEP_S = 20.0
SETTLE_S = 1.0        # hold only, before and after the sweep
IQ_MAX_A = 0.3
MAX_TRAVEL = 300      # counts from the start position
MAX_VEL = 10_000.0    # counts/s
HALF = 3              # LS kernel half-width (frames): 7 frames; 15 attenuated > 20 Hz


def chirp(tau):
    """Log sweep phase (cycles) and instantaneous frequency at tau in [0, SWEEP_S]."""
    k = math.log(F1 / F0) / SWEEP_S
    return F0 * (math.exp(k * tau) - 1.0) / k, F0 * math.exp(k * tau)


def run(node):
    os.makedirs("logs", exist_ok=True)
    path = os.path.join("logs", f"plant_id_{node}.csv")
    n = 2
    while os.path.exists(path):
        path = os.path.join("logs", f"plant_id_{node}_{n}.csv")
        n += 1
    m = MotorCANInterface(channel="can0", node_base=NODES[node], pair_dbg=False)
    m.start_listening()
    rows, reason = [], "done"
    try:
        time.sleep(0.5)
        idle = [r["iq_meas"] for r in m.drain_records()]
        if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
            print("ABORT: node missing or reset (run zero_iq_check.py first)")
            return 1
        m.send_start()
        t0 = time.monotonic()
        while not m.get_debug().get("run"):
            if time.monotonic() - t0 > 5:
                print("ABORT: no RUN")
                return 1
            m.send_set_iq(0.0)
            time.sleep(0.01)
        vel = AlphaBetaVelocity(55.0)
        recs = m.drain_records()
        x0 = recs[-1]["enc"] if recs else m.get_telemetry()["enc_count_unwrapped"]
        x, v, r_now, iq = x0, 0.0, 0.0, 0.0
        dt = 1.0 / RATE_HZ
        total = SETTLE_S + SWEEP_S + SETTLE_S
        t_start = time.monotonic()
        deadline = t_start
        while True:
            now = time.monotonic()
            tr = now - t_start
            if tr >= total:
                break
            for rr in m.drain_records():
                x = rr["enc"]
                v = vel.update(rr["mcu_time_us"], x)
                rows.append((rr["mcu_time_us"], x, rr["iq_meas"], r_now, iq))
            tel = m.get_telemetry()
            if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
                reason = "ABORT: stale telemetry"
                break
            if abs(x - x0) > MAX_TRAVEL or abs(v) > MAX_VEL:
                reason = f"ABORT: travel {x - x0:+.0f} counts / speed {v:.0f} counts/s"
                break
            tau = tr - SETTLE_S
            r_now = AMP * math.sin(2 * math.pi * chirp(tau)[0]) if 0 <= tau <= SWEEP_S else 0.0
            iq = max(-IQ_MAX_A, min(IQ_MAX_A, -KP * (x - x0) - KD * v + r_now))
            m.send_set_iq(iq)
            deadline += dt
            slack = deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                deadline = time.monotonic()
    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e}")
        m.stop_listening()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mcu_time_us", "enc", "iq_meas", "chirp", "iq_cmd"])
        w.writerows(rows)
    print(f"{reason}. {len(rows)} frames. Log: {os.path.abspath(path)}")
    if rows:
        e = np.array([r[1] for r in rows])
        print(f"travel from start: {e.min() - x0:+.0f}..{e.max() - x0:+.0f} counts")
    return 0 if reason == "done" else 1


def analyze(path, show=True):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t, p, i, r = d[:, 0] * 1e-6, d[:, 1], d[:, 2], d[:, 3]
    dtm = np.median(np.diff(t))
    k = np.arange(-HALF, HALF + 1)
    P = np.linalg.pinv(np.vstack([np.ones_like(k), k, k * k]).T.astype(float))
    a = np.convolve(p, 2 * P[2][::-1], mode="same") / dtm ** 2
    v = np.convolve(p, P[1][::-1], mode="same") / dtm
    s = slice(HALF, len(t) - HALF)
    a, i, r, v = a[s], i[s], r[s], v[s]
    # Coulomb friction biases a/i badly at small motion (validated in
    # simulation: 20 mA turns a true 3e6 into 0.15e6 at 1 Hz). Fit it
    # jointly in the time domain: a = G*i - G*c*sign(v) - b*v, then use the
    # friction-corrected current for the frequency response.
    X = np.vstack([i, -np.sign(v), -v]).T
    beta, *_ = np.linalg.lstsq(X, a, rcond=None)
    G_ls, c_fric = beta[0], beta[1] / beta[0]
    r2 = 1 - np.var(a - X @ beta) / np.var(a)
    i = i - c_fric * np.sign(v)
    # Welch cross-spectra, 2 s segments
    nseg = int(round(2.0 / dtm))
    win = np.hanning(nseg)
    def xs(u, w):
        acc = 0
        for j in range(0, len(u) - nseg + 1, nseg // 2):
            U = np.fft.rfft((u[j:j + nseg] - u[j:j + nseg].mean()) * win)
            W = np.fft.rfft((w[j:j + nseg] - w[j:j + nseg].mean()) * win)
            acc = acc + np.conj(U) * W
        return acc
    f = np.fft.rfftfreq(nseg, dtm)
    Sra, Sri, Srr, Sii = xs(r, a), xs(r, i), xs(r, r), xs(i, i)
    G = Sra / Sri
    coh = np.abs(Sri) ** 2 / (np.abs(Srr) * np.abs(Sii) + 1e-30)
    out = []
    for lo, hi in ((1, 2), (2, 3), (3, 5), (5, 7), (7, 10), (10, 14), (14, 20), (20, 28), (28, 40), (40, 50)):
        b = (f >= lo) & (f < hi)
        if b.any():
            g = np.mean(G[b])
            out.append((lo, hi, abs(g), np.degrees(np.angle(g)), float(np.mean(coh[b]))))
    if show:
        print(f"{path}: {len(t)} frames, {t[-1] - t[0]:.1f} s")
        print(f"  time-domain fit a = G*(i - c*sign v) - b*v: G {G_ls:,.0f} counts/s^2 per A, "
              f"Coulomb c {c_fric*1e3:.1f} mA, R^2 {r2:.3f}")
        print("  band Hz     |G| (counts/s^2 per A)   phase deg   coherence(r,i)")
        for lo, hi, g, ph, c in out:
            print(f"  {lo:4.0f}-{hi:<4.0f} {g:14,.0f}         {ph:7.1f}      {c:.2f}")
        good = [o for o in out if o[4] > 0.5 and o[0] >= 2 and o[1] <= 20]
        if good:
            g_lf = np.median([o[2] for o in good])
            print(f"  G (median of coherent bands 2-20 Hz): {g_lf:,.0f} counts/s^2 per A "
                  f"-> J = Kt/alpha = {0.0646 / (g_lf * 2 * math.pi / 4000):.2e} kg m^2 (Kt 64.6 mN m/A)")
            spread = max(o[2] for o in good) / min(o[2] for o in good)
            print(f"  |G| max/min over those bands: {spread:.2f} (rigid body ~1; a resonance shows as a peak/dip)")
    return out, G_ls, c_fric, r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", choices=NODES, default="A")
    ap.add_argument("--analyze")
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze)
        return 0
    return run(a.node)


if __name__ == "__main__":
    sys.exit(main())
