"""
Feasibility check (offline): can the follower's external-torque estimate
separate free motion from contact by a wide margin?

    python3 feasibility.py

Estimate: estimators.ExternalTorqueObserver (C1 form, 20 Hz, G_A from A2),
  tau = LPF(i - r) - w*(x - LPF(x)),  x = OBS_COMP * v / G_A
  free-motion runs: every TELEM frame (frames logs), AB 55 Hz velocity, mcu dt
  contact runs (A1 hand presses / foam): 500 Hz teleop CSV only (no frames
  logs then), legacy vel_a and iq_a_readback with loop_dt

Friction model (fitted on OTHER runs, leave-one-out over the free runs):
  tau_free = c0 + c1*sign(v)*(1 + c2*exp(-|v|/vs)) + c3*v     (Coulomb +
  Stribeck-like low-speed bump + viscous), least squares on c with vs fixed.
"""

import csv
import math

import numpy as np

from estimators import AlphaBetaVelocity, ExternalTorqueObserver

G_A = 9_025_000
FREE = {  # every-frame logs, follower = motor A
    "B1 (moderate)": "logs/teleop_KVs3e-05_FB-1_8_frames.csv",
    "run1 (gentle)": "logs/teleop_KVs3e-05_FB-1_9_frames.csv",
    "run2 (gentle)": "logs/teleop_KVs3e-05_FB-1_10_frames.csv",
    "run3 (vigorous)": "logs/teleop_KVs3e-05_FB-1_11_shm_frames.csv",
}
CONTACT = {"A1 hand presses": "logs/teleop_KVs3e-05_FB-1_5.csv",
           "A1 foam presses": "logs/teleop_KVs3e-05_FB-1_6.csv",
           "A1 free (500 Hz)": "logs/teleop_KVs3e-05_FB-1_2.csv"}
VS = 1500.0               # Stribeck speed scale, counts/s


def frames(path, motor="A"):
    t, p, i = [], [], []
    for r in csv.DictReader(open(path)):
        if r["motor"] == motor and int(r["loop_iter"]) >= 0:
            t.append(float(r["mcu_time_us"]) * 1e-6)
            p.append(float(r["enc"]))
            i.append(float(r["iq_meas"]))
    return np.array(t), np.array(p), np.array(i)


def observe_frames(t, p, i):
    ob, ve = ExternalTorqueObserver(G_A, 20.0), AlphaBetaVelocity(55.0)
    tau, v = np.empty(len(t)), np.empty(len(t))
    for k in range(len(t)):
        v[k] = ve.update(t[k] * 1e6, p[k])
        tau[k] = ob.update(v[k], i[k], (t[k] - t[k - 1]) if k else 1e-3)
    return tau[500:], v[500:]


def fric_cols(v):
    s = np.sign(v)
    return np.vstack([np.ones_like(v), s, s * np.exp(-np.abs(v) / VS), v / 1e4]).T


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main():
    print("FREE MOTION: follower external-torque estimate when nothing is touched")
    data = {}
    for name, path in FREE.items():
        t, p, i = frames(path)
        tau, v = observe_frames(t, p, i)
        data[name] = (tau, v, i[500:])
    for name, (tau, v, i) in data.items():
        # friction model fitted on the OTHER free runs (leave-one-out)
        X = np.vstack([fric_cols(data[n][1]) for n in data if n != name])
        y = np.concatenate([data[n][0] for n in data if n != name])
        c, *_ = np.linalg.lstsq(X, y, rcond=None)
        resid = tau - fric_cols(v) @ c
        still = np.abs(v) < 1  # AB rest detector -> exactly 0
        print(f"  {name:16s} |v| rms {rms(v):6.0f} c/s; raw follower current rms {rms(i)*1e3:6.1f} mA; "
              f"estimate rms raw {rms(tau)*1e3:5.1f} mA -> minus held-out friction model {rms(resid)*1e3:5.1f} mA"
              f"  (at rest {100*np.mean(still):.0f}% of time: {rms(resid[still])*1e3 if still.any() else float('nan'):.1f} mA)")
    X = np.vstack([fric_cols(d[1]) for d in data.values()])
    y = np.concatenate([d[0] for d in data.values()])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    print(f"  friction model (all runs): offset {c[0]*1e3:+.1f} mA, Coulomb {c[1]*1e3:.1f} mA, "
          f"low-speed bump {c[2]*1e3:+.1f} mA, viscous {c[3]*1e3:+.1f} mA per 10k c/s")
    all_tau = np.concatenate([d[0] - fric_cols(d[1]) @ c for d in data.values()])
    print(f"  pooled residual after friction: rms {rms(all_tau)*1e3:.1f} mA, p99 |.| "
          f"{np.percentile(np.abs(all_tau), 99)*1e3:.1f} mA, max {np.max(np.abs(all_tau))*1e3:.1f} mA")

    print("\nCONTACT vs FREE in the A1 logs (500 Hz, latest-value telemetry, legacy velocity)")
    for name, path in CONTACT.items():
        r = list(csv.DictReader(open(path)))
        ob = ExternalTorqueObserver(G_A, 20.0)
        tau = np.array([ob.update(float(x["vel_a"]), float(x["iq_a_readback"]),
                                  min(max(float(x["loop_dt"]), 1e-4), 0.01)) for x in r])[500:]
        v = np.array([float(x["vel_a"]) for x in r])[500:]
        tau_f = tau - fric_cols(v) @ c
        da = np.array([int(x["d_a"]) for x in r])[500:]
        # "held still" = follower moved <= 30 counts over the surrounding 0.2 s
        held = np.array([np.ptp(da[max(0, k - 50):k + 50]) <= 30 for k in range(len(da))])
        big = np.abs(tau_f) > 0.15
        print(f"  {name:17s}: estimate rms {rms(tau_f)*1e3:6.1f} mA; time with |est| > 150 mA: {100*np.mean(big):5.1f}%; "
              f"follower held still {100*np.mean(held):4.1f}% of time, |est| there median "
              f"{np.median(np.abs(tau_f[held]))*1e3 if held.any() else float('nan'):6.1f} mA, p90 "
              f"{np.percentile(np.abs(tau_f[held]), 90)*1e3 if held.any() else float('nan'):6.1f} mA")


if __name__ == "__main__":
    main()
