"""
Plan C1 (offline): follower external-torque estimator on the 2026-10-25
post-B4 data session (every-frame logs, rigid stop at ~4 cm radius).

    python3 c1_eval.py

Estimator (estimators.ExternalTorqueObserver, 20 Hz, G_A from A2), fed every
TELEM frame with AB 55 Hz velocity and mcu-clock dt. Two refinements over the
plain form, both found in the 2026-10-22 feasibility check:
  - ALIGN: the current is delayed by ALIGN_FRAMES (3 ms) so it lines up with
    the velocity estimate's ~2.6 ms lag (offline the best floor came from
    advancing velocity 3 ms; live we cannot look ahead, so delay current).
  - FRICTION: Coulomb + low-speed + viscous model fitted on the free run,
    subtracted from the estimate (option; reported both ways).

Sign: the observer computes the motor current NOT turned into acceleration
(i - a/G), i.e. the torque the motor exerts against the environment. The
torque the ENVIRONMENT exerts on the follower is the negative of that; the
brief's convention ("positive = environment pushes toward +counts") is
tau_env = -(i - a/G). Checked below on the rigid presses, where the stop
sits at +d_a.

Units: amps of equivalent current; torque = Kt * A with Kt = 64.6 mN m/A
(B7), force at the contact = torque / R_CONTACT.
"""

import csv

import numpy as np

from estimators import AlphaBetaVelocity, ExternalTorqueObserver

G_A = 9_025_000
KT = 0.0646                 # N m / A (B7, r = 160 mm run)
R_CONTACT = 0.04            # m, rigid stop / foam contact radius (user, ~4 cm)
ALIGN_FRAMES = 3            # 1 ms per frame
VS = 1500.0
RUNS = {"free": "logs/C1_free_frames.csv", "rigid_press": "logs/C1_rigid_press_frames.csv",
        "rigid_taps": "logs/C1_rigid_taps_frames.csv", "foam1": "logs/C1_foam1_frames.csv"}


def frames(path, motor="A"):
    t, p, i = [], [], []
    for r in csv.DictReader(open(path)):
        if r["motor"] == motor and int(r["loop_iter"]) >= 0:
            t.append(float(r["mcu_time_us"]) * 1e-6)
            p.append(float(r["enc"]))
            i.append(float(r["iq_meas"]))
    return np.array(t), np.array(p), np.array(i)


def fric_cols(v):
    s = np.sign(v)
    return np.vstack([np.ones_like(v), s, s * np.exp(-np.abs(v) / VS), v / 1e4]).T


def estimate(t, p, i, align=ALIGN_FRAMES):
    """Motor-side estimate m = LPF(i_delayed) - accel term, per frame."""
    ob, ve = ExternalTorqueObserver(G_A, 20.0), AlphaBetaVelocity(55.0)
    m, v = np.empty(len(t)), np.empty(len(t))
    i_d = np.concatenate([np.full(align, i[0]), i[:-align]]) if align else i
    for k in range(len(t)):
        v[k] = ve.update(t[k] * 1e6, p[k])
        m[k] = ob.update(v[k], i_d[k], (t[k] - t[k - 1]) if k else 1e-3)
    return m, v


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main():
    data = {n: frames(p) for n, p in RUNS.items()}
    est = {}
    for n, (t, p, i) in data.items():
        for align in (0, ALIGN_FRAMES):
            m, v = estimate(t, p, i, align)
            est[(n, align)] = (m, v)

    # --- friction model on the free run (aligned estimate)
    m_free, v_free = est[("free", ALIGN_FRAMES)]
    sk = 500
    c, *_ = np.linalg.lstsq(fric_cols(v_free[sk:]), m_free[sk:], rcond=None)
    fr = lambda v: fric_cols(v) @ c

    print("1) FREE FLOOR (C1_free, nothing touched)")
    t, p, i = data["free"]
    print(f"   follower range {np.ptp(p):.0f} counts, |v| rms {rms(v_free[sk:]):.0f} counts/s, raw |i| rms {rms(i[sk:])*1e3:.1f} mA")
    for align in (0, ALIGN_FRAMES):
        m, v = est[("free", align)]
        print(f"   align {align} ms: estimate rms {rms(m[sk:])*1e3:5.1f} mA; minus friction (fit on this run) "
              f"{rms(m[sk:] - fr(v[sk:]))*1e3:5.1f} mA; p99 |.| {np.percentile(np.abs(m[sk:] - fr(v[sk:])), 99)*1e3:5.1f} mA")
    m, v = est[("free", ALIGN_FRAMES)]
    floor = rms(m[sk:] - fr(v[sk:]))
    print(f"   friction model: offset {c[0]*1e3:+.1f}, Coulomb {c[1]*1e3:+.1f}, low-speed {c[2]*1e3:+.1f}, viscous {c[3]*1e3:+.1f} mA/10k c/s")
    print(f"   [C1] free-space rms {floor*1e3:.1f} mA = {floor*KT*1e3:.2f} mN m = {floor*KT/R_CONTACT*1e3:.0f} mN at {R_CONTACT*100:.0f} cm "
          f"(brief < 15 mA: {'PASS' if floor < 0.015 else 'FAIL'}; Gate A <= 25 mA: {'PASS' if floor <= 0.025 else 'FAIL'})")

    print("\n2) STATIC ACCURACY on rigid holds (a = 0, so the estimate should equal the measured current)")
    t, p, i = data["rigid_press"]
    m, v = est[("rigid_press", ALIGN_FRAMES)]
    mf = m - fr(v)
    holds, k = [], 0
    while k < len(t):
        j = np.searchsorted(t, t[k] + 0.5)
        if j < len(t) and np.ptp(p[k:j]) <= 5 and abs(i[k:j].mean()) > 0.05:
            e = j
            while e < len(t) and abs(p[e] - p[k]) <= 5:
                e += 1
            a, b = k + int(0.2 * (e - k)), e     # skip the settling start
            holds.append((t[k] - t[0], t[e - 1] - t[k], p[k], i[a:b].mean(), m[a:b].mean(), mf[a:b].mean()))
            k = e
        else:
            k += 50
    errs = []
    for h in holds:
        err = (h[4] - h[3]) / h[3]
        errs.append(err)
        print(f"   t={h[0]:5.1f}s {h[1]:.2f}s at enc {h[2]:.0f}: measured {h[3]:+.3f} A, estimate {h[4]:+.3f} A "
              f"({100*err:+.1f}%), with friction model {h[5]:+.3f} A; force {h[3]*KT/R_CONTACT:.2f} N")
    if errs:
        worst = max(abs(e) for e in errs)
        print(f"   [C1] static error worst {100*worst:.1f}% (within 10%: {'PASS' if worst <= 0.10 else 'FAIL'})")

    print("\n3) SIGN: stop at +counts; during holds the motor pushes +, so the environment pushes -")
    if holds:
        sgn = np.sign(np.mean([h[3] for h in holds]))
        print(f"   hold current sign {sgn:+.0f}, motor-side estimate sign {np.sign(np.mean([h[4] for h in holds])):+.0f} "
              f"-> brief-convention tau_env = -(estimate) reads {-np.mean([h[4] for h in holds]):+.3f} A (negative = toward -counts) OK")

    print("\n4) RIGID vs FOAM: estimated contact force vs penetration")
    for n in ("rigid_press", "foam1", "rigid_taps"):
        t, p, i = data[n]
        m, v = est[(n, ALIGN_FRAMES)]
        mf = m - fr(v)
        contact = mf > 5 * floor          # motor pushing into the contact
        if contact.sum() < 200:
            print(f"   {n}: too little contact ({contact.sum()} frames)")
            continue
        pc, fc = p[contact], mf[contact]
        x0 = np.percentile(pc, 2)          # contact onset position
        # stiffness: slope of estimate vs position over the contact frames
        slope = np.polyfit(pc - x0, fc, 1)[0]                       # A per count
        span = np.percentile(pc, 98) - np.percentile(pc, 2)
        k_nm = slope * KT / R_CONTACT / (2 * np.pi * R_CONTACT / 4000 * 1e3)   # N per mm at the contact
        print(f"   {n:12s}: contact {100*contact.mean():4.1f}% of frames; position span over contact {span:5.0f} counts "
              f"({span*2*np.pi*R_CONTACT/4000*1e3:5.1f} mm); force range {np.percentile(fc,5)*KT/R_CONTACT:.2f}..{np.percentile(fc,95)*KT/R_CONTACT:.2f} N; "
              f"stiffness fit {k_nm:7.2f} N/mm (corr {np.corrcoef(pc, fc)[0,1]:+.2f})")


if __name__ == "__main__":
    main()
