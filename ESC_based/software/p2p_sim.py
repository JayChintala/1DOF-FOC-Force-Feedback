"""
Offline sim: peer-to-peer (P2P) bilateral control with disturbance observers
(Keio-style acceleration-based 4-channel control) vs the existing PEER spring.

Run:  python3 p2p_sim.py            (prints the comparison tables)
      python3 p2p_sim.py --sweep    (also sweeps the DOB cutoff and gains)

Toy model, for trends and for choosing starting gains -- not a substitute for
the bench. What it does model, because each one limits this rig:
  * each ESC ticks at 1 kHz on its own clock (B offset 0.25 ms from A);
  * the peer's TELEM (encoder + 1 ms mean Iq) arrives ~0.13 ms after the
    peer's tick and is used on this ESC's next tick (0.1-1.1 ms old);
  * 4000 count/rev encoder, quantized; velocity from 1 ms differences + EMA;
  * current loop: first order, 440 Hz (the 50 % current-loop gain), the
    reference applied immediately (CommandIq), clamped at Iq_max 0.8 A;
  * 9 mA current wander (the measured 1 ms Iq std at 0 A, 50 % gain);
  * Coulomb friction with a stiction band; G_A 4.082e6 (3D arm), G_B 8.877e6.

The P2P law, run identically on both ESCs ("own" i, "peer" j):
  DOB   z_i  += T*g*(Iq_i + (g/G_i)*v_i - z_i);   d_i = z_i - (g/G_i)*v_i
        (d_i: disturbance in amps -- friction, gravity, the hand; Iq_i is the
         MEASURED 1 ms mean, so the observer cannot wind up at the clamp)
  RFOB  f_i  = -(d_i - model_i)   external force on shaft i, A-equivalent,
                                   + = pushing toward +counts
  modes ddx_com  = Kf * (f_i + f_j)                       force:    f_i + f_j -> 0
        ddx_diff = -Kp*(x_i - x_j) - Kv*(v_i - v_j)       position: x_i - x_j -> 0
  ref   ddx_i    = (ddx_com + ddx_diff) / 2
  out   Iq_i     = ddx_i / G_i + d_i

Kf sets the inertia you feel: a push f on one side accelerates both shafts
at Kf*f/2, so the apparent mass is 2/Kf (A per count/s^2). Passive coupling
of the two rotors is 1/2.80e6; Kf = 2*G_B makes the pair feel like motor B
alone.
"""

import argparse
import math
import random

G = {"A": 4.082e6, "B": 8.877e6}           # counts/s^2 per A (TEST_LOG 11-15)
FRIC = {"A": 0.025, "B": 0.017}            # Coulomb, A (follower model / leader_comp_B)
STICK = {"A": 0.03, "B": 0.02}             # breakaway, A (kept near Coulomb so
                                           # the model can still show a limit cycle)
IQ_MAX = 0.8
TAU_I = 1.0 / (2 * math.pi * 440.0)        # current loop, 50 % gain
IQ_WANDER = 0.009                          # A, 1 ms std
DT = 1.0 / 16000.0                         # plant step
TS = 1e-3                                  # controller tick
OFFSET = {"A": 0.0, "B": 0.25e-3}          # tick phase
WIRE = 0.13e-3                             # TELEM on the wire
VEL_ALPHA = 0.5


class Esc:
    """One ESC's controller state. Mirrors couple_ctrl.c."""

    def __init__(self, name, law, p):
        self.n, self.law, self.p = name, law, p
        self.peer = "B" if name == "A" else "A"
        self.x = self.v = None
        self.px = self.pv = None
        self.p_seq = -1
        self.z = {"own": None, "peer": None}
        self.d = {"own": 0.0, "peer": 0.0}
        self.x0 = self.px0 = None
        self.iq_cmd = 0.0

    def dob(self, key, g_plant, iq, v):
        g = self.p["g"]
        k = g / g_plant
        if self.z[key] is None:
            self.z[key] = iq + k * v   # start at d = iq (at rest: d = current)
        self.z[key] += TS * g * (iq + k * v - self.z[key])
        self.d[key] = self.z[key] - k * v
        return self.d[key]

    def tick(self, enc, iq_mean, peer_frame):
        # own velocity: 1 ms difference + EMA (as UpdateOwn)
        if self.x is None:
            self.x, self.v = enc, 0.0
        else:
            raw = (enc - self.x) / TS
            self.v = VEL_ALPHA * raw + (1 - VEL_ALPHA) * self.v
            self.x = enc
        # peer: only on a new frame (as UpdatePeer)
        if peer_frame is not None and peer_frame["seq"] != self.p_seq:
            if self.px is None:
                self.px, self.pv = peer_frame["enc"], 0.0
            else:
                dt = peer_frame["ts"] - self.p_ts
                raw = (peer_frame["enc"] - self.px) / dt if dt > 0 else 0.0
                self.pv = VEL_ALPHA * raw + (1 - VEL_ALPHA) * self.pv
                self.px = peer_frame["enc"]
            self.p_ts = peer_frame["ts"]
            self.p_seq = peer_frame["seq"]
            if self.law == "p2p":
                self.dob("peer", G[self.peer], peer_frame["iq"], self.pv)
        if self.law == "p2p":
            self.dob("own", G[self.n], iq_mean, self.v)
        if self.px is None:
            return 0.0           # no peer yet: not engaged
        if self.x0 is None:
            self.x0, self.px0 = self.x, self.px
        p = self.p
        err = (self.x - self.x0) - (self.px - self.px0)
        derr = self.v - self.pv
        if self.law == "peer":
            iq = -(p["kp"] * err + p["kd"] * derr)
        else:
            f_own = -(self.d["own"] - p["comp"] * self.model(self.n, self.v))
            f_peer = -(self.d["peer"] - p["comp"] * self.model(self.peer, self.pv))
            acc = 0.5 * (p["kf"] * (f_own + f_peer) - p["kpa"] * err - p["kva"] * derr)
            iq = acc / G[self.n] + self.d["own"]
        self.iq_cmd = max(-IQ_MAX, min(IQ_MAX, iq))
        return self.iq_cmd

    @staticmethod
    def model(name, v):
        # friction feedforward the RFOB subtracts (comp = 1): smooth Coulomb
        return FRIC[name] * max(-1.0, min(1.0, v / 300.0))   # as couple_ctrl.c


def simulate(law, p, scenario, T=3.0, seed=1):
    rnd = random.Random(seed)
    x = {"A": 0.0, "B": 0.0}
    v = {"A": 0.0, "B": 0.0}
    i_act = {"A": 0.0, "B": 0.0}
    i_ref = {"A": 0.0, "B": 0.0}
    wander = {"A": 0.0, "B": 0.0}
    acc_iq = {"A": 0.0, "B": 0.0}
    n_iq = {"A": 0, "B": 0}
    esc = {k: Esc(k, law, p) for k in "AB"}
    frames = {"A": [], "B": []}       # (arrive_t, frame) sent by that ESC
    latest = {"A": None, "B": None}   # latest frame FROM that ESC, as seen by the other
    seq = {"A": 0, "B": 0}
    next_tick = dict(OFFSET)
    log = []
    steps = int(T / DT)
    for s in range(steps):
        t = s * DT
        # deliver frames
        for k in "AB":
            while frames[k] and frames[k][0][0] <= t:
                latest[k] = frames[k].pop(0)[1]
        # controller ticks
        for k in "AB":
            if t >= next_tick[k]:
                next_tick[k] += TS
                enc = math.floor(x[k])
                iq_mean = acc_iq[k] / max(1, n_iq[k])
                acc_iq[k], n_iq[k] = 0.0, 0
                peer = "B" if k == "A" else "A"
                i_ref[k] = esc[k].tick(enc, iq_mean, latest[peer])
                seq[k] += 1
                frames[k].append((t + WIRE, {"seq": seq[k], "enc": enc, "iq": iq_mean, "ts": t}))
                wander[k] = rnd.gauss(0.0, IQ_WANDER)
        # plant
        f_ext = scenario(t, x, v)
        for k in "AB":
            i_act[k] += DT / TAU_I * (i_ref[k] + wander[k] - i_act[k])
            acc_iq[k] += i_act[k]
            n_iq[k] += 1
            drive = i_act[k] + f_ext[k]
            if abs(v[k]) < 20.0 and abs(drive) < STICK[k]:
                v[k] = 0.0
                continue
            fr = FRIC[k] * (1 if v[k] > 0 else -1 if v[k] < 0 else (1 if drive > 0 else -1))
            v[k] += G[k] * (drive - fr) * DT
            x[k] += v[k] * DT
        if s % 16 == 0:
            log.append((t, x["A"], x["B"], v["A"], v["B"], i_act["A"], i_act["B"],
                        f_ext["A"], f_ext["B"]))
    return log


# ---- scenarios: return external force on each shaft, A-equivalent ----

def hands_off_kick(t, x, v):
    """0.3 A on A for 5 ms at t = 0.2 s, then nobody touches it."""
    return {"A": 0.3 if 0.2 <= t < 0.205 else 0.0, "B": 0.0}


def hand_sweep(kh=2e-3, bh=2e-5, amp=1500.0, hz=1.0):
    """Hand on A tracking a 1 Hz sine through a grip impedance; B free."""
    def f(t, x, v):
        xd = amp * math.sin(2 * math.pi * hz * t) * min(1.0, t / 0.5)
        return {"A": kh * (xd - x["A"]) - bh * v["A"], "B": 0.0}
    return f


def hand_into_wall(kh=2e-3, bh=2e-5, wall=300.0, k_env=0.05, b_env=5e-5):
    """Hand pushes A toward 450 counts (0.3 A if A stopped at the wall);
    B hits a stiff wall at 300."""
    def f(t, x, v):
        xd = 450.0 * min(1.0, t / 1.0)
        fb = 0.0
        if x["B"] > wall:
            fb = -(k_env * (x["B"] - wall) + b_env * max(0.0, v["B"]))
            fb = min(0.0, fb)
        return {"A": kh * (xd - x["A"]) - bh * v["A"], "B": fb}
    return f


def hold_and_push(k_hold=5e-3, b_hold=3e-5, push=0.3):
    """Someone holds B firmly (grip impedance around 0); A gets a 0.3 A
    push from 0.2 s to 1.5 s. How far A gives is the springiness."""
    def f(t, x, v):
        return {"A": push if 0.2 <= t < 1.5 else 0.0,
                "B": -k_hold * x["B"] - b_hold * v["B"]}
    return f


def band_rms(sig, fs, lo, hi):
    """RMS of sig in [lo, hi] Hz via a plain DFT (short signals only)."""
    n = len(sig)
    m = sum(sig) / n
    sig = [s - m for s in sig]
    tot = 0.0
    for kf in range(1, n // 2):
        fr = kf * fs / n
        if lo <= fr <= hi:
            re = sum(sig[i] * math.cos(2 * math.pi * kf * i / n) for i in range(n))
            im = sum(sig[i] * math.sin(2 * math.pi * kf * i / n) for i in range(n))
            tot += 2 * (re * re + im * im) / (n * n)
    return math.sqrt(tot)


def metrics(law, p):
    out = {}
    # 1. hands off: does it ring after a kick?
    lg = simulate(law, p, hands_off_kick, T=1.2)
    tail = [r[1] - r[2] for r in lg if 0.7 <= r[0] < 1.2]
    out["ring_pp"] = max(tail) - min(tail)
    out["ring_40_90"] = band_rms(tail[::2], 500.0, 40, 90) if len(tail) > 10 else 0.0
    iq_tail = [r[5] for r in lg if 0.7 <= r[0] < 1.2]
    out["iq_pp"] = max(iq_tail) - min(iq_tail)
    out["kick_err"] = max(abs(r[1] - r[2]) for r in lg)
    # 2. free motion: springiness and how hard you have to push
    lg = simulate(law, p, hand_sweep(amp=600.0, hz=3.0), T=3.0)
    sl = [r for r in lg if r[0] >= 1.0]
    out["free_err_rms"] = math.sqrt(sum((r[1] - r[2]) ** 2 for r in sl) / len(sl))
    out["free_err_max"] = max(abs(r[1] - r[2]) for r in sl)
    out["hand_rms"] = math.sqrt(sum(r[7] ** 2 for r in sl) / len(sl))
    out["sat"] = sum(1 for r in sl if max(abs(r[5]), abs(r[6])) >= 0.79) / len(sl)
    # 3. hold B, push A: how far does A give?
    lg = simulate(law, p, hold_and_push(), T=2.0)
    out["give_peak"] = max(r[1] - r[2] for r in lg if r[0] < 1.5)   # A-B, the spring stretch
    out["give_ss"] = sum(r[1] for r in lg if 1.2 <= r[0] < 1.5) / sum(1 for r in lg if 1.2 <= r[0] < 1.5)
    out["give_B"] = sum(r[2] for r in lg if 1.2 <= r[0] < 1.5) / sum(1 for r in lg if 1.2 <= r[0] < 1.5)
    # 4. wall: does the hand feel the wall force?
    lg = simulate(law, p, hand_into_wall(), T=3.0)
    sl = [r for r in lg if 2.0 <= r[0] < 3.0]
    fh = sum(r[7] for r in sl) / len(sl)
    fw = sum(r[8] for r in sl) / len(sl)
    out["wall_hand"] = fh
    out["wall_env"] = fw
    out["wall_pen"] = sum(r[2] for r in sl) / len(sl) - 300.0
    out["wall_err"] = sum(r[1] - r[2] for r in sl) / len(sl)
    tail = [r[1] - r[2] for r in sl]
    out["wall_pp"] = max(tail) - min(tail)
    return out


PEER_BEST = {"kp": 0.007, "kd": 2.5e-5}             # TEST_LOG 2026-11-15
PEER_SOFT = {"kp": 0.004, "kd": 2.5e-5}             # the untried suggestion


def p2p(g=150.0, wn_hz=12.0, zeta=1.0, kf_scale=1.0, comp=1.0):
    wn = 2 * math.pi * wn_hz
    return {"g": g, "kpa": wn * wn, "kva": 2 * zeta * wn,
            "kf": kf_scale * 2 * G["B"], "comp": comp}


def row(name, m):
    print(f"{name:34s} {m['ring_pp']:6.1f} {m['ring_40_90']:6.2f} {m['iq_pp']*1e3:6.0f} "
          f"{m['kick_err']:6.0f} | {m['give_peak']:5.0f} {m['give_ss']:5.0f} {m['give_B']:5.0f} | {m['free_err_rms']:6.1f} {m['free_err_max']:6.0f} "
          f"{m['hand_rms']*1e3:6.1f} {m['sat']*100:5.1f} | {m['wall_hand']*1e3:6.0f} "
          f"{-m['wall_env']*1e3:6.0f} {m['wall_err']:6.1f} {m['wall_pp']:6.1f}")


def header():
    print(f"{'':34s} {'hands-off after kick':^27s} | {'hold B,push A':^17s} | {'free 3 Hz sweep, hand on A':^26s} | "
          f"{'hand pushes B into wall':^27s}")
    print(f"{'law':34s} {'pp':>6s} {'40-90':>6s} {'iq pp':>6s} {'kick':>6s} | "
          f"{'gap':>5s} {'A ss':>5s} {'B ss':>5s} | {'err':>6s} {'max':>6s} {'hand':>6s} {'sat%':>5s} | {'hand':>6s} "
          f"{'wall':>6s} {'err':>6s} {'pp':>6s}")
    print(f"{'':34s} {'cnt':>6s} {'cnt':>6s} {'mA':>6s} {'cnt':>6s} | {'cnt':>5s} {'cnt':>5s} {'cnt':>5s} | {'rms':>6s} "
          f"{'cnt':>6s} {'mA rms':>6s} {'':>5s} | {'mA':>6s} {'mA':>6s} {'cnt':>6s} "
          f"{'cnt':>6s}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    header()
    row("PEER Kp 0.007 Kd 2.5e-5 (11-15)", metrics("peer", PEER_BEST))
    row("PEER Kp 0.004 Kd 2.5e-5", metrics("peer", PEER_SOFT))
    row("P2P g150 wn12 z1 Kf=2G_B comp", metrics("p2p", p2p()))
    row("P2P g150 wn12 z1 Kf=2G_B no comp", metrics("p2p", p2p(comp=0.0)))
    if a.sweep:
        for g in (60.0, 100.0, 150.0, 250.0, 400.0):
            row(f"P2P g{g:g}", metrics("p2p", p2p(g=g)))
        for wn in (6.0, 9.0, 15.0, 20.0, 25.0):
            row(f"P2P wn{wn:g}", metrics("p2p", p2p(wn_hz=wn)))
        for z in (0.7, 1.5):
            row(f"P2P zeta{z:g}", metrics("p2p", p2p(zeta=z)))
        for kf in (0.5, 1.5, 2.0, 3.0):
            row(f"P2P Kf x{kf:g}", metrics("p2p", p2p(kf_scale=kf)))


if __name__ == "__main__":
    main()
