"""
Measure the motor parameters MC Workbench assumed (pmsm_motor_parameters.h),
on the running ESCs, from the Pi. Needs firmware CHANGE 11 (DIAG3: bus
voltage and 100 ms means of Vq, Vd, Iq). TEST_LOG 2026-11-23.

    python3 motor_params.py spin  --node B                 # shaft free
    python3 motor_params.py spin  --node A --speeds 0.5,1,1.5,2
    python3 motor_params.py stall --node A                 # A against the rigid stop
    python3 motor_params.py stall --node B                 # hold B's handle firmly
    python3 motor_params.py fit logs/motor_params_*.csv    # re-fit offline

Physics (amplitude-invariant dq frame, encoder-commutated FOC, Id = 0):
    Vq = Rs*Iq + lambda*we (+ Lq*dIq/dt, + dead-time ~ c*sign(I))
    phase-peak volts = Vq_s16 / 32768 * Vbus / sqrt(3)   (PWMC_SetPhaseVoltage)
    we = pp * w_mech;  Kt = 1.5 * pp * lambda  [N*m per A peak]
    Ke [Vrms ph-ph per krpm] = lambda * sqrt(3/2) * pp * 2*pi*1000/60
    J = Kt / G_rad, with G the measured plant gain (TEST_LOG 2026-11-15)

spin:  velocity PI in direct SET_IQ mode at 500 Hz, each speed in both
       directions; DIAG3 rows from the steady part of each step. Fit
       Vq = lambda*we + Rs*Iq + c*sign(we). Iq is small (friction only), so
       Rs is weakly determined here -- use stall for Rs.
stall: shaft blocked; steady Iq steps of both signs (A: one sign, into the
       stop -- pass --sign). Fit Vq = Rs*Iq + c*sign(Iq). Aborts if the shaft
       moves more than STALL_MOVE counts.
Ls is NOT measured: it needs a 16 kHz current-step capture (not in firmware).
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

POLE_PAIRS = 7
ENC = 4000
NODES = {"A": 0x000, "B": 0x020}
G_MEAS = {"A": 4.082e6, "B": 8.877e6}      # counts/s^2 per A (TEST_LOG 2026-11-15)
WB = {"Rs": 5.29, "Ls": 1.058e-3, "Ke": 5.0}  # pmsm_motor_parameters.h
IQ_MAX = 0.3
RATE_HZ = 500
SETTLE_S = 1.5
STALL_MOVE = 40          # counts


def lam_from_ke(ke):
    return ke / (math.sqrt(1.5) * POLE_PAIRS * 2 * math.pi * 1000 / 60)


def ke_from_lam(lam):
    return lam * math.sqrt(1.5) * POLE_PAIRS * 2 * math.pi * 1000 / 60


def report(node, lam=None, rs=None, c=None, extra=""):
    out = {}
    lam_wb = lam_from_ke(WB["Ke"])
    if lam is not None:
        kt = 1.5 * POLE_PAIRS * lam
        g_rad = G_MEAS[node] * 2 * math.pi / ENC
        out.update(lambda_Vs=lam, Ke_Vrms_krpm=ke_from_lam(lam), Kt_Nm_A=kt, J_kgm2=kt / g_rad)
        print(f"  flux linkage  {lam * 1e3:.3f} mV*s/rad   (Workbench {lam_wb * 1e3:.3f}: "
              f"{100 * (lam / lam_wb - 1):+.1f} %)")
        print(f"  Ke            {ke_from_lam(lam):.2f} Vrms/krpm   (Workbench {WB['Ke']:.2f})")
        print(f"  Kt            {kt * 1e3:.2f} mN*m/A   (Workbench-implied {1.5 * POLE_PAIRS * lam_wb * 1e3:.2f})")
        print(f"  inertia J     {kt / g_rad:.3e} kg*m^2   (= Kt / G, G {G_MEAS[node]:.3e} counts/s^2/A)")
    if rs is not None:
        out["Rs_ohm"] = rs
        print(f"  Rs            {rs:.3f} ohm   (Workbench {WB['Rs']:.2f}: {100 * (rs / WB['Rs'] - 1):+.1f} %)")
    if c is not None:
        out["deadtime_V"] = c
        print(f"  dead-time / offset term {c * 1e3:+.0f} mV")
    if extra:
        print("  " + extra)
    return out


# ------------------------------------------------------------------ fits ----
def fit_spin(steps):
    """steps: list of dicts with we (rad/s elec), vq (V), iq (A)."""
    we = np.array([s["we"] for s in steps])
    vq = np.array([s["vq"] for s in steps])
    iq = np.array([s["iq"] for s in steps])
    X = np.vstack([we, iq, np.sign(we)]).T
    (lam, rs, c), *_ = np.linalg.lstsq(X, vq, rcond=None)
    # lambda alone with Rs fixed at Workbench, as a robustness check
    lam_fixed = float(np.linalg.lstsq(np.vstack([we, np.sign(we)]).T, vq - WB["Rs"] * iq, rcond=None)[0][0])
    resid = vq - X @ np.array([lam, rs, c])
    return float(lam), float(rs), float(c), lam_fixed, float(np.sqrt(np.mean(resid ** 2)))


def fit_stall(steps):
    iq = np.array([s["iq"] for s in steps])
    vq = np.array([s["vq"] for s in steps])
    X = np.vstack([iq, np.sign(iq)]).T if len(set(np.sign(iq))) > 1 else np.vstack([iq, np.ones(len(iq))]).T
    (rs, c), *_ = np.linalg.lstsq(X, vq, rcond=None)
    resid = vq - X @ np.array([rs, c])
    return float(rs), float(c), float(np.sqrt(np.mean(resid ** 2)))


# --------------------------------------------------------------- hardware ----
class Rig:
    def __init__(self, node, channel, interface):
        from can_interface import MotorCANInterface, COUPLE_MODE_OFF
        from estimators import AlphaBetaVelocity
        self.n = node
        self.i = MotorCANInterface(channel=channel, bustype=interface, node_base=NODES[node])
        self.i.start_listening()
        self.vel = AlphaBetaVelocity(30.0)
        self.v = 0.0
        self.x = None
        self.started = False
        self.COUPLE_MODE_OFF = COUPLE_MODE_OFF

    def start(self):
        t0 = time.time()
        while self.i.get_telemetry()["enc_count_unwrapped"] is None:
            if time.time() - t0 > 2:
                raise SystemExit(f"no TELEM from {self.n}")
            time.sleep(0.02)
        self.i.send_couple_mode(self.COUPLE_MODE_OFF)
        self.i.send_start()
        self.started = True
        t0 = time.time()
        while not self.i.get_debug().get("run"):
            if time.time() - t0 > 5:
                raise SystemExit(f"{self.n} did not reach RUN (ESC power on?)")
            time.sleep(0.02)
        time.sleep(0.3)
        self.i.drain_records()
        self.i.drain_diag3_history()
        if self.i.get_diag().get("diag3") is None:
            time.sleep(0.3)
            if self.i.get_diag().get("diag3") is None:
                raise SystemExit("no DIAG3 frames: is CHANGE 11 flashed?")

    def poll(self):
        for r in self.i.drain_records():
            self.x = r["enc"]
            self.v = self.vel.update(r["mcu_time_us"], self.x)
        tel = self.i.get_telemetry()
        if tel["last_rx_time"] is None or time.time() - tel["last_rx_time"] > 0.05:
            raise RuntimeError("stale telemetry")

    def stop(self):
        try:
            self.i.send_set_iq(0.0)
            if self.started:
                time.sleep(0.02)
                self.i.send_stop()
            time.sleep(0.1)
        finally:
            self.i.stop_listening()


def run_spin(rig, speeds_rev_s, dwell):
    """Velocity PI per step; returns step summaries and raw DIAG3 rows."""
    g = G_MEAS[rig.n]
    kp = 30.0 / g          # ~30 rad/s velocity bandwidth
    ki = kp * 8.0
    steps, raw = [], []
    targets = [s * sgn for s in speeds_rev_s for sgn in (+1, -1)]
    dt = 1.0 / RATE_HZ
    for tgt in targets:
        vref = tgt * ENC
        integ = 0.0
        t0 = deadline = time.monotonic()
        v_samples = []
        while True:
            el = time.monotonic() - t0
            if el > SETTLE_S + dwell:
                break
            rig.poll()
            if abs(rig.v) > 1.6 * abs(vref) + 4000:
                raise RuntimeError(f"speed {rig.v:.0f} counts/s runaway (target {vref:.0f})")
            e = vref - rig.v
            u = kp * e + integ
            if abs(u) < IQ_MAX:
                integ += ki * e * dt
            u = max(-IQ_MAX, min(IQ_MAX, u))
            rig.i.send_set_iq(u)
            if el > SETTLE_S:
                v_samples.append(rig.v)
            if el <= SETTLE_S:
                rig.i.drain_diag3_history()        # discard the transient
            deadline += dt
            sl = deadline - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            else:
                deadline = time.monotonic()
        rows = rig.i.drain_diag3_history()
        rows = rows[1:]                             # first may straddle the settle edge
        if len(rows) < 5:
            raise RuntimeError("too few DIAG3 rows in a step")
        a = np.array(rows)
        vmean = float(np.mean(v_samples))
        we = vmean / ENC * 2 * math.pi * POLE_PAIRS
        st = {"target_rev_s": tgt, "speed_cnt_s": vmean, "speed_std": float(np.std(v_samples)),
              "we": we, "vbus": float(a[:, 1].mean()), "vq": float(a[:, 2].mean()),
              "vd": float(a[:, 3].mean()), "iq": float(a[:, 4].mean()), "n": len(rows)}
        steps.append(st)
        raw += [(tgt,) + tuple(r) for r in rows]
        print(f"  {tgt:+5.2f} rev/s: speed {vmean:+8.0f} (std {st['speed_std']:5.0f}) counts/s, "
              f"Vq {st['vq']:+.3f} V, Vd {st['vd']:+.3f} V, Iq {st['iq'] * 1e3:+6.1f} mA, Vbus {st['vbus']:.2f} V")
    rig.i.send_set_iq(0.0)
    return steps, raw


def run_stall(rig, currents, sign):
    steps, raw = [], []
    dt = 1.0 / RATE_HZ
    seq = []
    for c in currents:
        seq += [c * sign] if sign else [c, -c]
    rig.poll()
    x0 = rig.x
    for iq in seq:
        t0 = deadline = time.monotonic()
        while time.monotonic() - t0 < SETTLE_S + 2.0:
            rig.poll()
            if abs(rig.x - x0) > STALL_MOVE:
                raise RuntimeError(f"shaft moved {rig.x - x0:+.0f} counts: it is not blocked")
            rig.i.send_set_iq(iq)
            if time.monotonic() - t0 <= SETTLE_S:
                rig.i.drain_diag3_history()
            deadline += dt
            sl = deadline - time.monotonic()
            if sl > 0:
                time.sleep(sl)
        rig.i.send_set_iq(0.0)
        rows = rig.i.drain_diag3_history()[1:]
        a = np.array(rows)
        st = {"iq_cmd": iq, "vbus": float(a[:, 1].mean()), "vq": float(a[:, 2].mean()),
              "vd": float(a[:, 3].mean()), "iq": float(a[:, 4].mean()), "n": len(rows)}
        steps.append(st)
        raw += [(iq,) + tuple(r) for r in rows]
        print(f"  Iq {iq:+.2f} A: measured {st['iq'] * 1e3:+6.1f} mA, Vq {st['vq']:+.3f} V, "
              f"Vd {st['vd']:+.3f} V, Vbus {st['vbus']:.2f} V")
        time.sleep(0.3)                             # let it cool between steps
    return steps, raw


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("mode", choices=["spin", "stall", "fit"])
    ap.add_argument("files", nargs="*")
    ap.add_argument("--node", choices=["A", "B"], default="B")
    ap.add_argument("--speeds", default="1,2,4,6,8", help="rev/s (spin)")
    ap.add_argument("--dwell", type=float, default=3.0)
    ap.add_argument("--currents", default="0.1,0.15,0.2,0.25,0.3", help="A (stall)")
    ap.add_argument("--sign", type=int, choices=[-1, 0, 1], default=0,
                    help="stall: 0 = both signs (held by hand), +1/-1 = one sign (into a stop)")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--interface", default="socketcan")
    a = ap.parse_args()

    if a.mode == "fit":
        for f in a.files:
            d = json.load(open(f[:-4] + ".json" if f.endswith(".csv") else f))
            print(f"== {f}  [{d['mode']}, node {d['node']}]")
            if d["mode"] == "spin":
                lam, rs, c, lamf, r = fit_spin(d["steps"])
                report(d["node"], lam=lam, c=c, extra=f"(Rs from this fit {rs:.2f} ohm, weakly determined; "
                       f"lambda with Rs fixed at Workbench {lamf * 1e3:.3f} mV*s/rad; resid {r * 1e3:.1f} mV)")
            else:
                rs, c, r = fit_stall(d["steps"])
                report(d["node"], rs=rs, c=c, extra=f"(resid {r * 1e3:.1f} mV)")
        return 0

    os.makedirs("logs", exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = f"logs/motor_params_{a.mode}_{a.node}_{stamp}"
    rig = Rig(a.node, a.channel, a.interface)
    steps, raw, err = [], [], None
    try:
        rig.start()
        print(f"{a.mode} on motor {a.node}:")
        if a.mode == "spin":
            steps, raw = run_spin(rig, [float(s) for s in a.speeds.split(",")], a.dwell)
        else:
            steps, raw = run_stall(rig, [float(s) for s in a.currents.split(",")], a.sign)
    except (RuntimeError, KeyboardInterrupt) as e:
        err = str(e) or "interrupted"
        print("ABORT:", err)
    finally:
        rig.stop()
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "rx_time", "vbus_v", "vq_v", "vd_v", "iq_a"])
        w.writerows(raw)
    res = {}
    if steps and a.mode == "spin" and len(steps) >= 4:
        lam, rs, c, lamf, r = fit_spin(steps)
        res = report(a.node, lam=lam, c=c, extra=f"(Rs from this fit {rs:.2f} ohm, weakly determined; "
                     f"lambda with Rs fixed at Workbench {lamf * 1e3:.3f} mV*s/rad; resid {r * 1e3:.1f} mV)")
    elif steps and a.mode == "stall" and len(steps) >= 2:
        rs, c, r = fit_stall(steps)
        res = report(a.node, rs=rs, c=c, extra=f"(resid {r * 1e3:.1f} mV)")
    json.dump({"mode": a.mode, "node": a.node, "steps": steps, "result": res, "error": err,
               "args": vars(a)}, open(base + ".json", "w"), indent=1)
    print("log:", base + ".csv/.json")
    return 1 if err else 0


if __name__ == "__main__":
    sys.exit(main())
