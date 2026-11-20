"""
Peer-to-peer (P2P) teleop -- symmetric bilateral control, no leader and no
follower. Firmware CHANGE 9, COUPLE_MODE_P2P (couple_ctrl.h "P2P mode").

The second control architecture beside teleop_test.py (leader B drives
follower A from the Pi, force reflected back). Here both ESCs run the SAME
law on the ESC at 1 kHz, the Pi out of the loop:

  * a disturbance observer (DOB) on each motor cancels its friction, gravity
    and load, so each behaves like a nominal inertia;
  * a reaction force observer (RFOB) estimates the external force on each
    shaft -- your hand on one, your other hand or an object on the other;
  * differential mode  x_A - x_B -> 0   (position servo, wn / zeta)
    common mode        f_A + f_B -> 0   (force servo: action = reaction, Kf)

Why it is not springy like PEER mode: in PEER mode your push has to stretch
a spring (Kp, A/count) before the other shaft moves. Here your push drives
the COMMON mode -- both shafts accelerate together -- and the DOB's integral
action drives the steady position gap to zero (p2p_sim.py: hold one, push
the other with 0.3 A -> 2 counts of gap vs 45 in PEER mode).

Run from ESC_based/software/ on the Pi, venv active, hands OFF both shafts
until it says "Engaged":
    python3 p2p_teleop.py                      # defaults, until Ctrl+C
    python3 p2p_teleop.py --wn 25 --zeta 1.2   # stiffer differential mode
    python3 p2p_teleop.py --kf 150             # lighter (apparent mass 2/Kf)
    python3 p2p_teleop.py --fric 100           # cancel nominal Coulomb friction
Tuning knobs, all SET_PARAM (RAM-only, re-sent every run):
    --wn    differential bandwidth, Hz (1..40). Sim: unstable above Kv ~500/s,
            i.e. wn 40 @ zeta 1 or wn 30 @ zeta 1.5. Start low.
    --zeta  its damping ratio (0.2..3)
    --kf    force gain, % of 2*G_B (0..300). 100: the pair feels like motor B
            alone. Sim: 300 % went unstable against a stiff wall.
    --dob-g observer cutoff, rad/s (20..1000). Higher: stiffer, noisier.
    --fric  % of nominal Coulomb friction the RFOB subtracts (0..150).

The live line shows each side's estimated EXTERNAL force (A-equivalent,
1.058 N/A at the tip) from a Pi-side mirror of the firmware observers, and
their sum -- which the force servo drives to zero.

Logs: logs/p2p_<settings>.csv/.json/.png (same columns as coupling_monitor).
"""

import argparse
import bisect
import json
import math
import os
import sys
import time

from can_interface import (COUPLE_FLAG_PREDICT, COUPLE_MODE_OFF, STATUS_ENGAGED,
                           COUPLE_MODE_P2P, PEER_AGE_SATURATED_MS, TRIP_NAMES,
                           MotorCANInterface)
from coupling_monitor import (DBG_TIMEOUT_S, DRAIN_PERIOD_S, ENGAGE_TIMEOUT_S,
                              LOG_DIR, NODE_BASES, START_TIMEOUT_S,
                              TELEM_TIMEOUT_S, RunLog, status_tag,
                              unique_log_path, wait_for)
from plot_coupling import plot_coupling_log

# Must match couple_ctrl.h COUPLE_P2P_* (the mirror observer only; the ESCs
# use their own compiled-in copies).
G = {"A": 4.082e6, "B": 8.877e6}     # counts/s^2 per A
FC = {"A": 0.025, "B": 0.017}        # nominal Coulomb, A
FRIC_V0 = 300.0                      # counts/s
VEL_ALPHA = 0.5                      # COUPLE_VEL_FILTER_ALPHA

WN_HZ, ZETA, KF_PCT, DOB_G, FRIC_PCT = 20.0, 1.0, 100, 150, 0
IQ_MAX_A = 0.8
PRINT_PERIOD_S = 0.2
NEWTON_PER_A = 1.058                 # tip force per amp (TEST_LOG 2026-11-15)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--wn", type=float, default=WN_HZ, help="Hz, 1..40")
    p.add_argument("--zeta", type=float, default=ZETA, help="0.2..3")
    p.add_argument("--kf", type=int, default=KF_PCT, help="%% of 2*G_B, 0..300")
    p.add_argument("--dob-g", type=int, default=DOB_G, help="rad/s, 20..1000")
    p.add_argument("--fric", type=int, default=FRIC_PCT, help="%%, 0..150")
    p.add_argument("--iq-max", type=float, default=IQ_MAX_A, help="A, <= 0.8")
    p.add_argument("--predict", action="store_true",
                   help="extrapolate the peer position over its ~1 ms age")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds; 0 = until Ctrl+C (default)")
    p.add_argument("--channel", default="can0")
    p.add_argument("--interface", default="socketcan")
    p.add_argument("--no-plot", action="store_true")
    a = p.parse_args()
    if not 1.0 <= a.wn <= 40.0:
        p.error("--wn must be 1..40 Hz")
    if not 0.2 <= a.zeta <= 3.0:
        p.error("--zeta must be 0.2..3")
    if not 0 <= a.kf <= 300:
        p.error("--kf must be 0..300 %")
    if not 20 <= a.dob_g <= 1000:
        p.error("--dob-g must be 20..1000 rad/s")
    if not 0 <= a.fric <= 150:
        p.error("--fric must be 0..150 %")
    if not 0 < a.iq_max <= 0.8:
        p.error("--iq-max must be in (0, 0.8]")
    kv = 2 * a.zeta * 2 * math.pi * a.wn
    if kv > 500:
        print(f"[warn] Kv = {kv:.0f}/s is past where p2p_sim.py went unstable "
              f"(~500/s). Trips will catch a runaway; expect buzz.")
    return a


class MirrorObserver:
    """Pi-side copy of one firmware DOB/RFOB (couple_ctrl.c DobStep), run on
    that motor's TELEM records. For display and the log summary only."""

    def __init__(self, name, g, fric_pct):
        self.G, self.fc = G[name], FC[name]
        self.g, self.comp = g, fric_pct / 100.0
        self.t = self.x = None
        self.v = 0.0
        self.z = None
        self.f = 0.0

    def reset(self):
        self.z = None

    def update(self, t_us, x, iq):
        if self.t is not None and t_us > self.t:
            dt = (t_us - self.t) * 1e-6
            self.v = VEL_ALPHA * (x - self.x) / dt + (1 - VEL_ALPHA) * self.v
        else:
            dt = 1e-3
        self.t, self.x = t_us, x
        k = self.g / self.G
        if self.z is None or abs(iq) > 1.2:
            self.z = iq + k * self.v
        a = min(1.0, self.g * dt)
        self.z += a * (iq + k * self.v - self.z)
        d = self.z - k * self.v
        fr = self.fc * max(-1.0, min(1.0, self.v / FRIC_V0))
        self.f = -(d - self.comp * fr)
        return self.f


def readout(t_rel, ifaces, pos0, obs):
    parts = [f"t={t_rel:6.1f}s"]
    pos = {}
    for n, iface in ifaces.items():
        tel = iface.get_telemetry()
        p = tel["enc_count_unwrapped"]
        pos[n] = 0 if p is None else p - pos0[n]
    parts.append(f"A {pos['A']:+7.0f}  B {pos['B']:+7.0f}  gap {pos['A'] - pos['B']:+5.0f} cnt")
    fa, fb = obs["A"].f, obs["B"].f
    parts.append(f"force A {fa * NEWTON_PER_A:+.2f} N  B {fb * NEWTON_PER_A:+.2f} N  "
                 f"sum {(fa + fb) * NEWTON_PER_A:+.2f} N")
    tags = []
    for n, iface in ifaces.items():
        d = iface.get_debug()
        iq = d["iq_cmd"] if d["iq_cmd"] is not None else 0.0
        tags.append(f"{n} {iq:+.2f}A {status_tag(d)}")
    parts.append(" ".join(tags))
    print(" | ".join(parts), flush=True)


def summarize(rows_by_motor, args):
    """Gap and force-tracking stats over the engaged rows, motors aligned on
    bus time (the two 1 kHz streams are asynchronous -- TEST_LOG 11-15)."""
    eng = {n: [r for r in rows if r["engaged"]] for n, rows in rows_by_motor.items()}
    if not eng.get("A") or not eng.get("B"):
        return None
    tb = [r["t"] for r in eng["B"]]
    pairs = []
    for ra in eng["A"]:
        i = bisect.bisect_left(tb, ra["t"])
        best = min((j for j in (i - 1, i) if 0 <= j < len(tb)),
                   key=lambda j: abs(tb[j] - ra["t"]), default=None)
        if best is not None and abs(tb[best] - ra["t"]) < 0.0006:
            pairs.append((ra, eng["B"][best]))
    if not pairs:
        return None
    gaps = sorted(abs(a["pos"] - b["pos"]) for a, b in pairs)
    pick = lambda q: gaps[min(len(gaps) - 1, int(q * len(gaps)))]
    contact = [(a["f"], b["f"]) for a, b in pairs if max(abs(a["f"]), abs(b["f"])) > 0.05]
    out = {"ticks": len(pairs), "gap_p50": pick(0.5), "gap_p95": pick(0.95),
           "gap_max": gaps[-1], "contact_ticks": len(contact)}
    if contact:
        num = sum(abs(fa + fb) for fa, fb in contact)
        den = sum(max(abs(fa), abs(fb)) for fa, fb in contact)
        out["force_mismatch_pct"] = 100.0 * num / den
    return out


def main():
    args = parse_args()
    names = ["A", "B"]
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = (f"p2p_WN{args.wn:g}_Z{args.zeta:g}_KF{args.kf}_G{args.dob_g}"
                 f"_FR{args.fric}_A{args.iq_max:g}{'_PRED' if args.predict else ''}")
    log_path = unique_log_path(base_name)

    ifaces = {n: MotorCANInterface(channel=args.channel, bustype=args.interface,
                                   node_base=NODE_BASES[n]) for n in names}
    for iface in ifaces.values():
        iface.start_listening()
    obs = {n: MirrorObserver(n, args.dob_g, args.fric) for n in names}
    log = RunLog(log_path)
    summary_rows = {n: [] for n in names}
    started = no_data = False
    trip_msg = None
    t_start = time.time()
    engaged_seen = {n: False for n in names}

    def drain():
        for n, iface in ifaces.items():
            recs = iface.drain_records()
            log.write(n, recs)
            for r in recs:
                eng = r["flags"] is not None and bool(r["flags"] & STATUS_ENGAGED)
                if eng and not engaged_seen[n]:
                    obs[n].reset()       # the firmware resets at engagement
                engaged_seen[n] = eng
                f = obs[n].update(r["mcu_time_us"], r["enc"], r["iq_meas"])
                summary_rows[n].append({"t": r["bus_time"], "pos": r["enc"] - pos0[n],
                                        "f": f, "engaged": eng})

    pos0 = {}
    try:
        if not wait_for(lambda: all(i.get_telemetry()["enc_count_unwrapped"] is not None
                                    for i in ifaces.values()), TELEM_TIMEOUT_S):
            print("No TELEM from both ESCs -- check power, CAN wiring, can0 up at 1 Mbit/s.")
            no_data = True
            return 1
        if not wait_for(lambda: all(i.get_debug()["flags"] is not None
                                    for i in ifaces.values()), DBG_TIMEOUT_S):
            print("TELEM but no DBG -- a board is not running ESC_based firmware.")
            no_data = True
            return 1
        pos0 = {n: i.get_telemetry()["enc_count_unwrapped"] for n, i in ifaces.items()}

        for iface in ifaces.values():
            iface.send_couple_local(0.0, args.iq_max)   # Kd_local unused in P2P
            iface.send_p2p_params(args.wn, args.zeta, args.kf, args.dob_g, args.fric)
        print(f"P2P: wn {args.wn:g} Hz  zeta {args.zeta:g}  Kf {args.kf}%  "
              f"DOB g {args.dob_g} rad/s  fric comp {args.fric}%  Iq_max {args.iq_max:g} A"
              + ("  predict" if args.predict else ""))
        print("Sending START...")
        for iface in ifaces.values():
            iface.send_start()
        started = True
        if not wait_for(lambda: all(i.get_debug().get("run") for i in ifaces.values()),
                        START_TIMEOUT_S):
            stuck = [n for n, i in ifaces.items() if not i.get_debug().get("run")]
            print(f"ESC {', '.join(stuck)} did not reach RUN (motor power on? fault?).")
            return 1

        print("HANDS OFF both shafts -- engaging once both are still...")
        for iface in ifaces.values():
            iface.send_couple_mode(COUPLE_MODE_P2P, COUPLE_FLAG_PREDICT if args.predict else 0)
        if not wait_for(lambda: all(i.get_debug().get("engaged") for i in ifaces.values()),
                        ENGAGE_TIMEOUT_S):
            for n, i in ifaces.items():
                d = i.get_debug()
                if d.get("engaged"):
                    continue
                why = "a shaft is still moving, or a peer Iq readback is out of range"
                if (d["peer_age_ms"] or 0) >= PEER_AGE_SATURATED_MS:
                    why = "it is not receiving the other ESC's TELEM"
                elif d.get("trip"):
                    why = f"trip: {TRIP_NAMES.get(d['trip'], d['trip'])}"
                print(f"ESC {n} did not engage: {why}. Is CHANGE 9 flashed on both?")
            return 1

        print(f"Engaged. Logging to {log_path}. Ctrl+C to stop.\n"
              "One hand per shaft: move one and the other follows; hold one and "
              "push the other to feel it.\n")
        # Re-zero at engagement: START re-aligns the encoders (counter load),
        # so counts captured before START carry an arbitrary offset.
        drain()
        pos0.update({n: i.get_telemetry()["enc_count_unwrapped"] for n, i in ifaces.items()})
        for n in names:
            summary_rows[n].clear()
        t_eng = time.monotonic()
        next_print = t_eng
        while args.duration == 0 or time.monotonic() - t_eng < args.duration:
            drain()
            now = time.monotonic()
            if now >= next_print:
                readout(time.time() - t_start, ifaces, pos0, obs)
                next_print = now + PRINT_PERIOD_S
            for n, iface in ifaces.items():
                d = iface.get_debug()
                if d["flags"] is not None and not d["requested"] and d["trip"]:
                    trip_msg = f"ESC {n} tripped: {TRIP_NAMES.get(d['trip'], d['trip'])}"
            if trip_msg:
                readout(time.time() - t_start, ifaces, pos0, obs)
                print(f"\n{trip_msg}. Stopping.")
                break
            time.sleep(DRAIN_PERIOD_S)
        else:
            print("\nStopping (run complete)...")
    except KeyboardInterrupt:
        print("\nStopping (interrupted)...")
    finally:
        try:
            for iface in ifaces.values():
                iface.send_couple_mode(COUPLE_MODE_OFF)
            if started:
                time.sleep(0.02)
                for iface in ifaces.values():
                    iface.send_stop()
            time.sleep(0.1)
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        if pos0:
            drain()
        for iface in ifaces.values():
            iface.stop_listening()
        log.close()

        if no_data or log.rows == 0:
            os.remove(log_path)
            print("No data logged -- log discarded.")
        else:
            summ = summarize(summary_rows, args)
            config = {"law": "p2p", "mode": "p2p", "motors": names, "wn_hz": args.wn, "zeta": args.zeta,
                      "kf_pct": args.kf, "dob_g": args.dob_g, "fric_pct": args.fric,
                      "iq_max": args.iq_max, "predict": args.predict,
                      "duration_s": args.duration, "trip": trip_msg, "summary": summ,
                      "started": time.strftime("%Y-%m-%d %H:%M:%S",
                                               time.localtime(t_start))}
            with open(log_path[:-4] + ".json", "w") as f:
                json.dump(config, f, indent=2)
            print(f"Log saved: {log_path}  ({log.rows} rows)")
            for n, s in sorted(log.stats.items()):
                if s["engaged_ticks"]:
                    print(f"  ESC {n}: engaged {s['engaged_ticks'] / 1000:.1f} s, "
                          f"max |Iq cmd| {s['max_iq_cmd']:.3f} A, saturated "
                          f"{100 * s['sat_ticks'] / s['engaged_ticks']:.1f}% of ticks, "
                          f"worst peer age {s['max_age']:.1f} ms")
            if summ:
                line = (f"  gap |A-B|: p50 {summ['gap_p50']:.0f}  p95 {summ['gap_p95']:.0f}  "
                        f"max {summ['gap_max']:.0f} counts")
                if "force_mismatch_pct" in summ:
                    line += (f"  | force |f_A+f_B| / |f| in contact: "
                             f"{summ['force_mismatch_pct']:.1f}% "
                             f"({summ['contact_ticks']} ticks)")
                print(line)
            if not args.no_plot:
                png = plot_coupling_log(log_path)
                if png:
                    print(f"Plot saved: {png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
