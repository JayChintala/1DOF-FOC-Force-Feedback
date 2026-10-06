"""
Coupling monitor -- engages the ESC-side coupling and logs what both ESCs do.

The coupling law runs ON THE ESCs (ESC_based firmware): each one reads the
other's TELEM frame straight off the bus and closes its spring+damper
locally at 1 kHz. This script is not in that loop. It sends the gains,
starts the motors, requests the mode, and from then on only listens: a
terminal readout while it runs, a CSV + PNG in ./logs/ when it stops.

Modes:
    peer  bilateral coupling between ESC 1 (A) and ESC 2 (B)        [default]
    hold  each selected ESC holds its OWN position -- single-motor bring-up
          for stiff gains, before coupling the pair

Run from ESC_based/software/ with the venv active:
    python3 coupling_monitor.py                        # peer, gains below
    python3 coupling_monitor.py --kp 0.002 --kd 0.00003
    python3 coupling_monitor.py --mode hold --motors a --kp 0.005
Ctrl+C stops early. COUPLE_MODE off and STOP are always sent on the way out.

Re-plot an old log:  python3 plot_coupling.py logs/<name>.csv
"""

import argparse
import csv
import json
import os
import sys
import time

from can_interface import (COUPLE_MODE_HOLD, COUPLE_MODE_OFF, COUPLE_MODE_PEER,
                           PEER_AGE_SATURATED_MS, TRIP_NAMES, MotorCANInterface,
                           decode_status)
from plot_coupling import plot_coupling_log

# ---- Nodes ----
NODE_BASES = {"A": 0x000, "B": 0x020}   # A = ESC 1, B = ESC 2

# ---- Gains sent at startup ----
# Same law, units and starting point as position_mirror_test.py (and as the
# firmware's compiled-in defaults in couple_ctrl.h). Raise KP from here; the
# point of running it on the ESCs is that it should go much higher than the
# Pi ever could.
KP = 0.000265        # A/count of position error
KD = 0.00001         # A/(count/s) of relative velocity (own - peer)
KD_LOCAL = 0.0       # A/(count/s) of this shaft's own velocity
IQ_MAX_A = 0.8       # per-motor clamp; the firmware caps it at 0.8 A anyway

RUN_DURATION_S = 20.0
PRINT_PERIOD_S = 0.2
DRAIN_PERIOD_S = 0.05
TELEM_TIMEOUT_S = 1.0     # TELEM is 1 kHz; silence for this long = no ESC
DBG_TIMEOUT_S = 0.5       # DBG is 10 Hz before engaging
START_TIMEOUT_S = 5.0     # the first START after power-up includes encoder
                          # alignment, which takes a few seconds
ENGAGE_TIMEOUT_S = 3.0    # engaging waits for both shafts to be still

LOG_DIR = "logs"

CSV_FIELDS = ["t_s", "motor", "mcu_time_us", "pos", "iq_meas", "iq_cmd",
              "err", "run", "engaged", "saturated", "trip", "peer_age_ms"]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--mode", choices=["peer", "hold"], default="peer")
    p.add_argument("--motors", choices=["a", "b", "both"], default="both",
                   help="hold mode only: which ESC(s) hold position")
    p.add_argument("--kp", type=float, default=KP, help="A/count")
    p.add_argument("--kd", type=float, default=KD, help="A/(count/s)")
    p.add_argument("--kd-local", type=float, default=KD_LOCAL, help="A/(count/s)")
    p.add_argument("--iq-max", type=float, default=IQ_MAX_A, help="A, <= 0.8")
    p.add_argument("--duration", type=float, default=RUN_DURATION_S,
                   help="seconds; 0 = until Ctrl+C")
    p.add_argument("--channel", default="can0")
    p.add_argument("--interface", default="socketcan",
                   help="python-can interface (socketcan on the Pi)")
    p.add_argument("--no-plot", action="store_true")
    args = p.parse_args()

    if min(args.kp, args.kd, args.kd_local) < 0:
        p.error("gains must be >= 0 (a negative gain is positive feedback)")
    if not 0 < args.iq_max <= 0.8:
        p.error("--iq-max must be in (0, 0.8]")
    if args.mode == "peer" and args.motors != "both":
        p.error("--motors only applies to --mode hold; peer always uses both")
    return args


def unique_log_path(base_name):
    path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(path):
        path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1
    return path


def wait_for(predicate, timeout_s, period_s=0.02):
    """Polls predicate() until it is true or timeout_s passes."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


class RunLog:
    """Writes drained records as CSV rows, one per motor per 1 kHz tick."""

    def __init__(self, path):
        self.file = open(path, "w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.t0 = None
        self.enc0 = {}
        self.rows = 0
        self.stats = {}   # motor -> running summary

    def write(self, motor, records):
        for r in records:
            if self.t0 is None:
                self.t0 = r["bus_time"]
            enc0 = self.enc0.setdefault(motor, r["enc"])
            flags = r["flags"]
            st = decode_status(flags) if flags is not None else None
            row = {
                "t_s": f"{r['bus_time'] - self.t0:.6f}",
                "motor": motor,
                "mcu_time_us": int(r["mcu_time_us"]),
                "pos": int(r["enc"] - enc0),
                "iq_meas": f"{r['iq_meas']:.4f}",
                "iq_cmd": "" if r["iq_cmd"] is None else f"{r['iq_cmd']:.3f}",
                "err": "" if r["err"] is None else r["err"],
                "run": "" if st is None else int(st["run"]),
                "engaged": "" if st is None else int(st["engaged"]),
                "saturated": "" if st is None else int(st["saturated"]),
                "trip": "" if st is None else st["trip"],
                "peer_age_ms": "" if r["peer_age_ms"] is None else r["peer_age_ms"],
            }
            self.writer.writerow(row)
            self.rows += 1
            self._accumulate(motor, r, st)

    def _accumulate(self, motor, r, st):
        s = self.stats.setdefault(motor, {"engaged_ticks": 0, "sat_ticks": 0,
                                          "max_err": 0, "max_iq_cmd": 0.0,
                                          "max_age": 0.0})
        if st is None or not st["engaged"]:
            return
        s["engaged_ticks"] += 1
        s["sat_ticks"] += int(st["saturated"])
        s["max_err"] = max(s["max_err"], abs(r["err"]))
        s["max_iq_cmd"] = max(s["max_iq_cmd"], abs(r["iq_cmd"]))
        if r["peer_age_ms"] < PEER_AGE_SATURATED_MS:
            s["max_age"] = max(s["max_age"], r["peer_age_ms"])

    def close(self):
        self.file.close()


def status_tag(dbg):
    if dbg["flags"] is None:
        return "no DBG"
    if not dbg["run"]:
        return "STOPPED"
    if dbg["engaged"]:
        return "SAT" if dbg["saturated"] else "ENGAGED"
    if dbg["requested"]:
        return "waiting"
    return "direct"


def readout(t_rel, ifaces, pos0):
    parts = [f"t={t_rel:6.1f}s"]
    ages = []
    for name, iface in ifaces.items():
        tel = iface.get_telemetry()
        dbg = iface.get_debug()
        pos = tel["enc_count_unwrapped"]
        rel = 0 if pos is None else pos - pos0.get(name, pos)
        err = dbg["err"] if dbg["err"] is not None else 0
        iq_cmd = dbg["iq_cmd"] if dbg["iq_cmd"] is not None else 0.0
        iq_meas = tel["iq_readback"] if tel["iq_readback"] is not None else 0.0
        parts.append(f"{name}: pos {rel:+7.0f}  err {err:+6d}  "
                     f"iq {iq_cmd:+.3f}/{iq_meas:+.3f} A  [{status_tag(dbg)}]")
        if dbg["peer_age_ms"] is not None and dbg["mode_peer"]:
            ages.append(f"{name} {dbg['peer_age_ms']:.1f}")
    if ages:
        parts.append("peer age ms " + " ".join(ages))
    print(" | ".join(parts))


def main():
    args = parse_args()
    if args.mode == "peer":
        names, mode = ["A", "B"], COUPLE_MODE_PEER
    else:
        names = ["A", "B"] if args.motors == "both" else [args.motors.upper()]
        mode = COUPLE_MODE_HOLD

    os.makedirs(LOG_DIR, exist_ok=True)
    who = "" if args.mode == "peer" else f"_{''.join(names)}"
    base_name = (f"coupling_{args.mode}{who}_KP{args.kp:g}_KD{args.kd:g}"
                 f"_KDL{args.kd_local:g}_A{args.iq_max:g}")
    log_path = unique_log_path(base_name)

    ifaces = {n: MotorCANInterface(channel=args.channel, bustype=args.interface,
                                   node_base=NODE_BASES[n]) for n in names}
    for iface in ifaces.values():
        iface.start_listening()

    log = RunLog(log_path)
    started = False
    no_data = False
    trip_msg = None
    t_start = time.time()

    def drain():
        for n, iface in ifaces.items():
            log.write(n, iface.drain_records())

    try:
        if not wait_for(lambda: all(i.get_telemetry()["enc_count_unwrapped"] is not None
                                    for i in ifaces.values()), TELEM_TIMEOUT_S):
            missing = [n for n, i in ifaces.items()
                       if i.get_telemetry()["enc_count_unwrapped"] is None]
            print(f"No TELEM from ESC {', '.join(missing)} -- check power, CAN "
                  f"wiring and that can0 is up at 1 Mbit/s.")
            no_data = True
            return 1
        if not wait_for(lambda: all(i.get_debug()["flags"] is not None
                                    for i in ifaces.values()), DBG_TIMEOUT_S):
            missing = [n for n, i in ifaces.items() if i.get_debug()["flags"] is None]
            print(f"TELEM but no DBG frames from ESC {', '.join(missing)} -- that "
                  f"board is not running ESC_based firmware. Flash it first.")
            no_data = True
            return 1

        pos0 = {n: i.get_telemetry()["enc_count_unwrapped"] for n, i in ifaces.items()}
        for iface in ifaces.values():
            iface.send_couple_gains(args.kp, args.kd)
            iface.send_couple_local(args.kd_local, args.iq_max)

        print(f"Mode {args.mode} on ESC {'+'.join(names)}:  Kp={args.kp:g} A/count  "
              f"Kd={args.kd:g}  Kd_local={args.kd_local:g} A/(count/s)  "
              f"Iq_max={args.iq_max:g} A")
        print("Sending START...")
        for iface in ifaces.values():
            iface.send_start()
        started = True
        if not wait_for(lambda: all(i.get_debug().get("run") for i in ifaces.values()),
                        START_TIMEOUT_S):
            stuck = [n for n, i in ifaces.items() if not i.get_debug().get("run")]
            print(f"ESC {', '.join(stuck)} did not reach RUN within "
                  f"{START_TIMEOUT_S:g} s (fault? bus voltage?).")
            return 1

        print("Hands off -- engaging once " +
              ("the shaft is still..." if len(names) == 1 else "both shafts are still..."))
        for iface in ifaces.values():
            iface.send_couple_mode(mode)
        if not wait_for(lambda: all(i.get_debug().get("engaged") for i in ifaces.values()),
                        ENGAGE_TIMEOUT_S):
            for n, i in ifaces.items():
                d = i.get_debug()
                if d.get("engaged"):
                    continue
                why = "a shaft is still moving"
                if args.mode == "peer" and (d["peer_age_ms"] or 0) >= PEER_AGE_SATURATED_MS:
                    why = "it is not receiving the other ESC's TELEM (node IDs? both flashed?)"
                print(f"ESC {n} did not engage within {ENGAGE_TIMEOUT_S:g} s: {why}.")
            return 1

        print(f"Engaged. Logging to {log_path}" +
              ("" if args.duration == 0 else f" for {args.duration:g} s") +
              ". Ctrl+C to stop.\n")
        if args.mode == "peer":
            print("Turn one shaft and the other should follow; hold one and you "
                  "feel it in the other.\n")

        t_engaged = time.monotonic()
        next_print = t_engaged
        while args.duration == 0 or time.monotonic() - t_engaged < args.duration:
            drain()
            now = time.monotonic()
            if now >= next_print:
                readout(time.time() - t_start, ifaces, pos0)
                next_print = now + PRINT_PERIOD_S

            for n, iface in ifaces.items():
                d = iface.get_debug()
                if d["flags"] is not None and not d["requested"] and d["trip"]:
                    trip_msg = f"ESC {n} tripped: {TRIP_NAMES.get(d['trip'], d['trip'])}"
            if trip_msg:
                readout(time.time() - t_start, ifaces, pos0)
                print(f"\n{trip_msg}. It has zeroed its current and dropped the "
                      f"coupling. Stopping.")
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
            time.sleep(0.1)  # let the last frames in before the final drain
        except Exception as e:
            print(f"[warn] error during stop sequence: {e}")
        drain()
        unmatched = {n: i.unmatched_dbg for n, i in ifaces.items()}
        for iface in ifaces.values():
            iface.stop_listening()
        log.close()
        print("CAN interfaces closed.")

        if no_data or log.rows == 0:
            os.remove(log_path)
            print("No data logged -- log discarded.")
        else:
            config = {
                "mode": args.mode, "motors": names, "kp": args.kp, "kd": args.kd,
                "kd_local": args.kd_local, "iq_max": args.iq_max,
                "duration_s": args.duration, "trip": trip_msg,
                "started": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t_start)),
            }
            with open(log_path[:-4] + ".json", "w") as f:
                json.dump(config, f, indent=2)
            print(f"Log saved: {log_path}  ({log.rows} rows)")
            for n, s in sorted(log.stats.items()):
                if s["engaged_ticks"]:
                    print(f"  ESC {n}: engaged {s['engaged_ticks'] / 1000:.1f} s, "
                          f"max |err| {s['max_err']} counts, max |Iq cmd| "
                          f"{s['max_iq_cmd']:.3f} A, saturated "
                          f"{100 * s['sat_ticks'] / s['engaged_ticks']:.1f}% of ticks"
                          + (f", worst peer age {s['max_age']:.1f} ms"
                             if args.mode == "peer" else ""))
            if any(unmatched.values()):
                print(f"  DBG frames without their TELEM: {unmatched}")
            if not args.no_plot:
                png_path = plot_coupling_log(log_path)
                if png_path:
                    print(f"Plot saved: {png_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
