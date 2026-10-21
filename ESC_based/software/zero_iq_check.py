"""
Zero-current precheck: is TELEM Iq ~0 once the ESCs are in RUN?

While IDLE the firmware still computes Iqd from uncalibrated shunt readings
(FOC_CurrControllerM1 runs with PWM off), so TELEM reads ~+-25 A at rest.
Offsets are measured on START. This script STARTs both nodes, holds
SET_IQ 0 for HOLD_S, and logs every TELEM frame via drain_records() so the
idle-vs-RUN difference and the RUN noise floor are both visible. DBG's RUN
bit confirms the state machine actually got there.

Energizes both motors at 0 A. The first START after ESC power-up also runs
the encoder alignment (M1_ALIGNMENT_DURATION = 2000 ms of 0.8 A Id at 90
deg electrical), which moves each shaft and ends by loading the encoder
counter with the alignment angle -- a one-frame jump of hundreds of counts.
So the RUN-window statistics are taken from the first DBG frame with the
RUN bit set, not from a fixed delay after START.
"""

import csv
import os
import statistics as st
import sys
import time

from can_interface import MotorCANInterface

NODES = {"A": 0x000, "B": 0x020}
IDLE_S = 1.0          # log before START, for comparison
HOLD_S = 4.0          # after START; covers a 2 s alignment plus >= 1.5 s RUN
RESEND_S = 0.01       # SET_IQ 0 resend period, well inside the 200 ms timeout
RUN_IQ_PASS_A = 0.05  # |mean Iq| in RUN at 0 A command
MIN_RUN_S = 1.0       # need at least this much RUN data to judge
ENC_JUMP_COUNTS = 100  # one-frame step no shaft makes in 1 ms at 0 A
LOG_PATH = os.path.join("logs", "precheck_zero_iq.csv")


def main():
    os.makedirs("logs", exist_ok=True)
    ifaces = {name: MotorCANInterface(channel="can0", node_base=base)
              for name, base in NODES.items()}
    rows = []
    # Wall time of the first DBG frame with RUN set, per node. DBG is 10 Hz
    # outside coupling, so this lags the real transition by up to 100 ms.
    run_t = {name: None for name in NODES}

    def collect(phase):
        for name, m in ifaces.items():
            for r in m.drain_records():
                rows.append((r["bus_time"], name, phase, r["iq_meas"],
                             r["enc"], r["mcu_time_us"]))
            d = m.get_debug()
            if phase == "started" and run_t[name] is None and d.get("run"):
                run_t[name] = d["dbg_time"]

    for m in ifaces.values():
        m.start_listening()
    try:
        time.sleep(IDLE_S)
        collect("idle")
        t_start = time.time()
        for m in ifaces.values():
            m.send_start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < HOLD_S:
            for m in ifaces.values():
                m.send_set_iq(0.0)
            time.sleep(RESEND_S)
            collect("started")
    finally:
        for m in ifaces.values():
            m.send_set_iq(0.0)
        time.sleep(0.05)
        for m in ifaces.values():
            m.send_stop()
        time.sleep(0.5)
        collect("stopped")
        for m in ifaces.values():
            m.stop_listening()

    with open(LOG_PATH, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bus_time", "motor", "phase", "iq", "enc", "mcu_time_us"])
        w.writerows(rows)

    ok = True
    for name in NODES:
        if run_t[name] is None:
            print(f"{name}: RUN bit never seen in DBG")
            ok = False
            continue
        print(f"{name}: RUN seen {run_t[name] - t_start:.2f} s after START "
              f"(DBG at 10 Hz, so up to 0.1 s late)")
        started = [r for r in rows if r[1] == name and r[2] == "started"]
        for a, b in zip(started, started[1:]):
            if abs(b[4] - a[4]) > ENC_JUMP_COUNTS:
                print(f"  encoder jump {a[4]:.0f} -> {b[4]:.0f} in one frame at "
                      f"{b[0] - t_start:.3f} s after START (alignment counter load)")
        run_rows = [r for r in started if r[0] >= run_t[name]]
        if run_rows and run_rows[-1][0] - run_rows[0][0] < MIN_RUN_S:
            print(f"  only {run_rows[-1][0] - run_rows[0][0]:.2f} s of RUN data")
            ok = False
        groups = {"idle": [r for r in rows if r[1] == name and r[2] == "idle"],
                  "pre-run": [r for r in started if r[0] < run_t[name]],
                  "run0": run_rows,
                  "stopped": [r for r in rows if r[1] == name and r[2] == "stopped"]}
        for ph, sel in groups.items():
            if not sel:
                print(f"  {ph:8s} NO FRAMES")
                ok = ok and ph == "pre-run"
                continue
            x = [r[3] for r in sel]
            e = [r[4] for r in sel]
            gaps = [b[5] - a[5] for a, b in zip(sel, sel[1:])]
            print(f"  {ph:8s} n={len(x):5d} mean={st.mean(x):+8.4f} A "
                  f"std={st.pstdev(x):.4f} A  enc {min(e):.0f}..{max(e):.0f}  "
                  f"gaps>1050us={sum(g > 1050 for g in gaps)}")
            if ph == "run0" and abs(st.mean(x)) > RUN_IQ_PASS_A:
                ok = False
    print(f"Log: {os.path.abspath(LOG_PATH)}")
    print("PASS: Iq ~0 in RUN on both" if ok else "FAIL: see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
