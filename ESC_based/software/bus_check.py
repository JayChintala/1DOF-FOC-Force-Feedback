"""
Passive start-of-session check. Energizes nothing: receive-only on can0.

    python3 bus_check.py [--time 2]

Reports, per node: TELEM and DBG frame rates, MCU-clock gaps, idle Iq mean
(|mean| > RESET_IQ_A means the board was reset and its next START will run
the 2 s encoder alignment -- run zero_iq_check.py, with the user's OK,
before any test), last DBG status byte, and the Pi's throttle state.
Exit code 0 only if both nodes look healthy and calibrated.
"""

import argparse
import struct
import subprocess
import sys
import time

import can

NODES = {"A (ESC1, follower)": 0x000, "B (ESC2, leader)": 0x020}
TELEM, DBG = 0x012, 0x112
RESET_IQ_A = 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--time", type=float, default=2.0)
    a = ap.parse_args()

    ids = [b + TELEM for b in NODES.values()] + [b + DBG for b in NODES.values()]
    bus = can.interface.Bus(channel="can0", interface="socketcan",
                            can_filters=[{"can_id": i, "can_mask": 0x7FF} for i in ids])
    frames = {i: [] for i in ids}
    t0 = time.time()
    while time.time() - t0 < a.time:
        m = bus.recv(0.2)
        if m is not None and m.arbitration_id in frames and len(m.data) == 8:
            frames[m.arbitration_id].append(bytes(m.data))
    bus.shutdown()

    ok = True
    for name, base in NODES.items():
        tel = [struct.unpack("<fHH", d) for d in frames[base + TELEM]]
        dbg = [struct.unpack("<hhHBB", d) for d in frames[base + DBG]]
        if len(tel) < 0.9 * 1000 * a.time:
            print(f"{name}: TELEM {len(tel) / a.time:.0f}/s -- MISSING or slow")
            ok = False
            continue
        gaps = [(b[2] - a_[2]) % 65536 for a_, b in zip(tel, tel[1:])]
        bad = sum(not 950 <= g <= 1050 for g in gaps)
        iq = sum(x[0] for x in tel) / len(tel)
        status = dbg[-1][3] if dbg else None
        reset = abs(iq) > RESET_IQ_A
        ok = ok and not reset and bad == 0 and bool(dbg)
        print(f"{name}: TELEM {len(tel) / a.time:.0f}/s, DBG {len(dbg) / a.time:.0f}/s, "
              f"us-gaps outside 950..1050: {bad}, idle Iq mean {iq:+.3f} A "
              f"{'(RESET: uncalibrated -> zero_iq_check.py first)' if reset else '(calibrated)'}, "
              f"enc raw {tel[-1][1]}, DBG status {status}")
    try:
        thr = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True).stdout.strip()
    except OSError:
        thr = "vcgencmd unavailable"
    print(f"Pi: {thr} (want throttled=0x0)")
    ok = ok and thr.endswith("0x0")
    print("OK" if ok else "NOT OK -- see above")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
