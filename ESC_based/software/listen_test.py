"""
Listening test: which motor state makes the noise?

Phases (0 A only, nothing should move):
  1. both IDLE (PWM off)            -- baseline
  2. both RUN, SET_IQ 0             -- current loop + 16 kHz PWM only
  3. follower (A) RUN, leader idle  -- localize
  4. leader (B) RUN, follower idle  -- localize
  5. both IDLE

Prints "PHASE <n>: <text>" lines (flushed) so the host can announce them.
Also prints the measured Iq std per phase (TELEM is the 1 ms mean, so this
is the in-band noise, not the audio-band noise).

Usage: taskset -c 2,3 python3 listen_test.py [--seconds 8]
"""

import argparse
import statistics as st
import sys
import time

import can_interface
from can_interface import MotorCANInterface

NODES = {"A": 0x000, "B": 0x020}
RESEND_S = 0.01


def say(n, text):
    print(f"PHASE {n}: {text}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--both-only", action="store_true", help="phases 1-2 only")
    ap.add_argument("--gains", help="comma list of current-loop gain %% (SET_PARAM 2, CHANGE 8); runs both motors at 0 A once per value instead of phases 2-4")
    a = ap.parse_args()
    m = {k: MotorCANInterface(channel="can0", node_base=b) for k, b in NODES.items()}
    for x in m.values():
        x.start_listening()
    running = set()

    def hold(n, text, on):
        for k in m:
            if k in on and k not in running:
                m[k].send_start()
            if k not in on and k in running:
                m[k].send_set_iq(0.0)
                m[k].send_stop()
        running.clear()
        running.update(on)
        say(n, text)
        time.sleep(1.0)  # START offset calibration / settle, not scored
        for x in m.values():
            x.drain_records()
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.seconds - 1.0:
            for k in running:
                m[k].send_set_iq(0.0)
            time.sleep(RESEND_S)
        out = []
        for k, x in m.items():
            iq = [r["iq_meas"] for r in x.drain_records()]
            run = bool(x.get_debug().get("run"))
            if len(iq) > 10:
                out.append(f"{k}: run={run} Iq mean {st.mean(iq):+.4f} std {st.pstdev(iq) * 1000:.1f} mA")
        print("   " + " | ".join(out), flush=True)

    try:
        time.sleep(0.5)
        hold(1, "Both motors off. Baseline.", set())
        if a.gains:
            for i, g in enumerate(int(v) for v in a.gains.split(",")):
                can_interface.DEFAULT_CURR_GAIN_PCT = g  # used by send_start
                for x in m.values():
                    x.send_curr_gain(g)
                hold(2 + i, f"Both on, gain {g} percent.", {"A", "B"})
        else:
            hold(2, "Both motors on, zero amps.", {"A", "B"})
        if not a.both_only and not a.gains:
            hold(3, "Follower only, zero amps.", {"A"})
            hold(4, "Leader only, zero amps.", {"B"})
    finally:
        for x in m.values():
            x.send_set_iq(0.0)
        time.sleep(0.05)
        for x in m.values():
            x.send_stop()
            if a.gains:
                x.send_curr_gain(100)  # back to the Workbench default
        say(99, "Both motors off. Done.")
        time.sleep(0.3)
        for x in m.values():
            x.stop_listening()
    return 0


if __name__ == "__main__":
    sys.exit(main())
