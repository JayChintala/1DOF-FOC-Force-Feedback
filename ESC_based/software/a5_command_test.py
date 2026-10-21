"""
Phase A5: command-path test on one motor (default: follower A, shaft free).

    python3 a5_command_test.py --mode const  [--time 20]   # (b) SET_IQ 0 A at 500 Hz
    python3 a5_command_test.py --mode stair  [--time 20]   # (c) 0.05 A staircase, step every 2 ms
    python3 a5_command_test.py --analyze logs/A5_hold.csv logs/A5_const.csv logs/A5_stair.csv logs/A5_stair_cmd.csv

(a), HOLD with zero gains and no host traffic, is coupling_monitor.py:
    coupling_monitor.py --mode hold --motors a --kp 0 --kd 0 --kd-local 0 --duration 20

The staircase is symmetric -- 0, +STEP, 0, -STEP, repeating every 4 ticks --
so its mean is 0 and a free shaft does not creep. Each SET_IQ is applied as
a step on the ESC's next 1 kHz tick.

Logs every TELEM frame (drain_records: kernel bus_time, mcu_time_us, enc,
iq_meas) to logs/A5_<mode>.csv, and every command sent (host time.time(),
amps) to logs/A5_<mode>_cmd.csv. bus_time and time.time() share
CLOCK_REALTIME, so the step delay below is TX -> ESC applies -> Iq measured
-> TELEM received, transport included: a relative number for comparing
firmware variants (B4), not the ESC's internal delay.

Pass (brief A5): (a) and (b) spectra agree within 3 dB.
"""

import argparse
import csv
import os
import statistics as st
import sys
import time

import numpy as np

from can_interface import MotorCANInterface
from spin_test import RESET_IQ_A

NODES = {"A": 0x000, "B": 0x020}
RATE_HZ = 500
STEP_A = 0.05
STAIR = (0.0, STEP_A, 0.0, -STEP_A)
RUN_TIMEOUT_S = 5.0
PASS_DB = 3.0
BANDS_HZ = ((1, 10), (10, 50), (50, 150), (150, 300), (300, 450), (450, 500))


def run(mode, node, duration):
    os.makedirs("logs", exist_ok=True)
    m = MotorCANInterface(channel="can0", node_base=NODES[node])
    m.start_listening()
    frames, cmds = [], []
    try:
        time.sleep(0.5)
        idle = [r["iq_meas"] for r in m.drain_records()]
        if len(idle) < 400 or abs(st.mean(idle)) > RESET_IQ_A:
            print("ABORT: node missing or reset (run zero_iq_check.py first)")
            return 1
        m.send_start()
        t0 = time.monotonic()
        while not m.get_debug().get("run"):
            if time.monotonic() - t0 > RUN_TIMEOUT_S:
                print("ABORT: no RUN within 5 s")
                return 1
            m.send_set_iq(0.0)
            time.sleep(0.01)
        m.drain_records()
        dt = 1.0 / RATE_HZ
        t_end = time.monotonic() + duration
        deadline = time.monotonic()
        k = 0
        while time.monotonic() < t_end:
            amps = 0.0 if mode == "const" else STAIR[k % len(STAIR)]
            m.send_set_iq(amps)
            cmds.append((time.time(), amps))
            k += 1
            for r in m.drain_records():
                frames.append((r["bus_time"], r["mcu_time_us"], r["enc"], r["iq_meas"]))
            deadline += dt
            slack = deadline - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                deadline = time.monotonic()
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            m.send_set_iq(0.0)
            time.sleep(0.05)
            m.send_stop()
        except Exception as e:
            print(f"[warn] stop sequence: {e}")
        time.sleep(0.1)
        for r in m.drain_records():
            frames.append((r["bus_time"], r["mcu_time_us"], r["enc"], r["iq_meas"]))
        m.stop_listening()

    base = os.path.join("logs", f"A5_{mode}")
    with open(base + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bus_time", "mcu_time_us", "enc", "iq_meas"])
        w.writerows(frames)
    with open(base + "_cmd.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_time", "amps"])
        w.writerows(cmds)
    sends = np.diff([c[0] for c in cmds]) if len(cmds) > 1 else np.array([0])
    print(f"{mode}: {len(frames)} frames, {len(cmds)} SET_IQ sent "
          f"(period p50 {np.median(sends) * 1e3:.2f} ms, p95 {np.percentile(sends, 95) * 1e3:.2f} ms); "
          f"logs {base}.csv, {base}_cmd.csv")
    return 0


def load_iq(path):
    """(mcu time s, iq, bus_time) from an A5 log or a coupling_monitor HOLD log."""
    t, iq, bt = [], [], []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if "engaged" in r and r["engaged"] != "1":
                continue
            t.append(float(r["mcu_time_us"]) * 1e-6)
            iq.append(float(r["iq_meas"]))
            bt.append(float(r["bus_time"]) if "bus_time" in r else np.nan)
    return np.array(t), np.array(iq), np.array(bt)


def psd(iq, dt, nseg=1000):
    x = iq - iq.mean()
    win = np.hanning(nseg)
    segs = [x[i:i + nseg] * win for i in range(0, len(x) - nseg + 1, nseg // 2)]
    p = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0) / (win ** 2).sum() * dt * 2
    return np.fft.rfftfreq(nseg, dt), p          # A^2/Hz


def analyze(hold, const, stair, stair_cmd):
    res = {}
    for name, path in (("a hold", hold), ("b const", const), ("c stair", stair)):
        t, iq, _ = load_iq(path)
        dt = float(np.median(np.diff(t)))
        f, p = psd(iq, dt)
        res[name] = (f, p)
        print(f"{name:8s}: {len(iq)} samples, std {iq.std() * 1e3:.2f} mA, mean {iq.mean() * 1e3:+.2f} mA")
    print("\nBand power (mA rms) and (b) vs (a):")
    print(f"  {'band Hz':>10s} {'(a) hold':>9s} {'(b) const':>9s} {'(c) stair':>9s} {'b-a dB':>7s}")
    worst = 0.0
    for lo, hi in BANDS_HZ:
        rms = {}
        for name, (f, p) in res.items():
            s = (f >= lo) & (f <= hi)
            rms[name] = np.sqrt(np.trapezoid(p[s], f[s]))
        db = 20 * np.log10(rms["b const"] / rms["a hold"])
        worst = max(worst, abs(db))
        print(f"  {lo:4d}-{hi:<4d} {rms['a hold'] * 1e3:9.2f} {rms['b const'] * 1e3:9.2f} "
              f"{rms['c stair'] * 1e3:9.2f} {db:+7.2f}")
    f, pa = res["a hold"]
    _, pb = res["b const"]
    top = f[-1]
    print(f"  Nyquist bin {top:.0f} Hz: (a) {np.sqrt(pa[-1]) * 1e3:.3f}, (b) {np.sqrt(pb[-1]) * 1e3:.3f} mA/sqrt(Hz), "
          f"{10 * np.log10(pb[-1] / pa[-1]):+.2f} dB")
    print(f"  [A5] worst band difference (b) vs (a): {worst:.2f} dB "
          f"({'PASS' if worst <= PASS_DB else 'FAIL'} <= {PASS_DB:g} dB)")

    # Step response: average iq_meas around each command transition, host clock.
    _, iq, bt = load_iq(stair)
    cmd = np.loadtxt(stair_cmd, delimiter=",", skiprows=1)
    ct, ca = cmd[:, 0], cmd[:, 1]
    edges = [(ct[i], ca[i] - ca[i - 1]) for i in range(1, len(ca)) if ca[i] != ca[i - 1]]
    lags = np.arange(-1, 8) * 1e-3
    acc = np.zeros(len(lags))
    n = 0
    for te, dstep in edges:
        j = np.searchsorted(bt, te)
        if j < 2 or j + 8 >= len(bt):
            continue
        base = iq[j - 1]
        # sample-and-hold the measured response onto a 1 ms grid after the send
        resp = np.interp(te + lags, bt, iq) - base
        acc += resp * np.sign(dstep)
        n += 1
    if n:
        mean = acc / n
        half = 0.5 * STEP_A
        k = next((i for i in range(1, len(lags)) if mean[i] >= half), None)
        if k is not None:
            tk = lags[k - 1] + (half - mean[k - 1]) / (mean[k] - mean[k - 1]) * (lags[k] - lags[k - 1])
            print(f"\nStaircase: {n} steps of {STEP_A:g} A. Mean response vs time after send (ms): " +
                  " ".join(f"{l * 1e3:.0f}:{v * 1e3:+.1f}" for l, v in zip(lags, mean)) + " mA")
            print(f"  50 % crossing {tk * 1e3:.2f} ms after the host send (TX + apply tick + "
                  f"current loop + TELEM + RX; baseline for B4's <= 0.6 ms slower check)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("const", "stair"))
    ap.add_argument("--node", choices=NODES, default="A")
    ap.add_argument("--time", type=float, default=20.0)
    ap.add_argument("--analyze", nargs=4, metavar=("HOLD", "CONST", "STAIR", "STAIR_CMD"))
    a = ap.parse_args()
    if a.analyze:
        analyze(*a.analyze)
        return 0
    if not a.mode:
        ap.error("--mode or --analyze required")
    return run(a.mode, a.node, a.time)


if __name__ == "__main__":
    sys.exit(main())
