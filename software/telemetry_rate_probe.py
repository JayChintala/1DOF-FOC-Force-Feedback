"""
Telemetry rate probe -- diagnostic for the teleop latency budget.

WHY THIS EXISTS
teleop_test.py's logs show ~38% of control iterations reading an
*unchanged* encoder position even while the shaft is moving 80 counts per
iteration -- i.e. the effective encoder update rate is ~100 Hz, not the
1000 Hz the firmware sends (CAN_SendTelemetry() runs every
medium-frequency task tick, SPEED_LOOP_FREQUENCY_HZ = 1000). That missing
telemetry is what forces the heavy EMA filtering in teleop_test.py, and
that filtering is the phase lag behind the ~6 Hz oscillation. Raising
KV_SLOPE cannot fix a latency problem.

Three candidate causes need opposite fixes, so they have to be told apart
before changing any code:

  1. The USB-CAN adapter (this bus is gs_usb over USB, per
     `ip -d link show can0`) batching frames into URBs.
  2. Python/GIL cost: can_interface.py USED TO open one unfiltered socket
     per motor, so both RX threads parsed all ~6000 frames/s. This was the
     real cause; see below.
  3. Firmware TX FIFO drops: CAN_SendTelemetry() queues IQ -> ANGLE ->
     ENC, so ENC_COUNT is dropped first when the FIFO is full.

WHAT IT MEASURES
Three configurations, back to back, each for RUN_DURATION_S:

  (a) TWO unfiltered buses + two RX threads -- reproduces what
      can_interface.py did BEFORE the shared-socket change (kept as the
      control case: it is the arrangement whose cost this probe measured).
  (b) ONE bus filtered to the two ENC_COUNT IDs + one RX thread -- the
      cheapest possible Python path.
  (c) Two MotorCANInterface instances polled from a 200 Hz loop -- teleop's
      real path end to end, reporting the same fresh-sample fraction that
      teleop_test.py logs as fresh_a/fresh_b. Since the shared-socket change
      this is (b)'s shape internally, so (a) vs (c) is the before/after.

For each, per-arbitration-ID frame rates and inter-arrival gap
distributions, computed from BOTH clocks:

  - kernel timestamp (msg.timestamp): when the frame reached SocketCAN.
  - local receive time: when Python got around to it.

Those two clocks are what separate cause 2 from cause 1: if kernel gaps
are a clean 1 ms but local gaps are lumpy, Python is behind. If kernel
gaps are themselves bursty (near-zero, then a multi-ms hole), the USB
adapter is batching. If frames are simply missing from both clocks at a
steady rate, they were never sent or were dropped before the Pi -- read
s_encTxDropCount over the debugger to confirm.

HOW TO READ THE RESULT
  - ENC rate ~1000/s in BOTH configs -> telemetry is innocent; the lag is
    the EMAs plus loop pacing in teleop_test.py.
  - ENC rate short in (a) but ~1000/s in (b) -> Python/GIL. Fix
    can_interface.py: one shared filtered socket, one dispatch thread.
  - ENC rate short in BOTH, with bimodal kernel gaps -> the USB adapter is
    the ceiling; reduce frames/s (lower the telemetry rate,
    decimate telemetry) rather than chasing the Pi side.
  - IQ rate high while ENC rate is short -> firmware FIFO drops, since ENC
    is queued last.
  - Rates fine everywhere but config (a)'s handling lag p99 is many ms ->
    the frames arrive on time and Python gets to them late. This is the
    failure mode the staleness checks in teleop_test.py CANNOT see, because
    they timestamp frames at parse time: a frame handled 14 ms late still
    reports an age near zero.

FIRST MEASUREMENT -- 2026-08-31, both ESCs powered, shafts hand-spun
(logs/telemetry_rate_probe.csv, _2.csv). Result: THE CAN PATH IS CLEAN and
the starvation hypothesis this probe was written to test is WRONG.

  - Every telemetry ID arrives at 998-1002 frames/s, kernel inter-arrival
    gaps 1.00 ms median / 1.01 ms p95. Device rx_dropped and rx_missed were
    both 0. So the USB adapter and firmware are keeping up at the full
    6000 frames/s, and there is no meaningful frame loss.
  - Config (c) -- teleop's real path -- saw a NEW encoder sample on 100.0%
    of polls at 192 Hz, ENC age 0.40 ms median. teleop is NOT telemetry-
    starved. The ~38% of teleop rows with an unchanged position are the
    stretches where a shaft was simply being held still.
  - What IS real: with two unfiltered sockets (config a), 1.0% of frames on
    the first socket and 2.7% on the second are handled more than 5 ms
    late, p99 ~33 ms, worst 48 ms, in contiguous bursts -- the RX thread
    stalls, then catches up. Config (b), one filtered socket, has a p99 of
    0.27 ms and NO frame over 5 ms. Same wire, same firmware: the entire
    difference is Python parsing 6000 frames/s twice under the GIL.
  - Config (a) also lost ~0.2% of frames outright (9983 vs 10006 per 10 s),
    consistent with socket-queue overflow during those stalls.

So the fix for the RX path was config (b)'s shape -- one shared socket with
kernel-side filters, one dispatch thread -- and NOT firmware decimation.

AFTER that change (can_interface._SharedBus), re-measured under teleop's
full duty cycle (200 Hz poll of both nodes plus two SET_IQ sends per
iteration): handling lag median 0.056 ms, p99 0.205 ms, max 0.469 ms, and
ZERO frames over 5 ms, against 2.6-2.7% before. Fresh-sample rate 100.0%.
Re-run this probe if that ever seems to regress -- config (a) is still here
as the control case.

Run from software/ with the venv active, spinning both shafts by hand for
part of each window:
    python3 telemetry_rate_probe.py
"""

import csv
import os
import statistics
import subprocess
import sys
import threading
import time

import can

from can_interface import (
    MotorCANInterface,
    OFFSET_ENC_COUNT,
    OFFSET_IQ_MEAN,
    OFFSET_IQ_READBACK,
)

NODE_BASE_A = 0x000
NODE_BASE_B = 0x020

RUN_DURATION_S = 10.0

# Poll rate for config (c) -- matches CONTROL_RATE_HZ in teleop_test.py so the
# fresh-sample fraction is directly comparable to that log's fresh_a column.
CONSUMER_RATE_HZ = 200.0

# Firmware sends every telemetry frame once per medium-frequency task tick
# (SPEED_LOOP_FREQUENCY_HZ in drive_parameters.h) -- the rate every measured
# rate below is compared against.
EXPECTED_RATE_HZ = 1000.0

LOG_DIR = "logs"

TELEMETRY_IDS = {}
for _base, _tag in ((NODE_BASE_A, "A"), (NODE_BASE_B, "B")):
    TELEMETRY_IDS[_base + OFFSET_IQ_READBACK] = f"IQ_READBACK {_tag}"
    # IQ_MEAN also carries ELEC_ANGLE in bytes 4..5; the standalone 0x013
    # ELEC_ANGLE frame was retired (only 3 FDCAN Tx elements on the G4).
    TELEMETRY_IDS[_base + OFFSET_IQ_MEAN] = f"IQ_MEAN {_tag}"
    TELEMETRY_IDS[_base + OFFSET_ENC_COUNT] = f"ENC_COUNT {_tag}"

ENC_IDS = (NODE_BASE_A + OFFSET_ENC_COUNT, NODE_BASE_B + OFFSET_ENC_COUNT)


class Collector:
    """
    Arrival record keyed by (bus index, arbitration ID). The bus index
    matters: with two unfiltered sockets, BOTH receive every frame on the
    bus, so keying by ID alone would merge two independent streams and
    double every rate while inventing bogus inter-arrival gaps.

    Stores both clocks per frame -- the kernel's timestamp and the moment
    this Python thread got to it.
    """

    def __init__(self):
        self.kernel_times = {}
        self.local_times = {}
        self.lock = threading.Lock()

    def record(self, bus_idx, arb_id, kernel_t, local_t):
        key = (bus_idx, arb_id)
        with self.lock:
            self.kernel_times.setdefault(key, []).append(kernel_t)
            self.local_times.setdefault(key, []).append(local_t)

    def bus_indices(self):
        return sorted({k[0] for k in self.kernel_times})

    def total_frames(self):
        return sum(len(v) for v in self.kernel_times.values())


def _rx_loop(bus, bus_idx, collector, stop_event, parse_all):
    """
    One RX thread. parse_all mimics can_interface._handle_message: every
    frame on the bus is inspected, including the other node's, and only then
    discarded -- that per-frame Python work under the GIL is what config (a)
    is measuring.
    """
    while not stop_event.is_set():
        msg = bus.recv(timeout=0.2)
        if msg is None:
            continue
        local_t = time.time()
        if parse_all and msg.arbitration_id not in TELEMETRY_IDS:
            continue
        collector.record(bus_idx, msg.arbitration_id, msg.timestamp, local_t)


def _gap_stats(times):
    """Inter-arrival gaps in ms: median / p95 / max, plus the near-zero
    fraction. A high near-zero fraction alongside a large max is the
    signature of USB batching (a burst, then a hole)."""
    if len(times) < 3:
        return None
    gaps = [1000.0 * (b - a) for a, b in zip(times, times[1:])]
    gaps_sorted = sorted(gaps)
    p95 = gaps_sorted[int(0.95 * (len(gaps_sorted) - 1))]
    burst_frac = sum(1 for g in gaps if g < 0.2) / len(gaps)
    return {
        "median": statistics.median(gaps),
        "p95": p95,
        "max": max(gaps),
        "burst_frac": burst_frac,
    }


def _read_link_counters():
    """RX packets / dropped / overrun from `ip -s link show can0`. These are
    cumulative since the interface came up, so only deltas across a run
    mean anything."""
    try:
        out = subprocess.run(
            ["ip", "-s", "link", "show", "can0"],
            capture_output=True, text=True, timeout=5.0,
        ).stdout
    except Exception as e:
        print(f"[warn] could not read link counters: {e}")
        return None
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith("RX:") and i + 1 < len(lines):
            fields = lines[i + 1].split()
            if len(fields) >= 5:
                return {
                    "rx_packets": int(fields[1]),
                    "rx_errors": int(fields[2]),
                    "rx_dropped": int(fields[3]),
                    "rx_missed": int(fields[4]),
                }
    return None


def run_config(label, description, buses_specs, parse_all):
    """
    Listen for RUN_DURATION_S over the given bus specs (each a dict of
    can.interface.Bus kwargs), one RX thread per bus, and return the
    collected arrival record.
    """
    print(f"\n{'=' * 72}")
    print(f"CONFIG {label}: {description}")
    print(f"{'=' * 72}")

    buses = []
    threads = []
    stop_event = threading.Event()
    collector = Collector()

    try:
        for spec in buses_specs:
            buses.append(can.interface.Bus(channel="can0", bustype="socketcan", **spec))
        link_before = _read_link_counters()

        for bus_idx, bus in enumerate(buses):
            th = threading.Thread(
                target=_rx_loop,
                args=(bus, bus_idx, collector, stop_event, parse_all),
                daemon=True,
            )
            th.start()
            threads.append(th)

        print(f"Listening for {RUN_DURATION_S:.0f}s -- spin BOTH shafts by hand now.")
        t_start = time.time()
        while time.time() - t_start < RUN_DURATION_S:
            time.sleep(0.5)
            elapsed = time.time() - t_start
            print(f"  t={elapsed:4.1f}s  frames so far: {collector.total_frames()}")
        window = time.time() - t_start
    finally:
        stop_event.set()
        for th in threads:
            th.join(timeout=1.0)
        for bus in buses:
            bus.shutdown()

    link_after = _read_link_counters()
    return collector, window, link_before, link_after


def report(label, collector, window, link_before, link_after):
    print(f"\n--- CONFIG {label} results over {window:.2f}s ---")
    if collector.total_frames() == 0:
        print("NO TELEMETRY FRAMES AT ALL. The ESCs may be unpowered, or the")
        print("firmware may not have booted its control loop. This probe never")
        print("sends START -- bring the motors up with another script first.")
        return

    for bus_idx in collector.bus_indices():
        print(f"\n socket {bus_idx}:")
        print(f"  {'signal':16s} {'ID':>6s} {'frames':>7s} {'rate/s':>7s} {'loss%':>6s} "
              f"{'kern gap ms (med/p95/max)':>28s} {'handling lag ms (med/p99/max)':>31s}")
        for arb_id in sorted(TELEMETRY_IDS):
            name = TELEMETRY_IDS[arb_id]
            kt = collector.kernel_times.get((bus_idx, arb_id), [])
            lt = collector.local_times.get((bus_idx, arb_id), [])
            if not kt:
                print(f"  {name:16s} {arb_id:#06x} {0:>7d} {'--':>7s} "
                      f"{'--':>6s} {'(not received)':>28s}")
                continue
            rate = len(kt) / window
            loss = 100.0 * (1.0 - rate / EXPECTED_RATE_HZ)
            kg = _gap_stats(kt)
            gap_str = (f"{kg['median']:8.2f} /{kg['p95']:7.2f} /{kg['max']:7.2f}"
                       if kg else "n/a")
            # Handling lag = when Python touched the frame minus when the
            # kernel received it. This is the latency that actually shows up
            # as phase lag in the control loop, and it is invisible to the
            # staleness checks in teleop_test.py -- those timestamp frames at
            # PARSE time, so a late frame still looks fresh.
            lag = sorted(1000.0 * (l - k) for k, l in zip(kt, lt))
            lag_str = (f"{lag[len(lag) // 2]:9.2f} /{lag[int(0.99 * (len(lag) - 1))]:9.2f} "
                       f"/{lag[-1]:9.2f}")
            print(f"  {name:16s} {arb_id:#06x} {len(kt):>7d} {rate:>7.0f} {loss:>6.1f} "
                  f"{gap_str:>28s} {lag_str:>31s}")

    print(f"\n Total frames handled by this config (all sockets): "
          f"{collector.total_frames()}")
    if link_before and link_after:
        print(" Kernel socket counters (delta over the window):")
        for k in ("rx_packets", "rx_errors", "rx_dropped", "rx_missed"):
            print(f"   {k:12s} {link_after[k] - link_before[k]:>10d}")


def run_consumer_config(label):
    """
    CONFIG (c) -- the real teleop telemetry path, measured end to end.

    Configs (a) and (b) measure what arrives. This one measures what a
    control loop can actually SEE: two MotorCANInterface instances (the exact
    class teleop_test.py uses, so the same two unfiltered sockets and the
    same per-frame Python parsing) polled from a CONTROL_RATE_HZ loop that
    also competes for the GIL.

    It reports the same fresh_a/fresh_b fraction the instrumented
    teleop_test.py now logs. If this reproduces the ~60% seen there while
    config (b) shows a clean 1000 frames/s on the wire, the missing
    telemetry is being lost inside this process, not on the bus.

    Still passive: MotorCANInterface only sends when send_* is called, and
    this never calls them.
    """
    print(f"\n{'=' * 72}")
    print(f"CONFIG {label}: two MotorCANInterface instances polled at "
          f"{CONSUMER_RATE_HZ:.0f} Hz (teleop's real path)")
    print(f"{'=' * 72}")

    a = MotorCANInterface(channel="can0", node_base=NODE_BASE_A)
    b = MotorCANInterface(channel="can0", node_base=NODE_BASE_B)
    a.start_listening()
    b.start_listening()

    dt = 1.0 / CONSUMER_RATE_HZ
    iters = 0
    fresh_a = 0
    fresh_b = 0
    prev_ta = None
    prev_tb = None
    ages_a = []
    loop_dts = []

    try:
        time.sleep(0.3)  # let the first frames land
        print(f"Polling for {RUN_DURATION_S:.0f}s -- spin BOTH shafts by hand now.")
        t_start = time.time()
        prev_now = t_start
        while time.time() - t_start < RUN_DURATION_S:
            ta = a.get_telemetry()
            tb = b.get_telemetry()
            now = time.time()
            iters += 1
            loop_dts.append(now - prev_now)
            prev_now = now

            if ta["enc_count_time"] != prev_ta:
                fresh_a += 1
                prev_ta = ta["enc_count_time"]
            if tb["enc_count_time"] != prev_tb:
                fresh_b += 1
                prev_tb = tb["enc_count_time"]
            if ta["enc_count_time"] is not None:
                ages_a.append(1000.0 * (now - ta["enc_count_time"]))

            time.sleep(dt)
        window = time.time() - t_start
    finally:
        a.stop_listening()
        b.stop_listening()

    if iters == 0:
        print("No iterations completed.")
        return

    loop_hz = iters / window
    print(f"\n--- CONFIG {label} results over {window:.2f}s ---")
    print(f" Poll loop ran at {loop_hz:.0f} Hz ({CONSUMER_RATE_HZ:.0f} Hz nominal)")
    print(f" New ENC sample on {100.0 * fresh_a / iters:.1f}% of polls for A "
          f"({loop_hz * fresh_a / iters:.0f} Hz effective)")
    print(f" New ENC sample on {100.0 * fresh_b / iters:.1f}% of polls for B "
          f"({loop_hz * fresh_b / iters:.0f} Hz effective)")
    if ages_a:
        srt = sorted(ages_a)
        print(f" ENC age A (ms): med {srt[len(srt) // 2]:.2f}  "
              f"p95 {srt[int(0.95 * (len(srt) - 1))]:.2f}  max {srt[-1]:.2f}")
    if loop_dts:
        srt = sorted(loop_dts)
        print(f" Loop period (ms): med {1000 * srt[len(srt) // 2]:.2f}  "
              f"p95 {1000 * srt[int(0.95 * (len(srt) - 1))]:.2f}  "
              f"max {1000 * srt[-1]:.2f}")
    print(" NOTE: this ENC age understates the true latency -- enc_count_time is"
          "\n set when Python PARSES the frame, so a frame handled 14 ms late"
          "\n still reports an age near zero. Configs (a)/(b) measure that part.")


def write_csv(log_path, results):
    """One row per frame per config -- the raw record, for any offline
    analysis the printed summary doesn't cover (e.g. gap histograms)."""
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["config", "socket", "arb_id", "signal", "kernel_t", "local_t"])
        for label, collector, _window, _lb, _la in results:
            for (bus_idx, arb_id), kts in collector.kernel_times.items():
                lts = collector.local_times[(bus_idx, arb_id)]
                name = TELEMETRY_IDS.get(arb_id, "other")
                for kt, lt in zip(kts, lts):
                    w.writerow([label, bus_idx, f"{arb_id:#06x}", name,
                                f"{kt:.6f}", f"{lt:.6f}"])


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    base_name = "telemetry_rate_probe"
    log_path = os.path.join(LOG_DIR, f"{base_name}.csv")
    suffix = 2
    while os.path.exists(log_path):
        log_path = os.path.join(LOG_DIR, f"{base_name}_{suffix}.csv")
        suffix += 1

    print("PASSIVE probe -- no START/STOP/SET_IQ is ever sent.")
    print(f"Expecting {EXPECTED_RATE_HZ:.0f} frames/s per telemetry ID "
          f"({len(TELEMETRY_IDS)} IDs = "
          f"{len(TELEMETRY_IDS) * EXPECTED_RATE_HZ:.0f} frames/s on the bus).")

    results = []
    try:
        # (a) Today's arrangement: one unfiltered socket per motor, each RX
        # thread inspecting every frame on the bus.
        results.append(("a", *run_config(
            "a",
            "two unfiltered buses + two RX threads (what can_interface.py does)",
            [{}, {}],
            parse_all=True,
        )))

        # (b) Cheapest possible Python path: the kernel drops everything but
        # the two ENC_COUNT IDs, so nothing else is ever parsed.
        enc_filters = [{"can_id": i, "can_mask": 0x7FF} for i in ENC_IDS]
        results.append(("b", *run_config(
            "b",
            "one bus filtered to ENC_COUNT only + one RX thread",
            [{"can_filters": enc_filters}],
            parse_all=False,
        )))
    except KeyboardInterrupt:
        print("\nInterrupted.")

    for label, collector, window, lb, la in results:
        report(label, collector, window, lb, la)

    if not any(c.total_frames() for _l, c, _w, _lb, _la in results):
        print("\nNo data logged -- nothing written.")
        return

    write_csv(log_path, results)
    print(f"\nRaw arrival record saved: {log_path}")

    # (c) last: what a control loop actually sees through can_interface.py.
    try:
        run_consumer_config("c")
    except KeyboardInterrupt:
        print("\nInterrupted.")

    enc_a = [len(c.kernel_times.get((0, ENC_IDS[0]), [])) / w
             for _l, c, w, _lb, _la in results]
    if len(enc_a) == 2:
        print(f"\nENC_COUNT A on socket 0: config (a) {enc_a[0]:.0f}/s vs "
              f"config (b) {enc_a[1]:.0f}/s (expected {EXPECTED_RATE_HZ:.0f}/s).")
        print("Compare the handling-lag columns too: a rate that looks fine but a"
              " p99 lag of\nmany ms is still a control-loop latency problem, and it"
              " is the one the\nstaleness checks in teleop_test.py cannot see.")


if __name__ == "__main__":
    sys.exit(main())
