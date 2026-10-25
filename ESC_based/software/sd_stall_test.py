"""
Host diagnostic, no CAN, no motors: does writing teleop's CSV logs to the
SD card stall a 500 Hz loop?

    python3 sd_stall_test.py --dir logs/sdtest --time 75      # SD card
    python3 sd_stall_test.py --dir /dev/shm/sdtest --time 75  # RAM control

Reproduces teleop_test.py's per-iteration I/O with TELEOP_FRAME_QUEUE=1:
one 23-column main row and four frame rows (2 motors x 2 frames) per 2 ms
iteration, through csv.writer on files opened the same way, paced by the
same deadline loop. Times every iteration and the write calls separately,
and samples /proc/meminfo Dirty/Writeback, so a stall can be attributed to
the writes rather than to scheduling.
"""

import argparse
import csv
import os
import random
import sys
import time

RATE_HZ = 500


def meminfo():
    d = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":")
            if k in ("Dirty", "Writeback"):
                d[k] = int(v.split()[0])
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--time", type=float, default=75.0)
    a = ap.parse_args()
    os.makedirs(a.dir, exist_ok=True)
    tag = time.strftime("%H%M%S")
    main_f = open(os.path.join(a.dir, f"sd_main_{tag}.csv"), "w", newline="")
    frames_f = open(os.path.join(a.dir, f"sd_frames_{tag}.csv"), "w", newline="")
    mw, fw = csv.writer(main_f), csv.writer(frames_f)
    mw.writerow([f"c{k}" for k in range(23)])
    fw.writerow(["loop_iter", "motor", "bus_time", "mcu_time_us", "enc", "iq_meas"])

    dt = 1.0 / RATE_HZ
    deadline = t_prev = time.monotonic()
    t_end = t_prev + a.time
    loop_dt, write_dt, dirty_max, wb_max = [], [], 0, 0
    it = 0
    while time.monotonic() < t_end:
        now = time.monotonic()
        loop_dt.append(now - t_prev)
        t_prev = now
        w0 = time.monotonic()
        mw.writerow([f"{now:.4f}"] + [f"{random.uniform(-1, 1):.4f}" for _ in range(22)])
        for m in ("A", "A", "B", "B"):
            fw.writerow([it, m, f"{time.time():.6f}", f"{it * 2000:.0f}",
                         f"{random.randint(0, 40000)}", random.uniform(-0.1, 0.1)])
        write_dt.append(time.monotonic() - w0)
        if it % 250 == 0:
            mi = meminfo()
            dirty_max = max(dirty_max, mi["Dirty"])
            wb_max = max(wb_max, mi["Writeback"])
        it += 1
        deadline += dt
        slack = deadline - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            deadline = time.monotonic()
    main_f.close()
    frames_f.close()

    ld = sorted(loop_dt[1:])
    pct = lambda q: ld[int(q * (len(ld) - 1))] * 1e3
    over = lambda x: sum(1 for v in loop_dt[1:] if v > x)
    big_w = sorted(write_dt)[-5:]
    print(f"{a.dir}: {it} iterations, loop p50 {pct(.5):.3f} p95 {pct(.95):.3f} p99 {pct(.99):.3f} "
          f"max {ld[-1] * 1e3:.1f} ms; >5 ms {over(.005)}, >50 ms {over(.05)}, >100 ms {over(.1)}; "
          f"slowest write calls {', '.join(f'{v * 1e3:.1f}' for v in big_w)} ms; "
          f"Dirty max {dirty_max} kB, Writeback max {wb_max} kB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
