"""
Plots a coupling_monitor.py log as one PNG next to the CSV (same name).

Called automatically when coupling_monitor.py exits, or standalone:
    python3 plot_coupling.py logs/coupling_peer_....csv

Panels, top to bottom, all on one time axis:
  - shaft position of each ESC, relative to the start of the log
  - coupling error as each ESC computed it (mirror images in peer mode)
  - per ESC: Iq commanded by the coupling law vs Iq measured
  - peer mode only: age of the peer TELEM each ESC was using

Gray background = not coupled yet (START, alignment, settling); amber wash
on a current panel = that ESC was pinned at Iq_max; a red line = a trip.

If matplotlib isn't installed:
    pip install matplotlib --break-system-packages
"""

import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")  # no GUI needed -- just saves a PNG
import matplotlib.pyplot as plt

# Light-surface tokens from the dataviz reference palette. Each ESC keeps
# its color in every panel: A = categorical slot 1 (blue), B = slot 2
# (orange); the pair validates at CVD dE 24.7 / normal-vision dE 33.6.
# Measured current is the achromatic de-emphasis gray. Amber and red are
# status colors (warning, critical), used only for saturation and trips.
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e6e5e1"
MOTOR_COLOR = {"A": "#2a78d6", "B": "#eb6834"}
MEASURED = "#8a8984"
WASH_UNCOUPLED = "#f0efec"
SATURATED = "#fab219"
TRIP = "#d03b3b"

PEER_TIMEOUT_MS = 5.0       # COUPLE_PEER_TIMEOUT_US in couple_ctrl.h
PEER_AGE_SATURATED_MS = 25.5

TRIP_NAMES = {1: "peer TELEM timeout", 2: "error limit", 3: "velocity limit",
              4: "SET_IQ host timeout", 5: "motor left RUN"}


def load_coupling_log(path):
    """CSV -> {motor: columns}. DBG-derived columns only cover rows that had
    a DBG frame (every tick once coupling was requested, 10 Hz before).

    iq_meas is NaN (a gap in the plot) wherever the ESC was not in RUN, going
    by the most recent DBG status. Before the first START after power-up the
    current-sense offsets are uncalibrated (MCSDK measures them in
    OFFSET_CALIB), and TELEM's Iq reads tens of amps of pure offset -- real
    current is zero with the PWM off. Plotted, that would autoscale the
    current panels to +-30 A and flatten the data that matters. The CSV
    keeps the raw value."""
    data = {}
    running = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            motor = row["motor"]
            m = data.setdefault(motor, {
                "t": [], "pos": [], "iq_meas": [],
                "t_dbg": [], "iq_cmd": [], "err": [], "engaged": [],
                "saturated": [], "trip": [], "peer_age": []})
            t = float(row["t_s"])
            if row["run"] != "":
                running[motor] = row["run"] == "1"
            m["t"].append(t)
            m["pos"].append(float(row["pos"]))
            m["iq_meas"].append(float(row["iq_meas"]) if running.get(motor)
                                else float("nan"))
            if row["iq_cmd"] != "":
                m["t_dbg"].append(t)
                m["iq_cmd"].append(float(row["iq_cmd"]))
                m["err"].append(float(row["err"]))
                m["engaged"].append(row["engaged"] == "1")
                m["saturated"].append(row["saturated"] == "1")
                m["trip"].append(int(row["trip"]))
                m["peer_age"].append(float(row["peer_age_ms"]))
    return data


def spans(t, flags):
    """Contiguous [start, end] time spans where flags is true."""
    out, start = [], None
    for ti, f in zip(t, flags):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            out.append((start, ti))
            start = None
    if start is not None and t:
        out.append((start, t[-1]))
    return out


def coupled_span(data):
    """First and last time every ESC in the log was engaged, or None."""
    firsts, lasts = [], []
    for m in data.values():
        on = [t for t, e in zip(m["t_dbg"], m["engaged"]) if e]
        if not on:
            return None
        firsts.append(on[0])
        lasts.append(on[-1])
    start, end = max(firsts), min(lasts)
    return (start, end) if end > start else None


def first_trip(m):
    for t, code in zip(m["t_dbg"], m["trip"]):
        if code:
            return t, code
    return None


def style_axis(ax, title):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_2, labelsize=8)
    ax.set_title(title, loc="left", fontsize=9.5, color=TEXT, pad=4)


def legend(ax):
    # A surface-colored backing, so a trace or trip line behind the legend
    # cannot run through its text.
    leg = ax.legend(loc="upper right", fontsize=8, ncol=4, frameon=True,
                    facecolor=SURFACE, edgecolor="none", framealpha=0.9)
    for txt in leg.get_texts():
        txt.set_color(TEXT_2)


def plot_coupling_log(csv_path, out_path=None):
    """
    Reads a coupling_monitor.py CSV and saves the diagnostic PNG. Returns the
    output path, or None if there was nothing to plot.
    """
    data = load_coupling_log(csv_path)
    if not data:
        print(f"No data rows found in {csv_path} -- nothing to plot.")
        return None

    cfg = {}
    cfg_path = csv_path[:-4] + ".json"
    if os.path.exists(cfg_path):
        with open(cfg_path) as f:
            cfg = json.load(f)
    motors = sorted(data)
    peer = cfg.get("mode") == "peer" if cfg else len(motors) == 2
    iq_max = cfg.get("iq_max")

    panels = ["pos", "err"] + [f"iq_{n}" for n in motors] + (["age"] if peer else [])
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 1.0 + 2.2 * len(panels)),
                             sharex=True, facecolor=SURFACE)
    ax_of = dict(zip(panels, axes))

    window = coupled_span(data)
    t_all = [t for m in data.values() for t in m["t"]]
    t_lo, t_hi = min(t_all), max(t_all)
    trips = {n: first_trip(m) for n, m in data.items()}

    for ax in axes:
        if window:
            # Recede everything outside the coupled window.
            if window[0] > t_lo:
                ax.axvspan(t_lo, window[0], color=WASH_UNCOUPLED, lw=0, zorder=0)
            if window[1] < t_hi:
                ax.axvspan(window[1], t_hi, color=WASH_UNCOUPLED, lw=0, zorder=0)
        for n, tr in trips.items():
            if tr:
                ax.axvline(tr[0], color=TRIP, linewidth=1.2, zorder=3)

    # -- position --
    ax = ax_of["pos"]
    style_axis(ax, "Shaft position, from start of log  [counts, 4000 per rev]")
    for n in motors:
        ax.plot(data[n]["t"], data[n]["pos"], color=MOTOR_COLOR[n], linewidth=1.4,
                label=f"ESC {n}")
    legend(ax)

    # -- coupling error --
    ax = ax_of["err"]
    style_axis(ax, "Coupling error each ESC computed  [counts]" +
               ("  (mirror images when the pair agrees)" if peer else ""))
    ax.axhline(0, color=TEXT_2, linewidth=0.8, zorder=1)
    for n in motors:
        if data[n]["t_dbg"]:
            ax.plot(data[n]["t_dbg"], data[n]["err"], color=MOTOR_COLOR[n],
                    linewidth=1.4, label=f"ESC {n}")
    legend(ax)

    # -- current, one panel per ESC --
    for n in motors:
        ax = ax_of[f"iq_{n}"]
        m = data[n]
        style_axis(ax, f"ESC {n} current  [A]")
        for s0, s1 in spans(m["t_dbg"], m["saturated"]):
            ax.axvspan(s0, s1, color=SATURATED, alpha=0.25, lw=0, zorder=0)
        ax.plot(m["t"], m["iq_meas"], color=MEASURED, linewidth=0.7,
                label="measured (while running)", zorder=2)
        if m["t_dbg"]:
            ax.plot(m["t_dbg"], m["iq_cmd"], color=MOTOR_COLOR[n], linewidth=1.4,
                    label="commanded", zorder=3)
        if iq_max:
            for y in (iq_max, -iq_max):
                ax.axhline(y, color=TEXT_2, linewidth=0.8, linestyle=(0, (4, 3)),
                           zorder=1)
            ax.text(t_lo, iq_max, f"  Iq_max {iq_max:g} A", color=TEXT_2,
                    fontsize=7.5, va="bottom")
        if any(m["saturated"]):
            ax.fill_between([], [], color=SATURATED, alpha=0.25, label="at Iq_max")
        legend(ax)

    # -- peer TELEM age --
    if peer:
        ax = ax_of["age"]
        style_axis(ax, "Age of the peer TELEM each ESC used  [ms]")
        for n in motors:
            m = data[n]
            if m["t_dbg"]:
                ax.plot(m["t_dbg"], m["peer_age"], color=MOTOR_COLOR[n],
                        linewidth=1.0, label=f"ESC {n}")
        ax.axhline(PEER_TIMEOUT_MS, color=TEXT_2, linewidth=0.8,
                   linestyle=(0, (4, 3)))
        ax.text(t_lo, PEER_TIMEOUT_MS, "  timeout: coupling drops", color=TEXT_2,
                fontsize=7.5, va="bottom")
        # Floor just below zero so a 0.0 ms trace is not hidden by the axis.
        ax.set_ylim(-0.3, PEER_TIMEOUT_MS * 1.6)
        legend(ax)

    axes[-1].set_xlabel("time [s]", color=TEXT_2, fontsize=9)
    axes[-1].set_xlim(t_lo, t_hi)

    title = os.path.basename(csv_path)
    if cfg:
        title = (f"{cfg['mode']} coupling, ESC {'+'.join(cfg['motors'])}   "
                 f"Kp {cfg['kp']:g} A/count   Kd {cfg['kd']:g}   "
                 f"Kd_local {cfg['kd_local']:g} A/(count/s)   Iq_max {cfg['iq_max']:g} A"
                 + ("   predict on" if cfg.get("predict") else "")
                 + f"\n{os.path.basename(csv_path)}   {cfg.get('started', '')}")
    # Trips are named here rather than beside their red line, where the
    # text would collide with the legend or the data.
    for n, tr in trips.items():
        if tr:
            title += (f"\nRed line: ESC {n} tripped at {tr[0]:.3f} s "
                      f"({TRIP_NAMES.get(tr[1], tr[1])})")
    fig.suptitle(title, fontsize=10, color=TEXT, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    if out_path is None:
        out_path = os.path.splitext(csv_path)[0] + ".png"
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    plt.close(fig)
    return out_path


def main():
    if len(sys.argv) != 2:
        print("usage: python3 plot_coupling.py logs/<coupling log>.csv")
        return 1
    out = plot_coupling_log(sys.argv[1])
    if out:
        print(f"Plot saved: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
