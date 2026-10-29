"""
Reference plot of a C3-style teleop run (felt force from the follower's
external-torque estimate).

    python3 plot_reference.py logs/<run>.csv [--kf 1.0 --kb 0.35e-5 --db 0.02 --n-per-a 1.044] [--out <png>]

Five stacked panels on one time axis, one y-scale per panel (no dual axes):
  1. Shaft angles: leader and follower, degrees from their start.
  2. Contact force at the follower tip (N), from tau_env; contact shaded.
  3. What the operator feels: the leader's commanded current split into the
     contact term (K_F * deadband(tau_env)) and the leader's own damping
     (-KB_DAMP * v_b), with the 0.8 A motor rating marked.
  4. Follower current: commanded vs measured.
  5. Loop period (ms).

K_F, KB_DAMP and the dead band are not stored in the CSV; pass the values
the run used (defaults = the C3 operating point).
"""

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Reference palette (dataviz skill, light surface), categorical slots in order.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
COUNTS_PER_DEG = 4000 / 360.0
KT = 0.0646
# Force at the follower tip, measured against the scale (TEST_LOG 2026-10-28):
# 1.044 N/A, 22 % above KT / 75.7 mm. Logs before 2026-10-28 have tau_env_N
# computed with KT / r; this plot recomputes from tau_env either way.
N_PER_A_TIP = 1.044
I_RATED = 0.8


def load(path):
    rows = list(csv.DictReader(open(path)))
    col = lambda k: np.array([float(r[k]) for r in rows])
    d = {k: col(k) for k in ("t_s", "d_a", "d_b", "tau_env", "iq_b_cmd", "vel_b",
                             "iq_a_cmd", "iq_a_readback", "loop_dt")}
    if "iq_a_hold" in rows[0]:   # TELEOP_HOLD_K runs (contact spring)
        d["iq_a_hold"] = col("iq_a_hold")
    return d


def style(ax, ylabel):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.set_ylabel(ylabel, color=INK, fontsize=10)


def label_end(ax, t, y, text, color, dy=0):
    """Direct label at the right end of a series (identity not color-only).
    dy (points) staggers labels whose series end at similar values."""
    ax.annotate(text, xy=(t[-1], y[-1]), xytext=(6, dy), textcoords="offset points",
                color=INK, fontsize=9, va="center",
                bbox=dict(boxstyle="round,pad=0.2", fc=SURFACE, ec=color, lw=1.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--kf", type=float, default=1.0)
    ap.add_argument("--kb", type=float, default=0.35e-5)
    ap.add_argument("--db", type=float, default=0.02)
    ap.add_argument("--n-per-a", type=float, default=N_PER_A_TIP,
                    help="tip force per amp of tau_env (default: measured 1.044 N/A)")
    ap.add_argument("--out")
    a = ap.parse_args()
    d = load(a.csv)
    t = d["t_s"]
    felt = a.kf * np.sign(d["tau_env"]) * np.maximum(0, np.abs(d["tau_env"]) - a.db)
    damp = -a.kb * d["vel_b"]
    force_n = d["tau_env"] * a.n_per_a
    contact = np.abs(d["tau_env"]) > 0.08

    fig, axes = plt.subplots(5, 1, figsize=(13, 13), sharex=True,
                             gridspec_kw={"height_ratios": [2, 2, 2, 1.6, 1]})
    fig.patch.set_facecolor(SURFACE)

    def shade(ax):
        ax.fill_between(t, 0, 1, where=contact, transform=ax.get_xaxis_transform(),
                        color=GRID, alpha=0.9, linewidth=0, zorder=0)

    ax = axes[0]
    style(ax, "angle (deg)")
    shade(ax)
    ax.plot(t, d["d_b"] / COUNTS_PER_DEG, color=BLUE, lw=1.6, label="leader")
    ax.plot(t, d["d_a"] / COUNTS_PER_DEG, color=ORANGE, lw=1.6, label="follower")
    ax.legend(loc="upper left", fontsize=9, frameon=False)
    label_end(ax, t, d["d_b"] / COUNTS_PER_DEG, "leader", BLUE)
    label_end(ax, t, d["d_a"] / COUNTS_PER_DEG, "follower", ORANGE)
    ax.set_title("1  Shaft angles (from start)   — grey bands = contact (|tau_env| > 80 mA)",
                 loc="left", color=INK, fontsize=11)

    ax = axes[1]
    style(ax, "force (N)")
    shade(ax)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.plot(t, force_n, color=AQUA, lw=1.4)
    ax.set_title(f"2  Estimated contact force at the follower tip ({a.n_per_a:g} N/A, scale-calibrated; "
                 "+ = environment pushes toward +counts)", loc="left", color=INK, fontsize=11)

    ax = axes[2]
    style(ax, "leader current (A)")
    shade(ax)
    for y in (I_RATED, -I_RATED):
        ax.axhline(y, color=INK2, lw=0.9, ls="--")
    ax.text(t[0], I_RATED, "  0.8 A motor rating", color=INK2, fontsize=8, va="bottom")
    ax.plot(t, damp, color=ORANGE, lw=1.1, label=f"leader damping  -KB·v_b  (KB {a.kb:g})")
    ax.plot(t, felt, color=BLUE, lw=1.4, label=f"contact term  K_F·deadband(tau_env)  (K_F {a.kf:g}, DB {a.db * 1e3:g} mA)")
    ax.legend(loc="upper left", fontsize=9, frameon=False, bbox_to_anchor=(0.14, 1.0))
    label_end(ax, t, felt, "contact", BLUE, dy=9)
    label_end(ax, t, damp, "damping", ORANGE, dy=-9)
    ax.set_title("3  What the operator feels: leader current = contact term + damping",
                 loc="left", color=INK, fontsize=11)

    ax = axes[3]
    style(ax, "follower current (A)")
    shade(ax)
    ax.plot(t, d["iq_a_cmd"], color=BLUE, lw=1.1, label="commanded")
    ax.plot(t, d["iq_a_readback"], color=ORANGE, lw=0.9, alpha=0.85, label="measured")
    if "iq_a_hold" in d:
        ax.plot(t, d["iq_a_hold"], color=AQUA, lw=1.4, label="of which contact spring")
    ax.legend(loc="upper left", fontsize=9, frameon=False)
    label_end(ax, t, d["iq_a_cmd"], "commanded", BLUE, dy=9)
    label_end(ax, t, d["iq_a_readback"], "measured", ORANGE, dy=-9)
    ax.set_title("4  Follower current", loc="left", color=INK, fontsize=11)

    ax = axes[4]
    style(ax, "loop (ms)")
    ax.plot(t, d["loop_dt"] * 1e3, color=BLUE, lw=0.9)
    ax.set_ylim(0, max(10, float(np.percentile(d["loop_dt"] * 1e3, 99.9)) * 1.2))
    ax.set_title("5  Control-loop period", loc="left", color=INK, fontsize=11)
    ax.set_xlabel("time (s)", color=INK)

    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    # same definition as the C3 analysis: >= 0.3 s (150 rows at 500 Hz) from any contact
    free = np.ones(len(t), bool)
    for k in np.where(contact)[0]:
        free[max(0, k - 150):k + 150] = False
    fig.suptitle(f"{a.csv}\nfree: felt {rms(felt[free]) * 1e3:.1f} mA + damping {rms(damp[free]) * 1e3:.1f} mA rms   |   "
                 f"contact: felt {rms(felt[contact]) * 1e3 if contact.any() else 0:.0f} mA rms, "
                 f"peak force {np.abs(force_n).max():.2f} N   |   loop p95 {np.percentile(d['loop_dt'], 95) * 1e3:.2f} ms",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout(rect=(0, 0, 0.93, 0.96))
    out = a.out or a.csv[:-4] + "_reference.png"
    fig.savefig(out, dpi=110, facecolor=SURFACE)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
