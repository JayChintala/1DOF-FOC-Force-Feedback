"""
Contact figure for reporting: force vs follower indentation for rigid and
soft objects, plus the per-press stiffness (D2 preview, TEST_LOG 2026-10-29).

    python3 plot_contact_stiffness.py [--out PNG] [--side +1|-1]
        [--log LABEL=path.csv ...]   (default: rigid _14, squishy _15, sponge _16)

Panel 1: tip force (from tau_env, 1.044 N/A) against follower indentation
in mm at the tip, per object: median over presses in 0.05 N force bins,
with the 25-75 % band across presses. The slope is the stiffness the
follower sensed.
Panel 2: stiffness per press (stiffness_eval.presses: 0.3 -> 0.7 N,
quasi-static frames), log scale, median marked. Presses with <= 1 count of
travel are encoder-limited lower bounds (open markers).
"""

import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from stiffness_eval import F_LO, FMIN, MIN_PRESS_S, MM_PER_COUNT, VMAX, presses

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
DEFAULT_LOGS = [
    ("Rigid", "logs/teleop_KVs3e-05_FB-1_14.csv"),
    ("Squishy object", "logs/teleop_KVs3e-05_FB-1_15.csv"),
    ("Sponge", "logs/teleop_KVs3e-05_FB-1_16.csv"),
]
BINS = np.arange(0.15, 0.90, 0.05)


def press_curves(path, side):
    """Per press: median indentation (mm) in each force bin, NaN if empty."""
    r = list(csv.DictReader(open(path)))
    t, pa, va, f = (np.array([float(x[k]) for x in r]) for k in ("t_s", "pos_a", "vel_a", "tau_env_N"))
    f = side * f
    on = np.append(f > FMIN, False)
    curves, s = [], None
    for i, v in enumerate(on):
        if v and s is None:
            s = i
        elif not v and s is not None:
            if t[i - 1] - t[s] > MIN_PRESS_S:
                F, P, q = f[s:i], pa[s:i], abs(va[s:i]) < VMAX
                lo = q & (F < F_LO)
                hi = q & (F > 0.7)
                if lo.sum() >= 10 and hi.sum() >= 10:
                    p0 = np.median(P[lo])
                    sgn = 1.0 if np.median(P[hi]) >= p0 else -1.0
                    d = sgn * (P - p0) * MM_PER_COUNT
                    idx = np.digitize(F, BINS) - 1
                    curves.append([np.median(d[q & (idx == k)]) if (q & (idx == k)).sum() >= 3 else np.nan
                                   for k in range(len(BINS) - 1)])
            s = None
    return np.array(curves)


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", action="append", help="LABEL=path.csv (repeat; default rigid/squishy/sponge)")
    ap.add_argument("--side", type=int, choices=(1, -1), default=1)
    ap.add_argument("--out", default="../presentation/deck_assets/contact_stiffness.png")
    a = ap.parse_args()
    logs = [tuple(x.split("=", 1)) for x in a.log] if a.log else DEFAULT_LOGS
    colors = [BLUE, ORANGE, AQUA]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.6), gridspec_kw={"width_ratios": [1.6, 1]})
    fig.patch.set_facecolor(SURFACE)
    centers = 0.5 * (BINS[:-1] + BINS[1:])
    medians = []
    for j, ((label, path), col) in enumerate(zip(logs, colors)):
        ks = presses(path, a.side)
        k = np.array([p[4] for p in ks])
        bound = np.array([p[2] <= 1 for p in ks])
        km = float(np.median(k))
        medians.append(km)

        c = press_curves(path, a.side)
        n = np.sum(~np.isnan(c), axis=0)
        ok = n >= 3
        med = np.nanmedian(np.where(ok, c, np.nan), axis=0)
        q1 = np.nanpercentile(c[:, ok], 25, axis=0)
        q3 = np.nanpercentile(c[:, ok], 75, axis=0)
        ax1.fill_betweenx(centers[ok], q1, q3, color=col, alpha=0.18, lw=0)
        ax1.plot(med[ok], centers[ok], color=col, lw=2, marker="o", ms=4,
                 label=f"{label}: {km:.2f} N/mm ({len(ks)} presses)")

        x = np.full(len(k), j) + np.linspace(-0.12, 0.12, len(k))
        ax2.scatter(x[~bound], k[~bound], s=36, color=col, edgecolor=SURFACE, lw=1, zorder=3)
        ax2.scatter(x[bound], k[bound], s=36, facecolor=SURFACE, edgecolor=col, lw=1.5, zorder=3)
        ax2.plot([j - 0.25, j + 0.25], [km, km], color=INK, lw=2, zorder=4)
        ax2.annotate(f"{km:.2f}", (j + 0.27, km), va="center", fontsize=9, color=INK)

    style(ax1)
    ax1.set_xlabel("Follower indentation at the tip (mm)", color=INK2)
    ax1.set_ylabel("Sensed contact force (N)", color=INK2)
    ax1.set_title("Force vs indentation (slope = stiffness)", color=INK, fontsize=11, loc="left")
    ax1.set_xlim(left=min(-0.3, ax1.get_xlim()[0]))
    ax1.legend(frameon=False, fontsize=9, loc="lower right", labelcolor=INK)

    style(ax2)
    ax2.set_yscale("log")
    ax2.set_xticks(range(len(logs)))
    ax2.set_xticklabels([l for l, _ in logs], color=INK2)
    ax2.set_xlim(-0.5, len(logs) - 0.3)
    ax2.set_ylabel("Stiffness per press (N/mm, log)", color=INK2)
    ratio = medians[0] / medians[-1]
    ax2.set_title(f"Per-press stiffness: {logs[0][0].lower()} / {logs[-1][0].lower()} = {ratio:.0f}x",
                  color=INK, fontsize=11, loc="left")
    ax2.text(0.02, 0.02, "open = encoder-limited (lower bound)", transform=ax2.transAxes,
             fontsize=8, color=INK2)

    fig.suptitle("The follower senses contact stiffness without a force sensor (motor current only)",
                 color=INK, fontsize=12, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(a.out, dpi=200, facecolor=SURFACE)
    print(f"wrote {a.out}; medians {', '.join(f'{m:.2f}' for m in medians)} N/mm, ratio {ratio:.1f}x")


if __name__ == "__main__":
    main()
