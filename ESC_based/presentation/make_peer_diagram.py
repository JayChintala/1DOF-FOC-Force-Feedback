# -*- coding: utf-8 -*-
"""
diagram_peer.png: how peer-to-peer coupling works.

Same palette and box style as make_block_diagrams.py. Both halves are
mirror images on purpose -- that is the point of the slide.

Run: python3 make_peer_diagram.py   (writes next to this file)
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "diagram_peer.png")
plt.rcParams.update({"font.family": "DejaVu Sans", "mathtext.fontset": "dejavusans"})

INK, MUTED = "#1f2933", "#5f6b7a"
A_FILL, A_EDGE = "#e7effd", "#2a6fdb"
B_FILL, B_EDGE = "#fdeee4", "#d9682c"
KEY_FILL, KEY_EDGE = "#efe9f8", "#6f4fa8"

fig, ax = plt.subplots(figsize=(16, 6.4))
ax.set_xlim(0, 16); ax.set_ylim(0, 6.4); ax.axis("off")

BX, BW, BY, BH = 0.7, 5.0, 2.32, 3.15          # motor A box; B mirrors it
B2X = 16 - BX - BW

def motor(x, fill, edge, name, node, own, peer, cur):
    ax.add_patch(FancyBboxPatch((x, BY), BW, BH, boxstyle="round,pad=0,rounding_size=0.18",
                                fc=fill, ec=edge, lw=2.2, zorder=2))
    cx = x + BW/2
    ax.text(cx, BY + BH - 0.40, name, ha="center", va="top",
            fontsize=18, fontweight="bold", color=edge, zorder=3)
    ax.text(cx, BY + BH - 0.95, f"node {node} · FOC 16 kHz · law at 1 kHz",
            ha="center", va="top", fontsize=12.5, color=INK, zorder=3)
    ax.plot([x + 0.5, x + BW - 0.5], [BY + 1.95, BY + 1.95], color=edge, lw=1.0, alpha=0.45, zorder=3)
    ax.text(cx, BY + 1.78, f"err$_{own}$ = (θ$_{own}$ − θ$_{own}^0$) − (θ$_{peer}$ − θ$_{peer}^0$)",
            ha="center", va="top", fontsize=14, color=INK, zorder=3)
    ax.text(cx, BY + 1.06, f"$i_{own}$ = − ( Kp·err$_{own}$ + Kd·derr$_{own}$ )",
            ha="center", va="top", fontsize=14.5, fontweight="bold", color=edge, zorder=3)
    ax.text(cx, BY + 0.44, "same law, own and peer swapped", ha="center", va="top",
            fontsize=11.5, color=MUTED, style="italic", zorder=3)

motor(BX,  A_FILL, A_EDGE, "MOTOR A  ·  ESC 1", "0x000", "A", "B", "A")
motor(B2X, B_FILL, B_EDGE, "MOTOR B  ·  ESC 2", "0x020", "B", "A", "B")

# hands
for x0, x1, col, lab, ha in ((0.05, BX - 0.08, A_EDGE, "left hand", "left"),
                             (15.95, B2X + BW + 0.08, B_EDGE, "right hand", "right")):
    ax.annotate("", xy=(x1, BY + BH/2), xytext=(x0, BY + BH/2),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=2.4, mutation_scale=18))
    ax.text(x0, BY + BH/2 + 0.30, lab, ha=ha, va="bottom",
            fontsize=13, color=col, fontweight="bold")

# ---- the virtual spring between them ----------------------------------------
x0s, x1s = BX + BW + 0.45, B2X - 0.45
xs = np.linspace(x0s, x1s, 240)
zig = 0.32*np.sin(2*np.pi*(xs - x0s)/0.66)
taper = np.clip(np.minimum((xs - x0s)/0.55, (x1s - xs)/0.55), 0, 1)
ysp = BY + BH/2
ax.plot(xs, ysp + zig*taper, color=KEY_EDGE, lw=2.6, zorder=2, solid_capstyle="round")
for x, d in ((x0s, -1), (x1s, 1)):
    ax.annotate("", xy=(x + d*0.42, ysp), xytext=(x, ysp),
                arrowprops=dict(arrowstyle="-|>", color=KEY_EDGE, lw=2.2, mutation_scale=16))
ax.text(8.0, ysp + 1.44, "one virtual spring + damper", ha="center",
        fontsize=15.5, fontweight="bold", color=KEY_EDGE)
ax.text(8.0, ysp + 1.06, "Kp 0.007 A/count\nKd 2.5e-5 A/(count/s)", ha="center", va="top",
        fontsize=12, color=MUTED, linespacing=1.4)
ax.text(8.0, ysp - 0.44, "pulls both shafts\ntoward the same angle", ha="center", va="top",
        fontsize=12, color=MUTED, style="italic", linespacing=1.4)

# ---- CAN bus ----------------------------------------------------------------
BUSY = 1.60
ax.plot([1.2, 14.8], [BUSY, BUSY], color=INK, lw=4, solid_capstyle="round", zorder=2)
ax.text(8.0, BUSY + 0.18, "CAN bus · 1 Mbit/s", ha="center", va="bottom",
        fontsize=12.5, fontweight="bold", color=INK)
for x, col in ((BX + BW/2, A_EDGE), (B2X + BW/2, B_EDGE)):
    ax.annotate("", xy=(x, BUSY + 0.06), xytext=(x, BY - 0.04),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=2.2, mutation_scale=16))
ax.text(8.0, BUSY - 0.22, "each ESC reads the other's TELEM straight off the bus, every 1 ms "
        "— the Pi is not in this loop", ha="center", va="top", fontsize=12.5, color=MUTED)

# ---- the consequence --------------------------------------------------------
ax.add_patch(FancyBboxPatch((3.3, 0.14), 9.4, 0.98,
                            boxstyle="round,pad=0,rounding_size=0.16",
                            fc=KEY_FILL, ec=KEY_EDGE, lw=2.0, zorder=2))
ax.text(8.0, 0.90, "err$_B$ = − err$_A$     ⟹     $i_B$ = − $i_A$", ha="center", va="top",
        fontsize=16.5, fontweight="bold", color=KEY_EDGE, zorder=3)
ax.text(8.0, 0.49, "equal and opposite — neither motor is in charge", ha="center", va="top",
        fontsize=12.5, color=INK, zorder=3)

fig.subplots_adjust(left=0.008, right=0.992, top=0.992, bottom=0.008)
fig.savefig(OUT, dpi=190, facecolor="white")
print("wrote", OUT)
