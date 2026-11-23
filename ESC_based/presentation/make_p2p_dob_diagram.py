# -*- coding: utf-8 -*-
"""
diagram_p2p_dob.png: how peer-to-peer (P2P) control works since firmware
CHANGE 9/10 -- disturbance observers on both motors, a force channel and a
position channel, the same law on both ESCs. Replaces diagram_peer.png (the
old PEER spring) on the "Peer-to-Peer Coupling" slide.

Same palette and box style as make_peer_diagram.py; the halves mirror on
purpose. Run: python3 make_p2p_dob_diagram.py   (writes next to this file)
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "diagram_p2p_dob.png")
plt.rcParams.update({"font.family": "DejaVu Sans", "mathtext.fontset": "dejavusans"})

INK, MUTED = "#1f2933", "#5f6b7a"
A_FILL, A_EDGE = "#e7effd", "#2a6fdb"
B_FILL, B_EDGE = "#fdeee4", "#d9682c"
KEY_FILL, KEY_EDGE = "#efe9f8", "#6f4fa8"

fig, ax = plt.subplots(figsize=(16, 6.4))
ax.set_xlim(0, 16); ax.set_ylim(0, 6.4); ax.axis("off")

BX, BW, BY, BH = 0.7, 5.0, 2.32, 3.15
B2X = 16 - BX - BW


def motor(x, fill, edge, name, node, own):
    ax.add_patch(FancyBboxPatch((x, BY), BW, BH, boxstyle="round,pad=0,rounding_size=0.18",
                                fc=fill, ec=edge, lw=2.2, zorder=2))
    cx = x + BW / 2
    ax.text(cx, BY + BH - 0.40, name, ha="center", va="top",
            fontsize=18, fontweight="bold", color=edge, zorder=3)
    ax.text(cx, BY + BH - 0.95, f"node {node} · FOC 16 kHz · law at 1 kHz",
            ha="center", va="top", fontsize=12.5, color=INK, zorder=3)
    ax.plot([x + 0.5, x + BW - 0.5], [BY + 1.95, BY + 1.95], color=edge, lw=1.0, alpha=0.45, zorder=3)
    ax.text(cx, BY + 1.80, "observers on BOTH motors → forces $f_A$, $f_B$",
            ha="center", va="top", fontsize=12.5, color=INK, zorder=3)
    ax.text(cx, BY + 1.36, "$\\ddot{x}$ = ½ [ Kf·($f_A$+$f_B$) − Kp·err − Kv·derr ]",
            ha="center", va="top", fontsize=13.5, color=INK, zorder=3)
    ax.text(cx, BY + 0.86, f"$i_{own}$ = $\\ddot{{x}}$ / $G_{own}$ + $d_{own}$",
            ha="center", va="top", fontsize=15, fontweight="bold", color=edge, zorder=3)
    ax.text(cx, BY + 0.38, "same law, own and peer swapped", ha="center", va="top",
            fontsize=11.5, color=MUTED, style="italic", zorder=3)


motor(BX, A_FILL, A_EDGE, "MOTOR A  ·  ESC 1", "0x000", "A")
motor(B2X, B_FILL, B_EDGE, "MOTOR B  ·  ESC 2", "0x020", "B")

for x0, x1, col, lab, ha in ((0.05, BX - 0.08, A_EDGE, "hand", "left"),
                             (15.95, B2X + BW + 0.08, B_EDGE, "hand", "right")):
    ax.annotate("", xy=(x1, BY + BH / 2), xytext=(x0, BY + BH / 2),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=2.4, mutation_scale=18))
    ax.text(x0, BY + BH / 2 + 0.30, lab, ha=ha, va="bottom",
            fontsize=12, color=col, fontweight="bold")

# ---- the two modes, in place of the old spring -------------------------------
cx = 8.0
for y, head, body in ((BY + BH - 0.30, "force channel", "$f_A$ + $f_B$ → 0\naction = reaction"),
                      (BY + 1.30, "position channel", "$x_A$ − $x_B$ → 0\nshafts at the same angle")):
    ax.add_patch(FancyBboxPatch((cx - 1.85, y - 1.18), 3.7, 1.10,
                                boxstyle="round,pad=0,rounding_size=0.14",
                                fc=KEY_FILL, ec=KEY_EDGE, lw=1.8, zorder=2))
    ax.text(cx, y - 0.16, head, ha="center", va="top", fontsize=13.5,
            fontweight="bold", color=KEY_EDGE, zorder=3)
    ax.text(cx, y - 0.50, body, ha="center", va="top", fontsize=12, color=INK,
            linespacing=1.3, zorder=3)
for x, d in ((BX + BW + 0.10, 1), (B2X - 0.10, -1)):
    for y in (BY + BH - 0.88, BY + 0.72):
        ax.annotate("", xy=(x, y), xytext=(x + d * 0.62, y),
                    arrowprops=dict(arrowstyle="<|-|>", color=KEY_EDGE, lw=1.8, mutation_scale=14))

# ---- CAN bus ----------------------------------------------------------------
BUSY = 1.60
ax.plot([1.2, 14.8], [BUSY, BUSY], color=INK, lw=4, solid_capstyle="round", zorder=2)
ax.text(8.0, BUSY + 0.18, "CAN bus · 1 Mbit/s", ha="center", va="bottom",
        fontsize=12.5, fontweight="bold", color=INK)
for x, col in ((BX + BW / 2, A_EDGE), (B2X + BW / 2, B_EDGE)):
    ax.annotate("", xy=(x, BUSY + 0.06), xytext=(x, BY - 0.04),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=2.2, mutation_scale=16))
ax.text(8.0, BUSY - 0.22, "each ESC reads the other's encoder and current off the bus every 1 ms, "
        "and runs that motor's observer itself — the Pi is not in this loop",
        ha="center", va="top", fontsize=12, color=MUTED)

# ---- the consequence --------------------------------------------------------
ax.add_patch(FancyBboxPatch((2.6, 0.14), 10.8, 0.98, boxstyle="round,pad=0,rounding_size=0.16",
                            fc=KEY_FILL, ec=KEY_EDGE, lw=2.0, zorder=2))
ax.text(8.0, 0.90, "observer d = friction, cogging, gravity, the hand   ⟹   cancelled locally",
        ha="center", va="top", fontsize=14.5, fontweight="bold", color=KEY_EDGE, zorder=3)
ax.text(8.0, 0.49, "a push moves BOTH shafts instead of stretching a spring — "
        "no leader, no follower", ha="center", va="top", fontsize=12.5, color=INK, zorder=3)

fig.subplots_adjust(left=0.008, right=0.992, top=0.992, bottom=0.008)
fig.savefig(OUT, dpi=190, facecolor="white")
print("wrote", OUT)
