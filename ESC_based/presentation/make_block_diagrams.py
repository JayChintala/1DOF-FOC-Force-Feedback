"""
Block diagrams for the PI presentation (16:9, PNG + SVG):
  diagram_system.png      slide 3: hardware and data flow
  diagram_original.png    slide 4: original control law (felt = follower current)
  diagram_control.png     slide 7: current operating point (two observers, springs)

Run: python3 make_block_diagrams.py   (writes next to this file)
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

OUT = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "DejaVu Sans", "mathtext.fontset": "dejavusans"})

INK = "#1f2933"
MUTED = "#5f6b7a"
LEADER = ("#e7effd", "#2a6fdb")      # (fill, edge)
FOLLOWER = ("#fdeee4", "#d9682c")
OBSERVER = ("#e2f4ec", "#178a68")
LATCH = ("#efe9f8", "#6f4fa8")
PI = ("#eef1f4", "#46525f")
BAD = "#c0392b"


def canvas(ylim=(0, 9)):
    fig, ax = plt.subplots(figsize=(16, 9))
    ax.set_xlim(0, 16)
    ax.set_ylim(*ylim)
    ax.axis("off")
    fig.patch.set_facecolor("white")
    fig.subplots_adjust(0.01, 0.01, 0.99, 0.99)
    return fig, ax


def box(ax, x, y, w, h, colors, title=None, lines=(), title_size=16, size=13, lw=2.2):
    fc, ec = colors
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.18",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    n = len(lines) + (1 if title else 0)
    step = 0.42
    y0 = y + h / 2 + (n - 1) * step / 2
    if title:
        ax.text(x + w / 2, y0, title, ha="center", va="center", fontsize=title_size,
                weight="bold", color=ec, zorder=3)
        y0 -= step + 0.05
    for line in lines:
        ax.text(x + w / 2, y0, line, ha="center", va="center", fontsize=size, color=INK, zorder=3)
        y0 -= step


def terms_box(ax, x, y, w, h, colors, title, rows, footer=None, col2=2.45):
    """Command box: title, then rows of (math term, description)."""
    fc, ec = colors
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.18",
                                fc=fc, ec=ec, lw=2.2, zorder=2))
    ax.text(x + 0.22, y + h - 0.32, title, ha="left", va="center", fontsize=15,
            weight="bold", color=ec, zorder=3)
    yy = y + h - 0.82
    for term, desc, *new in rows:
        ax.text(x + 0.3, yy, term, ha="left", va="center", fontsize=14, color=INK, zorder=3)
        ax.text(x + col2, yy, desc, ha="left", va="center", fontsize=12, color=MUTED, zorder=3)
        if new and new[0]:
            ax.text(x + w - 0.15, yy, new[0], ha="right", va="center", fontsize=10,
                    weight="bold", color=ec, zorder=3)
        yy -= 0.44
    if footer:
        ax.text(x + 0.3, y + 0.28, footer, ha="left", va="center", fontsize=11.5,
                color=MUTED, style="italic", zorder=3)


def arrow(ax, pts, color=INK, lw=2.4, ls="-", label=None, label_at=None, label_size=13,
          label_color=None, ha="center", va="bottom"):
    """Polyline through pts with an arrowhead on the last segment."""
    for (x0, y0), (x1, y1) in zip(pts[:-2], pts[1:-1]):
        ax.plot([x0, x1], [y0, y1], color=color, lw=lw, ls=ls, solid_capstyle="butt", zorder=1)
    ax.annotate("", xy=pts[-1], xytext=pts[-2], zorder=1,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw, ls=ls,
                                mutation_scale=22, shrinkA=0, shrinkB=0))
    if label:
        lx, ly = label_at if label_at else ((pts[-2][0] + pts[-1][0]) / 2, (pts[-2][1] + pts[-1][1]) / 2 + 0.1)
        ax.text(lx, ly, label, ha=ha, va=va, fontsize=label_size,
                color=label_color or color, zorder=4,
                bbox=dict(fc="white", ec="none", pad=1.5, alpha=0.9))


def title(ax, text, sub=None, y=8.7):
    ax.text(0.3, y, text, ha="left", va="center", fontsize=22, weight="bold", color=INK)
    if sub:
        ax.text(0.3, y - 0.48, sub, ha="left", va="center", fontsize=13.5, color=MUTED)


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(os.path.join(OUT, f"{name}.{ext}"), dpi=200, facecolor="white")
    plt.close(fig)
    print("wrote", os.path.join(OUT, name + ".png/.svg"))


# ---------------------------------------------------------------- slide 3
def system_diagram():
    fig, ax = canvas((-0.4, 9.2))
    title(ax, "System architecture", "Current loop on each ESC; teleoperation loop on the Pi; everything over one CAN bus",
          y=8.85)

    box(ax, 5.0, 5.7, 6.0, 2.2, PI, "Raspberry Pi 4",
        ["teleop control loop, 500 Hz (teleop_test.py)",
         "observers · felt force · contact springs",
         "real-time priority, logs to RAM"])

    bus_y = 4.25
    ax.plot([0.8, 15.2], [bus_y, bus_y], color=INK, lw=6, solid_capstyle="round", zorder=1)
    ax.text(15.2, bus_y + 0.3, "CAN bus · 1 Mbit/s", ha="right", va="bottom", fontsize=14,
            weight="bold", color=INK)

    # Pi <-> bus
    arrow(ax, [(7.2, 5.7), (7.2, bus_y)], color=PI[1])
    ax.text(7.0, 4.95, "commands, 500 Hz\nSET_IQ (current, A)\nSTART / STOP · SET_PARAM",
            ha="right", va="center", fontsize=12.5, color=INK)
    arrow(ax, [(8.8, bus_y), (8.8, 5.7)], color=PI[1])
    ax.text(9.0, 4.95, "telemetry, 1 kHz per ESC\nIq (1 ms mean) · position · µs timestamp\n+ diagnostics, 10 Hz",
            ha="left", va="center", fontsize=12.5, color=INK)

    for x0, colors, name, node in ((0.9, LEADER, "ESC 2", "node 0x020"),
                                   (11.1, FOLLOWER, "ESC 1", "node 0x000")):
        ax.plot([x0 + 2.0, x0 + 2.0], [3.55, bus_y], color=INK, lw=3, zorder=1)
        box(ax, x0, 1.75, 4.0, 1.8, colors, f"{name}  ·  B-G431B-ESC1",
            [f"{node} · FOC current loop 16 kHz", "20 ms command timeout"], title_size=14.5, size=12.5)
        ax.plot([x0 + 2.0, x0 + 2.0], [1.05, 1.75], color=colors[1], lw=3, zorder=1)

    box(ax, 1.2, -0.05, 3.4, 1.1, LEADER, "LEADER  motor B",
        ["+ encoder (4000 counts/rev)"], title_size=14, size=12)
    box(ax, 11.4, -0.05, 3.4, 1.1, FOLLOWER, "FOLLOWER  motor A",
        ["+ encoder · arm r = 75.7 mm"], title_size=14, size=12)
    arrow(ax, [(0.05, 0.5), (1.2, 0.5)], color=LEADER[1], label="hand", label_at=(0.55, 0.62))
    arrow(ax, [(15.95, 0.5), (14.8, 0.5)], color=FOLLOWER[1], label="object", label_at=(15.4, 0.62))

    ax.text(8.0, 2.65, "Each ESC: closes its own current loop,\nmeasures current + position,\n"
            "streams them to the Pi.\nThe Pi decides every current command.",
            ha="center", va="center", fontsize=12.5, color=MUTED, style="italic")
    save(fig, "diagram_system")


# ---------------------------------------------------------------- slides 4 and 7
def plants(ax):
    box(ax, 0.75, 1.0, 2.1, 5.8, LEADER, "LEADER", ["motor B", "ESC 2", "", "turned by", "the operator"],
        size=12.5)
    box(ax, 13.15, 1.0, 2.1, 5.8, FOLLOWER, "FOLLOWER", ["motor A", "ESC 1", "", "arm touches", "the object"],
        size=12.5)
    arrow(ax, [(0.0, 3.9), (0.75, 3.9)], color=LEADER[1])
    ax.text(0.37, 4.15, "hand", ha="center", va="bottom", fontsize=12.5, color=LEADER[1], rotation=0)
    arrow(ax, [(16.0, 3.9), (15.25, 3.9)], color=FOLLOWER[1])
    ax.text(15.63, 4.15, "object", ha="center", va="bottom", fontsize=12.5, color=FOLLOWER[1])


def hand_observer(ax):
    box(ax, 3.55, 6.0, 3.55, 1.9, OBSERVER, "Hand-torque observer",
        [r"$\hat{\tau}_h = a_B/G_B - i_B$", "derivative-free, 20 Hz"], title_size=14.5)
    arrow(ax, [(2.85, 6.4), (3.55, 6.4)], color=LEADER[1], label=r"$v_B,\ i_B$",
          label_at=(3.2, 6.5), label_size=12)


def original_diagram():
    fig, ax = canvas((-0.6, 9.4))
    title(ax, "Original control law", "Felt force = the follower's own measured current", y=9.0)
    plants(ax)
    hand_observer(ax)

    terms_box(ax, 8.3, 5.6, 4.4, 2.3, FOLLOWER, r"FOLLOWER   $i_A$ =",
              [(r"$\hat{\tau}_h$", "hand feedforward"),
               (r"$+$ tanh-P$(v_B - v_A)$", "velocity tracking")],
              footer="clamp ±0.8 A")
    arrow(ax, [(7.1, 6.95), (8.3, 6.95)], color=OBSERVER[1], label=r"$\hat{\tau}_h$",
          label_at=(7.7, 7.05))
    arrow(ax, [(12.7, 6.4), (13.15, 6.4)], color=FOLLOWER[1], label=r"$i_A$", label_at=(12.93, 6.5))

    box(ax, 8.9, 0.9, 3.6, 1.6, ("#fbeaea", BAD), "Low-pass filter",
        [r"EMA of measured $i_A$, about 2.8 Hz"], title_size=14.5)
    arrow(ax, [(13.15, 1.7), (12.5, 1.7)], color=FOLLOWER[1], label=r"$i_A$", label_at=(12.83, 1.8))

    terms_box(ax, 3.3, 0.6, 4.4, 2.3, LEADER, r"LEADER   $i_B$ =",
              [(r"$-\,\mathrm{EMA}(i_A)$", "felt force"),
               (r"$-\ B\,v_B$", "damping")],
              footer="clamp ±0.8 A")
    arrow(ax, [(8.9, 1.7), (7.7, 1.7)], color=BAD)
    arrow(ax, [(3.3, 1.5), (2.85, 1.5)], color=LEADER[1], label=r"$i_B$", label_at=(3.08, 1.6))

    ax.text(8.0, 4.1,
            "Problem: the follower's current is mostly its own inertia and friction,\n"
            "plus the echo of the operator's push (measured ratio 1.00 ± 0.04).\n"
            "Rigid and foam contacts felt the same.",
            ha="center", va="center", fontsize=14, color=BAD,
            bbox=dict(fc="#fdf3f2", ec=BAD, lw=1.5, boxstyle="round,pad=0.5"))
    ax.text(5.32, 8.2, "FORWARD   hand → follower", ha="center", fontsize=13, color=MUTED, weight="bold")
    ax.text(8.0, 0.0, "REFLECTION   follower → hand", ha="center", fontsize=13, color=MUTED, weight="bold")
    save(fig, "diagram_original")


def control_diagram():
    fig, ax = canvas((-0.9, 9.4))
    title(ax, "How force is passed between the motors (current operating point)",
          "Two disturbance observers; each estimate is sent to the other motor as feedforward", y=9.0)
    plants(ax)
    hand_observer(ax)

    # forward path
    terms_box(ax, 8.3, 5.35, 4.4, 2.85, FOLLOWER, r"FOLLOWER   $i_A$ =",
              [(r"$\hat{\tau}_h$", "hand feedforward"),
               (r"$+$ tanh-P$(v_B - v_A)$", "velocity tracking"),
               (r"$+\ i_{hold}$", "contact spring", "★"),
               (r"$+\ K_{pos}(e - e_{ref})$", "slipping position")],
              footer="clamp ±0.8 A")
    arrow(ax, [(7.1, 6.95), (8.3, 6.95)], color=OBSERVER[1], label=r"$\hat{\tau}_h$",
          label_at=(7.7, 7.05))
    arrow(ax, [(12.7, 6.4), (13.15, 6.4)], color=FOLLOWER[1], label=r"$i_A$", label_at=(12.93, 6.5))

    # reflection path
    box(ax, 8.85, 0.75, 3.65, 1.95, OBSERVER, "Environment-torque observer",
        [r"$\hat{\tau}_{env} = -(i_A - a_A/G_A)$", "minus fitted friction + gravity",
         "20 Hz, every 1 kHz frame"], title_size=14.5, size=12.5)
    arrow(ax, [(13.15, 1.6), (12.5, 1.6)], color=FOLLOWER[1], label=r"$v_A,\ i_A$",
          label_at=(12.83, 1.7), label_size=11.5)

    terms_box(ax, 3.3, 0.3, 4.4, 2.85, LEADER, r"LEADER   $i_B$ =",
              [(r"$K_F\cdot\mathrm{db}(\hat{\tau}_{env})$", "felt force"),
               (r"$-\ B\,v_B$", "damping"),
               (r"$+\ c(\theta_B, v_B)$", "cogging cancel"),
               (r"$+$ leader spring", "contact stiffness", "★")],
              footer="clamp ±0.8 A")
    arrow(ax, [(8.85, 1.6), (7.7, 1.6)], color=OBSERVER[1], label=r"$\hat{\tau}_{env}$",
          label_at=(8.08, 1.7))
    arrow(ax, [(3.3, 1.45), (2.85, 1.45)], color=LEADER[1], label=r"$i_B$", label_at=(3.08, 1.55))

    # contact latch
    box(ax, 6.55, 3.55, 3.3, 1.35, LATCH, "★ Contact latch",
        [r"$|\hat{\tau}_{env}| > 80$ mA: record contact point"], title_size=14, size=11.5)
    arrow(ax, [(8.6, 1.6), (8.6, 3.55)], color=LATCH[1], lw=1.8)
    arrow(ax, [(9.4, 4.9), (9.4, 5.35)], color=LATCH[1], lw=1.8, ls="--")
    arrow(ax, [(7.0, 3.55), (7.0, 3.15)], color=LATCH[1], lw=1.8, ls="--")
    ax.text(9.95, 4.25, "★ terms act only\nwhile in contact", ha="left", va="center",
            fontsize=11.5, color=LATCH[1])

    ax.text(5.32, 8.2, "FORWARD   hand → follower", ha="center", fontsize=13, color=MUTED, weight="bold")
    ax.text(5.5, -0.2, "REFLECTION   object → hand", ha="center", fontsize=13, color=MUTED, weight="bold")
    ax.text(8.0, -0.72,
            r"$\hat{\tau}_h$ and $\hat{\tau}_{env}$ are feedforward: no added loop gain, so force level and "
            "stability are tuned separately.   Pi loop 500 Hz · ESC current loop 16 kHz · telemetry 1 kHz",
            ha="center", va="center", fontsize=12, color=MUTED)
    save(fig, "diagram_control")


if __name__ == "__main__":
    system_diagram()
    original_diagram()
    control_diagram()
