"""
springs.png: the two contact-latched springs.
  left:  ideal shape, current vs leader travel past the contact point (e - e0)
  right: one real press from run _20 (2026-10-29), latched 33.8-35.3 s

Run: python3 make_springs_figure.py   (writes next to this file)
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "..", "software", "logs", "teleop_KVs3e-05_FB-1_20.csv")
T0, T1 = 33.4, 35.7         # an isolated press: no other latch 4.9 s before or 0.5 s after

INK, MUTED, GRID = "#1f2933", "#5f6b7a", "#e3e7eb"
LEADER, FOLLOWER, NEUTRAL = "#2a6fdb", "#d9682c", "#8a96a3"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "axes.edgecolor": "#b8c2cc",
                     "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.spines.top": False, "axes.spines.right": False})

# operating point (TEST_LOG 0c UPDATE 3)
K, I_MAX = 0.5e-3, 0.05            # follower contact spring
K_L, I_L_MAX = 2e-3, 0.4           # leader contact spring
LEAK = 0.02                        # felt-force dead band, the main leak per pass


def style(ax, title):
    ax.set_title(title, loc="left", fontsize=12, color=INK, fontweight="bold")
    ax.grid(True, color=GRID, lw=0.8)
    ax.axhline(0, color="#b8c2cc", lw=1)


fig = plt.figure(figsize=(14, 7.2))
gs = fig.add_gridspec(2, 2, width_ratios=(1, 1.35), hspace=0.42, wspace=0.22,
                      left=0.085, right=0.98, top=0.86, bottom=0.09)
fig.suptitle("The two contact springs", x=0.085, ha="left", fontsize=17, fontweight="bold", color=INK)
fig.text(0.085, 0.915, "Both act only while a contact is latched, on how far the leader has gone past the "
         "contact point (e − e₀). Shown for a push toward +counts.", fontsize=11.5, color=MUTED)

# ---- ideal shapes
x = [i for i in range(-60, 301)]
ax = fig.add_subplot(gs[0, 0])
style(ax, "Follower contact spring  i_hold")
ax.plot(x, [min(max(K * v, 0), I_MAX) * 1e3 for v in x], color=FOLLOWER, lw=2.2)
ax.axhline(LEAK * 1e3, color=NEUTRAL, lw=1.2, ls="--")
ax.text(300, LEAK * 1e3 + 1.5, "20 mA leak per pass (dead band)", color=MUTED, fontsize=10, ha="right")
ax.plot([LEAK / K], [LEAK * 1e3], "o", ms=8, color=FOLLOWER, mec="white", mew=2, zorder=5)
ax.annotate("hold settles here:\nspring refills the leak\n(≈ 40 counts past contact)", (LEAK / K, LEAK * 1e3),
            (120, 3), color=INK, fontsize=10, arrowprops=dict(arrowstyle="-", color=INK, lw=0.8))
ax.text(300, 52.5, "cap 50 mA", color=MUTED, fontsize=10, ha="right")
ax.set_ylim(-8, 60)
ax.set_ylabel("follower current (mA)\n+ = into the object")
ax.set_xlabel("leader travel past contact, e − e₀ (counts)")

ax = fig.add_subplot(gs[1, 0])
style(ax, "Leader contact spring  i_L  (spring part)")
ax.plot(x, [-min(max(K_L * v, 0), I_L_MAX) for v in x], color=LEADER, lw=2.2)
ax.text(300, -0.455, "cap 0.4 A from 200 counts (18°)", color=MUTED, fontsize=10, ha="right")
ax.text(8, -0.13, "slope 2e-3 A/count\n(4× the follower's)", color=INK, fontsize=10)
ax.text(-55, 0.03, "0 before contact,\nand never pulls", color=MUTED, fontsize=10)
ax.set_ylim(-0.5, 0.1)
ax.set_ylabel("leader current (A)\n− = against the hand")
ax.set_xlabel("leader travel past contact, e − e₀ (counts)")

# ---- one real press
t, d, te, hold, lsp = [], [], [], [], []
with open(LOG) as f:
    for r in csv.DictReader(f):
        ts = float(r["t_s"])
        if T0 <= ts <= T1:
            t.append(ts - T0)
            te.append(float(r["tau_env"]))
            hold.append(float(r["iq_a_hold"]))
            lsp.append(float(r["iq_b_spring"]))
            d.append(float(r["err"]) - float(r["hold_e0"]) if r["hold_e0"] else float("nan"))
latched = [i for i, v in enumerate(d) if v == v]
assert latched and all(b - a == 1 for a, b in zip(latched, latched[1:])), "window holds more than one latch"
on, off = t[latched[0]], t[latched[-1]]

ax = fig.add_subplot(gs[0, 1])
style(ax, "Real press (run _20): leader travel past contact")
ax.axvspan(on, off, color="#efe9f8", lw=0)
ax.plot(t, d, color=INK, lw=2)
ax.text(on + 0.02, max(v for v in d if v == v) * 0.92, "latched", color="#6f4fa8", fontsize=10)
ax.set_ylabel("e − e₀ (counts)")
ax.set_xlim(0, T1 - T0)

ax = fig.add_subplot(gs[1, 1])
style(ax, "Real press: the currents")
ax.axvspan(on, off, color="#efe9f8", lw=0)
ax.plot(t, te, color=NEUTRAL, lw=1.6, label="τ̂_env  contact estimate (felt force before dead band)")
ax.plot(t, lsp, color=LEADER, lw=2, label="i_L  leader spring + contact damping")
ax.plot(t, hold, color=FOLLOWER, lw=2, label="i_hold  follower contact spring (0.3 s ramp-in)")
ax.set_ylabel("current (A)")
ax.set_xlabel("time (s)")
ax.set_xlim(0, T1 - T0)
ax.set_ylim(-0.95, 0.5)
ax.legend(loc="upper right", fontsize=9.5, frameon=False)

out = os.path.join(HERE, "springs.png")
fig.savefig(out, dpi=170, facecolor="white")
print("wrote", out, "latched", round(on + T0, 2), "-", round(off + T0, 2), "s")
