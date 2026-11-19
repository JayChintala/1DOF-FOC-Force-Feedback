"""
peer_to_peer.png: the symmetric (peer-to-peer) coupling, from 2026-11-15.

  left   equal and opposite -- i_A and -i_B overlay (Newton's third law)
  middle one object -- both shafts track within a few counts
  right  the open issue -- a ~57 Hz resonance, undamped unless a hand is on it

Run: python3 make_peer_figure.py   (writes next to this file)
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "..", "software", "logs",
                   "coupling_peer_KP0.007_KD2.5e-05_KDL0_A0.8_PRED_2.csv")
OUT = os.path.join(HERE, "peer_to_peer.png")

INK, MUTED, GRID = "#1f2933", "#5f6b7a", "#e3e7eb"
MOT_A, MOT_B, NEUTRAL = "#2a6fdb", "#d9682c", "#8a96a3"
ACCENT = "#6f4fa8"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "axes.edgecolor": "#b8c2cc",
                     "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.spines.top": False, "axes.spines.right": False})

def fl(v):
    try: return float(v)
    except Exception: return np.nan

rows = list(csv.DictReader(open(LOG)))
D = {}
for m in ("A", "B"):
    s = [x for x in rows if x["motor"] == m]
    D[m] = {c: np.array([fl(x[c]) for x in s]) for c in
            ("t_s", "pos", "iq_cmd", "err", "engaged")}
A, B = D["A"], D["B"]

# align B onto A's host timestamps (the two 1 kHz streams are asynchronous)
j = np.clip(np.searchsorted(B["t_s"], A["t_s"]), 1, len(B["t_s"]) - 1)
pick = np.where(np.abs(B["t_s"][j] - A["t_s"]) < np.abs(B["t_s"][j-1] - A["t_s"]), j, j-1)
t   = A["t_s"]
iqA, iqB = A["iq_cmd"], B["iq_cmd"][pick]
pA,  pB  = A["pos"],    B["pos"][pick]
errA = A["err"]
ok = (A["engaged"] > 0) & (B["engaged"][pick] > 0) & ~np.isnan(iqA) & ~np.isnan(iqB)

# No bold title here: the slide carries it. Keep only the law, as a strap line.
fig = plt.figure(figsize=(15.6, 4.55), dpi=200)
fig.text(0.052, 0.95, "Each board applies   "
         "i = \u2212(Kp\u00b7err + Kd\u00b7derr),    err = (own \u2212 own\u2080) \u2212 (peer \u2212 peer\u2080),    "
         "at 1 kHz on the ESC, with the Pi out of the loop.",
         ha="left", va="top", fontsize=12, color=MUTED)
gs = fig.add_gridspec(1, 3, left=0.055, right=0.985, top=0.80, bottom=0.145, wspace=0.30)

def style(ax, title):
    ax.set_title(title, loc="left", fontsize=12, color=INK, fontweight="bold", pad=8)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)

# ---- panel 1: equal and opposite -------------------------------------------
w = (t >= 24.0) & (t <= 28.0) & ok
ax = fig.add_subplot(gs[0, 0]); style(ax, "Equal and opposite")
ax.plot(t[w], iqA[w], color=MOT_A, lw=1.7, label="motor A current")
ax.plot(t[w], -iqB[w], color=MOT_B, lw=1.7, ls=(0, (4, 2.5)), label="− motor B current")
ax.set_xlabel("time (s)"); ax.set_ylabel("current (A)")
ax.legend(frameon=False, fontsize=10, loc="upper right", labelcolor=MUTED)
c = np.corrcoef(iqA[ok], -iqB[ok])[0, 1]
asym = 100*np.sqrt(np.mean((iqA[ok]+iqB[ok])**2))/np.sqrt(np.mean(iqA[ok]**2))
ax.text(0.03, 0.05, f"whole run: corr = +{c:.3f}\nresidual {asym:.1f}%",
        transform=ax.transAxes, fontsize=10, color=INK, va="bottom", bbox=dict(fc="white", ec="none", alpha=0.85, pad=2.5))

# ---- panel 2: one object ----------------------------------------------------
ax = fig.add_subplot(gs[0, 1]); style(ax, "The two shafts move as one")
b0 = np.nanmean(pA[w]) - np.nanmean(pB[w])
ax.plot(t[w], pA[w] - np.nanmean(pA[w]), color=MOT_A, lw=1.7, label="motor A position")
ax.plot(t[w], pB[w] + b0 - np.nanmean(pA[w]), color=MOT_B, lw=1.7, ls=(0, (4, 2.5)),
        label="motor B position")
ax.set_xlabel("time (s)"); ax.set_ylabel("position (counts, 4000 / rev)")
ax.legend(frameon=False, fontsize=10, loc="upper right", labelcolor=MUTED)
p50 = np.nanpercentile(np.abs(errA[ok]), 50)
ax.text(0.03, 0.05, f"median gap {p50:.0f} counts ({p50*0.119:.2f} mm at the tip)",
        transform=ax.transAxes, fontsize=10, color=INK, va="bottom", bbox=dict(fc="white", ec="none", alpha=0.85, pad=2.5))

# ---- panel 3: the resonance -------------------------------------------------
ax = fig.add_subplot(gs[0, 2]); style(ax, "Open issue: a ≈57 Hz resonance")
fs, W = 1000.0, 1000
quiet, moving = [], []
for a in range(0, len(t) - W, W):
    s = slice(a, a + W)
    if not np.all(ok[s]): continue
    e = errA[s] - np.nanmean(errA[s])
    if np.isnan(e).any(): continue
    P = np.abs(np.fft.rfft(e * np.hanning(len(e))))**2
    (quiet if (np.nanmax(pA[s]) - np.nanmin(pA[s])) < 150 else moving).append(P)
fr = np.fft.rfftfreq(W, 1/fs)
for arr, col, lab, ls in ((moving, NEUTRAL, "hands on, moving", "-"),
                          (quiet, ACCENT, "hands off", "-")):
    if arr:
        m = np.sqrt(np.mean(np.array(arr), axis=0))
        ax.semilogy(fr[1:], m[1:], color=col, lw=1.8, ls=ls, label=lab)
ax.axvspan(50, 70, color=ACCENT, alpha=0.08, lw=0)
ax.set_xlim(0, 150); ax.set_xlabel("frequency (Hz)")
ax.set_ylabel("coupling error (counts, log)")
ax.legend(frameon=False, fontsize=10, loc="upper right", labelcolor=MUTED)
ax.text(72, ax.get_ylim()[1]*0.25, "≈57 Hz\nundamped without\na hand on it",
        fontsize=10, color=ACCENT, va="top")

fig.savefig(OUT, facecolor="white")
print("wrote", OUT)
print(f"  corr(i_A, -i_B) = +{c:.5f}   residual asymmetry {asym:.1f}%   median |err| {p50:.0f} cnt")
