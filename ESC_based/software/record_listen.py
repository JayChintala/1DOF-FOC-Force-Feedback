"""
Record the bench mic (JayPC) while a test runs on the Pi, then analyse the
motor whine per phase. Runs on the Windows host, not the Pi.

  python record_listen.py --remote "taskset -c 2,3 python3 listen_test.py" --out <dir>

Saves <out>/audio.wav (48 kHz, mic channel), <out>/phases.json and
<out>/spectrogram.png. For each "PHASE n:" phase it prints the strongest
tonal peaks (above the phase-1 baseline) and, for the strongest one, how its
frequency wanders over time (min / max / period of the sweep).
"""

import argparse
import json
import os
import subprocess
import sys
import time
import wave

import numpy as np
import sounddevice as sd
from scipy import signal

FS = 48000
PI = "jayrpi4@192.168.1.164"
PI_DIR = "~/1DOF-FOC-Force-Feedback/ESC_based/software"


def record(remote, ch):
    chunks = []

    def cb(indata, frames, t, status):
        chunks.append(indata[:, ch].copy())

    phases = []
    with sd.InputStream(samplerate=FS, channels=ch + 1, dtype="float32", callback=cb):
        t0 = time.monotonic()
        p = subprocess.Popen(["ssh", "-o", "BatchMode=yes", PI, f"cd {PI_DIR} && {remote}"],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in p.stdout:
            t = time.monotonic() - t0
            print(f"[{t:6.2f}] {line.rstrip()}", flush=True)
            if line.startswith("PHASE "):
                n, text = line[6:].split(":", 1)
                phases.append({"n": int(n), "text": text.strip(), "t": t})
        p.wait()
        time.sleep(1.0)
    return np.concatenate(chunks), phases


def peaks(f, P, base, k=5):
    rel = 10 * np.log10(P / base)
    sel = (f > 200) & (f < 23000)
    idx, _ = signal.find_peaks(np.where(sel, rel, -99), prominence=6, distance=20)
    idx = idx[np.argsort(rel[idx])[::-1][:k]]
    return [(float(f[i]), float(rel[i]), float(10 * np.log10(P[i]))) for i in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--remote", default="taskset -c 2,3 python3 listen_test.py")
    ap.add_argument("--out", required=True)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--wav", help="analyse an existing recording instead")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    if a.wav:
        with wave.open(a.wav) as w:
            x = np.frombuffer(w.readframes(w.getnframes()), np.int16) / 32768.0
        phases = json.load(open(os.path.join(a.out, "phases.json")))
    else:
        x, phases = record(a.remote, a.channel)
        with wave.open(os.path.join(a.out, "audio.wav"), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS)
            w.writeframes((np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes())
        json.dump(phases, open(os.path.join(a.out, "phases.json"), "w"), indent=1)

    bounds = [(p, p["t"] + 1.5, (phases[i + 1]["t"] if i + 1 < len(phases) else p["t"] + 3) - 0.3)
              for i, p in enumerate(phases)]
    base = None
    for p, t1, t2 in bounds:
        seg = x[int(t1 * FS):int(t2 * FS)]
        if len(seg) < FS:
            continue
        f, P = signal.welch(seg, FS, nperseg=8192)
        if base is None:
            base = P.copy()
            print(f"\nPHASE {p['n']} ({p['text']}): baseline, rms {np.sqrt(np.mean(seg**2)):.5f}")
            continue
        pk = peaks(f, P, base)
        print(f"\nPHASE {p['n']} ({p['text']}): rms {np.sqrt(np.mean(seg**2)):.5f}")
        for fr, rel, ab in pk:
            print(f"   peak {fr:8.0f} Hz  +{rel:4.1f} dB over baseline")
        if pk:
            f0 = pk[0][0]
            fs_, ts, S = signal.spectrogram(seg, FS, nperseg=4096, noverlap=3072)
            band = (fs_ > f0 - 1500) & (fs_ < f0 + 1500)
            track = fs_[band][np.argmax(S[band], axis=0)]
            tr = track - track.mean()
            if len(tr) > 8 and tr.std() > 0:
                spec = np.abs(np.fft.rfft(tr * np.hanning(len(tr))))
                fr_ = np.fft.rfftfreq(len(tr), ts[1] - ts[0])
                fm = fr_[1:][np.argmax(spec[1:])]
            else:
                fm = 0.0
            print(f"   strongest tone track: {track.min():.0f}..{track.max():.0f} Hz "
                  f"(std {track.std():.0f} Hz), sweep rate ~{fm:.2f} Hz")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fs_, ts, S = signal.spectrogram(x, FS, nperseg=4096, noverlap=3072)
        plt.figure(figsize=(14, 6))
        plt.pcolormesh(ts, fs_ / 1000, 10 * np.log10(S + 1e-14), shading="auto", vmin=-130, vmax=-60)
        for p in phases:
            plt.axvline(p["t"], color="w", lw=0.8)
            plt.text(p["t"] + 0.2, 22.5, f"P{p['n']}", color="w")
        plt.ylabel("kHz"); plt.xlabel("s"); plt.colorbar(label="dB")
        plt.tight_layout(); plt.savefig(os.path.join(a.out, "spectrogram.png"), dpi=90)
        print(f"\nspectrogram: {os.path.join(a.out, 'spectrogram.png')}")
    except Exception as e:  # plotting is optional
        print("plot failed:", e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
