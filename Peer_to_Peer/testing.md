# P2P Testing Log

Results of every peer-to-peer (P2P) test, newest at the bottom. For what P2P is and what was built, see [README.md](README.md). The project-wide running log is `ESC_based/software/TEST_LOG.md`; P2P entries there are summarized here, and **new P2P results go in this file**.

## How to read this file

- **Motors:** A = ESC 1 (node 0, CAN base 0x000), with the 3D-printed arm; G_A = 4.082e6 counts/s² per A. B = ESC 2 (node 1, base 0x020), the bare handle; G_B = 8.877e6.
- **Units:**
  - **Position:** counts, 4000 per rev; 1 count ≈ 0.12 mm at A's tip.
  - **Current:** A or mA. 1.058 N per A at the tip.
- **Gap:** x_A − x_B, taken from the firmware's own DBG `err` (zeroed at engagement). The Pi's position columns have an arbitrary offset, because START re-aligns the encoders after the Pi's first sample. `p2p_teleop.py` re-zeros at engagement since 2026-11-19.
- **Common mode:** (x_A + x_B)/2, the two shafts moving together.
- **Logs:** on the Pi, in `~/1DOF-FOC-Force-Feedback/ESC_based/software/logs/`.
  - `p2p_<settings>.csv` holds one row per motor per 1 kHz tick.
  - `.json` holds the settings and the end-of-run summary.
  - `_cal.csv` / `_cal.json` hold the ripple calibration sweep and fit.
  - Filenames encode the settings: `WN` ωn Hz, `Z` ζ, `KF` Kf %, `G` observer cutoff rad/s, `FR` friction comp %, `R` ripple comp %, `W` warm-up s, `A` Iq max.
- **Run from Windows** (the CAN bus is on the Pi):
  ```
  ssh -t jayrpi4@192.168.1.164 "cd ~/1DOF-FOC-Force-Feedback/ESC_based/software && python3 p2p_teleop.py <args>"
  ```
  After a power cycle or a flash, run `zero_iq_check.py` first. "RUN bit never seen" with live telemetry means the ESC motor supply is off.
- **Analyse A/B runs:** `python3 p2p_ab.py logs/<run1>.csv logs/<run2>.csv` (see the 2026-11-19 protocol below). Copy logs to the PC with `scp` if needed.

## Current operating point (2026-11-19)

`p2p_teleop.py --kf 50 --dob-g 80` with the defaults ωn 20 Hz, ζ 1.0, Iq max 0.8 A, ripple comp 100 % (in-session calibration), friction comp 0 %. Firmware: CHANGE 10 (commit `44244ea`, branch `feature/p2p-dob-bilateral`). User verdict on this point: **"It felt good."**

---

## 2026-11-18: Offline simulation (`p2p_sim.py`)

Toy model: per-ESC 1 kHz ticks 0.25 ms apart, ~1 ms peer delay, encoder quantization, 1 ms mean Iq, 440 Hz current loop, 9 mA current noise, Coulomb friction, measured G values. **No structural modes.**

| Law | Hold B, push A 0.3 A: steady gap | 3 Hz free sweep: hand force | B against a wall: hand / wall force, gap |
|---|---|---|---|
| PEER, Kp 0.007, Kd 2.5e-5 | 45 counts | 114 mA rms | 229 / 216 mA, 31 counts |
| P2P ωn 20, ζ 1, g 150, Kf 100 % | ≈ 2 counts | 54 mA (24 with friction comp) | 288 / 287 mA, 0.2 counts |

- **Stability:** unstable when Kv = 2ζωn > ~500 /s, or Kf at 300 % against a stiff wall.
- **Transient:** a sudden 0.3 A push against a held shaft opens an ~81-count gap for ~20 ms.
- **Later shown optimistic:** the hardware has a ~20 Hz common-mode wobble the sim doesn't have (see below).

## 2026-11-18: Flash and no-motion checks (CHANGE 9)

- **Builds:** both 0 warnings. `flash_and_verify.ps1` PASS; `zero_iq_check.py` PASS.
- **Parameters:** all five P2P SET_PARAMs (ids 3–7) are accepted on both boards. An out-of-range value is rejected (DIAG bit 6).
- **Bug found and fixed:** the firmware latches SET_PARAM in one latest-wins slot, so two frames within 1 ms kept only the last. `send_start()` was sending Tafter and the current-loop gain ~0.1 ms apart. `send_set_param` now waits 3 ms after each frame; Tafter 400 is confirmed in the DIAG2 readback.

## 2026-11-19: First hardware runs, hands off (0.4 A cap, 10 s each)

All engaged for the full run, **no trips, 0 % saturated**, worst peer age 0.6 ms.

| Settings | Gap p50 / p95 / max | Gap 40–90 Hz rms | Common-mode rms | Common 10–30 Hz | Max \|Iq\| A |
|---|---|---|---|---|---|
| ωn 20, Kf 100, g 150 (defaults) | 3 / 10 / 22 counts | 2.59 | 15.8 counts | 13.8 | 0.216 A |
| ωn 20, Kf 50, g 150 | 3 / 9 / 19 | 2.54 | 8.0 | 5.8 | 0.182 |
| ωn 20, Kf 50, g 80 | 3 / 8 / 17 | 1.97 | 9.1 | 4.8 | 0.167 |
| ωn 15, Kf 50, g 80 | 3 / 10 / 20 | — | 7.5 | 4.4 | 0.121 |

- **Gap:** hands-off buzz in the gap is ~3× lower than PEER mode's on the same metric (7.72 counts on 2026-11-15).
- **Open issue: a ~20 Hz common-mode wobble.** Both shafts move together at 19.5 Hz; the gap also shows ~21, ~30 and ~41 Hz. Halving Kf halves it.
- **It doesn't move with ωn** (20 → 15 Hz), so it isn't the position servo. Inferred: a structural mode (arm A or its mount) excited by the observers' noise. A tap or HOLD test on each arm would confirm it.

## 2026-11-19: First hand test (0.4 A cap, no ripple compensation)

- **User:** "limited to 0.4 A" (the `--iq-max 0.4` first-run cap) and "a lot of cogging".
- **Saturated 33 % of ticks;** gap p95 1527, max 1852 counts. The cap was too low to hold one shaft against the other.
- **Angle-locked ripple in each motor:**
  - B: 12/rev 38 mA, 7/rev 24 mA, 14/rev 23 mA.
  - A: 7/rev 25 mA, 12/rev 14 mA, 28/rev 12 mA.
- **Diagnosis:** each motor's observer reported its own ripple as external force, so the force servo passed both motors' ripple to the hand. Fixed by CHANGE 10 (in-session ripple calibration).

## 2026-11-19: Ripple calibration, 0.8 A (user hand test, 60 s)

`--kf 50 --dob-g 80`, Iq max 0.8 A, ripple 100 %. No trip.

- **Saturation fixed:** **3.7 %** of ticks (was 33 %). Gap p50 3, p95 67, max 892 counts. Force mismatch in contact 10 %.
- **Calibration** (both motors, 1.25 rev each way, ~35 s):

  | Motor | Largest harmonics | Angle-locked rms, raw → after fit |
  |---|---|---|
  | A | 12/rev 18.5, 28/rev 15.4, 7/rev 14.5, 1/rev 13.8 (gravity) mA | 28.2 → 13.9 mA |
  | B | 12/rev 36.2, 28/rev 13.0 mA | 28.6 → 10.7 mA |

- **Ripple left in the hand force** (calibrated ripple minus the angle-locked part of iq − a/G):
  - A: 24 → **14 mA rms**.
  - B: 29 → **31 mA**, no gain. 12/rev fell 36 → 27, but 7/rev rose 6 → 20.
  - Caveats: no back-to-back uncompensated run for comparison, and cross-talk between the locked shafts.
- **User:** "It felt good."

## 2026-11-19: Warm-up A/B #1 (no warm-up vs 300 s warm-up): inconclusive

**Hypothesis:** B's 7/rev grows as the boards warm after START (TEST_LOG 2026-10-28: τ ≈ 160 s, ~65 mA warm), so a calibration done cold under-cancels it.

**The two runs were used very differently,** so the felt-ripple comparison is invalid:

| | Run 1, no warm-up (23:29) | Run 2, 300 s warm-up (23:36) |
|---|---|---|
| Motion | A 63k, B 77k counts of travel | A 12k, B 11k counts, all inside a ~250-count span (8 % of the circle) |
| Saturation | 27 % (lots of hard hold-and-push) | 0 % |
| Gap p95 / max | 1100 / 1693 counts | 7 / 21 counts |

- **Run 1:** with heavy saturation, the angle-locked fit mostly picks up hand force.
- **Run 2:** too little angle coverage to fit at all.

**The calibrations themselves are comparable,** and they **do not support** the warm-up hypothesis:

| 7/rev at calibration | A | B |
|---|---|---|
| 23:05 (cold) | 15 mA | 6 mA |
| 23:29 (Run 1, no warm-up) | 4 mA | 26 mA |
| 23:36 (Run 2, after warm-up) | 21 mA | 13 mA |

- **7/rev changes from START to START** on both motors with no warm-up trend. Possibly CHANGE 5 (switching offset calibration, flashed after the 10-28 measurement) changed the behaviour.
- **12/rev and 28/rev repeat** to within a few mA, so the calibration itself is consistent.
- **Conclusion:** in-session calibration stays. Warm-up is not shown to help, so it isn't worth 5 min per session yet.
- **Lesson:** A/B runs need identical movements. Hence `--guide` and `p2p_ab.py` (next section).

## A/B protocol from 2026-11-19 on: `--guide` + `p2p_ab.py`

`p2p_teleop.py --guide` runs exactly 60 s and prints a cue at each phase:

| Phase | Time | Do |
|---|---|---|
| settle | 0–5 s | Hands off |
| slow_B | 5–25 s | Turn B slowly and steadily (~1 turn per 4 s), back and forth; hand off A |
| slow_A | 25–45 s | The same on A; hand off B |
| push | 45–60 s | Hold B firmly; push A back and forth |

The phase times are saved in the run's JSON (`guide`, `engage_mcu_us`). `p2p_ab.py` reports two metrics for each slow phase, on the hand-turned shaft:

- **Speed ripple:** angle-locked harmonics of the shaft's speed, as % of mean speed. Model-free, so it works for `--no-cal` runs too.
- **Force ripple in the hand:** in mA, for calibrated runs only.

A phase whose turning covers < 95 % of the circle is reported as "insufficient coverage" instead of a number.

## P2P vs the old teleop controller: protocol and baselines (set up 2026-11-22)

**Roles:** the operator holds **B only** (teleop's leader); A touches the environment (teleop's follower). Each run uses `p2p_teleop.py --guide <routine> --surface <what A touches>` at the operating point `--kf 50 --dob-g 80`. Taps and contact runs pause after the ripple calibration so the surface can be placed under A; the calibration swings A 1.25 turns.

| Run | Command flags | Phases |
|---|---|---|
| Free space | `--guide free --surface air` | 30 s free motion with B; A touches nothing |
| Taps | `--guide taps --surface rigid` | 30 s of discrete taps of A onto the rigid stop |
| Contact, rigid | `--guide contact --surface rigid` | 20 s slow presses, 8 s gentle hold, 8 s firm hold |
| Contact, sponge | `--guide contact --surface sponge` | The same, on the sponge |

**Analysis:** `python3 p2p_compare.py <the four csv>`. Forces come from replaying the firmware observers on the logged data. Definitions and detector code match the teleop evaluation scripts:

| Metric | Teleop baseline (TEST_LOG) | P2P measure |
|---|---|---|
| Free-space RMS | `iq_b_cmd` 18.7 mA (C3 op point); free felt 10.1 mA (HOLD3); 26.2 mA (run 21) | RMS of B's current, plus RMS of the hand-force estimate on B (B's current also carries its own friction/inertia cancellation, so the hand force is the fairer felt number) |
| Tap rise | median 4.0 ms (C3 taps-only; p25 4, p75 34); original path 66 ms | `analyze_teleop_baseline.impacts()`, the same detector, on B's current and on the hand force |
| Holds | plateau slope −0.03 / +0.02 A/s (HOLD3, contact spring); onset flips 2 | `hold_eval.py` definitions on \|f_A\|, plus static matching \|f_A + f_B\| / \|f_A\|. Teleop's "static ≤ 5 %" was measured against a scale, so this is an analogue, not the same test |
| Rigid vs sponge | follower k 1.62 N/mm rigid (lower bound) vs 0.08 sponge (19.9×); leader travel to 0.7 N: 382 rigid / 473 sponge counts | `stiffness_eval.py` definition on A, plus B's travel from contact to 0.7 N |

**Results:** pending (runs not done yet).

---

## Open issues

1. **~20 Hz common-mode wobble** (hands off). Probably structural; not yet located.
2. **Is B's residual cogging the 7/rev?** Varies per START; the warm-up A/B was inconclusive. Pending the guided A/B.
3. **Transient gap under sudden pushes** (sim: ~81 counts for ~20 ms against a held shaft). Not yet measured on hardware.
4. **Friction compensation (`--fric`)** has not been tried on hardware yet.
