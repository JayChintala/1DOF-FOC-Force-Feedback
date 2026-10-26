# ESC_based test log: sensorless force-feedback plan

Running log of the Phase A–D plan, run on the Pi against the ESC_based
firmware on both B-G431B-ESC1 boards. Newest entries at the bottom.

## TL;DR (updated 2026-10-20, end of session; paused by the user)

**Status:** Phase A done; Phase B partly done.

| Step | Status |
|---|---|
| A1–A6 | done (A4 FAIL on the brief's model, extended model accepted) |
| B1 | PASS (live) |
| B2 | PASS (offline) |
| B3 | PASS (offline) |
| B4 | PASS (flashed 2026-10-25; noise 9.7× lower; aliased peak reduced, not gone) |
| B5 | dropped by the user |
| B6 | FAIL (offline) |
| B7 | **PASS (levels 1–6)**: Kt = 64.6 mN·m/A at r ≈ 160 mm (−2 % vs nominal). The 4 cm rerun confirms the scaling (3×, rig wobble). Motor mount not rigid: clamp it before Phase C data |
| C1 | **Gate A PASS (offline)**: free 10.3 mA (gentle motion), static ≤ 2.8 %, sign settled, rigid ≥ 6 N/mm vs foam ≈ 0.3 N/mm |
| C2 | **PASS (live, 3D-printed arm, fitted model)**: free floor 19.1 mA (contact excluded; brief's 15 mA not met), static ≤ 5.0 %, loop p95 2.09 ms |
| C3 | **Operating point K_F 1.0, KB_DAMP 0.5e-5, DB 20 mA**: stable, free felt 35.9 mA (target 34.7), user "felt better". KB 0.25e-5 oscillated in contact (stopped). Contact capped by the about 0.8 A ceiling. Tap rise TBD (taps-only run) |

**Blocker for closed-loop work:** host stalls of up to 2 s.

- **To resume, read "Handoff → 0. Resume here" at the bottom of this
  file** first. Then use the start-of-session checklist.
- The code is committed and pushed on branch
  `tune/current-sensing-gain-16x`.
- **Logs are NOT in git.** `logs/` is git-ignored (about 183 MB). They
  stay on the Pi in `ESC_based/software/logs/`.
- Firmware work for the dev machine is in **"Firmware changes"**, below
  the TL;DR.

**Headline findings:**

- **The felt force is an echo, not the environment.** In every hand-clamp
  and foam press, iq_b_cmd = −tau_h (ratio 1.00 ± 0.04), and the
  follower's command is 95–125 % feedforward. Table and foam presses are
  indistinguishable. Phase C exists to fix this.
- **Baseline to beat:**

  | Metric | Now | C3 / D3 target |
  |---|---|---|
  | Free-space RMS felt current (iq_b_cmd) | 0.1156 A | ≤ 0.035 A |
  | Hand-clamp tap 10–90 % rise | 66 ms (truncated edges, 0.55 A preload) | ≤ 30 ms |

- **Plant gains (A2):** G_A = 9.025e6 and G_B = 9.672e6 counts/s² per A,
  with standard errors 2.7 % and 2.0 %. That is +18 % and +22 % on the
  constants in `teleop_test.py`.
  - The user chose **not** to apply them yet. Apply them through B3's
    observer config.
- **Friction is about 100× what the brief assumes.**
  - Breakaway: 0.05–0.09 A. Running: about 0.04 A. Both shafts.
  - Stick-slip below about 2,000 counts/s.
  - Coasting at 0 A stops within 0.1 rev.
  - The brief's "Coulomb 0.0004 A" is wrong for this rig.
- **Ripple (A4):** the brief's model set **fails** (20–43 % explained).
  - Angle-locked, current-independent: 12/rev (about 22 mA, largest),
    24, 28 and 36/rev.
  - Grows with current: 14, 42 and 84/rev.
  - **7/rev is loop-driven and must not be tabulated.**
  - Gravity is negligible (1/rev ≤ 5 mA).
- **Noise (A3):** raw single-sample std 0.030–0.043 A at rest, and it
  varies 1.3–2× with angle. Drift is 3.1 mA per 10 min (PASS). Values sit
  on a lattice. There is a 247 Hz peak, probably aliased.
- **Command path (A5):** host traffic adds no noise (PASS on still
  segments). Step response reaches 50 % at 1.47 ms after the host send.

**Hazards and gotchas:**

- **Idle Iq is garbage** (about ±25 A) until a board has been STARTed
  once after power-up.
- **First START after ESC power-up** runs a 2 s, 0.8 A alignment.
  - SET_IQ is dropped until it finishes.
  - When it ends, the encoder count jumps 200–500 counts in one frame.
  - After any reset, run `zero_iq_check.py` (with the user's OK) before
    teleop. `bus_check.py` detects a reset passively.
- **Both ESCs sometimes reset together**, mid-run, at low current. The
  cause is unknown; a shared supply is suspected.
- **Host loop stalls of up to 1,045 ms** (4 in about 220 s of teleop).
  - Lead: the RX thread's SCHED_FIFO request **fails silently**
    (`ulimit -r` = 0, PermissionError), so it runs at normal priority.
- The CAN HAT IRQ is on CPU0, and `setup_can_realtime.sh` cannot find it.
  Run control scripts with `taskset -c 2,3`.
- **Direct SET_IQ is not clamped on the direct path** per the code (B5
  dropped by the user, who says a max current is set). The firmware
  zeroes current 200 ms after the last SET_IQ.
- The Pi's under-voltage was fixed this session. Check `throttled=0x0`
  each session.

**Conventions**

- Each entry gives what ran, the command, the numbers against the pass
  threshold, PASS/FAIL, and the log paths. Paths are relative to
  `ESC_based/software/`.
- **(inferred)** marks a claim reasoned from code or context but not measured.
- Motor A = ESC 1, node base 0x000, follower. Motor B = ESC 2, node base
  0x020, leader.

**Scripts** (all in `ESC_based/software/`)

| Script | Energizes? | Purpose |
|---|---|---|
| `bus_check.py` | no | start-of-session check: frame rates, gaps, idle Iq (reset detection), throttle |
| `zero_iq_check.py` | yes, 0 A | START both, hold 0 A for 4 s; re-aligns after a reset; checks Iq ≈ 0 in RUN |
| `teleop_test.py` | yes | copied unchanged from `software/` (the A1 baseline) |
| `plot_run.py` | no | copied unchanged; teleop imports it |
| `analyze_teleop_baseline.py` | no | A1 RMS and rise; A6 loop_dt; `--impacts` (not specific; see A1 run 4) |
| `plant_gain_test.py` | yes, ≤ 0.15 A bang-bang | A2. Ported: resend every tick, mcu_time_us, reset precheck |
| `spin_test.py` | yes | host PI velocity loop: `--node --speed --time --iq-max --kv --ki --coast`; overspeed floor 10,000 counts/s |
| `coupling_monitor.py` | yes | firmware HOLD/PEER coupling; HOLD with zero gains = 0 A with no host traffic (A3, A5a) |
| `analyze_noise.py` | no | A3: std, lattice, per-angle std, Welch spectrum, `--drift` |
| `analyze_ripple.py` | no | A4: angle-domain r_eq = i_cmd − a/G, harmonics, phases, `--harmonics` |
| `a5_command_test.py` | yes, ≤ 0.05 A | A5 (b) const and (c) staircase; `--analyze` |
| `estimators.py` | no (library) | per-instance classes: `LsVelocity`, `QuadLsVelocity`, `AlphaBetaVelocity`, `LegacyVelocity`, `HandTorqueObserver`, `RippleModel`, `ExternalTorqueObserver` |
| `b2_eval.py` | no | B2 offline estimator comparison |
| `b6_ripple.py` | no | B6 fit (saves `logs/ripple_model_A.json`) and held-out test |
| `b7_force_cal.py` | yes, ≤ 0.8 A | B7 schedule run (`--sign`) and fit (`--fit … --grams …`) |

`can_interface.py` gained three aliases in `get_telemetry()`
(`iq_readback_time`, `enc_count_time`, `iq_readback_bus_time`) so the
copied teleop runs without edits. One TELEM frame carries Iq and position,
so these were already equal in the old interface. No other change.

---

## Firmware changes (for the dev machine)

> **Authoritative instructions: `ESC_based/FIRMWARE_CHANGES.txt`**
> (2026-10-25), a self-contained file to hand to a Claude Code session on
> the dev machine. It supersedes the code snippets below where they
> differ. In particular, B4 there also removes the then-unused `iqd`
> variable (the raw read moves into the `#else` branch) and updates the
> TELEM comment in `can_driver.h`.

The Pi does not edit firmware. Make these on the dev machine by **hand
edit only**: never regenerate from Motor Control Workbench, because the
`.ioc` still says PGA 8x and would revert the 16x gain and the
hand-written FDCAN code. Flash **both** boards. Paths are under
`ESC_based/STM32/MCWorkbench/`.

| # | Change | Plan step | Status |
|---|---|---|---|
| 1 | TELEM sends the 1 ms boxcar mean of Iq, not one raw sample | B4 | **to do** |
| 2 | Shorter direct-mode host timeout: 200 ms → 20 ms | safety (from B1) | **to do (recommended)** |
| — | Clamp direct SET_IQ at ±0.8 A | B5 | **dropped**: user says a max current is already set (see the note below) |

### 1. Send the 1 ms mean of Iq in TELEM (B4)

**Why:**

- **Noise:** the current is sampled 16,000 times a second, but TELEM
  reports one raw sample per millisecond. That sample's noise is
  0.030–0.043 A std at rest (A3), while C1's free-space target is
  < 0.015 A RMS.
- **Aliasing:** keeping 1 sample in 16 with no anti-alias filter folds
  500 Hz–8 kHz content into the 0–500 Hz band the Pi sees. The 247 Hz
  peak in A3's spectrum is probably that (inferred).
- **Already computed:** the firmware already accumulates the 16 samples
  and computes their mean every period, then discards it. A 16-tap
  boxcar has nulls at exactly 1, 2, 3 kHz…, where the aliasing folds
  from.

**Where:** `STM32CubeIDE/Application/User/can_driver.c`,
`SendTelemetry()` (about lines 409–445).

- `mc_parameters.h` is already included there for `scaleParams_M1`.
- The accumulator is `IqTelem_UpdateHF` / `IqTelem_GetMeanAndReset` in
  `Src/mc_tasks_foc.c`, in raw s16A units.

```c
#ifndef TELEM_IQ_MEAN
#define TELEM_IQ_MEAN 1   /* 0 = raw single sample (pre-B4 behaviour) */
#endif
...
#if TELEM_IQ_MEAN
  float const iq_tx = IqTelem_GetMeanAndReset() * scaleParams_M1.current;
#else
  (void)IqTelem_GetMeanAndReset();   /* still drain it every period */
  float const iq_tx = iqd.q;
#endif
  memcpy(&payload[0], &iq_tx, sizeof(float));   /* replaces &iqd.q */
```

- Remove the existing unconditional `(void)IqTelem_GetMeanAndReset();`
  (about line 430), so the accumulator is drained exactly once per
  period.
- The TELEM layout is unchanged, so **no host change is needed**.

**Pass (B4):**

- Iq std ≥ 2× lower than before at the same angle. Before: 0.030 A free
  (raw ≈ 3996), 0.034 A at rest.
- The 247 Hz peak is gone.
- A5(c) step 50 % ≤ 1.47 + 0.6 = **2.07 ms**.

### 2. Shorten the direct-mode host timeout (recommended)

**Why:**

- If no SET_IQ arrives within `COUPLE_DIRECT_TIMEOUT_US`, the firmware
  zeroes the current. Today that is 200 ms.
- In B1 the Pi froze for 2.08 s, and the follower's last command
  (−0.07 A) stayed applied for those 200 ms. The free follower spun
  **1.8 rev**.
- Teleop can command up to 1 A, so the same freeze at a high command
  would be far worse.
- Host freezes keep happening: 4 of up to 1 s in Phase A, one of 2 s in
  B1. Their cause is not fixed yet (lead: the RX thread does not get
  SCHED_FIFO). This timeout bounds the damage meanwhile.

**Where:** `Inc/couple_ctrl.h`.

```c
#define COUPLE_DIRECT_TIMEOUT_US 20000u   /* was 200000u */
```

- **Compatibility:** every ESC_based script resends SET_IQ at least every
  10 ms (teleop, spin_test and a5 every 2 ms; plant_gain 1 ms;
  zero_iq_check 10 ms). HOLD/PEER coupling has no host timeout.
- A host stall over 20 ms now zeroes the current (trip HOST_TIMEOUT) and
  resumes on the next SET_IQ.

**Check:** none needed beyond normal runs. Optionally, stop sending
SET_IQ mid-run and confirm DBG shows trip HOST_TIMEOUT within about
20 ms.

### Note on the dropped clamp (B5)

The user says the firmware already has a max current set. For the
record, this repo's code shows:

- `IQMAX_A` = 0.8 (`Inc/drive_parameters.h`) limits the **speed PI
  output** (`Src/mc_config.c:52-55`).
- `MaxPositiveTorque` = NOMINAL_CURRENT (0.8 A) limits the **torque-ramp**
  path.
- A **direct SET_IQ** goes `Couple_SetDirectIq` →
  `MC_SetCurrentReferenceMotor1_F` → `MCI_SetCurrentReferences_F` (scale
  by CURRENT_CONV_FACTOR and cast to int16) → `MCI_ExecBufferedCommands`
  (`Src/mc_interface.c:723-726`), which writes `FOCVars.Iqdref` directly.
  Neither limit is applied there.
- The hardware OCP comparator (`DAC_OCP_Threshold` = 16334) is a fault
  trip that stops the motor, not a regulation limit.

So by this reading a SET_IQ of 2.0 A would be applied (inferred from
code). A 1 s test settles it: SET_IQ 2.0 with the shaft held, then read
DBG `iq_cmd`. The host-side limits stay as they are: teleop
`IQ1_MAX_A` / `IQ2_MAX_A` = 1.0, `spin_test --iq-max`, and B7's own
0.8 A cap.

### After flashing

1. The boards power-cycle, so `bus_check.py` shows RESET.
2. Run `zero_iq_check.py` (ask the user first). It re-aligns, and must
   print the alignment counter jump and PASS.
3. Run the B4 checks:
   - A3: `coupling_monitor.py --mode hold --motors a --kp 0 --kd 0
     --kd-local 0 --duration 30`, then `analyze_noise.py`. Same angle as
     `A3_free` if possible.
   - A5(c): `a5_command_test.py --mode stair`, then `--analyze`.
4. Log the results as a "B4" entry in this file.

---

## 2026-10-20: P0, bus and power precheck (no motor energized)

**What ran:** passive CAN receive for 3–4 s; `vcgencmd get_throttled`; `dmesg`.

**Results:**

- **Frames on the wire:** only TELEM (0x012 / 0x032, 8 B, 1000/s each)
  and DBG (0x112 / 0x132, 8 B, 10/s each). This matches `can_interface.py`
  and `can_driver.h`. MCU µs gaps were 999–1006 µs, with 0 outside
  1000 ± 50 over 4 s. Bus ERROR-ACTIVE, berr-counter 0/0.
- **Idle Iq is not a current.** Before any START since power-up:
  A = −24.41 A (std 0.066), B = +25.28 A (std 0.044).
  - **Cause:** `FOC_HighFrequencyTask` → `FOC_CurrControllerM1()` computes
    and stores Iqd every 16 kHz cycle even with PWM off, and TELEM sends
    `MC_GetIqdMotor1_F()`. Before the first START the shunt offsets are
    uncalibrated. Confirmed in P1: the idle reading after STOP is ≈ 0.
  - **Rule:** never use TELEM Iq from a node that has not been STARTed
    since power-up.
- **Iq quantisation is not a fixed 29.4 mA lattice.** 29.4 mA is one phase
  ADC count. Iq is a Park transform of the phase readings, so its step
  depends on rotor angle (16.5 mA observed at one fixed angle).
- **Power:** before the supply fix, `throttled=0x50005` (under-voltage and
  throttling active) with 97 undervoltage events in 34 min. After the user
  fixed the supply and rebooted: `throttled=0x0`, 0 events.
- **IRQ:** the CAN HAT interrupt (IRQ 58, `spi0.0`, GPIO25) is serviced on
  CPU0. `setup_can_realtime.sh` cannot find this IRQ (it looks for an IRQ
  named `can0`, which fits the old USB adapter). `rmem_max` is already
  4 MB. Workaround: run control scripts with `taskset -c 2,3`.

**Verdict:** PASS for the bus. Power is fixed.

---

## 2026-10-20: P1, 0 A precheck, first START after power-up

**What ran:** `python3 zero_iq_check.py` (first version: fixed 0.5 s
settle, 3 s hold). Both motors energized at 0 A.

**Log:** `logs/precheck_zero_iq_first_start_alignment.csv`

**Results:**

The first START after power-up ran the encoder alignment
(`drive_parameters.h`: `M1_ALIGNMENT_DURATION` 2000 ms,
`FINAL_I_ALIGNMENT_A` 0.8 A on Id, `M1_ALIGNMENT_ANGLE_DEG` 90).

- During alignment both shafts moved: A by 238 counts, B by 278, at
  0.1–0.3 s after START. Iq stayed ≈ 0 (mean A +0.002, B +0.002) because
  the alignment current is on the d axis.
- At **2.018 s** after START, both encoder counters jumped in a single
  frame to the same unwrapped value, 4142 (raw 142): A 3940 → 4142, B
  3627 → 4142. That frame's Iq read exactly 0.0.
  - 90° electrical at 7 pole pairs = 142.9 counts. This is the
    alignment's counter load plus `FOC_Clear()` on entry to RUN. It is not
    motion.
- **In RUN** (from 2.018 s to the end of the hold, 1.49 s):

  | Motor | Iq mean (A) | Iq std (A) | Drift at 0 A (counts) |
  |---|---|---|---|
  | A | +0.0023 | 0.0523 | 4142 → 4179 |
  | B | +0.0010 | 0.0295 | 4142 → 4214 |

  The drift is plausibly the links settling under gravity (inferred).
- **Idle before START:** A −28.22 A, B +25.14 A. **After STOP:** A
  +0.020 A, B −0.005 A. Offset calibration on START removes the idle
  artifact.

**Verdict:** PASS (|mean Iq| ≤ 0.05 A in RUN on both). The script's fixed
0.5 s settle was too short, so its "RUN" window included 1.5 s of
alignment. The script was fixed to start the window at the first DBG
frame with the RUN bit, and to flag one-frame encoder jumps.

**Consequence for `teleop_test.py`:** teleop waits 0.5 s after START and
then takes its position baselines. On an ESC that has just powered up, or
reset (e.g. ESC1 browning out), the alignment is still running then. When
it ends, the counter jumps 200–500 counts in 1 ms, roughly 250,000
counts/s on the velocity estimate. That is above `WATCHDOG_VEL_LIMIT_CNT_S`
(200,000). (inferred, not tested.)

- **Rule:** after any ESC power cycle, run `zero_iq_check.py` once before
  teleop.

---

## 2026-10-20: P2, 0 A precheck, aligned boards

**What ran:** `python3 zero_iq_check.py` (fixed version: RUN window from
the RUN bit, 4 s hold). Both motors energized at 0 A.

**Log:** `logs/precheck_zero_iq.csv`

**Results:**

- RUN bit seen 0.01 s after START on both, so no alignment ran. DBG is
  also sent on status change, so it is not limited to 10 Hz here.

| Motor | Phase | n | Iq mean (A) | Iq std (A) | Encoder (counts) |
|---|---|---|---|---|---|
| A | idle (calibrated) | 999 | −0.0431 | 0.0769 | 3255 |
| A | RUN, 0 A | 3992 | −0.0005 | 0.0760 | 3167..3274 |
| A | stopped | 553 | +0.0133 | 0.0651 | 3201..3211 |
| B | idle (calibrated) | 999 | −0.0024 | 0.0260 | 214 |
| B | RUN, 0 A | 3991 | +0.0005 | 0.0286 | 211..214 |
| B | stopped | 553 | −0.0023 | 0.0263 | 211 |

- **Single-sample noise:** A's raw Iq is about 2.6× noisier than B's
  (0.076 vs 0.029 A std). It is the same with PWM off (idle 0.077), so it
  comes from A's current sensing, not from the current loop (inferred from
  the PWM-off match). For scale: the brief's ripple is 0.009–0.016 A and
  Coulomb friction is 0.0004 A. Raw A samples are 5–8× noisier than the
  ripple being modelled. This feeds A3 and B4.
- **START transient:** each board's TELEM stream skips about 2 frames at
  START (one 3.0 ms gap), then has one short gap (A 500 µs, B 43 µs). The
  1 kHz tick re-phases at START (inferred). B1's "0 missing frames" check
  should exclude the first 10 ms after START.
- **A at 0 A:** A wandered 107 counts in RUN. With nothing commanded, this
  is the follower link moving freely (inferred).

**Verdict:** PASS (|mean Iq| A 0.0005, B 0.0005 ≤ 0.05 A; both in RUN).

---

## 2026-10-20: A1 run 1, free motion (aborted)

**What ran:** `timeout -s INT 75 taskset -c 2,3 python3 teleop_test.py`,
unchanged. Both motors energized. User on the leader.

**Logs:** `logs/teleop_KVs3e-05_FB-1.csv`, `logs/teleop_KVs3e-05_FB-1.png`

**Result:** the teleop watchdog tripped at t = 1.516 s with
`vel_a = 321412` counts/s (limit 200,000). The script zeroed both motors
and sent STOP. Only 756 rows were logged.

**Cause: ESC1 ran the encoder alignment during the run, and ESC2 most
likely did too.** Both had lost their aligned state between P2 and this
START, so they had been reset or power-cycled. The reset itself is not
explained.

- **Timing:** teleop's t = 0 is about 0.5 s after START, so the trip came
  2.02 s after START. That is the alignment end measured in P1 (2.018 s).
- **Magnitude:** A's last logged position was −781 unwrapped (raw 3219).
  The counter load to raw 142 reads as +923 counts through the unwrapper.
  Over the 2.01 ms velocity window that is 458,700 counts/s, and after the
  0.7 EMA, 321,400 counts/s. The watchdog printed 321,412.
- **Commands were not applied:** outside RUN the firmware drops SET_IQ
  (`couple_ctrl.c` `Couple_Tick`). The regression slope of measured Iq on
  commanded Iq was +0.04…+0.06 for A and −0.01…−0.08 for B, at lags of
  0–10 ms and with the command low-passed at 2–30 Hz. A's readback RMS
  (0.079 A) equals its 0 A noise floor. During the leader flick at
  t = 1.44–1.456 s, iq_a_cmd reached −0.9 A, A moved 6 counts, and A's
  readback stayed between −0.10 and +0.08 A.
- **Follower motion:** in the first 0.11 s A swept −479 counts, peaking at
  −11,800 counts/s, and stopped within 18 ms. It then crept to about −1000
  counts and buzzed ±5 counts at about 30 Hz. That is the 0.8 A Id
  alignment pulling the rotor to 90° electrical and holding it there, not
  gravity or the teleop loop (inferred from the timing above).
- **Leader:** B's readback did not track its command either, so B was
  most likely aligning too (inferred; the trip row is not logged, so B's
  own counter jump is not visible).

**After the run** (20 s passive monitor): both nodes at 1000 frames/s, no
µs gaps outside 950–1050, status byte 0. Idle Iq A +0.019 (std 0.026),
B +0.007 (std 0.030), i.e. calibrated. So no further reset, and the
alignment completed (A now at raw 224, near the 142 load plus drift).

**Side note on A's noise:** A's idle Iq std was 0.077 A at raw 3255 in P2
but 0.026 A now at raw 224. If one phase sensor is noisy, its share of Iq
changes with rotor angle (inferred). A3 should log noise against angle,
not as one number.

**Verdict: FAIL.** The baseline was not measured (no free-motion data in
RUN). This is the P1 consequence: teleop's fixed 0.5 s wait does not cover
an alignment after an ESC reset.

**Proposed fix (not applied, needs approval):** gate `teleop_test.py`
behind a `WAIT_FOR_RUN` flag (default on).

- After START, poll DBG until both nodes report RUN, or abort after 3 s.
- Take the position baselines only after both report RUN.
- During the loop, abort if either node's DBG RUN bit clears.

This changes no control law. It only refuses to run when commands would
be dropped.

---

## 2026-10-20: A1 run 1 (rerun), free motion

**What ran:**

1. A 1 s passive check: idle Iq A +0.029 A, B −0.013 A, so both boards
   were still calibrated and aligned.
2. `timeout -s INT 75 taskset -c 2,3 python3 teleop_test.py`, unchanged.
   User on the leader. No `WAIT_FOR_RUN` gate (user's call).

After the run, DBG showed both boards at status 0 and 0 mA, so both had
stopped. `throttled=0x0` before and after.

**Logs:** `logs/teleop_KVs3e-05_FB-1_2.csv`, `logs/teleop_KVs3e-05_FB-1_2.png`
(72.7 s, 34,817 rows after the first 1 s)

**Commands applied this time:** slope of measured Iq on commanded Iq at a
2 ms lag was +0.967 for A (corr 0.891) and +0.993 for B (corr 0.940).

**A1 metric:**

| Metric | Value |
|---|---|
| Free-space RMS of iq_b_cmd | **0.1156 A** (mean −0.0114, std 0.1151) |
| Same, excluding 0.3 s after each stall | 0.1156 A (unchanged) |
| vel_a RMS / max | 6,096 / 56,110 counts/s |
| vel_b RMS / max | 3,696 / 44,120 counts/s |
| tau_h RMS (= iq_a_ff RMS) | 0.1128 A, range −0.50…+0.78 A |
| iq_a_cmd | hit the +1.0 A clamp at least once |
| Position error d_b − d_a | RMS 7,599 counts, max 16,420 (no anchor, drift expected) |

The tap rise time needs run 2 (table).

**A6 timing (from this log):**

| Metric | Value | Threshold | Result |
|---|---|---|---|
| loop_dt p50 / p95 / p99 | 2.000 / 2.040 / 2.230 ms | p95 ≤ 2.1 ms | **PASS** |
| Fresh-frame fraction | A 99.8 %, B 99.7 % | — | — |

**The p95 hides a bad tail.** loop_dt histogram:

| < 1.9 ms | 1.9–2.0 | 2.0–2.1 | 2.1–2.5 | 2.5–3 | 3–5 | > 5 |
|---|---|---|---|---|---|---|
| 638 | 6,652 | 27,252 | 580 | 86 | 78 | 30 |

Three iterations were long stalls:

- **1045 ms at t = 37.42 → 38.47 s.** The leader moved 2,271 counts
  during it, so the follower got no new command for a second. The
  firmware's 200 ms SET_IQ timeout should have zeroed A (inferred).
- **87 ms at t = 66.54 s.** enc_lag_a was 85.6 ms, so the RX thread
  stalled too: the whole process stopped, not just the loop.
- **741 ms at t = 67.6 → 68.35 s.**

There is no kernel message (mmc, hung task, undervoltage, throttle) for
any of them. The cause is unknown. Candidates: an SD-card write stall on
the CSV, or the Python process descheduled (inferred, not tested). These
make the "loop p95 ≤ 2.1 ms" criterion blind to the failure that matters
for teleop. B1 and C2 should also gate on max loop_dt.

**Verdict:** A1 free-space number recorded (0.1156 A). A6 PASS on p95,
but three stalls ≥ 87 ms need explaining before closed-loop work (C3).

---

## 2026-10-20: A1 run 2, table presses, attempt 1 (discarded)

**What ran:** idle-Iq precheck (A −0.018 A, B +0.014 A), then
`timeout -s INT 40 taskset -c 2,3 python3 teleop_test.py`. Both boards were
at status 0 and 0 mA afterwards.

**Log:** `logs/teleop_KVs3e-05_FB-1_3.csv` (37.6 s). **Discarded:** the user
reported doing the presses wrong.

**Timing:**

| Metric | Value |
|---|---|
| loop_dt p50 / p95 / p99 | 2.000 / 2.030 / 2.170 ms |
| loop_dt max | 8.84 ms |
| Fresh-frame fraction | 99.9 % on both |

No long stalls this time.

**Tap detection needs fixing:** the `iq_b_cmd` threshold detector fired 27
times, many on free-motion transients with peaks of 0.000–0.03 A.

- **Fix:** detect contact from the follower instead: blocked (|vel_a| ≈ 0)
  at a repeated d_a while |iq_a| is held.
- In this log the follower parked repeatedly at d_a ≈ −1600…−1950 with
  |iq_a_filt| ≤ 0.18 A. Those were touches, not sustained presses.

---

## 2026-10-20: A1 run 2, table presses, attempt 2 (aborted: both ESCs reset)

**What ran:** idle-Iq precheck (A −0.000 A, B +0.003 A), then
`timeout -s INT 40 taskset -c 2,3 python3 teleop_test.py`.

**Log:** `logs/teleop_KVs3e-05_FB-1_4.csv` (2.81 s, 1,408 rows)

**What happened:**

1. At t = 2.768 s both nodes' TELEM stopped in the same frame. A's last
   fresh row was 2.768 s and B's was 2.766 s.
2. teleop's stale check (50 ms) zeroed both motors.
3. The next `send_set_iq` raised `CanOperationError: Transmit buffer
   full`, so the script exited with a traceback. Its stop sequence also hit
   "Transmit buffer full".

**After the run:**

- The Pi's CAN controller logged error-warning 1 and error-passive 2, with
  tx berr-counter 22. With no node ACKing, the Pi's frames failed until
  the ESCs came back.
- Both ESCs then streamed again at 1 kHz with idle Iq A +28.24 A and
  B +28.61 A, uncalibrated, and encoder raw = 0 on both.
- **So both MCUs rebooted.** DBG status was 0, i.e. IDLE.

**State at the moment of the reset:** not a contact or high-current
event.

| Quantity | Value |
|---|---|
| Max \|iq_a_readback\| over the run | 0.36 A |
| Max \|iq_a_cmd\| over the run | 0.31 A |
| At reset: iq_a_cmd / iq_a_readback | +0.17 A / −0.03 A |
| At reset: vel_a | −5,000 counts/s, free swing (current opposing motion) |
| At reset: iq_b_cmd / iq_b_readback | −0.06 A / −0.02 A |
| At reset: vel_b | ≈ 0 |

**Interpretation (inferred):** two independent MCUs rebooting in the same
millisecond at low current points to something they share, most likely
their supply (VBUS dropout) or a shared ground or reset path.

- This is probably also the unexplained double reset between P2 and A1
  run 1.
- An over-voltage from regen would latch a FAULT, not reboot the MCU
  (inferred).
- The ESC firmware does not report VBUS over CAN, so it cannot be checked
  from these logs.

**Verdict: FAIL.** Blocker for every motor test: the ESCs can reset during
a run. The next START will re-run the 2 s alignment, so run
`zero_iq_check.py` before any teleop.

**Robustness note:** teleop's stop path assumes `send` cannot fail. If the
TX queue filled while the ESCs were alive, STOP might never go out. The
firmware's 200 ms SET_IQ timeout is the only backstop then.

---

## 2026-10-20: A1 run 2, table presses, attempt 3

**What ran:**

1. `zero_iq_check.py` to re-align after the reset. PASS: the alignment
   counter load came 2.015 s after START (A raw 1147 → 142). RUN Iq mean
   A +0.0004 A (std 0.057), B +0.0010 A (std 0.040).
2. `timeout -s INT 40 taskset -c 2,3 python3 teleop_test.py`, unchanged.
   User doing slow table presses through the leader.

After the run, both boards were idle (status 0), calibrated (idle Iq
A −0.032 A, B +0.024 A), and the bus was ERROR-ACTIVE with berr 0/0.
**No ESC reset this time.**

**Logs:** `logs/precheck_zero_iq_realign_after_reset.csv`,
`logs/teleop_KVs3e-05_FB-1_5.csv`, `logs/teleop_KVs3e-05_FB-1_5.png` (37.6 s).
P2's precheck log was preserved as `logs/precheck_zero_iq_P2.csv`.

**High-current segments** (|iq_a_filt| > 0.25 A for ≥ 0.2 s): 9 found, all
with the same signature.

| t (s) | d_a change | med \|vel_a\| | med \|vel_b\| | peak \|iq_a_rb\| (A) | iq_b_cmd / −tau_h | iq_a_ff / iq_a_cmd |
|---|---|---|---|---|---|---|
| 5.64–5.99 | −81 | 959 | 0 | 0.523 | 1.01 | 0.98 |
| 7.80–8.15 | −171 | 500 | 76 | 0.566 | 1.00 | 0.97 |
| 12.29–13.15 | −843 | 1028 | 75 | 0.872 | 1.00 | 0.98 |
| 18.00–19.19 | −47 | 1145 | 0 | 0.847 | 1.00 | 1.00 |
| 22.48–23.13 | −114 | 1325 | 0 | 0.577 | 1.00 | 1.02 |
| 23.72–24.85 | +1630 | 1455 | 7 | 1.210 | 1.01 | 0.96 |
| 26.88–27.42 | +139 | 2344 | 402 | 0.790 | 1.00 | 0.98 |
| 29.22–30.75 | −257 | 963 | 1 | 1.083 | 1.01 | 0.99 |
| 32.30–34.03 | +1790 | 1072 | 197 | 1.201 | 1.03 | 0.95 |

- **The felt force is the feedforward echo, measured:** in every segment
  the leader is held still, the follower's command is 95–102 % feedforward,
  and the felt current equals the hand-torque estimate (ratio 1.00–1.03).
  - With B still, the observer reads B's own felt current as hand torque
    (tau_h = −LPF(i_b) = +iq_a_filt). FF_GAIN = 1 puts that into A, and
    A's current comes back to B.
  - The loop gain is ≈ 1, so the felt force latches and wanders instead of
    reflecting the environment.
  - This holds with correct current readings, so the old −7.94 A artifact
    was not the whole explanation for teleop's "93 % feedforward" note.
- **Contact was not verified:** in five segments the follower's net
  position barely moved (−47…−257 counts) while it buzzed at about
  ±1000 counts/s. That may be chatter against the table (inferred).
  - In the other four it moved 139–1790 counts, so it was not blocked.
  - d_a spans −5,695…+4,030 counts (2.4 rev) over the run, so the table
    was not a fixed stop on every swing.

**A1 metric, as defined (table tap rise):** not measurable from this run.
The user did slow presses, not taps. The felt-force 10–90 % rise during
the 9 presses:

| Statistic | Value |
|---|---|
| Median rise | **847 ms** |
| Range | 436–1578 ms |
| Felt-force peaks | 0.28–0.77 A |

This mixes the user's press speed with the echo build-up. It is not a
transport or filter time constant. The 2.8 Hz IQ_FILTER alone would give
about 125 ms.

The whole-run RMS iq_b_cmd is 0.2639 A. This is not a free-space number:
it includes the presses.

**A6 timing:**

| Metric | Value |
|---|---|
| loop_dt p50 / p95 / p99 | 2.000 / 2.030 / 2.130 ms (PASS) |
| Fresh-frame fraction | 99.9 % on both |

One more stall: **822 ms at t = 31.35 → 32.18 s.** During it the leader
moved +6,572 counts and the follower −2,432. That makes 4 stalls over
about 110 s of teleop in two runs (1045, 87, 741, 822 ms).

- **Next diagnostic:** log to `/dev/shm` to rule out SD-card writes
  (inferred candidate).

**Verdict:** recorded. A1 has no pass threshold.

**Findings carried forward:**

1. The felt path is dominated by the FF echo (motivates C1–C3).
2. Host stalls of up to 1 s.
3. Unexplained simultaneous ESC resets (1 of 3 runs since P2).

---

## 2026-10-20: Leader spin, 5 s (user request)

**What ran:** new `spin_test.py --node B --speed 4000 --time 5 --iq-max 0.15`.
It is a host P velocity loop: KV = 1.5e-5 A/(count/s), resent every tick,
LS velocity over 8 frames on mcu_time_us, 0.5 s ramps, overspeed abort at
3× target. `software/spin_motor2.py` was not used: it sends SET_IQ once
(the ESC_based firmware zeroes current after 200 ms) and runs open loop.

**Log:** `logs/spin_B_+4000_0.15A.csv`

**Result:**

| Metric | Value |
|---|---|
| Speed at target | mean 667 counts/s, std 1263 (target 4000) |
| Distance turned | +0.82 rev in 5 s |
| iq_cmd | mean +0.050 A, never saturated |

- **Stick-slip:** the shaft sits at 0 counts/s with iq_cmd 0.06 A and
  iq_meas 0.04–0.13 A, then slips in bursts of up to ~5,000 counts/s and
  sticks again.
- **Not gravity:** it advanced through all angles, sticking at raw 3693,
  405, 1392, 2123, 2413, 2708 and 3109.

**Interpretation:** the leader's breakaway friction is > 0.06 A, about
4 mN·m at nominal Kt (inferred from the stuck phases). That is two orders
of magnitude above the brief's "Coulomb friction ≈ 0.0004 A", which was
probably measured on the follower or before direct drive. It matches the
2026-09-22 note that Motor 2 barely moved at 0.2 A.

Consequences:

- A P-only loop with this KV cannot spin the leader. It needs integral
  action, or KV high enough to reach the cap while stuck.
- For teleop, 0.06 A of leader stiction is felt as a dead band before any
  reflected force below that level gets through (inferred).
- The leader observer assumes no friction, so stiction appears in tau_h
  as hand torque.

**Verdict:** spin not achieved at 0.15 A with P-only control. Recorded
as a leader friction finding.

### Rerun with integral action

**What ran:** `spin_test.py --node B --speed 4000 --time 5 --iq-max 0.15 --ki 2e-4`.

- New `--kv` / `--ki` flags. The default `--ki 0` keeps the P-only
  behaviour.
- The integrator is clamped to ±iq-max (anti-windup) and integrates over
  the measured loop period.

**Log:** `logs/spin_B_+4000_0.15A_kv1.5e-05_ki0.0002.csv`

**Result:** spun. 5.44 rev total; at target, speed mean 4003 counts/s
(target 4000), std 2507.

| Quantity | Value |
|---|---|
| iq_cmd while moving | mean +0.038 A, std 0.039 A |
| iq_cmd saturation | never hit the 0.15 A cap |
| Breakaway at start | 0.05–0.08 A |

- **Running friction ≈ 0.04 A** (inferred from the mean command at steady
  speed). Speed still swings about 1,600–7,100 counts/s.
- **Harmonic fit at 1, 2, 7, 14, 28, 42 and 84/rev:** none explains more
  than 5 % of speed variance (largest 28/rev, 796 counts/s, 5 %) or of
  iq_cmd variance (largest 28/rev, 0.011 A, 4 %; 14/rev 0.0087 A, 3 %).
- So the speed variation is mostly not angle-locked. It is the PI loop
  limit-cycling against stiction, not ripple (inferred).
- **For A4:** at this speed and current the angle-locked content is small
  next to friction-driven variation. The ripple fit needs either higher
  speed or the acceleration-based method the brief describes, and a
  friction term in the model.

---

## 2026-10-20: A1 run 3, foam presses

**What ran:** idle-Iq precheck (A −0.003 A, B −0.000 A), then
`timeout -s INT 40 taskset -c 2,3 python3 teleop_test.py`, unchanged. User
doing slow foam presses.

After the run, both boards were idle (status 0) and calibrated (A −0.034,
B +0.026 A), and the bus was ERROR-ACTIVE with berr 0/0. **No reset.**

**Logs:** `logs/teleop_KVs3e-05_FB-1_6.csv`, `logs/teleop_KVs3e-05_FB-1_6.png` (37.6 s)

**Timing:**

| Metric | Value |
|---|---|
| loop_dt p50 / p95 / p99 | 2.000 / 2.030 / 2.150 ms (PASS) |
| loop_dt max | 9.17 ms (no stall this run) |
| Fresh-frame fraction | 99.9 % on both |

**High-current segments** (|iq_a_filt| > 0.25 A for ≥ 0.2 s): 21.

| Quantity | Range over the 21 segments |
|---|---|
| iq_b_cmd / −tau_h | 0.99–1.04 |
| iq_a_ff / iq_a_cmd | 1.01–1.25 |
| Felt-force peak | 0.44–0.81 A |
| Median \|vel_b\| | 1,155–5,457 counts/s |
| d_a change per segment | −4,054…+3,666 counts, typically 1,000–4,000 |
| 10–90 % felt rise | median **608 ms**, range 8–1,006 ms |

- The echo signature again: the felt force equals the hand-torque
  estimate, and the follower's command is all feedforward.
- Unlike the table run, the leader was moving during the segments.
- The follower moved 90–360° in most segments (only +38 counts at
  13.62 s). Foam cannot absorb that travel at 0.031 mm/count, so in most
  segments the follower was being driven around by the echoed feedforward,
  not held in the foam (inferred).
- Contact cannot be identified from this log.
- d_a range over the run: −3,920…+3,015 counts.

Whole-run RMS iq_b_cmd is 0.3707 A (mean −0.178). Not a free-space
number.

**Verdict:** recorded. A1 has no pass threshold.

---

## A1 summary

| Run | Log | Duration | Key number |
|---|---|---|---|
| Free motion | `teleop_KVs3e-05_FB-1_2.csv` | 72.7 s | free-space RMS iq_b_cmd **0.1156 A** |
| Table presses | `teleop_KVs3e-05_FB-1_5.csv` | 37.6 s | 9 presses, felt peaks 0.28–0.77 A, rise median 847 ms |
| Foam presses | `teleop_KVs3e-05_FB-1_6.csv` | 37.6 s | 21 segments, felt peaks 0.44–0.81 A, rise median 608 ms |

The table and foam runs are **not distinguishable** by felt-force peak.
In both, felt force = −tau_h at a ratio of 1.00 ± 0.04. The current path
reflects the observer's echo of the leader's own felt current, not the
environment. This is the baseline Phase C has to beat.

**Tap metric:** measured later in run 4 with hand-clamp taps; see "A1
run 4, Re-analysis". Felt rise median 66 ms (reflected term 83 ms), on
truncated edges with a 0.55 A preload. "Table" in runs 2 and 4 means the
user's hand clamping the follower.

**Open issues** (none blocks A2–A6 analysis; both block C3):

1. **Simultaneous ESC resets:** 1 of 5 teleop runs since P2, plus one
   between runs.
2. **Host loop stalls up to 1 s:** 4 in about 220 s of teleop.

---

## 2026-10-20: A1 run 4, table taps (inconclusive)

**What ran:** idle-Iq precheck (A +0.001 A, B +0.007 A), then
`timeout -s INT 20 taskset -c 2,3 python3 teleop_test.py`, unchanged.
After the run, both boards were idle and calibrated (A +0.102 A,
B +0.001 A, status 0). **No reset.**

**Log:** `logs/teleop_KVs3e-05_FB-1_7.csv` (17.7 s)

**Analysis tool:** `analyze_teleop_baseline.py --impacts`. A tap is the
follower stopping from ≥ 3,000 counts/s to ≤ 20 % of that within 20 ms
while the leader keeps moving the same way at ≥ 1,500 counts/s. The rise
is 10–90 % of iq_b_cmd from impact to peak.

**Result: not specific.**

| Log | Contents | "Impacts" found | Rise median (range) |
|---|---|---|---|
| Free motion (negative control) | nothing in the path | 73 | 96 ms (2–373) |
| Table presses | presses | 24 | — |
| Table taps | taps | 23 | 10–383 ms |

- The follower turned −21,840…+1,944 counts (about 6 rev) during the tap
  run. It was not stopped at a fixed angle each time.
- **Clustering by raw angle:** the only tight cluster is raw 1688–1721
  (5 stops). Its stops come from both directions (v0 −29,269, −20,384,
  −25,192, −4,211 and +24,982 counts/s), which a one-sided table cannot
  produce.
- The free-motion control also has a cluster at raw 1633–1682 (5 stops).

**If** the raw ≈ 1700 cluster is the table: felt-force 10–90 % rise is
215–381 ms (median about 297 ms), peaks 0.22–1.02 A (inferred,
unverified).

**Verdict (angle-clustering method):** inconclusive.

### Re-analysis: "table" = the user's hand clamping the follower

The user confirmed there is no table. Contact in runs 2–4 is the user
clamping the follower by hand, so it can happen at any angle and from
either side. That explains the clustering result above.

**Clamp detector:** the follower is still (d_a range ≤ 30 counts over
100 ms) while the leader moves ≥ 100 counts in the same 100 ms. Only
**push-in** events are kept: the felt force grows against the leader's
motion by ≥ 0.1 A.

| Log | Push-in clamps | Held time | Total rise median (range) | Felt change median |
|---|---|---|---|---|
| Free motion (control) | 7, all at t = 24–27 s | 2.4 s of 73 s | 175 ms (113–469) | 0.468 A |
| Hand presses (run 2) | 1 | 0.2 s of 38 s | 171 ms | 0.598 A |
| Foam presses (run 3) | 35 | 7.3 s of 38 s | 96 ms (39–366) | 0.210 A |
| **Hand taps (run 4)** | **24** | 6.9 s of 18 s | **66 ms (16–266)** | 0.339 A |

- **Control:** the free-motion events all fall at t = 24–27 s. Possibly
  the user touched the follower then (inferred); the detector is not
  proven specific.
- **Run 2:** the follower was rarely held still (it slipped or kept
  moving), so run 2's 9 high-current segments were mostly not clamps.

**Split of the felt force:** iq_b_cmd = −iq_a_filt − KB_DAMP·vel_b.

| Log | Reflected (−iq_a_filt) rise / change | Damping (−KB_DAMP·vel_b) rise / change | Reflected share of felt change |
|---|---|---|---|
| Hand taps | 83 ms (26–212) / 0.414 A | 68 ms (8–237) / 0.219 A | 78 % |
| Foam presses | 97 ms (29–405) / 0.129 A | 78 ms (28–227) / 0.087 A | 72 % |
| Free control | 151 ms (102–462) / 0.338 A | 90 ms (30–264) / 0.092 A | 91 % |

- The damping term reacts to leader speed with no filter. The reflected
  term goes through the 2.8 Hz IQ_FILTER, whose 10–90 % step rise is
  about 125 ms.
- The reflected medians are below that floor, so the clamps ended before
  the felt force plateaued. These rises are on truncated edges (inferred).
- In taps the felt force at clamp onset was already **0.55 A** (median),
  preloaded by the echoed feedforward. Several taps hit the 1.0 A clamp
  (−0.97…−1.00 A), which also shortens the measured rise.

**A1 tap metric (recorded, with caveats):** felt-force 10–90 % rise on
hand-clamp taps, median **66 ms** (iq_b_cmd), reflected term **83 ms**.
Against the brief's ≤ 30 ms target for C3/D3, both the 2.8 Hz filter and
the felt-force preload matter.

**Analysis tooling:**

- `analyze_teleop_baseline.py --impacts`: impact detector. Not specific
  (73 false events on free motion). Kept, but not used for the number.
- The clamp detector above was run ad hoc. It should move into the
  analyser before Phase D.

---

## 2026-10-20: A2, plant gain re-identification

**What ran:** `plant_gain_test.py`, ported to ESC_based (see its
docstring). Five consecutive runs, each doing motor A then motor B.
Bang-bang current at 0.06 / 0.10 / 0.15 A, reversing at ±10,000 counts/s,
1.5 s per level. Links fitted, hands off. `throttled=0x0` before and after.

**Changes from `software/plant_gain_test.py`:**

- SET_IQ resent every tick (the original would hit the firmware's 200 ms
  timeout on a stuck shaft).
- Every TELEM frame, timed by mcu_time_us.
- An extra gravity-aware fit.
- A reset precheck.

The fit was checked on synthetic data first: G 5.00e6 true → 5.04e6
recovered (+0.8 %).

**Logs:** `logs/plant_gain.csv`, `logs/plant_gain_2.csv` … `logs/plant_gain_5.csv`

**Result:** no watchdog trips, speed stayed in band, 104–142 reversals per
motor per run.

| Run | G_A (counts/s² per A) | G_B | G_A / G_B |
|---|---|---|---|
| 1 | 9,021,558 | 10,081,817 | 0.89 |
| 2 | 9,860,237 | 9,491,551 | 1.04 |
| 3 | 8,392,060 | 9,730,029 | 0.86 |
| 4 | 8,783,494 | 10,016,176 | 0.88 |
| 5 | 9,069,391 | 9,039,212 | 1.00 |
| **Mean** | **9,025,000** | **9,672,000** | **0.93** |
| Run-to-run std | 538,000 (6.0 %) | 425,000 (4.4 %) | |
| **Standard error** | **241,000 (2.7 %)** | **190,000 (2.0 %)** | |
| Old value (2026-08-31, pre-direct-drive) | 7,622,000 | 7,915,000 | 0.96 |
| Change | **+18.4 %** | **+22.2 %** | |

**Verdict: PASS** (standard error ≤ 5 % on both). G changed by more than
5 %, so new values are proposed below. **Not applied:** they need user
approval.

**Caveats:**

- **Not linear at low current.** accel/A at the 0.06 A level is only
  10–92 % of the 0.10/0.15 A levels: A 0.86–4.9e6, B 3.9–9.5e6. Run 4 A
  has only 2 segments at 0.06 A.
  - That is breakaway friction the Coulomb term does not capture,
    consistent with the leader's ~0.06 A stiction from the spin test.
  - The 0.10 and 0.15 A levels agree within about 20 %.
  - G is therefore a "moving shaft" gain. Below about 0.06 A the plant is
    friction-dominated.
- **Fit residual 8.8–12.2 % of range** (the old version had about 12 %).
- **Gravity-aware fit discarded: ill-conditioned.** It gave "offsets" of
  −0.76…+0.54 A and 1/rev amplitudes of 0.06–0.79 A. Those are physically
  impossible: a 0.5 A gravity load would be obvious at rest.
  - Cause: each run spans only 49–112° of angle, so the constant, cos and
    sin columns are nearly collinear.
  - Gravity has to be fitted from a run covering a full revolution (A4).
- The viscous/Coulomb split is not identified, as the original warns:
  b = 7–65 1/s, c = −0.0034…+0.0079 A.

**Proposed constants for `teleop_test.py`:**

| Constant | Current | Proposed |
|---|---|---|
| `G_A_CNT_S2_PER_A` (unused by teleop) | 7,622,000 | 9,025,000 |
| `G_B_CNT_S2_PER_A` | 7,915,000 | 9,672,000 |

- Effect: the leader observer's inertia term (x = OBS_COMP·v_b/G_B)
  shrinks by 18 %. The KV_SLOPE stability limit estimate drops to about
  3.5–4.2e-5 at T = 4.5 ms, close to the current KV_SLOPE of 3e-5.

---

## 2026-10-20: A3, noise floor (follower, HOLD, zero gains)

**Setup:** `coupling_monitor.py --mode hold --motors a --kp 0 --kd 0
--kd-local 0 --iq-max 0.8 --duration <s> --no-plot`. The firmware applies
0 A every tick, with no host traffic after engage and no host timeout.
Only the follower is energized.

**Analysis:** `analyze_noise.py <log> --drift`, which uses only rows
where the mode was engaged.

The decision to leave teleop's G constants unchanged (user, after A2)
stands.

### Run 1: 30 s, shaft clamped by hand

**Log:** `logs/A3_clamped.csv` (30,048 samples, 0 timestamp gaps)

| Measure | Value |
|---|---|
| Applied command | max \|iq_cmd\| = 0.0000 A |
| Shaft travel | 8 counts |
| **Iq mean / std** | **−0.0005 A / 0.0434 A** (p1…p99: −0.125…+0.101 A) |
| Distinct values | 197 |
| Finest step | 1.80 mA = 1 s16A unit (1/544.6 A) |
| Most common values | +0.0220, −0.0037, −0.0073, +0.0202, −0.0349, 0.0000, +0.0257, +0.0514 A |
| Spectrum floor (Welch, 0.98 Hz bins) | 2.32 mA/bin |
| Peaks | 246–248 Hz up to 6.13 mA; 1.0 Hz 3.98 mA; others ≤ 2.8 mA |
| 10 s means range | 0.93 mA over 30 s (too short to judge drift) |

- **On a lattice, not spread out.** The values cluster on combinations of
  the two phase readings at this angle, on the 1.8 mA s16 grid. The
  generator steps are roughly 25.7 mA and 3.6 mA (inferred from the
  common values, not fitted).
- **The 247 Hz peak is probably aliased**, from a component above 500 Hz
  folded back by the 1 kHz raw sampling (inferred). It is a candidate for
  the B4 boxcar mean to remove.
- This clamped std (0.043 A) is lower than A's 0.076 A in P2 (raw 3255)
  and higher than 0.026 A after A1 run 1 (raw 224). Noise depends on
  angle, which run 2 was meant to map.

### Runs 2 and 3: 30 s free, then 10 min at rest (hands off)

**Logs:** `logs/A3_free.csv` (30,030 samples), `logs/A3_rest_10min.csv`
(599,998 samples). No timestamp gaps in either. max |iq_cmd| = 0.0000 A.
`throttled=0x0`.

| | Clamped | Free 30 s | **Rest 10 min** |
|---|---|---|---|
| Iq mean (A) | −0.0005 | +0.0003 | +0.0001 |
| Iq std (A) | 0.0434 | 0.0304 | **0.0340** |
| p1…p99 (A) | −0.125…+0.101 | −0.070…+0.068 | −0.077…+0.079 |
| Distinct values | 197 | 120 | 186 |
| Shaft travel (counts) | 8 | 14 | 40 |
| Spectrum floor (mA/bin) | 2.32 | 1.46 | 1.59 |
| 247 Hz peak (mA) | 6.13 | not in top 8 | 2.58 |
| 10 s means range (mA) | 0.93 | 0.89 | **3.10** (60 windows) |

**A3 drift: PASS.** Over 10 min the range of the 10 s means is 3.10 mA
(< 5 mA). The linear trend is +0.09 mA per 10 min.

**Notes:**

- **Low-frequency wander:** the spectrum's lowest bins fall off
  monotonically: 1.0 Hz 15.1 mA, 2.0 Hz 7.2, 2.9 Hz 4.5, 3.9 Hz 3.5,
  4.9 Hz 2.8 (rest run). That is red, 1/f-like wander plus window leakage
  from below 1 Hz, not a 1 Hz tone (inferred). It matches the 3.1 mA range
  of the 10 s means.
- **Clamped vs free:** clamping raised the std from 0.030 to 0.043 A.
  Holding the shaft by hand moved it a few counts, and the current loop
  reacts to that motion and back-EMF even at a 0 A reference (inferred).
- **Angle dependence not mapped:** the link stayed put (14–40 counts), so
  all three runs sample essentially one angle (raw ≈ 3950–4000). The
  angle-dependent noise seen in P2 (0.077 A) vs A1 (0.026 A) remains open.
  A4's constant-speed runs cover all angles.
- **Scale for later phases:** the raw single-sample floor is 0.034 A std
  at rest. C1's free-space target is RMS < 0.015 A, so the estimate needs
  about 5× noise reduction from filtering and B4's boxcar mean.
  - At 20 Hz bandwidth from 1 kHz white samples, the gain would be about
    √(1000/(2·20)) = 5× (inferred, ignoring the red part).

**Verdict: PASS** (drift 3.10 mA < 5 mA).
2. A2: plant gain re-identification (`plant_gain_test.py` is in
   `software/`; port to ESC_based first).
3. Open: ESC reset cause; host stall cause.
2. A1 run 3: slow foam presses.
3. Open: why both ESCs reset between P2 and A1 run 1; what causes the
   host stalls.

---

## 2026-10-20: A4, ripple characterisation, unloaded, low speed (partial)

**What ran:** `spin_test.py --node A --speed ±1000 --time 20 --iq-max 0.3
--ki 2e-4 --coast 3`, planned as a 6-speed sequence (±1k, ±4k, ±10k).
New in `spin_test.py`:

- a `--coast` phase (0 A straight after the hold);
- a `phase` column;
- unique log names;
- an overspeed floor `OVERSPEED_MIN_CNT_S = 10,000`.

**Analysis:** `analyze_ripple.py` (new), angle-domain:

1. Bin crossing times on mcu_time_us give v(θ) over 500 bins/rev.
2. a = v·dv/dpos by FFT.
3. r_eq(θ) = i_cmd − a/G with G_A = 9.025e6.
4. Harmonics at 1, 7, 14, 28, 42 and 84/rev.

**Attempt 1:** +1000 aborted after 0.06 rev. A stick-slip breakaway burst
(1,048 → 3,821 counts/s in 7 ms at 0.08 A) tripped the old 3×-target
overspeed limit. Log: `logs/spin_A_+1000_0.3A_kv1.5e-05_ki0.0002_coast3.csv`.
The overspeed floor was added after this.

**Attempt 2:**

- **+1000:** done. 5.00 rev, mean 1,000 counts/s, **std 1,427**, i_cmd
  mean +0.044 A. Log: `logs/spin_A_+1000_0.3A_kv1.5e-05_ki0.0002_coast3_2.csv`.
- **−1000:** aborted at 17.7 s on **overspeed −10,263 counts/s** after
  4.41 rev. It stuck at raw ≈ 336, the integrator wound to −0.116 A, and
  the release burst exceeded 10,000 counts/s. The rest of the sequence
  was skipped. Log: `logs/spin_A_-1000_0.3A_kv1.5e-05_ki0.0002_coast3.csv`.

| Harmonic r_eq (mA) | 1 | 7 | 12 | 14 | 28 | 42 | 84 |
|---|---|---|---|---|---|---|---|
| +1000 | 2.56 | 27.06 | 17.00 | 9.51 | 13.37 | 3.92 | 6.53 |
| −1000 | 3.54 | 19.23 | 24.89 | 8.05 | 19.87 | 7.25 | 5.81 |

| Check | +1000 | −1000 |
|---|---|---|
| Model harmonics explain | 45 % (FAIL) | 31 % (FAIL) |
| 42/14 ratio | 0.41 | 0.90 |
| Sensing view (iq_meas − iq_cmd): model harmonics explain | 1 % | 4 % |
| Raw iq_meas − iq_cmd std per 1/16 rev | 0.047–0.080 A (×1.7) | 0.042–0.084 A (×2.0) |

**Interpretation:**

- **Invalid regime:** 1,000 counts/s is stick-slip on the follower too.
  The angle-averaged speed (2,963 counts/s) is far off the real mean
  (1,000), showing the smooth-motion assumption is broken. So the FAIL
  and the 42/14 ratio are **not trustworthy here**. Do not drop the
  median-filter idea on this data.
- **Gravity is negligible:** the 1/rev term is 2.6–3.5 mA. The −1000
  burst was a stiction release, not the link falling.
- **Strongest components:** 7/rev (one electrical period; the classic
  signature of a phase-current offset) and 12/rev (inferred: slot-related).
  28/rev is also strong. 14/rev, the brief's "gain mismatch", is only
  8–10 mA here.
- **The sensing view sees almost none of it,** as the brief expects: the
  current loop holds iq_meas at i_cmd.
- **Noise vs angle while moving:** 2× variation across the revolution.

**Verdict:** A4 incomplete. The low-speed runs are stick-slip dominated.
Next: ±4,000 and ±10,000 counts/s (smooth motion), then loaded runs
(shaft against a plate for steady friction, per the user).

---

## 2026-10-20: A4, ripple characterisation, unloaded, ±4k and ±10k counts/s

**What ran:** `spin_test.py --node A --speed {±4000 for 10 s, ±10000 for 8 s}
--iq-max 0.3 --ki 2e-4 --coast 3`. All 4 completed.

| Target (counts/s) | Mean (counts/s) | Std | Rev | i_cmd mean (A) |
|---|---|---|---|---|
| +4,000 | 3,999 | 2,372 | +10.19 | +0.038 |
| −4,000 | −3,997 | 2,179 | −10.16 | −0.042 |
| +10,000 | 9,987 | 2,430 | +20.66 | +0.032 |
| −10,000 | −9,997 | 1,805 | −20.69 | −0.035 |

**Logs:** `logs/spin_A_{+4000,-4000,+10000,-10000}_0.3A_kv1.5e-05_ki0.0002_coast3.csv`

**Analysis:**

- **Bug fixed before these numbers:** `analyze_ripple.py` multiplied the
  acceleration by the travel direction, but a = |v|·d|v|/dx is already in
  the +counts frame. The negative-direction r_eq values in the **low-speed
  entry above are affected**; the positive runs are not.
- **New options:** `--harmonics` (an alternative model set) and the
  explained ratio over harmonics ≤ 100/rev. At 10,000 counts/s the 1 kHz
  telemetry resolves about ≤ 200/rev, and the unexplained 143, 186 and
  247/rev there are likely sampling artifacts (inferred).

**Explained share of r_eq harmonic variance:**

| Model | +4k | −4k | +10k | −10k |
|---|---|---|---|---|
| Brief set (1, 7, 14, 28, 42, 84), all harmonics | 43 % | 22 % | 31 % | 20 % |
| Same, ≤ 100/rev | 61 % | 37 % | 61 % | 47 % |
| + 12/24/36 family, all harmonics | 64 % | 53 % | 42 % | 35 % |
| **+ 12/24/36 family, ≤ 100/rev** | **90 %** | **89 %** | **85 %** | **83 %** |
| 12/24/36 family without 7/rev, ≤ 100/rev | 46 % | 78 % | 37 % | 56 % |

**Phase of r_eq by run** (+4k, −4k, +10k, −10k), and amplitude:

| n/rev | Phases (deg) | Amp (mA) | Angle-locked? |
|---|---|---|---|
| 1 | −134, 4, −152, 8 | 2.0–4.9 | flips with direction → once-per-rev friction, not gravity |
| 7 | −69, −86, −166, 92 | 11.6–30.0 | **no**: phase wanders |
| 12 | −172, −178, 164, −148 | 18.3–22.3 | yes |
| 14 | 95, 98, 79, 113 | 5.7–9.9 | yes |
| 24 | −106, −112, −103, −102 | 6.5–8.4 | yes |
| 28 | 32, 42, 43, 21 | 9.5–13.2 | yes |
| 36 | −166, −160, −172, 162 | 6.8–10.2 | yes |
| 42 | 111, 95, 114, 87 | 3.0–5.2 | yes |
| 84 | −58, −109, −36, −112 | 6.8–9.6 | roughly |

**Other results:**

- **Sector-spike test:** the 42/14 ratio is 0.31–0.91, i.e. ≥ 0.1, so the
  median-filter idea is **not** dropped. In absolute terms 42/rev is small
  (3–5 mA).
- **Sensing view** (iq_meas − iq_cmd): the model harmonics explain only
  5–20 %. The ripple shows up in acceleration, not in measured Iq, as the
  brief anticipated.
- **Noise vs angle:** raw iq_meas − iq_cmd std is 0.052–0.079 A across
  the revolution (×1.3–1.5).
- **Coasting at Iq_ref = 0 is unusable.** The follower stops within
  0.0–0.1 rev (tens of ms) even from 10,000 counts/s, because running
  friction is about 0.04 A (about 3.6e5 counts/s² of deceleration). The
  fits are degenerate. The brief assumed Coulomb friction of 0.0004 A.
- **Brief's "known ripple" 14/rev, 0.009–0.016 A:** measured 5.7–9.9 mA,
  phase-locked. The 12/rev family (12/24/36, 18–22 / 7–8 / 7–10 mA) is
  larger and was not in the brief. It may be slot-related; the cause is
  inferred, not known.

**Verdict: FAIL** on the brief's model (20–43 %). Stopped here per the
FAIL rule.

**Proposed fix:**

1. Extend the ripple model to {1, 12, 14, 24, 28, 36, 42, 84} as
   angle-locked terms. With 7/rev these explain 83–90 % of the resolved
   harmonic variance.
2. Treat 7/rev separately: it is large (12–30 mA) but not angle-locked,
   so a fixed table cannot compensate it.
   - **Test (host only):** repeat ±4,000 at two other KV/KI settings. If
     7/rev moves with the loop gains, it is speed-loop interaction, not a
     plant property.
   - **Firmware angle (dev machine, not here):** a phase-current offset
     gives a 1× electrical torque ripple.
3. Loaded runs (shaft against a plate) to spread i_mean and identify
   g(θ). The current fit's condition number is 27 with i_mean within
   ±0.04 A.

---

## 2026-10-20: A4, 7/rev loop-gain check

**What ran:** `spin_test.py --node A --speed ±4000 --time 10 --iq-max 0.3`
at low gains (KV 7.5e-6, KI 1e-4), then planned high gains (KV 2.5e-5,
KI 4e-4).

- **+4000 low:** completed, 10.44 rev, mean 4,002 counts/s, std 2,834.
  Log: `logs/spin_A_+4000_0.3A_kv7.5e-06_ki0.0001.csv`.
- **−4000 low:** aborted at 0.41 rev on overspeed −12,071 counts/s
  (limit 12,000 = 3× target). A stick-slip burst; the weaker loop lets it
  run further. Log: `logs/spin_A_-4000_0.3A_kv7.5e-06_ki0.0001.csv`.
- **High-gain runs:** skipped by the sequence's stop-on-fail.

**Same speed (+4000), two gain settings:**

| n/rev | Low gains | Baseline gains |
|---|---|---|
| 7 | 12.9 mA @ 15° | 30.0 mA @ −69° |
| 12 | 20.0 @ −170° | 21.1 @ −172° |
| 14 | 10.6 @ 113° | 9.9 @ 95° |
| 28 | 12.0 @ 33° | 13.1 @ 32° |

7/rev changes with the loop gains; the angle-locked harmonics do not.

**Speed-ripple regressor:** each run's r_eq(θ) (≤ 100/rev) was fitted on
the locked set {1, 12, 14, 24, 28, 36, 42, 84} plus the speed ripple v(θ).

| Run | locked only | locked + v | b / KV, from i_cmd | b / KV, from iq_meas |
|---|---|---|---|---|
| +4k low | 73 % | 84 % | −0.83 | −0.75 |
| +4k base | 46 % | 90 % | −0.87 | −0.67 |
| −4k base | 78 % | 90 % | −0.65 | −0.63 |
| +10k base | 37 % | 83 % | −0.79 | −0.54 |
| −10k base | 56 % | 81 % | −0.75 | −0.57 |

- A single v(θ) term does as well as the 7/rev pair (83–90 %). Its
  coefficient scales with KV at about −0.8 × KV.
- So the non-locked "7/rev" is mostly the **loop's own P-term current
  ripple appearing in r_eq**, not a fixed motor ripple.
- **It persists with measured Iq** (−0.54…−0.75 × KV), so it is not only
  the timing of the logged command.
- In other words, current ripple at the loop's frequencies (7–17 Hz here)
  produces only about 20–50 % of the acceleration that G (from A2's
  bang-bang) predicts.
- **Two candidates (both inferred):**
  1. The plant gain falls with frequency, e.g. compliance between rotor
     and encoder or link.
  2. An artifact of averaging non-angle-locked speed ripple over
     revolutions.
- A small current chirp on the follower (e.g. 1–50 Hz, ±0.05 A, at
  constant speed) would separate them.

**Conclusions for later phases:**

- **B6** (ripple compensation, estimate side): use only the angle-locked
  set {1, 12, 14, 24, 28, 36, 42, 84}. Per-harmonic amplitudes are
  2–22 mA. Do not tabulate 7/rev.
- **C1** (follower observer, i − a/G): if the plant gain really drops at
  7–17 Hz, the observer will read loop-driven motion as environment
  torque. Check G vs frequency before C1.

**A4 verdict:** still **FAIL** on the brief's model (20–43 %). With the
extended locked set plus a speed term: 81–90 % of resolved harmonic
variance.

---

## 2026-10-20: A4, loaded runs (shaft against a plate)

**What ran:** `spin_test.py --node A --speed ±4000 --time 24 --iq-max 0.45
--ki 2e-4`. The user pressed a plate against the follower shaft, stepping
the pressure up during the hold.

- **+4000:** aborted at 12.2 s of hold on overspeed +12,107 counts/s. The
  heavy stage nearly stalled the shaft (12–13 s: speed about 1,900,
  i_cmd 0.33–0.42 A), then it broke free. Log:
  `logs/spin_A_+4000_0.45A_kv1.5e-05_ki0.0002.csv`.
  - Usable: 0–7 s at i_cmd +0.16 A (7 rev), and 7–12 s rising from +0.20
    to +0.29 A (mean +0.24 A, 5 rev).
- **−4000:** aborted at 5.0 s on overspeed −12,929 counts/s. The pressure
  rose to −0.36 A by 4–6 s. Log:
  `logs/spin_A_-4000_0.45A_kv1.5e-05_ki0.0002.csv`.
  - Usable: 0–4 s, mean −0.145 A, 3.7 rev.
- No saturation at 0.45 A except about 1 % in the −4000 run.
  `throttled=0x0`.

**Fit:** per piece, plain FFT harmonics of r_eq = i_cmd − a/G (no speed
regressor). Pieces: unloaded ±4k (+0.038 / −0.042 A), loaded +0.162,
+0.240 and −0.145 A, so i spans −0.145…+0.240 A. Then complex least
squares r_n = c_n + i·g_n across the 5 pieces.

- The speed regressor was dropped for this fit because it is collinear
  with the locked harmonics' own speed response. With it, the 12/rev
  amplitude on the unloaded run fell from 21 to 7 mA, so the split was
  not unique.

| n/rev | c (mA @ deg) | g (mA/A) | i·g at 0.3 A (mA) | R² of c + i·g | Behaviour |
|---|---|---|---|---|---|
| 12 | 21.6 @ −172 | 14.2 | 4.3 | 0.55 | mostly current-independent |
| 24 | 8.5 @ −101 | 4.8 | 1.4 | 0.17 | current-independent |
| 28 | 13.4 @ 41 | 1.1 | 0.3 | 0.02 | current-independent |
| 36 | 9.2 @ −155 | 4.7 | 1.4 | 0.22 | current-independent |
| 14 | 8.3 @ 96 | 24.1 | 7.2 | **0.87** | grows with current |
| 42 | 2.6 @ 112 | 20.7 | 6.2 | **0.88** | grows with current |
| 84 | 6.0 @ −75 | 34.2 | 10.3 | **0.88** | grows with current |
| 1 | — | — | — | 0.26 | excluded |

**Interpretation:**

- The **12/24/28/36 family is current-independent**, i.e. cogging or
  mechanical. It does not change with load and was not in the brief.
- **14, 42 and 84/rev scale with current.** They are 2×, 6× and 12× the
  electrical frequency (7 pole pairs). That supports the brief's inferred
  "14/rev = phase-sensor gain mismatch" (∝ i), with 42 and 84 as the
  usual torque-ripple orders (inferred).
- **1/rev is excluded:** the short loaded −0.145 A piece has 21 mA of
  1/rev, while the others have 2–5 mA. Most likely the plate rubbing an
  eccentric shaft (inferred).
- 7/rev, for reference, is 30.0 / 24.6 / 28.4 / 11.6 / 7.9 mA over the
  same pieces. It is still not angle-locked; see the gain check above.
- **Limits:** five pieces, only one loaded piece on the negative side
  (3.7 rev), and |i| ≤ 0.24 A. The brief's ±0.3 A was not reached cleanly
  because heavier pressure stalls the shaft and ends in a breakaway burst.

**A4 overall:**

- Brief's model set: **FAIL** (20–43 %).
- **Proposed model for B6:** ripple(θ, i) = c(θ) + i·g(θ), with c on
  {12, 14, 24, 28, 36, 42, 84}/rev and g on {14, 42, 84}/rev.
  - These locked terms (with 1/rev) explain **37–78 %** of the resolved
    (≤ 100/rev) harmonic variance in the unloaded runs.
  - Adding the non-locked 7/rev, or the speed-ripple term, raises that to
    81–90 %. That part is loop-driven and cannot go into a static table.
  - The loaded fits follow the table above.
- **Open:** 7/rev and G vs frequency (chirp test), before C1.

---

## 2026-10-20: A5, command-path test (follower free)

**What ran** (20 s each, follower A, shaft free, hands off):

| Run | Command | Result |
|---|---|---|
| (a) | `coupling_monitor.py --mode hold --motors a --kp 0 --kd 0 --kd-local 0 --duration 20` | engaged 20.1 s, max \|iq_cmd\| 0.000 A, max \|err\| 118 counts |
| (b) | `a5_command_test.py --mode const`: SET_IQ 0 A at 500 Hz | 9,997 sends, period p50 2.00 / p95 2.05 ms |
| (c) | `a5_command_test.py --mode stair`: 0, +0.05, 0, −0.05 A, one step per 2 ms (mean 0) | 9,974 sends |

`throttled=0x0`.

**Logs:** `logs/A5_hold.csv`, `logs/A5_const.csv` (+ `_cmd.csv`),
`logs/A5_stair.csv` (+ `_cmd.csv`).

**Analysis:** `a5_command_test.py --analyze …`, with Welch PSD and band RMS.

**Whole runs:**

| Band (Hz) | (a) hold | (b) const | (c) stair | (b) − (a) |
|---|---|---|---|---|
| 1–10 | 16.48 | 14.73 | 12.70 | −0.97 dB |
| 10–50 | 20.31 | 11.37 | 8.81 | −5.04 dB |
| 50–150 | 29.01 | 17.12 | 34.29 (the 125 Hz staircase itself) | −4.58 dB |
| 150–300 | 37.00 | 21.14 | 16.48 | −4.86 dB |
| 300–450 | 34.92 | 21.18 | 20.85 | −4.34 dB |
| 450–500 | 20.01 | 12.55 | 9.60 | −4.05 dB |
| std | 69.2 mA | 44.1 mA | 48.8 mA | |

Band values are mA RMS. Worst |Δ| = 5.04 dB, **FAIL** as the brief
defines it. But (b) is *quieter* than (a), so host traffic did not add
noise.

**Why the whole-run comparison fails:** the hold run was disturbed. Its
shaft moved 176 counts; from 2 to 10 s the angle swung from 3901 to 3987
and back, with noise of 79–90 mA per 2 s chunk. That was external, at
0 A, with nothing commanded: a touch or bench vibration (inferred). In
its still last 8 s (raw 3891–3896) the std was 48–51 mA.

**Still segments, equal length (8 s):** (a) 12–20 s vs (b) 12–20 s.

| Band (Hz) | 1–10 | 10–50 | 50–150 | 150–300 | 300–450 | 450–500 |
|---|---|---|---|---|---|---|
| (b) − (a) | −0.26 | −1.07 | −0.86 | −0.66 | −0.95 | −1.70 dB |

Worst |Δ| = **1.70 dB, PASS** (≤ 3 dB).

- **Caveat:** the shaft angles differ (3894 vs 3702), and noise depends
  on angle.
- There is no 500 Hz artifact from host traffic: (b)'s top bin is 5.2 dB
  below (a)'s.

**Step response (c),** host clock, averaged over 9,973 steps of 0.05 A:

| ms after send | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| Response (mA) | +0.1 | +7.4 | +44.8 | +48.7 | +48.7 | +41.4 (next step) |

- **50 % crossing at 1.47 ms** after the host send. That includes CAN TX,
  the ESC's next 1 kHz tick, the current loop, TELEM and RX.
- This is the baseline for B4's "step delay ≤ 0.6 ms slower".
- It settles to 97 % of the step within 3 ms.

**Verdict:** **PASS on still segments** (1.7 dB). The whole-run FAIL is
explained by an external disturbance during (a). Rerun (a) if a clean
whole-run number is needed.

---

## 2026-10-20: B1, consume every frame

**Code changes** (default behaviour unchanged):

- **`can_interface.py`:** `MotorCANInterface(..., pair_dbg=True)`. With
  `pair_dbg=False`, each TELEM record is queued on arrival instead of
  being held for its DBG frame (in direct mode DBG is 10 Hz, so pairing
  added about 1 ms).
  - Unit-tested both modes offline with synthetic frames: paired mode
    releases T1 only after T2 or a matching DBG; unpaired releases it
    immediately; `get_debug()` updates in both.
- **`teleop_test.py`:** `USE_FRAME_QUEUE` (default False; override with
  `TELEOP_FRAME_QUEUE=1`).
  - When on, each iteration drains every frame from both motors into
    `<log>_frames.csv` (loop_iter, motor, bus_time, mcu_time_us, enc,
    iq_meas), plus the pre-loop frames (loop_iter −1) and the final ones
    (−2).
  - The control law still uses `get_telemetry()` (the newest frame), so
    behaviour is unchanged.
  - The latest-value path remains for rollback.
- **`analyze_teleop_baseline.py --frames`:** gap and missing-frame checks
  over loop frames only (loop_iter ≥ 0), so the known 2-frame drop at
  START is excluded. On synthetic data it caught one inserted drop.

**Live test:** `TELEOP_FRAME_QUEUE=1 timeout -s INT 75 taskset -c 2,3
python3 teleop_test.py`, free motion, user on the leader. `bus_check.py`
was OK before and after (no reset). `throttled=0x0`.

**Logs:** `logs/teleop_KVs3e-05_FB-1_8.csv`, `logs/teleop_KVs3e-05_FB-1_8_frames.csv`,
`.png` (72.5 s)

| Criterion | Result | Threshold | Verdict |
|---|---|---|---|
| mcu_time_us gaps, A / B | 991–1009 µs; 0 outside (72,496 / 72,486 frames) | 1000 ± 50 µs | **PASS** |
| Missing frames, A / B | 0 / 0 over 72.5 s | 0 in 60 s | **PASS** |
| loop_dt p95 | 2.090 ms (p50 2.000, p99 2.250) | ≤ 2.1 ms | **PASS** (margin 0.01 ms) |
| loop_dt max | **2,082 ms** | (not in brief) | see below |

- Frame logging costs about +0.05 ms at p95 (2.04 → 2.09 ms against A1).
- Free-space RMS iq_b_cmd in this run: 0.1231 A (A1 0.1156; normal
  run-to-run variation).
- Other long iterations: 31, 39 and 44 ms.

**The 2 s stall (t = 26.74 → 28.82 s) and its consequence:**

- **The whole process stopped.** enc_lag_a was 47.6 ms on the first
  iteration after it. The kernel kept buffering, and one drain then
  returned 3,939 frames, so **no frame was lost**.
- **The follower turned −7,312 counts (1.8 rev) during the stall.**
  - The last command was iq_a_cmd −0.07 A, with the follower moving
    about +1,800 counts/s.
  - From the buffered frames: 4,677 → 677 in the first 200 ms (1 rev in
    200 ms), → −2,631 by 400 ms, then stopped.
  - Interpretation (inferred): the last SET_IQ persisted until the
    firmware's 200 ms host timeout, then friction stopped the shaft.
- teleop's stale check fired once after the stall and zeroed both
  motors.

**Verdict:** **B1 PASS** on the brief's criteria (gaps, 0 missing,
p95 ≤ 2.1 ms).

**Blocker for closed-loop work (C3 and beyond):** host stalls of up to
2 s. During one, the last command stays applied for up to 200 ms. Even
−0.07 A spun the free follower 1.8 rev; a 1 A command would be far worse.

**Proposed fixes:**

1. **RT priority for the RX thread and the control loop** (system change,
   needs user approval). SCHED_FIFO currently fails silently
   (`ulimit -r` = 0). Then re-measure max loop_dt over several runs.
2. **[DEV MACHINE]** Lower `COUPLE_DIRECT_TIMEOUT_US` from 200,000 to
   about 20,000 µs (10 host periods at 500 Hz) in `couple_ctrl.h`. That
   bounds what a stall can do.
3. Still to rule out: SD-card writes. Test by logging to `/dev/shm`.

---

## 2026-10-20: B2, velocity estimator (offline)

**Code:**

- **New `estimators.py`**, with per-instance state and dt from
  `mcu_time_us`:
  - `LsVelocity(window_s)`: linear least squares.
  - `QuadLsVelocity(window_s)`: quadratic, slope at the newest sample.
  - `AlphaBetaVelocity(bandwidth_hz)`: alpha-beta tracker, alpha from dt
    per sample, β = α²/(2 − α).
  - `LegacyVelocity`: an offline replica of teleop's original scheme
    (latest frame per 2 ms tick, Δpos over ≥ 2 ms of kernel time, EMA 0.7).
  - The LS and AB estimators share a **rest detector**: output exactly 0
    when the position range over the last 20 ms is ≤ 1 count. Side
    effect: true speeds below about 50 counts/s also read 0, which is the
    encoder's resolution floor over that window.
- **New `b2_eval.py`.** Reference velocity: central difference on mcu
  time plus a zero-phase FFT low-pass at 100 Hz (no scipy on the Pi).
  - New estimators see every frame; legacy sees the latest frame per
    2 ms tick. All are scored at 500 Hz ticks.
  - noise = RMS(est − ref shifted by the best lag); lag searched over
    0–15 ms.
- **Data:** `spin_A_{±4000,±10000}` hold phases (constant speed),
  `teleop_KVs3e-05_FB-1_8_frames.csv` (B1, real motion, both motors), and
  `A3_rest_10min.csv` (rest).

**Results, coarse lag grid (0.25 ms)** — noise in counts/s @ lag:

| Dataset | legacy | LS 4 | LS 6 | LS 8 | AB 30 | AB 60 | AB 100 |
|---|---|---|---|---|---|---|---|
| spin +4000 | 278 @ 3.2 | 141 @ 2.0 | 88 @ 3.0 | 97 @ 4.0 | 171 @ 5.2 | 95 @ 2.2 | 200 @ 1.2 |
| spin −4000 | 268 @ 3.0 | 137 @ 2.0 | 86 @ 3.0 | 95 @ 4.0 | 176 @ 5.2 | 93 @ 2.5 | 199 @ 1.2 |
| spin +10000 | 352 @ 3.0 | 152 @ 2.0 | 103 @ 3.0 | 127 @ 4.0 | 222 @ 5.2 | 107 @ 2.2 | 219 @ 1.2 |
| spin −10000 | 306 @ 3.2 | 148 @ 2.0 | 96 @ 3.0 | 117 @ 4.0 | 215 @ 5.2 | 101 @ 2.5 | 214 @ 1.2 |
| teleop B1 A | 565 @ 3.2 | 184 @ 2.0 | 170 @ 3.0 | 249 @ 4.0 | 501 @ 5.2 | 130 @ 2.5 | 231 @ 1.2 |
| teleop B1 B | 225 @ 3.2 | 133 @ 2.0 | 79 @ 3.0 | 83 @ 4.0 | 165 @ 5.2 | 88 @ 2.5 | 191 @ 1.2 |

**Discarded:**

- QLS 12/16/20 ms: 2–3× *noisier* than legacy. End-point slopes are very
  noise-sensitive.
- LS 5 ms: 2.47–2.91× at 2.5 ms.
- AB 45 Hz: lag 3.5 ms.

**Fine lag grid (0.05 ms):**

| Estimator | Noise ratio legacy/est, constant speed | Same, teleop | Lag | Meets ≥ 3× and < 3 ms? |
|---|---|---|---|---|
| LS 6 ms | 3.12–3.41× | 2.85–3.34× | 2.95 ms | yes (barely) |
| AB 50 Hz | 3.43–3.87× | 2.86–4.07× | 2.90–2.95 ms | yes |
| AB 52 Hz | 3.47–3.96× | 2.90–4.49× | 2.80–2.85 ms | yes |
| **AB 55 Hz** | **3.36–3.91×** | 2.84–4.91× | **2.60–2.65 ms** | **yes, chosen** (0.35 ms lag margin) |
| AB 57 Hz | 3.24–3.80× | 2.75–4.99× | 2.50–2.55 ms | yes |

Legacy's lag is 3.10–3.30 ms.

**Rest (A3_rest_10min, 599,998 frames, 3,356 position changes; the shaft
crept 40 counts in 10 min):**

| Estimator | Ticks exactly 0 | Notes |
|---|---|---|
| legacy | 0.96 % | max \|v\| 974 counts/s, rms 39.5 |
| AB 55 Hz | 98.25 % | |

- **On every tick where the shaft was truly still** (20 ms range ≤ 1
  count, the detector's window), AB 55 Hz output is **exactly 0 in 100 %
  of 294,714 ticks**.
- Nonzero outputs occur only during real creep: 87 % of them have a net
  displacement ≥ 2 counts within ±50 ms.

**Verdict: B2 PASS with AlphaBetaVelocity(55 Hz):** 3.36–3.91× lower
noise at constant speed (≥ 3×), lag 2.60–2.65 ms (< 3 ms), exactly 0 at
rest.

**Wired in behind a flag:** `teleop_test.py` `VEL_ESTIMATOR = "legacy" | "ab55"`.

- Default "legacy". Override with `TELEOP_VEL_ESTIMATOR=ab55`; it requires
  `TELEOP_FRAME_QUEUE=1`.
- In "ab55" the estimator replaces vel_a_filt / vel_b_filt everywhere:
  tracking, damping, watchdog and observer.
- **Smoke-tested offline** with a fake CAN interface (synthetic frames
  served in real time): both modes run at 500 Hz and log every frame.
  Peak vel_b: legacy ±6,478 and ab55 ±6,345, true ±6,283.
- **Not yet run live:** it changes loop behaviour, so ask the user first.
  At the faster estimate, teleop's KV_SLOPE margin improves slightly
  (0.5–0.7 ms less lag; inferred).
- `spin_test.py` keeps its own 8-frame `LsVelocity` (no rest detector), so
  A4's runs stay reproducible.

---

## 2026-10-20: B3, leader observer class (offline)

**Code:** `estimators.HandTorqueObserver(G, fc_hz)`, with per-instance
state.

- `update(v, i, dt)` computes x = OBS_COMP·v/G, then
  tau_h = w·(x − LPF_w(x)) − LPF_w(i), the same derivative-free form as
  the inline code.
- The LPF alpha **and** OBS_COMP = 1/(1 − π·fc·dt) are both computed from
  the dt passed with each sample. The inline code uses the nominal
  1/CONTROL_RATE_HZ for both (alpha via `dt` at teleop line ~717,
  OBS_COMP at line 410).

**Replay:** inputs `vel_b` (= vel_b_filt), `iq_b_readback` and
`loop_dt`, compared with the logged `tau_h`. The logged values have
4 decimals, so the comparison floor is about 0.03 mA.

| Log | Logged tau_h rms | old G, nominal dt (**B3 test**) | old G, measured dt | A2 G, measured dt |
|---|---|---|---|---|
| `teleop_KVs3e-05_FB-1_2.csv` (A1 free) | 112.8 mA | **0.03 %** (0.029 mA) | 2.35 % | 3.22 % |
| `teleop_KVs3e-05_FB-1_5.csv` (hand presses) | 256.8 mA | **0.01 %** | 0.25 % | 0.87 % |
| `teleop_KVs3e-05_FB-1_6.csv` (foam) | 360.8 mA | **0.01 %** | 0.46 % | 0.56 % |
| `teleop_KVs3e-05_FB-1_8.csv` (B1) | 122.0 mA | **0.02 %** | 2.59 % | 2.94 % |

- "Measured dt" here is the host `loop_dt`, clipped to 2 ms for the first
  row and for stalls over 50 ms. Teleop CSVs have no mcu time. The live
  "mcu" option uses ESC timestamps instead.
- Switching to measured dt and A2's G changes tau_h by ≤ 3.2 % RMS. Small,
  as expected: tau_h is dominated by −LPF(i_b).

**Verdict: B3 PASS.** The replay reproduces the old tau_h within
0.01–0.03 % RMS on 4 logs (≤ 1 % required).

**Wired in behind flags** (defaults reproduce the original exactly):

- `OBSERVER_IMPL = "inline" | "class"` (`TELEOP_OBSERVER`)
- `OBS_G_SOURCE = "old" | "A2"` (`TELEOP_OBS_G`; A2 = 9,672,000)
- `OBS_DT_SOURCE = "nominal" | "mcu"` (`TELEOP_OBS_DT`). "mcu" uses the
  ESC-clock interval between B's newest frames and skips the update when
  no new frame arrived.
- G/dt overrides are refused with the inline observer.
- Smoke-tested with the fake interface, with
  `TELEOP_FRAME_QUEUE=1 TELEOP_VEL_ESTIMATOR=ab55` and each of
  inline / class-old-nominal / class-A2-mcu. All run; inline and
  class-nominal give matching tau_h (−0.0544…+0.0542 vs
  −0.0542…+0.0542 A on synthetic data).
- **Not yet run live.**

---

## 2026-10-20: B6, ripple compensation, estimate side (offline) — FAIL

**Code:**

- **`estimators.RippleModel`:** r(θ, i) = Σ c_n(θ) + i·Σ g_m(θ), θ from
  the raw count. Has `columns()` for fitting, `eval()` per sample for
  live use, and `save()`/`load()` as JSON.
- **`estimators.ExternalTorqueObserver(G, fc, ripple)`:**
  tau = LPF_w(i − r) − w·(x − LPF_w(x)), the C1 form. The sign follows
  the brief and is still to be verified in C1.
- **`b6_ripple.py`:**
  - Fits at the sample level: r = iq_meas − a/G_A, with a and v from a
    centred 15-frame quadratic LS kernel on mcu time. Moving frames only
    (|v| > 1,500 counts/s), with friction nuisance terms [1, sign v, v].
  - Then runs the observer (20 Hz, G_A = 9.025e6, AB 55 Hz velocity) with
    and without the model, and measures each harmonic left in its output
    by regression on cos/sin(nθ).
- **Data:**
  - Train: unloaded ±4k (baseline gains), loaded +4k (0–12 s) and −4k
    (0–4 s).
  - Held out: unloaded ±10k and +4k low-gain.

**Results:** 14/rev in the observer output, without → with model.

| Variant | +10k | −10k | +4k low-gain | 14/rev in-sample |
|---|---|---|---|---|
| shared friction terms, c+g (first fit, saved to `logs/ripple_model_A.json`) | 8.0→8.5 (−7 %) | 1.8→5.4 (−198 %) | 8.7→5.6 (+35 %) | −1…+44 % |
| A: per-run friction, c+g | −6 % | −243 % | +31 % | −148…+14 % |
| B: per-run friction, c only, unloaded train | −15 % | +18 % | +14 % | +17…+31 % |
| C: per-run friction, c only, all train | −16 % | +13 % | +22 % | +18…+55 % |

- **Other harmonics do better,** for example on the +4k low-gain held-out
  run (variant C): 12/rev −45 %, 24/rev −60 %, 28/rev −45 %, 36/rev
  −87 %, 42/rev −57 %, 84/rev −44 %. Output rms falls only about 2–5 mA
  (e.g. 29.9 → 27.1 mA).
- **The first fit's numbers are distorted:** g14 = 147 mA/A, against
  24 mA/A in the A4 angle-domain fit, and a shared "Coulomb" term of
  101 mA. The plate's extra friction in the loaded runs leaked into the
  i-dependent columns. Per-run friction terms (A–C) reduce that but do
  not fix 14/rev.

**Verdict: B6 FAIL** (14/rev reduction on held-out runs −198…+35 %,
≥ 60 % required).

**Diagnosis (inferred):**

1. A static angle-locked model removes only about 20–50 % of the
   harmonics in the observer output, **even in-sample**. So that output
   ripple is not purely angle-locked.
   - This matches A4's finding that current ripple at the loop's
     frequencies (7–35 Hz here) produced only 20–50 % of the acceleration
     G predicts.
   - If the plant gain is lower at those frequencies (compliance), the
     observer's i − a/G does not cancel the loop-driven motion, and the
     residual depends on speed and loop gains rather than angle.
2. The held-out set is weak for 14/rev:
   - At ±10k, 14/rev is 35 Hz, attenuated by the observer's 20 Hz LPF
     (baseline only 1.8–8.0 mA).
   - Only one held-out run (+4k low-gain) is at the training speed.

**Proposed fix (needs motor time; ask the user):**

1. **Run A4b, the G-vs-frequency chirp** (follower free, about 25 s; see
   Handoff section 4).
   - If G(f) falls between 5 and 40 Hz, model it in the observer's
     acceleration path, then refit.
   - This also decides C1's design.
2. **Record dedicated held-out runs** at the training speed:
   `spin_test.py --node A --speed ±4000 --time 10`, baseline gains, about
   25 s.
3. Then refit (variant C or A with per-run friction) and re-test.
   - Note: `logs/ripple_model_A.json` is the first (shared-friction) fit.
     Regenerate it after the fix.
4. **Leader:** there is no ripple model for B yet (A4 measured the
   follower only). Apply B6 to the leader observer only after equivalent
   leader runs.

---

## 2026-10-20: B7, static force calibration — script ready, waiting for bench

**`b7_force_cal.py`:**

- **Run:** `--sign ±1` holds 0.05, 0.10, 0.20, 0.30, 0.45, 0.60 and
  0.80 A for 5 s each, with 4 s at 0 A between and a 5 s lead-in. It
  prints the schedule first. SET_IQ is resent every 2 ms, and every frame
  is logged to `logs/B7_cal.csv` (refuses to overwrite).
- **Fit:** `--fit logs/B7_cal.csv --grams g1,…,g7` fits
  F = g·9.81/1000 N against mean **measured** |Iq| over the last half of
  each hold. Pass: R² ≥ 0.99, slope within 15 % of 3.3 N/A.
- **Safety:**
  - Never above 0.8 A.
  - Aborts if the shaft moves faster than 2,000 counts/s, or travels more
    than 100 counts within a level. A wrong `--sign` lifts the link, which
    aborts at level 1.
  - Aborts on stale telemetry, and on a reset node (idle-Iq precheck).
- **Offline check:** on synthetic data (true 3.0 N/A, noise 0.03 A per
  sample, 1 g per reading) the fit gives 2.960 N/A, R² 0.9999.

**Needs:** the user at the bench, the follower link resting on a kitchen
scale on a rigid support, tared at 0 A. The user reads the steady value
near the end of each hold and reports the 7 readings.

---

## 2026-10-22: Host real-time priority (stall fix, step 1 of 2)

**Why:** the host loop stalled for up to 2,082 ms (B1). The CAN RX
thread's `SCHED_FIFO` request in `can_interface.py`
(`_apply_rx_thread_priority`, priority 20) failed silently, because the
user's RT limit was 0 (`ulimit -r` = 0, PermissionError). The thread ran
at normal priority, competing with everything else.

**Change (system, approved by the user):** new file
`/etc/security/limits.d/99-rtprio.conf`:

```
jayrpi4  -  rtprio  50
```

- PAM applies it on login: `pam_limits` is in `/etc/pam.d/sshd` (line
  40), `lightdm`, `login`, `sudo` and `cron`.
- This Claude Code session runs under the VS Code server, which inherits
  its limits from the SSH login that started it. The **new limit only
  applies after a reboot or re-login.**
- **Revert:** `sudo rm /etc/security/limits.d/99-rtprio.conf`, then
  reboot.

**Verified (before reboot):** in a fresh PAM login session
(`sudo -i -u jayrpi4`), `ulimit -r` = 50 and
`os.sched_setscheduler(0, SCHED_FIFO, 20)` succeeds. The old session
still shows 0, as expected.

**Step 2, after the reboot and on proper power** (not done yet):

1. Confirm `ulimit -r` = 50 inside the Claude Code shell.
2. Run 3 × 60 s teleop free-motion runs with `TELEOP_FRAME_QUEUE=1` and
   compare max `loop_dt` with B1 (2,082 ms) and Phase A (up to 1,045 ms).
3. Optionally put the control loop on RT too, e.g.
   `chrt -f 10 taskset -c 2,3 python3 teleop_test.py`, below the RX
   thread's 20. That is not done in code yet.
4. If stalls persist, test logging to `/dev/shm` (SD-card writes).

---

## 2026-10-22: Host stall fix, step 2: verification runs

**Setup:**

- After the reboot: `ulimit -r` = 50 and SCHED_FIFO(20) succeeds in the
  Claude Code shell. The Pi is on the proper supply: 0 under-voltage
  events, `throttled=0x0`.
- Both ESCs had been power-cycled. `zero_iq_check.py` re-aligned them:
  counter jump to 142 at 2.016 s on both; RUN Iq mean A +0.0000 A,
  B +0.0004 A; PASS. Log: `logs/precheck_zero_iq_realign_2026-10-22.csv`.
- **Runs:** three free-motion teleop runs,
  `TELEOP_FRAME_QUEUE=1 timeout -s INT 75 taskset -c 2,3 python3 teleop_test.py`,
  with the user on the leader. `bus_check.py` was OK after each.

| Run | Log | Logged to | Leader vel rms (counts/s) | max loop_dt | Iterations > 5 ms | 2.5–5 ms | p95 / p99 | Missing frames |
|---|---|---|---|---|---|---|---|---|
| 1 | `teleop_KVs3e-05_FB-1_9.csv` | SD | 749 (gentle) | 36.0 ms | 16 | 81 | 2.09 / 2.21 ms | 0 |
| 2 | `teleop_KVs3e-05_FB-1_10.csv` | SD | 651 (gentle) | **1,034 ms** (also 369 ms) | 16 | 71 | 2.07 / 2.20 ms | 0 |
| 3 | `teleop_KVs3e-05_FB-1_11_shm.csv` | **RAM** (`/dev/shm`, copied back) | 6,902 (vigorous) | **14.8 ms** (start-up iteration) | 1 | 14 | 2.06 / 2.16 ms | 0 |

For comparison, before the RT fix: B1 had a max of 2,082 ms; Phase A
had up to 1,045 ms.

**RT priority fixed the RX thread:**

- In B1 the 2,082 ms stall left `enc_lag_a` (parse − kernel time) at
  47.6 ms, so the RX thread had stalled too.
- In run 2 the lag after both stalls was 0.11 ms and 0.06 ms, and the
  1,016 frames buffered during the 1,034 ms stall arrived every 1.00 ms
  (max 1.04 ms) and were parsed on time.
- **Only the main control loop still stalled.**

**Logging to RAM removed the stalls in run 3**, with the most vigorous
motion of the three. That points to blocking SD-card writes in the main
loop (two CSV files written every iteration) as the remaining cause.

- Inferred, and strongly suggestive, but one clean run is not proof:
  run 1 on SD was also free of long stalls.
- Felt-current RMS is not comparable across these runs, because motion
  intensity differed by 10×. Run 3's iq_b_cmd RMS is 0.258 A at
  vel_b rms 6,902.

**Status: DEFERRED by the user (2026-10-22).** Logged as a potential
issue, not to be worked on for now.

**Proposed fix (not applied):** keep the live loop off the SD card.

- **Option A (simplest):** teleop writes its CSVs to a RAM directory
  during the run and moves them to `logs/` on exit, via an env/config
  `TELEOP_LOG_DIR`, defaulting to `logs`.
- **Option B:** a writer thread with a queue. File writes release the
  GIL, so an SD stall would block only the writer.
- Then confirm with 3 more runs (≥ 1 vigorous) logged that way: max
  loop_dt < 50 ms, and no iteration over 100 ms.
- The same applies to every live script (`spin_test`, `a5_command_test`,
  `plant_gain_test`, `b7_force_cal`). Those buffer their data in memory
  and write only at the end, so they are already safe, except
  `coupling_monitor.py`, which writes as it goes (inferred from code).

---

## 2026-10-22: Feasibility check (offline): can the follower estimate separate contact from free motion?

**Why:** the user asked for reasonable assurance, from the data, that the
goal is achievable, and for a timeline.

**Script:** `feasibility.py`, plus ad-hoc fits recorded below. Estimator:
`ExternalTorqueObserver` (C1 form, 20 Hz, G_A = 9.025e6) on every-frame
logs, with AB 55 Hz velocity and mcu dt.

**Free-motion floor** (nothing touched; follower = motor A):

| Run | Leader \|v\| rms (counts/s) | Estimate rms | − friction (held-out fit) | Moving | At rest | p99 |
|---|---|---|---|---|---|---|
| B1 `_8_frames` (moderate) | 8,991 | 47.8 mA | 43.8 | 44.6 | 23.7 | 125 |
| run 1 `_9_frames` (gentle) | 3,601 | 36.0 | 37.7 | 45.8 | 19.8 | 178 |
| run 2 `_10_frames` (gentle) | 6,699 | 38.5 | 42.5 | 47.5 | 25.3 | 144 |
| run 3 `_11_shm_frames` (vigorous, t = 30–47 s excluded) | 12,142 | 45.3 | 41.7 | 41.8 | 32.3 | 109 |

- **Run 3 contains contact.** From 33 to 46 s the follower was nearly
  stationary (span 15–450 counts) at 0.4–0.85 A. The estimate correctly
  read 0.5–1.0 A of external torque. Excluded from the floor.
  - Either the follower was blocked or held, or it hit something. Not
    confirmed with the user.
  - The fast stretch at 30.5–32 s (400–570 mA peaks at about 10,000
    counts/s) is ambiguous, so it was excluded too.
- **Friction model barely helps** (Coulomb 26 mA, low-speed term −32 mA,
  viscous −7 mA per 10,000 counts/s).

**Error grows with acceleration** (pooled):

| \|a\| (counts/s²) | n | Residual rms | Median a/G |
|---|---|---|---|
| < 1e5 | 121,751 | 28.8 mA | 0.4 mA |
| 1e5–3e5 | 64,217 | 37.5 | 20 |
| 3e5–1e6 | 66,513 | 44.8 | 57 |
| 1e6–3e6 | 15,741 | 70.3 | 145 |
| > 3e6 (about 1 %) | 2,337 | 164.4 | 489 |

**Cause: timing, not G.** Mean floor over the 4 runs by G and velocity
advance:

| G \ advance | 0 ms | 2 ms | 3 ms | 4 ms |
|---|---|---|---|---|
| 6.0e6 | 48.5 | 40.0 | 39.0 | 40.4 |
| 8.0e6 | 42.8 | 35.5 | 34.5 | 35.8 |
| **9.025e6 (A2)** | 41.9 | 35.3 | **34.4** | 35.5 |

- A2's G is confirmed as best.
- Aligning velocity and current (the AB 55 Hz estimator lags 2.6 ms)
  cuts the floor by 18 %, to 34.4 mA mean (24.9 mA on the gentle run).
- **Live implementation:** delay the current by about 3 ms to match the
  velocity, rather than advancing velocity. That adds 3 ms latency,
  small against the 30 ms tap target (inferred).

**Contact signal:**

- A1 hand-clamp presses: 0.28–0.85 A of follower current (A1 run 2
  entry).
- Run 3's blocked stretch: estimate 0.5–1.0 A.
- **Margin over the free floor (34–44 mA rms, p99 110–180 mA):** about
  7–25× at RMS and about 2–8× against p99.
- Force terms (nominal 3.3 N/A): floor about 0.11–0.15 N rms; contacts
  1–3 N.

**What this shows and does not show:**

- **Shown (measured):** with the data we already log, the estimate
  separates free motion from pressing on the follower by roughly an order
  of magnitude. G is right, and the largest fixable error found so far
  (timing) is understood.
- **Supported, not yet shown (inferred):**
  - B4's averaged Iq (raw noise ≥ 2× lower) and the angle-locked ripple
    (12/rev about 20 mA) are the next floor reductions. A realistic
    target is about 15–25 mA rms, so the brief's 15 mA is ambitious but
    plausible.
  - **Rigid vs soft** depends on rendering force against penetration.
    The sensing resolution supports it by a wide margin: 0.031 mm/count,
    foam of e.g. 0.5 N/mm reaches 2 N at about 4 mm (130 counts), while a
    rigid stop reaches 2 N within a few counts, and the force floor is
    about 0.1 N. But it has not been demonstrated: no rigid-stop or foam
    data at a fixed angle exists yet.
- **Not yet known:** closed-loop stability of the felt force (C3),
  through the hand with about 5 ms of loop delay. That is the largest
  remaining technical risk.

---

## 2026-10-22: SD-card write stalls confirmed (no motors, no CAN)

**Why:** the user asked to re-test the SD hypothesis, and asked why the
Pi would not crash if the SD card were "failing".

- It is not a failure. `write()` normally copies into the page cache and
  returns at once.
- When the card falls behind (dirty-page limits, ext4 journal commit
  every 5 s), the kernel blocks the *writing thread* until the card
  catches up. Consumer SD cards routinely pause for hundreds of ms during
  internal housekeeping (erase, wear-levelling).
- Only that thread waits; nothing crashes. This matches run 2, where the
  RX thread kept parsing while the main loop froze.

**Test:** new `sd_stall_test.py`. It reproduces teleop's per-iteration
I/O with the frame queue on (one 23-column row plus four frame rows every
2 ms, `csv.writer`, the same deadline loop), with no CAN and no motors. It
times every iteration and every write call, and samples `/proc/meminfo`
Dirty/Writeback. Run with `taskset -c 2,3`, 75 s each.

| Run | Target | p95 / p99 loop | max loop | > 5 / > 50 / > 100 ms | Slowest write calls | Dirty / Writeback max |
|---|---|---|---|---|---|---|
| 1 | SD (`logs/sdtest`) | 2.013 / 2.040 ms | **1,641.5 ms** | 23 / 2 / 2 | 1,641.4, 1,117.4, 31.7 ms | 6,760 / 4,812 kB |
| 2 | SD | 2.012 / 2.029 ms | **853.1 ms** | 7 / 2 / 1 | 853.1, 68.1, 27.7 ms | 6,504 / 5,928 kB |
| 3 | SD | 2.011 / 2.028 ms | 57.9 ms | 7 / 1 / 0 | 57.9, 42.4 ms | 6,976 / 6,556 kB |
| 4 | RAM (`/dev/shm/sdtest`) | 2.004 / 2.015 ms | **5.4 ms** | 1 / 0 / 0 | 2.3 ms max | 1,924 / 1,672 kB |

- **On SD, the longest loop iteration equals the slowest single write
  call in every run.** The loop was blocked inside `write()`.
- RAM never exceeded 5.4 ms. There was no kernel message (no hung task,
  no mmc error), and `throttled=0x0`.

**Verdict:** **CONFIRMED.** Teleop's remaining 0.4–2 s main-loop stalls
are SD-card write stalls. Measured here without motors; that teleop's
stalls are the same mechanism is inferred, strongly supported by run 3
(teleop on RAM, max 14.8 ms).

**Status:** still a deferred potential issue (user, 2026-10-22). **It must
be fixed before C3.**

- Simplest fix: log to RAM and move the files at exit, or use a writer
  thread.
- A longer-term option is a higher-endurance SD card or USB SSD (inferred
  to stall less; untested).
- The `logs/sdtest/sd_*.csv` files are throwaway test output.

---

## 2026-10-25: B4, TELEM sends the 1 ms mean of Iq (after the firmware flash)

**What changed:** the user flashed both boards with
`ESC_based/FIRMWARE_CHANGES.txt`: B4 (`TELEM_IQ_MEAN` = 1) plus
`COUPLE_DIRECT_TIMEOUT_US` = 20,000 µs. B5 was not included (dropped).

**Passive check right after the flash** (uncalibrated idle, no START):

| Board | Idle Iq std before (P1) | After flash |
|---|---|---|
| ESC1 | 77.9 mA | 34.2 mA (2.3× lower) |
| ESC2 | 47.6 mA | 31.9 mA (1.5× lower) |

ESC1 also showed 404 distinct values in 8 s, against about 40–200
before. Consistent with averaging (inferred).

**What ran** (`throttled=0x0`):

1. **`zero_iq_check.py`:** alignment counter jump to 142 at 2.016 s;
   PASS. RUN noise at 0 A: A std **4.7 mA**, B **6.7 mA** (2026-10-22
   re-align: 45.5 / 29.4 mA). Log: `logs/precheck_zero_iq_B4.csv`.
2. **HOLD, zero gains, 30 s, follower free:** `coupling_monitor.py --mode
   hold --motors a --kp 0 --kd 0 --kd-local 0 --duration 30`. Log:
   `logs/B4_hold_free.csv`. Shaft drifted 287 counts.
3. **A5(c) staircase, 20 s:** `a5_command_test.py --mode stair`. Log:
   `logs/A5_stair.csv` (+ `_cmd`). The pre-B4 run was kept as
   `logs/A5_stair_preB4.csv`.

| Criterion | Before | After B4 | Threshold | Verdict |
|---|---|---|---|---|
| Iq std, follower free at 0 A | 30.4 mA (`A3_free`, raw ≈ 3996) | **3.12 mA** (p1…p99 −7.5…+7.2 mA) | ≥ 2× lower | **PASS (9.7×)** |
| Step 50 % crossing (host send → TELEM) | 1.47 ms | **1.45 ms** | ≤ 2.07 ms | **PASS** |
| Aliased peak (246–248 Hz) | 6.13 mA (clamped) on a 2.32 mA floor | **0.76 mA** on a 0.16 mA floor | gone | **not gone** (8× smaller, still 4.7× above floor) |
| Drift (3 × 10 s means) | 0.89 mA | 0.01 mA | — | — |

- The shaft angle differed between the before and after runs. Noise
  varies ≤ 1.8× with angle here (per-bin std 3.0–5.2 mA), so the
  reduction is ≥ 5× even in the worst case.
- **The 247 Hz peak** is consistent with aliasing that the 16-tap boxcar
  only partly removes: its nulls are at 1, 2, 3 kHz…, while components
  near 753 or 1,247 Hz, which also fold to 247 Hz, are cut only 3–5×.
  It could also be real 247 Hz content (inferred either way).
  - It is negligible in absolute terms (0.76 mA against a 3.1 mA std).
  - If it matters later, the firmware's EWMA, or a longer boxcar across
    2 periods, would attack it.
- **Step response:** at 1 ms after the send it reads 10.6 mA (was 7.4),
  because the mean includes the samples after the step within that tick.
  Settling is unchanged (48.9 mA at 3 ms).

**Verdict: B4 PASS** on noise (9.7×) and delay. The aliased-peak
criterion is only partly met: the peak is 8× smaller but still visible.

**Consequences:**

- C1's free-floor target (15 mA rms) looks much more reachable. The
  sensor-noise part of the 34–44 mA floor in the feasibility check came
  from raw samples (about 0.03–0.04 A std); it is now about 3 mA (inferred
  until C1 is re-run on new data).
- **All pre-B4 Iq data (Phase A, B1, B6 fits, feasibility) is "raw
  sample" telemetry.** Any refit (B6, C1) should use post-B4 logs.

---

## 2026-10-25: B7 attempt 1, aborted: both ESCs would not START (latched fault)

**What happened:**

1. The user set up the scale.
2. `b7_force_cal.py --sign 1` printed its schedule, then **"ABORT: no
   RUN"**: the follower never reported RUN within 5 s of START. No
   current was applied.
3. Passive check afterwards: both nodes streaming at 1 kHz, calibrated
   (idle Iq A −0.106 A, B −0.012 A, so no MCU reset). DBG status 128 =
   trip 4 (HOST_TIMEOUT, latched from the end of the previous run),
   `run` False.
4. `zero_iq_check.py` (user OK): **"RUN bit never seen" on both A and
   B.** An hour earlier, in this session, both reached RUN (the B4
   checks).

**Cause (inferred from code):**

- The CAN START handler only calls `MC_StartMotor1()`
  (`can_driver.c:361-363`).
- After any motor-control fault (under-voltage, over-voltage,
  over-current), MCSDK latches FAULT_OVER and ignores START until
  `MC_AcknowledgeFaultMotor1()` is called. Nothing in this firmware calls
  it, and DBG does not report the fault code.
- Both boards affected at once points to the shared motor supply:
  most likely a VBUS dip below `UD_VOLTAGE_THRESHOLD_V` = 8 V while the
  bench was rearranged for the scale. The user confirmed the supply is on
  and that cables moved.
- The 20 ms host timeout (trip 4) does not block START.

**Fix now:** restore a steady supply, then power-cycle both ESCs (clears
the latch). Then `bus_check.py` → `zero_iq_check.py` (re-align) → B7
again.

**Firmware improvement for later** (dev machine; added to the open
issues, not done):

- Acknowledge faults on START: call `MC_AcknowledgeFaultMotor1()` before
  `MC_StartMotor1()`.
- Put the MCSDK fault code (`MC_GetOccurredFaultsMotor1()`) into DBG, so
  a latched fault is diagnosable over CAN instead of looking like a
  silent "no RUN".
- It may also explain some earlier "simultaneous ESC" events (inferred).

---

## 2026-10-25: B7 attempt 2 aborted: both ESCs rebooted again (no motor activity)

**What happened:**

1. After the user power-cycled the ESCs: `bus_check.py` showed RESET on
   both, as expected, with status 0.
2. The user lifted the link; `zero_iq_check.py` re-aligned both. PASS.
   RUN noise A 4.2 mA, B 3.3 mA. ESC1's counter load was 219 → 142, a
   77-count jump, below the check's 100-count report threshold. Log:
   `logs/precheck_zero_iq_realign_B7.csv`.
3. The user put the link back on the scale and re-tared.
   `b7_force_cal.py` then aborted at its pre-check, **"node missing or
   reset"**. No current was applied.
4. `bus_check.py`: **both** ESCs uncalibrated again (idle Iq A +22.5 A,
   B +28.6 A), encoder counters restarted near 0. **Both MCUs rebooted
   within about 2 minutes, with nothing commanded.** The Pi was
   unaffected: `throttled=0x0`, CAN error-active.

**Shared-power events so far:**

| Date | Event | Motor activity |
|---|---|---|
| 2026-10-20 | between P2 and A1 run 1: both reset | none |
| 2026-10-20 | A1 run 2, attempt 2: both reset in the same ms | about 0.2 A |
| 2026-10-25 | after scale setup: both latched a fault (likely a VBUS dip) | none |
| 2026-10-25 | during link placement and re-tare: both reset | none |

**Interpretation (inferred, strong):** a fault in what the two boards
share, i.e. the motor supply, its leads or a connector. Not the firmware
or the Pi. The last two events coincided with the user handling the
setup, which suggests an intermittent connection.

**Action:** stop motor tests until it is found. Proposed:

- A receive-only reset monitor (1 ms resolution) while the user wiggles
  each connector.
- Check the supply's voltage, current limit and CV/CC state.

---

## 2026-10-25: B7, static force calibration — data taken; FAIL as specified (geometry question open)

**What ran:**

1. After the user reseated connections (both ESCs rebooted, so
   re-aligned with the link lifted, PASS), the link was placed on the
   kitchen scale on a rigid support and tared at 0 A.
2. `b7_force_cal.py --sign 1`: all 7 levels completed. Shaft travel was
   **0 counts** in the second half of every hold, so the link pressed
   rigidly. `throttled=0x0`.

**Log:** `logs/B7_cal.csv`. The earlier aborted attempts are
`B7_cal_attempt1_noRUN.csv` and `B7_cal_attempt2_reset.csv`.

| Level | iq_cmd | iq_meas mean (std) | Scale (user) | Force | g per A |
|---|---|---|---|---|---|
| 1 | 0.05 A | 0.0496 (8.6 mA) | 3 g | 0.0294 N | 60.5 |
| 2 | 0.10 | 0.0991 (8.6) | 6 | 0.0589 | 60.5 |
| 3 | 0.20 | 0.1981 (8.6) | 9 | 0.0883 | 45.4 |
| 4 | 0.30 | 0.2986 (9.7) | 13 | 0.1275 | 43.5 |
| 5 | 0.45 | 0.4464 (12.1) | 19 | 0.1864 | 42.6 |
| 6 | 0.60 | 0.5911 (34.7) | 26 | 0.2551 | 44.0 |
| 7 | 0.80 | **0.6754 (82.0)** | 40 | 0.3924 | 59.2 |

**Fits** (F vs mean measured |Iq|):

| Levels | Slope | Intercept | R² |
|---|---|---|---|
| All 7 | 0.508 N/A | −0.0087 N | 0.934 |
| 1–6 | **0.404 N/A** | +0.0111 N | **0.996** |
| 1–5 | 0.384 N/A | +0.0143 N | 0.996 |
| Through origin, all 7 | 0.490 N/A | — | — |

**Verdict: FAIL as specified** (all 7: R² 0.934 < 0.99; slope −85 % vs
3.3 N/A). But:

- **Levels 1–6 are linear (R² 0.996).** Level 7 is the outlier, and its
  current measurement is unreliable: it fell short of the command
  (0.675 vs 0.80 A) with 10× normal noise. That suggests the current
  loop is out of headroom near 0.8 A at stall, for example a supply
  current limit, or the firmware limit the user mentioned (inferred).
- **The slope's expected value depends on geometry.** The brief's
  3.3 N/A assumes the force acts 2 cm from the shaft (66 mN·m/A ÷
  0.02 m). F = τ/r, so 0.40 N/A matches the nominal Kt if the contact
  point is about 16 cm from the shaft axis: 0.40 × 0.16 = 64 mN·m/A
  (inferred). **The user has been asked for the contact radius.**
- **Low levels read high** (60 g/A vs about 43 at levels 3–6): 1 g scale
  resolution (±17 % at 3 g), and bearing stiction (0.05–0.09 A
  breakaway) can carry part of a static load.

**Next:** with the radius r, report Kt = slope × r (mN·m/A), and rescale
the brief's force numbers to the real geometry: N/A = Kt / r, and
mm/count = 2π·r / 4000 (the brief's 0.031 mm/count assumes r = 2 cm).
Consider repeating with more mid-range levels and a finer scale if
better accuracy is needed for D1.

---

## 2026-10-25: B7 re-evaluated with the real contact radius (r ≈ 160 mm, user)

**Torque constant:** Kt = slope × r.

| Fit | Slope | Kt (r = 0.160 m) | vs nominal 66 mN·m/A |
|---|---|---|---|
| Levels 1–6 | 0.404 N/A | **64.6 mN·m/A** | −2 % |
| Levels 1–5 | 0.384 N/A | 61.4 mN·m/A | −7 % |
| All 7 | 0.508 N/A | 81 mN·m/A | +23 % (level 7 distorted) |

- The user's radius is "roughly 160 mm". ±1 cm on r is ±6 % on Kt.

**Verdict: B7 PASS on levels 1–6.**

- R² 0.996 (≥ 0.99).
- Slope within 2 % of the geometry-correct expectation, 66/0.160 =
  0.41 N/A (≤ 15 %).
- **Level 7 is excluded explicitly:** the current loop did not reach the
  0.80 A command (measured 0.675 A, std 82 mA).
- With all 7 levels it is a FAIL (R² 0.934). The brief's 3.3 N/A assumed
  a 2 cm link and does not apply to this rig.

**Geometry conversion for later phases.** The brief's numbers assume
r = 2 cm; this rig contacts at about 160 mm.

| Quantity | Brief (r = 2 cm) | This rig (r ≈ 160 mm) |
|---|---|---|
| Force per amp at the contact point | 3.3 N/A | **0.40 N/A** (measured 0.404) |
| Tip displacement per count | 0.031 mm | **0.25 mm** (2π·160/4000) |
| Force at a 0.6 A ceiling (level 7 shows current falling short near 0.8 A) | about 2 N | **about 0.24 N (about 25 g)** |
| Free-motion estimate floor (feasibility, 34 mA) | about 0.11 N | about 14 mN (2.2 mN·m) |
| B4 sensor noise (3 mA) | — | about 1.2 mN (0.2 mN·m) |

- **Proposal:** restate the Phase D force thresholds as **torque**, so
  they do not depend on where the follower is pressed.
  - D1's 0.05 N floor at 2 cm is 1.0 mN·m.
  - D1's "0.2 N up to the ceiling" becomes about 4 mN·m to the ceiling
    (about 40 mN·m at 0.6 A).
  - D2's stiffness uses 0.25 mm/count at the tip, or rad/count at the
    shaft.
- **Not changed in the plan text yet:** this needs the user's agreement.

---

## 2026-10-25: B7 rerun at a shorter lever arm (about 4 cm)

**Why:** at r ≈ 160 mm the tip force was only about 0.24 N at 0.6 A. The
user shortened the lever arm to about 4 cm (not measured precisely).

**What ran:** the same `b7_force_cal.py --sign 1`. Levels 1–5 completed;
it **aborted at level 6** on the travel check (+101 counts within the
level, about 6 mm of arc at 4 cm). Travel was already 20 counts at
level 5 and 53 at level 6. The user reports the measurement rig wobbled:
the lever arm moved the motor, so the motor mount is not rigid.
`throttled=0x0`.

**Log:** `logs/B7_cal_r40mm.csv`. The earlier run is now
`logs/B7_cal_r160mm.csv`.

| Level | iq_meas mean (std) | Scale (user, "scuffed") |
|---|---|---|
| 1 | 0.0495 A (8.1 mA) | 3 g |
| 2 | 0.0991 (8.0) | 10 |
| 3 | 0.1980 (8.6) | 22 |
| 4 | 0.2982 (10.0) | 34 |
| 5 | 0.4401 (25.0), 20 counts travel | 60 |

**Fits:**

| Levels | Slope | Intercept | R² |
|---|---|---|---|
| 1–4 | **1.214 N/A** | −0.027 N | 0.999 |
| 1–5 | 1.397 N/A | −0.050 N | 0.990 |

- Expected at exactly 40 mm: 66/0.040 = 1.65 N/A. Measured 1–4 is
  **3.0×** the 160 mm run; geometry predicts 4.0×.
- **Likely causes (inferred):** motor-mount compliance absorbing part of
  the push (the user saw the motor move), the scale readings being
  uncertain, and/or a true contact radius above 4 cm (1.214 N/A would
  imply about 54 mm at 66 mN·m/A).

**Verdict:** confirms the scaling (much stronger at a short arm). The
**calibration value stays Kt = 64.6 mN·m/A** from the 160 mm run (rigid
contact, 0 travel). The user moved on to the next phase.

**Important for Phase C/D:** the **motor mount moves under load.** A
compliant mount makes a rigid contact look soft, which is exactly the
rigid-vs-soft distinction D2/D4 must measure. **Clamp the motor mount
rigidly before the contact-data session.**

---

## 2026-10-25: Phase C data session + C1 offline evaluation

**Data session.** First post-B4 contact data, recorded with:

- the motor mount clamped (user);
- the follower arm hovering above a rigid surface at about 4 cm
  contact radius, unable to rotate 360°;
- every-frame logging **to RAM**: teleop run from `/dev/shm/c1data/<run>`
  and copied back. That is the SD-stall workaround.

Boards were still aligned from earlier. Each run was
`TELEOP_FRAME_QUEUE=1 timeout -s INT 50..70 taskset -c 2,3 python3 teleop_test.py`,
with the user on the leader and the felt path unchanged.

| Run | Log | Frames / missing | max loop_dt | Notes |
|---|---|---|---|---|
| Free motion | `logs/C1_free.csv` + `_frames` | 67.6 s / 0 | 15.1 ms | follower range only 278 counts (25°), leader \|v\| rms 252, so **gentle** |
| Rigid presses | `logs/C1_rigid_press.csv` + `_frames` | 0 missing | 17.0 ms | idle for the first 34 s; 6 holds of about 1 s at 0.18–0.62 A |
| Rigid taps | `logs/C1_rigid_taps.csv` + `_frames` | 0 missing | 12.9 ms | about 29 contacts, median impact about 6,900 counts/s |
| Foam (1 piece) | `logs/C1_foam1.csv` + `_frames` | 0 missing | 12.8 ms | contact 92 % of the time |

**C1 estimator** (`c1_eval.py`): `ExternalTorqueObserver` (20 Hz, G_A =
9.025e6) on every frame, with AB 55 Hz velocity and mcu dt.

- **Alignment:** the current is delayed 3 frames (3 ms) to match the
  velocity lag. On this gentle data it made no measurable difference.
- **Friction model:** fitted on the free run. Degenerate on this gentle
  motion (cancelling ±0.2 A terms); not trusted.
- **Units:** Kt = 64.6 mN·m/A (B7), r = 0.04 m.

| Check | Result | Threshold | Verdict |
|---|---|---|---|
| 1. Free-space rms (C1_free) | **10.3 mA** = 0.67 mN·m ≈ 17 mN at 4 cm (p99 46 mA) | brief < 15 mA; Gate A ≤ 25 mA | **PASS**, but on gentle motion: mostly the at-rest floor |
| 2. Static accuracy, 6 rigid holds (0.26–1.03 N) | estimate vs measured current **+2.8, +1.4, +1.2, +1.5, +1.6, −0.2 %** | within 10 % of B7 | **PASS** |
| 3. Sign | holds: current +, estimate + → environment pushes −; **tau_env (brief convention) = −(i − a/G)** | — | settled |
| 4. Rigid vs foam | see below | D2: rigid ≥ 5× stiffest foam | **≈ 18× or more** (preview) |

**Check 4: quasi-static frames only** (|v| < 300 counts/s, estimate
> 0.08 A): median raw position per 0.1 N force bin.

- **Rigid presses:** 0.45–1.15 N → 3175…3177 (2 counts over 0.7 N), so
  **≥ about 6 N/mm**, encoder-limited (0.063 mm/count at 4 cm).
  - The 0.25–0.35 N bins sit at 3203: an earlier press 28 counts higher,
    consistent with the mount shifting under load (inferred).
  - The 0.15 N bin (2830) is a slow free-space push, not contact, and is
    excluded. Including it gave a meaningless 0.11 N/mm.
- **Rigid taps:** ≥ 0.45 N → 3172…3176, so ≥ about 5 N/mm.
- **Foam:** 0.15 → 1.15 N gives 2818 → 2876 (58 counts), **about
  0.3 N/mm** (3.4 mm/N).

**Gate A: PASS.**

- Free floor 10.3 mA ≤ 25 mA.
- Static error 2.8 % ≤ 10 %.
- Contact forces of 0.26–1.03 N are 15–60× the 17 mN floor.

**Caveats (to close in C2 or with a short extra run):**

1. The free run was gentle. The inertia and friction path is barely
   exercised, so a more dynamic free run within the allowed range is
   needed to confirm the floor.
2. The friction model is ill-posed on this data.
3. Ripple compensation is not applied (B6 refit on post-B4 constant-speed
   data is still to do).
4. Only one foam. The radius is about 4 cm, not measured precisely.
5. The motor-mount shift (28 counts) would distort D2. Re-clamp before
   Phase D.

**Next:** C2, shadow mode. Compute and log tau_env live in teleop
(flagged), with the felt path unchanged. Then C3 needs the SD fix in
code, not the cwd workaround, before closing the loop.

---

## 2026-10-25: C2 shadow mode, run 1 (vigorous free motion) — FAIL; follower plant changed with the new arm

**Code (teleop, flagged):** `SHADOW_TAU_ENV` (`TELEOP_SHADOW_TAU_ENV=1`,
needs the frame queue).

- Every follower frame goes through `ExternalTorqueObserver` (20 Hz),
  with its own AB 55 Hz velocity, the current delayed 3 frames, and
  G_A = 9.025e6.
- Logged as `tau_env` (brief sign) and `tau_env_N` (Kt 64.6 mN·m/A,
  r = 0.04 m). No feedback into control or the felt path.
- **Smoke-tested** on a physically consistent fake plant (i = a/G): free
  phase mean +0.5 mA, a +0.1 A motor-side push reads −100.01 mA.

**Run:** `TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1 timeout -s INT 70
taskset -c 2,3 python3 teleop_test.py`, from `/dev/shm/c2/free` (RAM
logging), with the user swinging vigorously.

- **Logs:** `logs/C2_free.csv` + `_frames`.
- 0 missing frames; loop p95 2.06 ms, max 23.2 ms.
- Follower covered **3,812 counts (343°)**; |vel_a| rms 6,249 counts/s;
  raw current rms 137.5 mA.

**Result:** live tau_env rms **127.8 mA**, p99 406 mA. An offline replay
matches (127.5 mA). Gate A requires ≤ 25 mA, so **FAIL**.

**Diagnosis:**

- **Contact is minor:** stalls with force cover only 2.0 % of the time.
- **The residual grows with acceleration:** 92 → 186 → 360 mA. The
  leftover-inertia factor is k = +6.4, so far more current per
  acceleration than G_A predicts.
- **Effective follower plant gain on this run** (centred 15-frame LS for
  v and a, moving frames, with Coulomb and viscous terms):
  **G_eff ≈ 790,000 counts/s² per A**, consistent from 786k to 797k for
  current delays of 0–4 ms. Coulomb about 30 mA; R² only 0.72.
  - That is **about 11× below A2's 9.025e6**, measured on the original,
    longer and apparently lighter link.
  - The arm was replaced with a shorter (about 4 cm) one for B7 and the
    C data. That it has about 11× the inertia is **inferred**; the user
    has not been asked about the arm's mass yet.
  - The C1 static holds (a = 0) and the gentle C1 free run were
    insensitive to G, which is why C1 did not see this.

**Replays with corrected G** (`c1_eval` estimator):

| G | C2 free (vigorous) | C1 free (gentle) |
|---|---|---|
| 9.025e6 | 127.5 mA (p99 405) | 10.9 |
| **0.79e6** | **78.4 mA** (p99 195) | 11.1 |
| 0.70e6 / 0.90e6 | 77.9 / 80.9 | 11.2 / 11.0 |

**What does not explain the remaining ~78 mA** (fit on the first half,
tested on the second half):

- **Friction model** (Coulomb + low-speed + viscous): 86.8 → 83.0 mA.
  The residual is **98 mA even at \|a\| < 1e4 counts/s²**.
- **1/rev "gravity" term** (73 mA fitted): 86.8 → 76–82 mA, but it
  makes C1 free *worse* (11 → 64 mA). C1 sat at rest at those angles with
  about 0 current, so it is **not gravity**. The angle pattern in C2's
  mean residual (+81 mA near raw 1,100, −102 mA near raw 2,600) is more
  likely direction-of-travel correlated with angle (inferred).

**Interpretation (inferred):** the follower and arm do not behave as one
rigid inertia. Compliance or a resonance between motor and arm (a
flexible coupling, a flexing arm, or the clamp) would give
current-without-acceleration and a low R². It is consistent with A4's
G-vs-frequency suspicion.

**Verdict: C2 FAIL** on free motion under vigorous swings. Static
accuracy is unaffected (G-independent). C2 run 2, the static holds, has
not been run yet.

**Proposed fix (needs the user):**

1. **Ask the user** what the new arm is: mass, material, how it is
   attached (rigid hub vs flexible coupling), and whether it flexes.
2. **Re-identify the follower plant on the current hardware** within its
   limited range.
   - A new host script: a small position hold (PD) around a mid angle
     plus a ±0.05 A current chirp, 1–50 Hz, about 20 s, travel ±50
     counts.
   - Fit G(f) = A(f)/I(f) from `iq_meas` and the LS acceleration.
   - This is the A4b chirp, adapted for limited travel.
3. **Then update the estimator** (G, and G(f) if needed: e.g. a
   low-pass on the acceleration path matching the measured roll-off),
   and rerun C2.
4. **Re-run A2 on this arm if possible.** The bang-bang test needs free
   rotation, so it may not fit the limited range.

---

## 2026-10-25: Follower arm identified (user): 22.5 cm stainless chopstick, 9 g, hot-glued to a shaft mount

**Inertia check (inferred from measurements):**

| | G (counts/s² per A) | α (rad/s² per A) | J = Kt / α |
|---|---|---|---|
| A2, old link | 9.025e6 | 14,137 | **4.6e-6 kg·m²** |
| C2, new arm | 0.79e6 | 1,241 | **5.2e-5 kg·m²** |
| Added by the arm | | | **about 4.7e-5 kg·m²** |

- A 9 g, 22.5 cm rod about its centre is m·L²/12 = 3.8e-5 kg·m².
- The measured value matches a pivot about 3 cm off the rod's centre:
  0.009·(L²/12 + d²) gives d ≈ 3.1 cm. Or the mount and glue add a
  little.
- So the arm is about 10× the rotor's inertia.

**Implications (inferred):**

- **Hot-glue joint compliance** and the rod's bending, combined with that
  inertia, give low resonances (a few to a few tens of Hz). That matches
  where A4 and C2 saw current that does not match acceleration
  (7–35 Hz), and C2's R² of 0.72.
- **Imbalance:** a pivot offset of about 3 cm gives gravity of about
  m·g·d ≈ 2.7 mN·m (≈ 40 mA) versus angle.
  - The earlier "not gravity" conclusion was too strong. The fitted 1/rev
    term (73 mA) had a phase contaminated by direction-of-travel
    correlation. C1's rest region (raw 2774–3052) may lie near the
    gravity null.
  - Measure gravity directly (quasi-static holds at several angles)
    before modelling it.
- **No benefit from the length:** contact is at about 4 cm, so the
  22.5 cm length adds inertia, imbalance and bending without improving
  force sensing.

**Recommendation (awaiting the user's decision):**

- Replace the arm with a short (about 5 cm), light (about 2 g), stiff
  arm. That adds about 2e-6 kg·m², about 1.4× the original total
  inertia instead of 11×.
- Fix it rigidly (set-screw or clamp hub, or epoxy; not hot glue), and
  roughly balance it.
- Then re-identify the plant with a limited-range chirp (new script),
  and rerun C2.
- **Alternative:** keep the chopstick but epoxy it, then identify and
  compensate G, gravity and resonance in the estimator. More work, and
  less robust.

---

## 2026-10-25: New 3D-printed follower arm (r = 75.7 mm) — plant re-identified; C2 free floor 23 mA (offline replay)

**Hardware (user):** a 3D-printed arm replaces the chopstick. Contact
radius is 75.7 mm from the centre of rotation, so force at the tip =
0.853 N/A (Kt 64.6 mN·m/A), and 0.119 mm of tip travel per count. The
arm could now turn freely: 13.8 rev in this run. Teleop's
`SHADOW_R_CONTACT_M` was updated to 0.0757.

**`plant_id.py`** (new; limited-range PD hold plus a 1–50 Hz current
chirp, then G(f)):

- Validated in simulation: the pipeline is exact without friction (7-frame
  kernel: 2.9–3.0e6 flat to 40 Hz against a true 3.0e6).
- **It fails with realistic stiction:** 20 mA of Coulomb friction → G −18 %;
  40 mA → off by 16×. The held shaft sticks.
- **Not used on hardware.** The limitation is documented in its docstring.

**Run:** C2 free motion on the new arm.
`TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1 timeout -s INT 70 taskset -c 2,3 python3 teleop_test.py`
from `/dev/shm/c2/free_arm3d`, with the user swinging vigorously.

- **Logs:** `logs/C2_free_arm3d.csv` + `_frames`.
- 0 missing frames; loop p95 2.06 ms, max 27.9 ms.
- Follower range 55,226 counts (13.8 rev); |v| rms 13,711 counts/s;
  |a| rms 3.8e5 counts/s²; raw current rms 105.9 mA.

**Plant fit** (centred 15-frame LS for v and a, moving frames |v| > 1,500
counts/s, 60 % of the time):

| Model | G_eff (counts/s² per A) | Coulomb | Viscous | Gravity | R² |
|---|---|---|---|---|---|
| a + friction | **3,847,031** | 24.4 mA | 3.3 mA/10k | — | **0.934** |
| + 1/rev gravity | 3,864,074 | 23.9 | 3.5 | **12.9 mA** | 0.938 |

| Arm | G | R² |
|---|---|---|
| A2 link | 9.0e6 | — |
| Chopstick (hot-glued) | 0.79e6 | 0.72 |
| **3D-printed** | **3.85e6** | **0.93** |

- Inertia about 2.3× the original link.
- Close to a rigid single inertia, and nearly balanced.

**Estimator replay** (`c1_eval.estimate`, 3 ms current alignment; friction
and gravity fitted on the first half, tested on the second):

| G | Estimate rms (whole run) | Held-out half raw → with friction + gravity |
|---|---|---|
| 9.025e6 (A2) | 56.3 mA (p99 171) | 52.0 → 48.0 mA |
| **3.85e6** | 37.0 mA (p99 79) | 30.2 → **23.0 mA (p99 63)** |

- **Held-out residual by |a|:** 21.8 (< 1e5), 24.5 (1e5–3e5), 21.2
  (3e5–1e6), 33.3 mA (> 1e6, 4.4 % of the time). **Inertia is now
  cancelled** (flat versus acceleration).
- **At rest** (v = 0, 18 % of the time): 18.7 mA. That is stiction
  holding part of the commanded current while stationary; it is
  indistinguishable from external torque without motion (inferred). It
  is the main barrier to the brief's 15 mA.

**Verdict:** with the new arm's model, the vigorous free floor is **23 mA
held-out: Gate A (≤ 25 mA) PASS; brief (< 15 mA) not met.** This is an
offline replay. The live shadow still uses A2's G (it read high).

**Next:**

1. A fitted follower model file (G, friction, gravity) loaded by teleop's
   shadow estimator.
2. C2 rerun live: free motion plus static holds on the rigid surface.
3. Later: a stiction-aware dead band in the felt path, so static
   friction is not felt as contact.

---

## 2026-10-25: Follower model file + live shadow uses it (C2 code)

**Code:**

- **`fit_follower_model.py <frames.csv> [--out]`** fits two things:
  - **G** by LS on moving frames;
  - **friction + gravity terms** on the estimator's own output: offset,
    Coulomb, low-speed, viscous, cos/sin θ.

  It saves JSON with a held-out check.
- **Fitted on `logs/C2_free_arm3d_frames.csv`:** G = 3,847,031 (R²
  0.934). Coeffs: offset −1.7, Coulomb +24.5, low-speed −26.9,
  viscous +2.3 per 10k, gravity cos −13.5, sin −2.4 mA. Held-out 30.2 →
  23.0 mA.
- **Files:** `logs/follower_model.json` (git-ignored), plus a **tracked
  copy, `follower_model_arm3d.json`**, next to the scripts.
- **`teleop_test.py`:** `TELEOP_FOLLOWER_MODEL=<json>` (with
  `TELEOP_SHADOW_TAU_ENV=1`). The shadow estimator uses the model's G and
  subtracts `fit_follower_model.model_terms()`. Unset leaves the C1
  estimator unchanged.

**Live-path check, offline:** the recorded `C2_free_arm3d` frames were
replayed in real time through teleop with a fake CAN interface (15 s
starting 20 s in).

| Path | tau_env rms | p99 |
|---|---|---|
| Live path | 35.4 mA | 182 mA |
| Offline estimator, same window, same cold start | 36.1 mA | 189 mA |

- **The live code matches.** That window is the most vigorous stretch
  (|v| rms 20,322 counts/s, about 5 rev/s, against 13,711 for the whole
  run). So the floor depends on motion intensity: about 23 mA over the
  whole held-out half, about 36 mA in the most violent 15 s.

**Next:** C2 live rerun with the model: free motion plus static holds on
the rigid surface, at r = 75.7 mm.

---

## 2026-10-25: C2 live with the fitted follower model — PASS

**Runs:** `TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1
TELEOP_FOLLOWER_MODEL=follower_model_arm3d.json`, from `/dev/shm/c2/...`
(RAM logging). The user was on the leader; the felt path was unchanged.

**Run 1, free oscillations, no intended contact.**
`logs/C2_live_free.csv` + `_frames`.

- Follower range 35,548 counts (8.9 rev), |vel_a| rms 11,931 counts/s.
- 0 missing frames; loop p95 2.09 ms, p99 2.25 ms, max 58.5 ms.

| Window (s) | \|vel_a\| rms | tau_env rms | p99 |
|---|---|---|---|
| 1–11 | 10,104 | 19.4 mA | 53 |
| 11–21 | 11,701 | 22.2 | 75 |
| **21–31** | 11,871 | **76.1** | **469** |
| 31–41 | 12,853 | 19.5 | 56 |
| 41–51 | 14,734 | 17.8 | 53 |
| 51–61 | 11,072 | 18.0 | 53 |
| 61–71 | 9,773 | 16.3 | 49 |

- **21–31 s was a contact,** not estimator error: all |tau_env| > 150 mA
  rows fall at t = 26.2–28.1 s, within raw angle 2,180–2,390, with the
  follower decelerating to a standstill. Peak −692 mA. The estimator
  flagged it correctly.
- **Excluding it: live tau_env rms 19.1 mA, p99 56 mA.**

**Run 2, static holds.** `logs/C2_live_holds.csv` + `_frames`.

- 0 missing frames; loop p95 2.09 ms, max 54.5 ms.
- 16 holds (≥ 0.5 s within 5 counts, |iq| > 50 mA) at **two contact
  positions**: raw about 990 (iq +0.27…+0.65 A) and raw about 1,620
  (iq −0.49…−0.69 A).
- Live tau_env vs measured current: **−1.1 … +5.0 %** (typically about
  +3 %). Tip force 0.22–0.61 N.
- The sign is correct in both directions: tau_env = −iq at standstill.

| Criterion | Live result | Threshold | Verdict |
|---|---|---|---|
| Free floor (vigorous, contact excluded) | 19.1 mA rms (p99 56) | Gate A ≤ 25 mA (brief < 15) | **PASS** (brief: not met) |
| Static accuracy | worst 5.0 % | ≤ 10 % | **PASS** |
| Loop p95 | 2.09 ms | ≤ 2.1 ms | **PASS** (margin 0.01 ms; max 54–58 ms) |

**Verdict: C2 PASS.**

**Notes:**

- The +3 % static bias is likely from the model's offset and gravity
  terms at those angles (inferred).
- Max loop_dt 54–58 ms with RAM logging is new compared with the
  12–28 ms of earlier RAM runs. Possibly the per-frame numpy work in
  `model_terms()` (inferred); watch it in C3.
- **Before C3:** the SD-stall fix must be in code; today relies on the
  run-from-`/dev/shm` workaround. The firmware's 20 ms host timeout now
  bounds the damage from a stall.

---

## 2026-10-25: C3 preparation (offline): RAM logging in code, felt-force switch, dead band

**Code (`teleop_test.py`, all flagged; defaults unchanged):**

- **`TELEOP_LOG_DIR=<dir>`** fixes the confirmed SD-write stalls. The CSVs
  (main + frames) are written to `<dir>` during the run, e.g.
  `/dev/shm/teleop`. At exit they and the plot are moved into `logs/`
  under the usual unique name. This replaces the run-from-`/dev/shm`
  workaround.
- **`TELEOP_FELT_SOURCE=tau_env`** (default `iq_a_filt`) sets the felt
  command `iq_b_cmd = clamp(K_F · deadband(tau_env) − KB_DAMP · v_b)`.
  - It requires `TELEOP_SHADOW_TAU_ENV=1` and `TELEOP_FOLLOWER_MODEL`.
  - `TELEOP_K_F` defaults to 0.25 and must be in [0, 1].
  - `TELEOP_FELT_DEADBAND_A` defaults to 0: a soft dead band,
    |x| ≤ DB → 0, else sign·(|x| − DB).
  - The 2.8 Hz `IQ_FILTER` is bypassed on this path (the estimator is
    20 Hz).

**Sign/scale check on recorded data** (`C2_live_holds`, `C2_live_free`):

- **Static holds** (1,202 rows): new felt term at K_F = 1 (`tau_env`) vs
  old (`FEEDBACK_GAIN·iq_a_filt`): **sign agreement 100 %, median ratio
  1.01.** So at K_F = 1 the static felt force equals the old path's.
- **Free motion, contact excluded:** old felt term rms 38 mA vs new 19 mA
  at K_F = 1, i.e. **4.8 mA at K_F = 0.25.**
- **Note:** the leader's own damping, KB_DAMP·v_b, is in iq_b_cmd for
  both paths and dominates under vigorous motion (2e-5 × 10,000 counts/s
  = 0.2 A). C3 will report the felt term and the damping separately.

**Smoke test** (recorded frames replayed through teleop via a fake CAN
interface, with all of: `TELEOP_FELT_SOURCE=tau_env`, `TELEOP_K_F=0.25`,
`TELEOP_FELT_DEADBAND_A=0.02`, `TELEOP_LOG_DIR=/dev/shm/teleop_smoke`):

- 500 Hz.
- 3 files moved to `logs/`; the RAM dir was left empty.
- iq_b_cmd equals 0.25·deadband(tau_env) − KB_DAMP·vel_b to within
  0.06 mA (log rounding).

**Status of the SD-stall issue:** fixed in code, behind
`TELEOP_LOG_DIR`. Confirm in the live C3 runs (max loop_dt).

---

## 2026-10-25: C3 step 1, felt force = 0.25 · tau_env (dead band 20 mA)

**Run:** `TELEOP_FRAME_QUEUE=1 TELEOP_SHADOW_TAU_ENV=1
TELEOP_FOLLOWER_MODEL=follower_model_arm3d.json TELEOP_FELT_SOURCE=tau_env
TELEOP_K_F=0.25 TELEOP_FELT_DEADBAND_A=0.02 TELEOP_LOG_DIR=/dev/shm/teleop_c3
timeout -s INT 55 taskset -c 2,3 python3 teleop_test.py`. The user did
free motion, then presses, then taps.

- **Logs:** `logs/C3_kf025.csv` + `_frames` + `.png` (moved from RAM by
  the new code).
- 0 missing frames; loop p95 2.09 ms, max 46.9 ms. No watchdog trip.

**User report:** *"no change in resistance between free spinning, the
taps, and pushing."*

**Measured (iq_b_cmd split into its terms):**

| | Free (≥ 0.3 s from contact) | Contact |
|---|---|---|
| Felt contact term 0.25·deadband(tau_env) | **3.8 mA rms** | up to **137 mA** |
| Leader damping −KB_DAMP·v_b (KB_DAMP = 2e-5) | **79.7 mA rms** | up to about 400 mA (iq_b_cmd max 424 mA) |
| Total iq_b_cmd | 80.6 mA rms | — |

- Leader stiction is about 60 mA breakaway (spin test) on top of that.
- **Interpretation:** the new path removed the follower's own
  inertia/friction from the felt force (3.8 mA in free motion). But at
  K_F = 0.25 the contact term (≤ 137 mA ≈ 9 mN·m) is the same order as
  the leader's damping and stiction, which are present everywhere. So
  contact is masked, consistent with the user's report.

**C3 criteria at this step:**

| Criterion | Result | Verdict |
|---|---|---|
| Free felt current ≤ 30 % of A1 (≤ 34.7 mA) | 80.6 mA (79.7 of it leader damping) | **FAIL** |
| Ringing: largest 5–40 Hz PSD bin vs 0.3–3 Hz mean | −15.5 dB (at 5.4 Hz) | PASS (≤ +10 dB) |
| Watchdog trips | none | PASS |
| Tap rise ≤ 30 ms | not evaluated yet | — |

**Next:** step 2 at K_F = 0.5 (one gain at a time), then 1.0. If contact
is still masked, reduce KB_DAMP as its own step and watch for ringing.

---

## 2026-10-25: C3 step 2, K_F = 0.5 (dead band 20 mA)

**Run:** same as step 1 with `TELEOP_K_F=0.5`.

- **Logs:** `logs/C3_kf050.csv` + `_frames` + `.png`.
- 0 missing frames; loop p95 2.08 ms, max 50.2 ms. No watchdog trip.

| | Free | Contact |
|---|---|---|
| Felt contact term 0.5·deadband(tau_env) | 7.4 mA rms | **145 mA rms, max 279 mA** |
| Leader damping −KB_DAMP·v_b | 104.7 mA rms | 69 mA rms |
| Total iq_b_cmd (free) | 107.1 mA rms | — |

- Contact was 30 % of the time and clearly free only 8 %, so the free
  numbers rest on little data.
- **Ringing:** largest 5–40 Hz PSD bin −18.5 dB vs 0.3–3 Hz (at 5.1 Hz),
  so **PASS**.
- In contact, the felt term is now about 2× the leader damping (it was
  about 1× at K_F = 0.25).
- **User feel report:** "I did not feel contact."

---

## 2026-10-25: C3 step 3, K_F = 1.0 (dead band 20 mA)

**Run:** same as steps 1–2 with `TELEOP_K_F=1.0`. Step 2's user report
was recorded first: "I did not feel contact."

- **Logs:** `logs/C3_kf100.csv` + `_frames` + `.png`.
- 0 missing frames; loop p95 2.09 ms, max 56.8 ms. No watchdog trip.

| | Free | Contact |
|---|---|---|
| Felt contact term 1.0·deadband(tau_env) | 13.8 mA rms | **531 mA rms**, max 1,482 mA (iq_b_cmd clamped at 1.0 A) |
| Leader damping | 75.8 mA rms | 67 mA rms |
| Total iq_b_cmd (free) | 79.8 mA rms | — |

- Contact was 33 % of the time; clearly free 17 %.
- The follower command also hit its 1.0 A clamp during presses.
- **Contact with the leader nearly still** (|v_b| < 300, 4 % of the time):
  felt term rms 547 mA. It does not fade at this gain.
- **Ringing:** −17.6 dB (largest 5–40 Hz bin at 5.9 Hz), **PASS**.
- In contact, the felt term is about 8× the leader damping, about
  34 mN·m rms, up to the 1.0 A ceiling (about 65 mN·m).
- **User feel report:** "the contact is a bit better."

---

## 2026-10-25: C3 tap-rise (K_F 1.0) and damping step 1 (KB_DAMP 1e-5)

**Tap-rise on `C3_kf100`:** 38 contact onsets (after ≥ 40 ms quiet, peak
> 150 mA). Felt 10–90 % rise median **35.9 ms** (2–190 ms); peaks median
640 mA, spread 39 %. Target ≤ 30 ms, so a **marginal FAIL**. The onsets
include slow presses as well as taps; a taps-only run is needed. The old
path's A1 tap rise was 66 ms.

**Code:**

- `TELEOP_KB_DAMP` env override for the leader damping (default 2e-5
  unchanged; teleop prints the value used).
- `fit_follower_model.model_terms()` gets a plain-math scalar fast path
  (4.2 µs/call). It is identical to the array path at test points
  (0.00 nA difference).
  - **Why:** loop p95 crept to 2.11 ms with numpy-on-scalars every frame.

**Run:** `TELEOP_K_F=1.0 TELEOP_KB_DAMP=1e-5` (other flags as in C3
steps 1–3).

- **Logs:** `logs/C3_kf100_kb1e-5.csv` + `_frames` + `.png`.
- 0 missing frames. **Loop p95 2.11 ms (marginal FAIL)**, max 61.5 ms,
  before the fast path above.

| | Free | Contact |
|---|---|---|
| Felt term | 8.6 mA rms | **523 mA rms** |
| Leader damping | 60.9 mA rms (\|v_b\| rms 6,086: faster motion than before) | **35 mA** (was 67 at 2e-5) |
| Total iq_b_cmd (free) | 62.1 mA rms (was 79.8) | — |

- **Ringing:** −18.8 dB (5.6 Hz), **PASS**.
- Rise median 85 ms over 36 onsets. Not comparable: it depends on how
  contact was made.
- **User feel report:** "it was lighter, the contact was about the same."

---

## 2026-10-25: C3 damping step 2 (KB_DAMP 0.5e-5), K_F 1.0

**Run:** `TELEOP_K_F=1.0 TELEOP_KB_DAMP=0.5e-5`, other flags as before.
The `model_terms` scalar fast path is in.

- **Logs:** `logs/C3_kf100_kb5e-6.csv` + `_frames` + `.png`.
- 0 missing frames. **Loop p95 2.08 ms (PASS), max 27.5 ms**, against
  61 ms before the fast path.

| | Free | Contact |
|---|---|---|
| Felt term | 12.5 mA rms | **508 mA rms** |
| Leader damping | 32.6 mA rms (\|v_b\| rms 6,510) | **19 mA**, so contact about 27× damping |
| **Total iq_b_cmd (free)** | **35.9 mA rms** vs target ≤ 34.7 mA: just above | — |

- Contact 41 %, free only 7 %, so the free figure rests on little data.
- **Ringing:** −17.9 dB (5.9 Hz), **PASS**.

**C3 progression of the free felt current:**

| KB_DAMP | Free felt current |
|---|---|
| 2e-5 | 79.8 mA |
| 1e-5 | 62.1 mA |
| 0.5e-5 | **35.9 mA** |

Stable at every step.

- **User feel report:** "it felt better, but the contact force topped out
  quickly."

**Current ceiling check** (measured ÷ commanded, C3 logs):

| \|cmd\| | Leader (B) | Follower (A) |
|---|---|---|
| 0.2–0.6 A | 0.99–1.00 | 0.99–1.02 |
| 0.6–0.8 A | 0.98 | 0.96–0.98 |
| 0.8–0.95 A | 0.94–0.97 (about 0.79 A delivered) | 0.89 (about 0.76 A) |
| ≥ 0.95 A | 0.66 (0.66 A) | 0.77–0.82 (about 0.76–0.80 A) |

- **Delivered current saturates at about 0.8 A,** consistent with B7
  (0.675 A measured at 0.8 A commanded at stall).
- Felt torque ceiling ≈ 0.8 × 64.6 ≈ **52 mN·m**. The follower's push is
  capped at about 0.76 A ≈ 0.65 N at its 75.7 mm tip.
- At K_F = 1.0, contact already reaches the ceiling. **More gain only
  reaches it sooner.** More felt force needs mechanics: a shorter lever
  at the leader grip, or a stronger motor or supply.
- Whether the ceiling is the user's firmware limit or current-loop
  voltage headroom is not determined (inferred either way).

---

## 2026-10-25: C3 damping step 3 (KB_DAMP 0.25e-5) — user felt oscillations; stop per the brief

**Run:** `TELEOP_K_F=1.0 TELEOP_KB_DAMP=0.25e-5`, other flags as before.

- **Logs:** `logs/C3_kf100_kb2.5e-6.csv` + `_frames` + `.png`.
- 0 missing frames; loop p95 2.07 ms, max 28.6 ms.

| | Free | Contact |
|---|---|---|
| Felt term | 6.5 mA rms | 429 mA rms |
| Leader damping | 12.8 mA rms | 9 mA rms |
| **Total iq_b_cmd (free)** | **14.6 mA rms: PASS** vs ≤ 34.7 mA | — |

- Contact 25 %, free 33 % of the time.
- **Whole-run ringing metric:** −16.9 dB, which does *not* flag it.
- **User feel report:** "felt good, some oscillations."

**Windowed search** (0.5 s windows, the 5–60 Hz part of iq_b_cmd):

| KB_DAMP | Bursts |
|---|---|
| **0.25e-5** | **78 mA at 16 Hz at t ≈ 33 s (58 % in contact)**; 45 mA at 14 Hz (t 8 s); 30 mA at 32 Hz (in contact) |
| 0.5e-5 | 65 mA at 28 Hz (in contact), 77 mA at 8 Hz (t 22 s, mostly free); not felt by the user |

- **Interpretation (inferred):** contact chatter. With little leader
  damping, the arm bounces against the object instead of settling. The
  whole-run PSD averages it away.
- **For future runs:** judge ringing with the windowed metric (burst
  amplitude in contact), not the whole-run PSD.

**Verdict: STOP lowering KB_DAMP** (brief: stop at the first sign of
ringing).

**C3 operating point** (stable, user "felt better"): K_F = 1.0,
KB_DAMP = 0.5e-5, dead band 20 mA.

| Criterion | At the operating point |
|---|---|
| Free felt current | 35.9 mA vs ≤ 34.7: essentially at target; marginal FAIL by 1.2 mA |
| Stability | no ringing felt or flagged |
| Contact | about 27× damping, but limited by the about 0.8 A current ceiling |
| Tap rise | about 36 ms (K_F 1.0, KB 2e-5) on mixed presses and taps; a taps-only run is needed |

**Options:**

- Try KB_DAMP ≈ 0.35e-5 to pass the free-current target with margin to
  the oscillation (optional).
- A taps-only run for the rise time.
- Leader stiction compensation.
- Then C4 (anchor).

---

## Phase A summary (2026-10-20)

| Step | Result | Key number |
|---|---|---|
| A1 baseline | recorded | free RMS iq_b_cmd 0.1156 A; felt force = −tau_h (echo); tap rise 66 ms (hand clamp, caveats) |
| A2 plant gain | **PASS** | G_A 9.025e6 (±2.7 %), G_B 9.672e6 (±2.0 %), +18 / +22 % vs old; not applied (user) |
| A3 noise floor | **PASS** | drift 3.1 mA / 10 min; raw std 0.030–0.043 A at one angle |
| A4 ripple | **FAIL** (brief model 20–43 %) | locked c(θ) 12/24/28/36 + i·g 14/42/84; 7/rev loop-driven; friction 0.04 A, about 100× the brief |
| A5 command path | **PASS** (still segments, 1.7 dB) | step 50 % at 1.47 ms |
| A6 timing | **PASS** on p95 (2.03–2.04 ms) | but host stalls up to 1,045 ms (4 in about 220 s) |

**Open issues carried into Phase B:**

1. Simultaneous ESC resets.
2. Host loop stalls.
3. Leader and follower stiction (0.05–0.09 A breakaway).
4. The FF echo in the felt path.
5. G vs frequency (chirp test).
6. Angle-dependent Iq noise (×1.3–2).

---

## Handoff: how to continue (written 2026-10-20, end of Phase A)

This section is for the next Claude session (or person) picking this up
cold. The original plan is the user's "sensorless force-feedback
improvement plan" brief, Phases A–D. Its steps, pass thresholds and
rules are restated below where they matter. Everything here was checked
against the code and the logs above.

### 0. Resume here (state at the end of the 2026-10-20 session)

**Where things stand:** Phase A done; B1, B2 and B3 pass; B4 waits for
the user's dev machine; B5 dropped (user); B6 failed offline; B7 is ready
for the bench.

- The user paused the work and asked for everything to be in this file.
- Phase C has **not** started. Do not start C3 (closed-loop felt force)
  until the host stalls are fixed.

**What is waiting on whom** (do them in this order unless the user
chooses otherwise):

| # | Item | Needs | How |
|---|---|---|---|
| 1 | Host stall fix: RT priority for the RX thread and the loop | **RT done and verified 2026-10-22. The remaining main-loop stalls (likely SD writes) are a potential issue, DEFERRED by the user. Revisit before C3 (closed-loop felt force).** | Create `/etc/security/limits.d/99-rtprio.conf` with `jayrpi4 - rtprio 50` (sudo), then restart Claude Code / log in again. Check: `ulimit -r` shows 50, and `python3 -c "import os; os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))"` succeeds. Then re-measure max loop_dt over 3 × 60 s teleop runs (`TELEOP_FRAME_QUEUE=1`). If stalls persist, test logging to `/dev/shm`. |
| 2 | B4 + timeout firmware | **DONE 2026-10-25**, flashed; B4 PASS (see the entry) | Section "Firmware changes" near the top. After flashing: `bus_check.py` (shows RESET) → `zero_iq_check.py` → the B4 checks. |
| 3 | B6 fix: A4b G-vs-frequency chirp | **user OK**, follower free, about 25 s | Not written yet. Add `--chirp f0,f1,amp` to `spin_test.py` (design in section 4), run ±4000, and fit H(f) = A(f)/I(f) with I from `iq_meas`. |
| 4 | B6 fix: held-out runs | **user OK**, follower free, about 25 s | `spin_test.py --node A --speed ±4000 --time 10 --iq-max 0.3 --ki 2e-4` (baseline gains). Then refit in `b6_ripple.py`: add these to HELDOUT, use per-run friction (variant C or A in the B6 entry), and model G(f) if A4b shows a roll-off. |
| 5 | B7 force calibration | **user at the bench** with the kitchen scale | Link on the scale on a rigid support, tared at 0 A. `python3 b7_force_cal.py --sign +1` (if level 1 aborts on travel, delete `logs/B7_cal.csv` and use `--sign -1`). The user reports 7 readings in grams; then `python3 b7_force_cal.py --fit logs/B7_cal.csv --grams g1,…,g7`. |
| 6 | Optional live trial of the B2/B3 flags | **user OK**, user on the leader | `TELEOP_FRAME_QUEUE=1 TELEOP_VEL_ESTIMATOR=ab55 TELEOP_OBSERVER=class TELEOP_OBS_G=A2 TELEOP_OBS_DT=mcu timeout -s INT 75 taskset -c 2,3 python3 teleop_test.py`. Compare against `logs/teleop_KVs3e-05_FB-1_8.csv` (same protocol, flags off except the queue). |
| 7 | Then Phase C | — | C1 offline first (section 6). Read its sign and friction warnings. |

**Feature flags in `teleop_test.py`.** All defaults reproduce the
original A1 behaviour exactly; overrides are environment variables.

| Flag (env var) | Default | Options | Step |
|---|---|---|---|
| `USE_FRAME_QUEUE` (`TELEOP_FRAME_QUEUE=1`) | off | on: drain and log every frame to `<log>_frames.csv` | B1 |
| `VEL_ESTIMATOR` (`TELEOP_VEL_ESTIMATOR`) | `legacy` | `ab55` (needs the frame queue) | B2 |
| `OBSERVER_IMPL` (`TELEOP_OBSERVER`) | `inline` | `class` | B3 |
| `OBS_G_SOURCE` (`TELEOP_OBS_G`) | `old` (7,915,000) | `A2` (9,672,000) | B3 |
| `OBS_DT_SOURCE` (`TELEOP_OBS_DT`) | `nominal` | `mcu` | B3 |

**Changes to shared code:**

- `can_interface.py`: `get_telemetry()` aliases, plus
  `MotorCANInterface(pair_dbg=True)`. Default unchanged; `False` releases
  each frame on arrival.

**Data that matters for the next steps** (all in `ESC_based/software/logs/`
on the Pi, **not in git**):

| Purpose | Logs |
|---|---|
| A1 baselines | `teleop_KVs3e-05_FB-1_2.csv` (free), `_5` (hand presses), `_6` (foam), `_7` (taps) |
| B1 | `teleop_KVs3e-05_FB-1_8.csv` + `_8_frames.csv` (every frame, both motors) |
| A2 | `plant_gain.csv`, `plant_gain_2…5.csv` |
| A3 | `A3_clamped.csv`, `A3_free.csv`, `A3_rest_10min.csv` |
| A4 / B6 training | `spin_A_±4000_0.3A_kv1.5e-05_ki0.0002_coast3.csv`, `spin_A_±4000_0.45A_kv1.5e-05_ki0.0002.csv` (loaded) |
| A4 / B6 held out | `spin_A_±10000_0.3A_kv1.5e-05_ki0.0002_coast3.csv`, `spin_A_+4000_0.3A_kv7.5e-06_ki0.0001.csv` |
| A5 | `A5_hold.csv`, `A5_const.csv` (+ `_cmd`), `A5_stair.csv` (+ `_cmd`) |
| B6 model (first fit, FAIL) | `ripple_model_A.json` |

If the logs are ever lost, the numbers in this file are the record.
Re-collecting needs the same commands as each entry.

**Decisions already made by the user (don't re-ask):**

- Teleop's G constants stay "old" by default. A2 values are available
  via `TELEOP_OBS_G=A2`.
- The A4 extended ripple model was accepted.
- B5 (clamp) was dropped.
- "Table" in A1 means the hand clamping the follower.
- Loaded A4 runs use a plate pressed on the shaft.

**Session note, 2026-10-22:**

- The Pi was powered from a PC USB port: 13 under-voltage events in the
  first 9 min, `throttled=0x50000`.
- **Switch back to the proper 5 V / 3 A supply before any motor test.**
  Under-voltage coincided with ESC1 bus dropouts before and would distort
  loop-timing measurements.
- Both ESCs had been power-cycled (RESET in `bus_check.py`), so run
  `zero_iq_check.py` first.
- The firmware is not flashed yet (user, 2026-10-22).

**Hardware reminders:**

- Ask before every energizing command.
- After any ESC power cycle, run `zero_iq_check.py` (alignment) before
  teleop.
- Both ESCs have reset together during runs. If `bus_check.py` shows
  RESET, re-align, then consider asking the user about the ESC supply.

### 1. Start-of-session checklist

1. **Read** the TL;DR at the top, then this section. Skim the entry for
   any step you are about to touch.
2. **Do the non-energizing checks:**
   - `cd ESC_based/software && python3 bus_check.py`
   - It expects both nodes at TELEM 1000/s and DBG 10/s, 0 µs gaps, idle
     Iq |mean| < 1 A ("calibrated"), and `throttled=0x0`.
   - Also check `ip -details -statistics link show can0`: ERROR-ACTIVE,
     berr-counter 0/0.
3. **If a node shows "RESET"** (idle Iq about ±25 A), it has power-cycled.
   Its next START runs the 2 s alignment.
   - Ask the user, then run `python3 zero_iq_check.py`. It must print
     `encoder jump … (alignment counter load)` and `PASS`.
   - Only then run any other motor test.
4. **Ask the user before every command that energizes a motor.** They
   must confirm the shafts or links are clear and someone is present.
   - Never chain motor tests unattended. One approved sequence of runs
     is fine.
   - The Claude Code auto-mode classifier has blocked motor commands
     before; the user then has to allow them.
5. **Run control scripts as** `timeout -s INT <s> taskset -c 2,3 python3 -W ignore <script>`,
   in the background for long runs. Read the output afterwards.

### 2. Rules from the brief (still binding)

- **No firmware edits on the Pi, and never regenerate MCSDK** from Motor
  Control Workbench.
  - The `.ioc` still says PGA 8x, and regenerating would revert the 16x
    gain and the hand-written FDCAN code.
  - B4 (and the timeout change) are firmware steps for the user's dev machine. **Stop and
    tell the user** when you reach them.
- **Prototype offline against logged CSVs** before changing live control
  code.
  - Every behaviour change goes behind a flag or config switch, with the
    old behaviour as the default.
- **Host code must be port-friendly:** per-instance estimator and
  observer state (classes), and dt from timestamps (`mcu_time_us`), never
  a nominal constant.
- **Commit only when the user asks.** Branch first if on main.
- **Report** what ran, the numbers against the pass threshold, PASS/FAIL,
  the log paths, and what is inferred.
  - **On FAIL, stop and propose a fix; don't move on** unless the user
    says so.
- **Log every step in this file**, as a new `## <date>: <step>` entry
  above "Phase A summary"/"Handoff", in the same format as the existing
  entries.

**User decisions so far:**

- Teleop's G constants stay unchanged. Apply the A2 values via B3's
  observer config.
- The A4 extended ripple model was accepted, and the user moved on to A5.
- Loaded A4 runs: the user presses a plate on the follower shaft.
- "Table" contacts in A1 were the user's hand clamping the follower.

### 3. Facts and constants to use

| Quantity | Value | Source |
|---|---|---|
| Plant gain G_A (follower) | 9,025,000 counts/s² per A (SE 2.7 %) | A2 |
| Plant gain G_B (leader) | 9,672,000 counts/s² per A (SE 2.0 %) | A2 |
| G constants in teleop (old, pre-direct-drive) | G_A 7,622,000 (unused), G_B 7,915,000 | `teleop_test.py:350-351` |
| Breakaway / running friction, both shafts | 0.05–0.09 A / about 0.04 A | spin tests, A4 |
| Stick-slip regime | ≤ about 2,000 counts/s | A4 |
| Raw Iq noise (single sample, rest) | 0.030–0.043 A std, angle-dependent ×1.3–2 | A3 |
| Iq drift | 3.1 mA per 10 min | A3 |
| Command step, host send → TELEM, 50 % | 1.47 ms (97 % by 3 ms) | A5 |
| Loop timing (teleop, 500 Hz) | p95 2.03–2.04 ms, but stalls up to 1,045 ms | A1, A6 |
| KV stability estimate | about 3.5–4.2e-5 A/(count/s) at T = 4.5 ms (teleop KV_SLOPE = 3e-5) | A2 |
| Encoder | 4000 counts/rev; 7 pole pairs (alignment loads raw 142 = 90° electrical) | P1 |
| At rest | encoder dithers ±1 count: 3,356 changes in 600,000 frames | A3 |

**Ripple model r(θ, i) = Σ c_n·e^{inθ} + i·Σ g_n·e^{inθ}** (A4 loaded
fit, 4,000 counts/s; amplitudes in mA, phases in deg):

| n/rev | c (mA @ deg) | g (mA/A @ deg) | Use |
|---|---|---|---|
| 12 | 21.6 @ −172 | 14.2 @ −72 | c |
| 24 | 8.5 @ −101 | — | c |
| 28 | 13.4 @ 41 | — | c |
| 36 | 9.2 @ −155 | — | c |
| 14 | 8.3 @ 96 | 24.1 @ 147 | c + g |
| 42 | 2.6 @ 112 | 20.7 @ −74 | c + g |
| 84 | 6.0 @ −75 | 34.2 @ 22 | c + g |
| 7 | — | — | **do not tabulate** (loop-driven) |
| 1 | — | — | excluded (plate-induced in loaded runs) |

- **Phase convention:** `analyze_ripple.harmonics()` is `rfft` over 500
  bins. Bin k covers raw counts [8k, 8k + 8), so the phase reference is
  the bin **start**, and the average sits half a bin later: n·180/500
  degrees, which is 30° at 84/rev.
- **Do not hand-copy these phases into code.** Regenerate the
  coefficients programmatically from the logs, ideally with a
  sample-level least-squares fit on `(enc % 4000, iq, acc)`.

**Telemetry and API facts:**

- **`drain_records()`** (ESC_based `can_interface.py`) returns every
  TELEM frame with `bus_time`, `mcu_time_us`, `enc`, `iq_meas`, and DBG
  fields when paired.
  - In direct mode, DBG is only 10 Hz, so each record is held until the
    *next* TELEM arrives: +1 ms latency (inferred from code).
- **Teleop CSVs lack `mcu_time_us`.** Spin, coupling, A3 and A5 logs
  have it.

### 4. Pending Phase A follow-ups (optional; do G(f) before C1)

**A4b: G vs frequency, current chirp.**

- **Why:** the loop's current ripple at 7–17 Hz produced only 20–50 % of
  the acceleration that A2's G predicts. If G falls with frequency, C1's
  i − a/G will read the follower's own motion as environment torque.
- **How:** in `spin_test.py`, add `--chirp f0,f1,amp`. Superimpose
  amp·sin(2π f(t) t), log-swept 1→50 Hz over about 10 s with
  amp = 0.05 A, on the PI command at a constant +4,000 counts/s.
  - Run ±4,000. That is about 25 s of motor time, follower free.
- **Analysis:** take acceleration from local quadratic fits of `enc` vs
  `mcu_time_us` (9–15 sample windows). Estimate the transfer function
  H(f) = A(f)/I(f) with I from `iq_meas` (sampled with the position),
  using Welch cross-spectra.
- **Expected and pass (proposed):** |H|/G within ±15 % from 1 to 30 Hz,
  or a documented roll-off curve for C1 to use.

**A3b: noise vs angle.** The A3 runs sampled one angle. Per-angle noise
can come from the spin logs (`analyze_ripple.py` prints the per-1/16-rev
std of iq_meas − iq_cmd), or from HOLD zero-gain runs at 4–8 hand-set
angles (30 s each).

**A5(a) rerun.** Only needed for a clean whole-run (a) vs (b) number. The
first (a) was disturbed from 2 to 10 s.

**A1 tap rerun.** Only if a cleaner tap number is wanted. Use geometry
where contact happens at one angle and from one side, and start each tap
from rest.

### 5. Phase B plans

**B1. Consume every frame (host).** — **DONE, PASS** (see the B1 entry).

- **State:** ESC_based `drain_records()` already queues every TELEM
  frame (B1 is mostly built). `teleop_test.py` still uses the
  latest-value `get_telemetry()` (lines 480–544), so the 500 Hz loop sees
  every other frame.
- **Do:**
  1. Add `USE_FRAME_QUEUE = False` to `teleop_test.py`. When True, each
     iteration drains both motors' records and feeds **all** of them
     (in order, with `mcu_time_us`) to the estimators. The control output
     is still computed once per iteration from the newest state.
  2. Keep the old path for rollback.
  3. Add an option to `can_interface.py` so records are released without
     waiting for DBG pairing, e.g. a constructor flag
     `pair_dbg=False`. This removes the +1 ms hold in direct mode.
  4. Log `mcu_time_us` in teleop's CSV.
- **Offline first:** unit-test the drain path against a recorded frame
  stream, e.g. `A5_const.csv`.
- **Live test:** 60 s free-motion teleop with the flag on; the user on
  the leader. Ask first.
- **Pass:**
  - mcu_time_us gaps 1000 ± 50 µs.
  - 0 missing frames in 60 s, excluding the first 10 ms after START (a
    known two-frame drop).
  - loop p95 ≤ 2.1 ms. **Also report max loop_dt** (stalls).

**B2. Velocity estimator class (host).** — **DONE, PASS offline** with AB 55 Hz, flagged and not run live (see the B2 entry).

- **Do:** create `estimators.py`.
  - `LsVelocity`: sliding least squares on `(mcu_time_us, pos)`, window
    4–8 ms. It exists in `spin_test.py`; move it and keep an import
    alias.
  - `AlphaBetaVelocity`: a tracker on the same inputs.
  - Both take dt from the timestamps.
- **"Exactly 0 at rest":** the encoder dithers ±1 count about 0.5 % of
  frames, so add a rest detector, e.g. zero output when the window's
  position range ≤ 1 count. A plain LS fit will not give exactly 0.
- **Offline comparison:** A1 teleop logs have no mcu time. Use:
  - `spin_A_±4000_*` and `±10000_*` for constant speed;
  - `A3_rest_10min.csv` for rest;
  - B1's new teleop logs for real motion.
- **Metrics:**
  - RMS noise at constant speed vs the current scheme (Δpos over ≥ 2 ms
    of kernel time, then EMA α = 0.7). Reimplement that scheme offline.
  - Lag against an offline zero-phase reference (centred LS over
    ±10 ms, or filtfilt).
  - Output at rest.
- **Pass:** ≥ 3× lower RMS at constant speed, lag < 3 ms, exactly 0 at
  rest. Then wire in the winner behind `VEL_ESTIMATOR = "legacy"`.

**B3. Leader observer class (host).** — **DONE, PASS offline**, flagged and not run live (see the B3 entry).

- **State:** teleop lines 632–640.
  - `obs_alpha` uses `dt = 1/CONTROL_RATE_HZ` (nominal), and
    `OBS_COMP = 1/(1 − π·fc/CONTROL_RATE_HZ)` (line 410) is also nominal.
  - Input is `vel_b_filt` and the measured `iq_b`.
- **Do:** `HandTorqueObserver(G, fc_hz)` in `estimators.py`, with
  per-instance state. `update(v, i, dt)` computes α and OBS_COMP from
  that sample's dt.
- **Offline:** replay `logs/teleop_KVs3e-05_FB-1_2.csv`. Inputs:
  `vel_b`, `iq_b_readback`, `loop_dt`; use the **old**
  G_B = 7,915,000. Compare with the logged `tau_h`.
  - The logged `loop_dt` is the measured period; nominal-dt replay is
    the reference for "reproduce".
- **Pass:** tau_h reproduced within 1 % RMS. Then add config
  `G_B_SOURCE = "old" | "A2"` (A2 = 9,672,000), default "old" until the
  user says otherwise.

**B4. [DEV MACHINE: stop and tell the user] TELEM boxcar mean.** — **DONE 2026-10-25, PASS** (see the B4 entry).

- **Firmware change:** in `can_driver.c` `SendTelemetry()`
  (`STM32CubeIDE/Application/User/can_driver.c:409-445`), behind a new
  `TELEM_IQ_MEAN` flag, send `IqTelem_GetMeanAndReset() * scaleParams_M1.current`
  instead of `iqd.q`.
  - The accumulator already exists (`Src/mc_tasks_foc.c:168`, raw s16A).
    It is drained and **discarded** at `can_driver.c:430` today.
- **After flashing:** the ESCs power-cycle, so the next START aligns. Run
  `bus_check.py`, then `zero_iq_check.py` (ask first), then:
  - A3: HOLD zero gains, 30 s free, at the **same angle** as `A3_free`
    (raw ≈ 3996) if possible.
  - A5(c): `a5_command_test.py --mode stair`, then `--analyze`.
- **Pass:**
  - Iq std ≥ 2× lower than before at the same angle (before: 0.030 A
    free, 0.034 A rest).
  - The 247 Hz peak is gone.
  - Step 50 % ≤ 1.47 + 0.6 = **2.07 ms**.

**B5. DROPPED (user decision 2026-10-20: firmware already has a max current). See 5b for the code-path note.** Original plan, kept for reference:

- **Firmware change:** in `couple_ctrl.c` `Couple_SetDirectIq()` (line
  359) or `RunDirect()` (line 257), clamp to ±`NOMINAL_CURRENT_A`. Then,
  on the host, set teleop's `IQ1_MAX_A` / `IQ2_MAX_A` to 0.8.
- **Test:** **shaft held or clamped** (0.8 A on a free follower spins it
  fast). Send SET_IQ 2.0 for about 150 ms, resending every 2 ms, and read
  DBG `iq_cmd`. DBG is only 10 Hz in direct mode, so hold ≥ 150 ms.
- **Pass:** DBG reports 800 mA.

**B6. Ripple compensation, estimate side only (host).** — **FAIL offline**. The fix needs A4b plus held-out runs (see the B6 entry).

- **Do:** a `RippleModel` class, built from the logs (see section 3),
  with c on {12, 14, 24, 28, 36, 42, 84} and g on {14, 42, 84}.
  - Subtract r(θ, i) inside the observers' current input. **Do not
    touch the motor command.**
- **Held-out data:** record new constant-speed free runs, e.g.
  `spin_test.py --node A --speed ±4000 --time 10` (ask first), not used
  for fitting.
- **Pass:** ≥ 60 % lower 14/rev amplitude in the observer output on the
  held-out logs.
  - Also report 12/rev, the largest term, which the brief didn't know
    about.

**B7. Static force calibration (user at bench).** — **SCRIPT READY** (`b7_force_cal.py`), waiting for a bench session.

- **Setup:** the follower presses on a kitchen scale on a rigid support,
  at 5–8 steady SET_IQ levels from 0.05 to 0.8 A.
  - Clamp to 0.8 A in the script (B5 was dropped).
- **New script `b7_force_cal.py`:** for each level, prompt the user,
  hold SET_IQ for about 3 s (resend every 2 ms), log every frame, and
  have the user enter the scale reading in grams. F = m·9.81 N.
- **Friction:** breakaway friction (0.05–0.09 A) does not matter in a
  static press, but don't wiggle the link during a reading.
- **Pass:** R² ≥ 0.99, slope within 15 % of 3.3 N/A (1 A ≈ 66 mN·m at a
  2 cm link).
- **Keep this data separate from Phase D** (its own log names).

### 5b. Dev-machine firmware changes

These moved to the top-level **"Firmware changes"** section near the top
of this file, which has the code, the rationale and the after-flash
checks.

### 6. Phase C plans

**C1. Offline follower observer.**

- **Do:** `ExternalTorqueObserver(G_A, fc, ripple)`:
  tau_env = LPF_w(i_f − a_f/G_A − r(θ, i_f)), in the derivative-free form
  (as the leader observer), in amps. Input is every frame (B1), with dt
  from mcu time.
- **Sign check required.** The brief says "positive = environment pushes
  the follower toward +counts". But with J·a = Kt·i + τ_ext, a push
  toward +counts makes a/G > i, so i − a/G is **negative**.
  - Verify with a hand push of known direction and fix the sign
    explicitly.
- **Friction:** the brief says "no friction term", but running friction
  is about 0.04 A. During free motion it will appear in tau_env as
  ±0.04 A, against a target RMS of < 0.015 A.
  - Expect C1's free-space pass to **fail without a friction model**
    (Coulomb plus a Stribeck-like low-speed term; inferred).
  - Propose that to the user rather than silently adding it.
- **G(f):** run A4b first if possible.
- **Data:** new free, hand-clamp and foam logs recorded after B1–B6, with
  every frame and mcu time.
- **Pass:** free-space RMS < 0.015 A; static contact within 10 % of B7.

**C2. Shadow mode.** Compute and log tau_env live in teleop behind
`SHADOW_TAU_ENV = True`; the felt path is unchanged.

- **Pass:** C1's metrics hold live, and loop p95 ≤ 2.1 ms (also report
  max).

**C3. Switch the felt force.**

- **Do:** behind `FELT_SOURCE = "iq_a_filt" | "tau_env"`, set
  iq_b_cmd = −K_F·tau_env − KB_DAMP·v_b, and replace the 2.8 Hz
  `IQ_FILTER_ALPHA` with the observer's 20 Hz.
  - Ramp K_F through 0.25 → 0.5 → 1.0 with FF_GAIN fixed, one gain at a
    time, a short run each. The user is on the leader; ask first.
- **Loop-gain note:** the FF path (FF_GAIN·tau_h) and the felt path
  close a loop through the hand, with measured gain ≈ 1 today. Watch for
  ringing as K_F rises, and stop at the first sign.
- **Pass:**
  - Free RMS felt current ≤ 30 % of 0.1156, i.e. **≤ 0.0347 A**.
  - No 5–40 Hz peak more than 10 dB above the 0.3–3 Hz content.
  - No watchdog trips.
  - Tap rise ≤ 30 ms. Use the clamp detector from A1 run 4's
    re-analysis; port it into `analyze_teleop_baseline.py` first.

**C4. Bounded leaky anchor** (follower command only, never the felt
path), as in the brief.

- e = (d_b − d_a) − offset.
- i_anchor = clamp(K_a·deadband(e, D), ±I_a_max), applied only while
  |tau_env| < gate.
- The offset follows (d_b − d_a) with tau_leak. During contact, freeze
  the anchor and let only the leak run.
- **Starting values:** D = 20–50 counts, I_a_max ≤ 0.015 A,
  tau_leak = 2–5 s, gate = 3× the free-space RMS of tau_env.
- **Note:** I_a_max (0.015 A) is below the 0.05–0.09 A breakaway, so the
  anchor alone cannot move a stuck follower.
- **Pass:** |d_b − d_a| < D + 50 counts over 60 s of free motion; < 0.015 A
  change in felt force during a 5 s static press.

### 7. Phase D plans (user at bench; Claude analyses)

- **D1 force accuracy:** fresh presses at ≥ 6 levels from 0.2 N up to the
  ceiling, against the scale. Pass: error ≤ max(0.05 N, 10 %); leader
  output within 15 % of the environment force.
- **D2 stiffness:** foam and balloon on the scale. Compare k from scale
  force vs encoder displacement (about 0.031 mm/count) with k from
  tau_env. Pass: within 20 % for soft samples; rigid ≥ 5× the stiffest
  foam.
- **D3 taps:** 20 taps on a rigid surface. Pass: felt rise ≤ 30 ms;
  peaks repeatable within 20 %. This needs a real rigid stop at a fixed
  angle; the hand clamp is not rigid.
- **D4 blind trials:** follower hidden, noise-masking headphones, rigs
  mechanically isolated. Randomised order, ≥ 20 trials per pair, plus a
  control (force path off, or the old path).
  - Pass: rigid vs soft ≥ 17/20 correct, with the control near chance.
  - Report a psychometric curve for the intermediate foams.
  - Needs a small script: randomiser plus a per-trial answer log.

### 8. Open issues and suggested investigations

1. **Simultaneous ESC resets.** Seen once mid-run at about 0.2 A, and
   once between runs. Both MCUs rebooted in the same millisecond
   (encoder raw 0, uncalibrated Iq).
   - **Suspect:** the shared VBUS supply or wiring (inferred). Ask the
     user about the supply type and current limit.
   - VBUS is not on CAN today. Adding it to DBG would be a dev-machine
     firmware change.
   - `bus_check.py` and the scripts' reset prechecks catch resets
     between runs.
2. **Host loop stalls up to 2 s (B1 run: 2,082 ms). POTENTIAL ISSUE, DEFERRED by the user (2026-10-22).**
   - The RX thread is fixed by RT priority.
   - The main loop still stalled for up to 1,034 ms when logging to SD,
     and not when logging to RAM. **Cause CONFIRMED 2026-10-22 by
     `sd_stall_test.py`** (no motors): SD write calls blocked for up to
     1,641 ms; RAM max 5.4 ms.
   - Revisit before C3: a stall during closed-loop felt force holds the
     last command up to the 200 ms firmware timeout. During a stall the
   last SET_IQ stays applied for 200 ms. In B1, −0.07 A spun the free
   follower 1.8 rev.
   - Proposed firmware mitigation (dev machine): lower
     `COUPLE_DIRECT_TIMEOUT_US` from 200 ms to about 20 ms.
   - **First fix (applied 2026-10-22, verify after reboot):** let Python
     get SCHED_FIFO; it used to fail silently (`ulimit -r` = 0). See the
     "Host real-time priority" entry. Options, which are system changes and
     need the user's approval:
     - an `rtprio` entry in `/etc/security/limits.d/`;
     - `sudo setcap cap_sys_nice+ep` on the venv's Python;
     - running under `sudo chrt`.
   - Then re-measure max loop_dt over several 60 s runs.
   - **Second suspect:** SD-card writes. Test by logging to `/dev/shm`.
3. **Stiction 0.05–0.09 A on both shafts.** Sub-breakaway reflected
   forces on the leader may be lost (inferred). Consider friction
   compensation in the leader observer (tau_h currently reads stiction
   as hand torque).
4. **G vs frequency.** See A4b.
5. **Angle-dependent Iq noise ×1.3–2.** See A3b. B4 may reduce it.
7. **Latched motor-control faults are invisible over CAN** (2026-10-25).
   - START is ignored after any fault until it is acknowledged, and
     nothing acknowledges it. The DBG frame has no fault code.
   - Proposed (dev machine): acknowledge on START, and report the fault
     code in DBG.
   - Workaround: power-cycle the ESCs.
6. **The FF echo.** By construction, tau_h ≈ −LPF(i_b) while the leader
   is held. C3 replaces the felt source; watch the hand-closed loop gain.

### 9. Timeline to the goal (drafted 2026-10-22)

Effort is in focused work, not calendar days. "Bench" means the user at
the rig. "Claude" means offline analysis or code with no motors. The
order is chosen so each gate is met before more effort goes in.

**Fixtures to prepare before session 3** (user, one-off):

- A **rigid stop** the follower meets at one angle, from one side, e.g. a
  screwed-down block.
- **Foam samples** of 2–3 stiffnesses, plus a balloon, mountable at that
  angle.
- The **kitchen scale** on a rigid support (B7, D1, D2).

| # | Session | Who | Bench time | Claude time | Steps | Gate / exit criterion |
|---|---|---|---|---|---|---|
| 1 | Firmware | user (dev machine) + bench | ~15 min | ~30 min | B4 averaged Iq + 20 ms timeout; flash both boards; re-align; A3/A5(c) rerun | B4 pass: Iq std ≥ 2× lower, 247 Hz gone, step ≤ 2.07 ms |
| 2 | Calibration | bench (scale) | ~20 min | ~30 min | B7 force calibration | R² ≥ 0.99, slope within 15 % of 3.3 N/A, i.e. N per A known |
| 3 | Contact data | bench (fixtures) | ~30 min | ~1 h | Free motion, rigid-stop presses and taps, foam presses; every-frame logging (`TELEOP_FRAME_QUEUE=1`, logs to RAM) | Clean labelled data for C1 |
| 4 | C1 offline | Claude | — | ~2–4 h | Follower estimator with the 3 ms current/velocity alignment, friction, ripple (B6 refit on session-3 data); sign check | **Gate A:** free floor ≤ 25 mA rms (target 15), rigid press within 10 % of B7, contact ≥ 10× floor |
| 5 | C2 shadow | bench | ~20 min | ~1 h | Estimator live, logged only; nothing felt changes | C1 metrics hold live; loop p95 ≤ 2.1 ms |
| 6 | C3 felt force | bench (several short runs) | 30–60 min, possibly 2 sessions | ~2 h | Fix SD-write stalls first (deferred issue). Felt force = −K_F·tau_env, K_F 0.25 → 0.5 → 1.0, 20 Hz path | **Gate B:** stable at K_F ≥ 0.5 (no 5–40 Hz ringing, no watchdog trips); free felt RMS ≤ 0.035 A; tap rise ≤ 30 ms. **First point where rigid vs soft should feel different** — informal check by the user here |
| 7 | C4 anchor | bench | ~20 min | ~1 h | Bounded leaky position anchor | Drift < D + 50 counts over 60 s; < 15 mA felt change in a 5 s press |
| 8 | D1–D3 | bench (scale, fixtures) | ~60–90 min | ~2 h | Force accuracy ≥ 6 levels; stiffness foam, balloon and rigid; 20 taps | D1 ≤ max(0.05 N, 10 %); D2 within 20 %, rigid ≥ 5× foam; D3 rise ≤ 30 ms, peaks within 20 % |
| 9 | D4 blind trials | bench + helper | ~60–90 min | ~2 h (randomiser and analysis script first) | ≥ 20 trials per pair plus a control; follower hidden; masking headphones | rigid vs soft ≥ 17/20, control near chance; psychometric curve |

**Totals:** about 5–7 h of bench time over 8–10 sessions, and about
12–16 h of Claude work.

- At 2–3 bench sessions a week, that is roughly **3–5 weeks** of calendar
  time, depending mostly on Gate B (C3 stability) iterations.
- **Optional, any time before session 4:** the A4b G(f) chirp (5 min of
  bench time) if Gate A misses on motion-related error.

**Biggest risks, in order:**

1. **Closed-loop stability of the felt force (C3).** Unknown until
   tried. Mitigations: lower FF_GAIN, the leader damping KB_DAMP, a lower
   estimator bandwidth.
2. **Free-floor target:** 15 mA is ambitious. 25–35 mA is likely enough
   for rigid vs soft (contacts are 300–1,000 mA), but it may miss the
   brief's free-space RMS number.
3. **Leader stiction about 0.06 A (about 0.2 N):** forces below that
   reach the hand poorly. Rigid vs soft contacts are well above it.
4. **Reliability:** occasional simultaneous ESC resets (cause unknown,
   suspect supply), and SD-write stalls (deferred; must be fixed before
   C3).
