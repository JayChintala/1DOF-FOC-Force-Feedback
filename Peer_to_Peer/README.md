# Peer-to-Peer (P2P) Bilateral Teleop: Changes Summary

This folder summarizes everything that was added to run **peer-to-peer teleop**: symmetric bilateral control with no dedicated leader or follower, following the disturbance-observer approach from Keio University (Ohnishi, Katsura, Shimono). The code itself lives in `ESC_based/`; this page says what changed, where it is, and why.

- **Branch:** `feature/p2p-dob-bilateral`
- **Commits:** `c12dd52` (firmware + Pi code), `dc37905` (test log)
- **Firmware change number:** CHANGE 9 (full entry in `ESC_based/FIRMWARE_CHANGES.txt`)
- **Status (2026-11-18):** built (0 warnings), flashed on both ESCs, parameters verified over CAN. **Not yet run with motion on the bench.**

---

## 1. The two control architectures

| | Teleop (existing) | P2P (new) |
|---|---|---|
| Roles | Motor B = leader, Motor A = follower | None: both motors are equal |
| Where the law runs | Pi (`teleop_test.py`), ~500 Hz, sends `SET_IQ` | On both ESCs, 1 kHz, Pi out of the loop |
| Position | Follower tracks leader | Differential mode: `x_A − x_B → 0` |
| Force | Follower's current reflected to the leader | Common mode: `f_A + f_B → 0` (action = reaction) |
| Feel | Light (the follower is powered to track) | Lighter than PEER mode; stiff in steady state |

P2P also replaces the earlier symmetric attempt, **PEER mode** (`COUPLE_MODE 2`), a plain spring and damper between the shafts. PEER mode felt **springy**: a push had to stretch the virtual spring before the other shaft moved, it saturated at 114 counts, and stiffer gains hit a 57 Hz buzz (TEST_LOG 2026-11-15).

### Why P2P should not be springy

Each motor gets a **disturbance observer (DOB)** that cancels its friction, gravity and load, so it behaves like a nominal inertia. A **reaction-force observer (RFOB)** estimates the external force on each shaft. Control then splits into two independent modes:

- **Common mode (force servo):** your push accelerates *both* shafts together instead of stretching a spring.
- **Differential mode (position servo):** keeps the shafts aligned. The DOB acts as an integrator, so the steady-state gap goes to zero.

```
          ESC 1 (motor A)                          ESC 2 (motor B)
   enc_A, Iq_A ──► DOB_A ──► f_A            enc_B, Iq_B ──► DOB_B ──► f_B
   TELEM_B ─────► DOB_B ──► f_B            TELEM_A ─────► DOB_A ──► f_A
                   │                                        │
        ddx = ½[Kf(f_A+f_B) − Kp·err − Kv·derr]   (same law, own/peer swapped)
        Iq_A = ddx/G_A + d_A                      Iq_B = ddx/G_B + d_B
                   └──────────── CAN, 1 kHz TELEM each way ──┘
```

Each ESC runs **both** observers: its own, plus the peer's computed from the peer's TELEM frame. That frame already carries Iq and the encoder count, so **no new CAN frame or ID was needed**.

---

## 2. The control law

Run every 1 kHz tick on each ESC, for shaft k ∈ {own, peer}:

```
DOB    z_k += min(1, g·dt) · (i_k + (g/G_k)·v_k − z_k)
       d_k  = z_k − (g/G_k)·v_k                      disturbance current [A]
RFOB   f_k  = −(d_k − comp·Fc_k·sat(v_k / 300))      external force [A-equiv], + = toward +counts

err    = (x_own − x_own0) − (x_peer − x_peer0)       [counts]
derr   = v_own − v_peer                              [counts/s]
ddx    = ½ · ( Kf·(f_own + f_peer) − Kp·err − Kv·derr )   [counts/s²]
Iq     = clamp( ddx / G_own + d_own, ±Iq_max )
```

- `i_k` is the **measured** 1 ms mean current (from TELEM), not the command, so the observers stay correct when the output clamps (no wind-up).
- `Kp = ωn²`, `Kv = 2ζωn`: bandwidth and damping of the position (differential) mode.
- `Kf` sets the felt inertia: apparent mass = `2/Kf`. At 100 % (`2·G_B`) the pair feels like motor B alone; PEER mode felt like both rotors, about 3.2× heavier.
- Plant constants: `G_A = 4.082e6`, `G_B = 8.877e6` counts/s² per A; Coulomb friction `Fc_A = 0.025 A`, `Fc_B = 0.017 A`.
- Torque sign `COUPLE_TORQUE_SIGN` is honoured throughout.

---

## 3. Firmware changes (CHANGE 9)

All paths are under `ESC_based/STM32/MCWorkbench/`.

| File | Change |
|---|---|
| `Inc/couple_ctrl.h` | New mode `COUPLE_MODE_P2P = 3`. Full law documented ("P2P mode" section). Defaults, SET_PARAM ranges, plant table (`COUPLE_P2P_G_NODE0/1`, `COUPLE_P2P_FC_NODE0/1`), `COUPLE_P2P_KF_REF = 2·G_B`, sanity limit `COUPLE_P2P_IQ_SANE_A = 1.2 A`. New trip `COUPLE_TRIP_OBS_RANGE = 6`. Declares `Couple_SetP2PParam()`. |
| `STM32CubeIDE/Application/User/couple_ctrl.c` | Observer struct `Dob_t`, `DobReset()`, `DobStep()`. Own observer stepped in `UpdateOwn()`; peer observer stepped once per new peer frame in `UpdatePeer()` using the peer's own timestamps. P2P branch in `RunLaw()`. Engagement also waits for sane Iq readbacks on both sides and resets both observers at engage, so the first output is ~0 A. `Couple_SetP2PParam()` with range checks; changing `g` shifts `z` so the estimate doesn't jump. PEER and HOLD are unchanged. |
| `Inc/can_driver.h` | `CAN_TickSample_t.iq_a` (own 1 ms mean Iq) and `CAN_PeerSample_t.iq_a` (peer's TELEM Iq). SET_PARAM ids `CAN_PARAM_P2P_*` 3–7. Protocol comments updated. |
| `STM32CubeIDE/Application/User/can_driver.c` | Peer RX now also stores TELEM bytes 0..3 (Iq). The Iq mean is drained by `SampleIq()` in `CAN_ControlTick()` **before** `Couple_Tick()`, so the observer uses exactly the value TELEM then sends (TELEM bytes unchanged). `ApplySetParam()` forwards ids 3–7 to `Couple_SetP2PParam()`; invalid values are counted as rejects (DIAG bit 6). |

### Protocol additions

| Item | Value |
|---|---|
| `COUPLE_MODE` (base+0x006) | `3` = P2P. Optional flag bit 0 = predict peer position (as in PEER mode). |
| SET_PARAM (base+0x007) id 3 | ωn, 0.1 Hz units, 10..400, default **200** (20 Hz) |
| id 4 | ζ, %, 20..300, default **100** |
| id 5 | Kf, % of 2·G_B, 0..300, default **100** |
| id 6 | DOB cutoff g, rad/s, 20..1000, default **150** |
| id 7 | RFOB friction compensation, %, 0..150, default **0** |
| DBG status bit 3 | Now "mode uses the peer" (PEER **or** P2P) |
| Trip 6 | Observer input out of range: an Iq readback beyond 1.2 A, e.g. the peer left RUN (IDLE reads about ±28 A) |

No new CAN IDs, no RX filter changes, no change to the TELEM or DBG layouts. Parameters are RAM-only and take effect immediately, engaged or not.

### Safety behaviour

- **Before engaging:** both shafts must be still, peer TELEM fresh, and both Iq readbacks under 1.2 A.
- **At engage:** both observers reset to the at-rest state.
- **While engaged:** the existing trips still apply (peer timeout 5 ms, error > 10 rev, velocity > 50 rev/s, motor left RUN), plus trip 6.
- **Output clamp:** Iq is clamped at `Iq_max` (≤ 0.8 A hard ceiling).

---

## 4. Pi code

All paths are under `ESC_based/software/`.

| File | Change |
|---|---|
| `can_interface.py` | `COUPLE_MODE_P2P`, `PARAM_P2P_*` ids, trip 6 name, `send_p2p_params(wn_hz, zeta, kf_pct, dob_g, fric_pct)`. **Bug fix:** `send_set_param()` now waits 3 ms after each frame (see below). |
| `p2p_teleop.py` | **New.** Entry point for the P2P architecture: sends parameters, START, requests mode 3, waits for engagement, then prints a live line (positions, gap, each side's estimated force in N, Iq, status). A Pi-side mirror of the firmware observers provides the force readout. Logs CSV/JSON/PNG to `logs/p2p_*.csv`; prints gap percentiles and a force-mismatch figure at the end. Reuses logging and engage helpers from `coupling_monitor.py`. |
| `p2p_sim.py` | **New.** Offline model used to choose the defaults and compare P2P against PEER mode (see §5). |

### Bug fixed: dropped SET_PARAMs

The firmware latches SET_PARAM in a single latest-wins slot that the 1 kHz loop drains. Two frames inside the same millisecond kept only the second. `send_start()` was sending Tafter and the current-loop gain ~0.1 ms apart, so one was probably being lost; this is inferred, not checked against old logs. With 3 ms pacing, Tafter = 400 is now confirmed by the DIAG2 readback on both boards.

---

## 5. Simulation results (`p2p_sim.py`)

The model includes: per-ESC 1 kHz ticks 0.25 ms apart, ~1 ms peer delay, encoder quantization, 1 ms mean Iq, 440 Hz current loop, 9 mA current noise, Coulomb friction, measured G values.

| Law | Hold B, push A 0.3 A: steady gap | 3 Hz free sweep: hand force | B against a wall: hand / wall force, gap |
|---|---|---|---|
| PEER, Kp 0.007, Kd 2.5e-5 | 45 counts | 114 mA rms | 229 / 216 mA, 31 counts |
| **P2P defaults** | **≈ 2 counts** | **54 mA** (24 mA with friction comp 100 %) | **288 / 287 mA, 0.2 counts** |

- **Stability limit:** about Kv = 2ζωn > 500 /s (e.g. ωn 40 Hz at ζ 1). The defaults use 251 /s. Kf at 300 % went unstable against a wall.
- **Transient:** a sudden 0.3 A push against a held shaft opens a gap of ~81 counts for ~20 ms (PEER: 50).
- **Limitation:** the model has no structural resonances. It did **not** reproduce PEER mode's real 57 Hz buzz, so treat its stability margins as optimistic.

---

## 6. How to run

**Build and flash** (on the Windows host):
```
powershell -ExecutionPolicy Bypass -File ESC_based/STM32/flash_and_verify.ps1 -Yes
```
**Re-align** after any flash or power-up (on the Pi):
```
cd ~/1DOF-FOC-Force-Feedback/ESC_based/software
python3 zero_iq_check.py
```
**Run P2P.** Keep hands off both shafts until it prints "Engaged":
```
python3 p2p_teleop.py --duration 30 --iq-max 0.4     # cautious first run
python3 p2p_teleop.py                                # defaults, until Ctrl+C
python3 p2p_teleop.py --wn 25 --zeta 1.2 --kf 150    # stiffer, lighter
python3 p2p_teleop.py --fric 100                     # cancel nominal friction
```
**Sim:**
```
python3 p2p_sim.py            # comparison table
python3 p2p_sim.py --sweep    # gain sweeps
```

### Tuning guide

| Symptom | Try |
|---|---|
| Buzz or vibration hands-off | Lower `--wn`, or lower `--dob-g` |
| Shafts drift apart or feel loose | Raise `--wn` (keep 2·ζ·2π·ωn under ~500) |
| Feels heavy | Raise `--kf` in steps (100 → 150 → 200) |
| Feels sticky | `--fric 50`, then `--fric 100` |
| Trip 6 | A board isn't in RUN or its current readback is bad; re-run `zero_iq_check.py` |

---

## 7. Next steps

1. First bench run: hands-off buzz check, free motion, hold-one-push-other, then with `--fric 100`; compare against PEER mode (`coupling_monitor.py --kp 0.007 --kd 2.5e-5 --predict`).
2. Chirp / frequency-response test to measure the real plant, including the ~57 Hz mode, and fit it into the sim.
3. Add a linear margin analysis (`python-control`) to explain the stability limit.
4. Optional: gravity compensation for arm A in the RFOB, so its weight isn't felt as a force.
