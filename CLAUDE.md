# 1-DOF FOC Force-Feedback — guide for Claude Code

Two B-G431B-ESC1 boards (motor **A** = ESC 1, node 0, 3D-printed arm; motor **B** = ESC 2, node 1, bare handle) on one CAN bus with a Raspberry Pi. Two control architectures: **leader–follower teleop** (Pi, `teleop_test.py`) and **peer-to-peer (P2P)** (both ESCs, disturbance observers, firmware CHANGE 9/10).

**Active work is on branch `feature/p2p-dob-bilateral`.** `ESC_based/` is the current system; the top-level `STM32/` and `software/` are the older Pi-computes-everything design.

## Read these first

| Question | Where |
|---|---|
| Status, history, every test result | `ESC_based/software/TEST_LOG.md`. Read its TL;DR at the top and "0c. RESUME HERE". Dated entries are chronological and end just above `## Phase A summary`, so **the newest entry is the last one before that heading**. |
| P2P: design, firmware/Pi changes, references | `Peer_to_Peer/README.md` |
| P2P: all test results, incl. P2P vs teleop | `Peer_to_Peer/testing.md` (new P2P results go here) |
| Firmware changes 1–11 | `ESC_based/FIRMWARE_CHANGES.txt` |
| Raw logs in git | `Peer_to_Peer/data/` (2026-11-22 P2P vs teleop), `ESC_based/data/` (2026-11-23 motor parameters). Live logs are on the Pi (git-ignored). |

## The slide deck

- File: `ESC_based/presentation/Force-Aware Telepresence for Robot Learning.pptx` (26 slides).
- **To read or answer questions about it, use `ESC_based/presentation/deck_outline.md`**. It's a text mirror with every slide's text, tables, the exact values behind each chart, figure names and speaker notes. Regenerate it after any deck change: `python3 ESC_based/presentation/export_deck_outline.py`.
- P2P section: slides 16–18 (how it works, measured, two approaches), 19–22 (P2P vs teleop: free space, taps/holds, rigid vs soft, scorecard), 23 (remaining work).
- Built by scripts in the same folder: `add_software_and_peer_slides.py` (2026-11-15 slides), `add_p2p_vs_teleop_slides.py` (slides 16–23 updates and 19–22), `make_p2p_dob_diagram.py` (slide 16 figure), with data in `p2p_measured_window.json`.
- These scripts rebuild from a **local, untracked backup** of the earlier deck; `add_p2p_vs_teleop_slides.py` refuses to run without it. On a fresh clone, edit the deck directly, or restore the backup from the deck at commit `8ecf246`.
- Visual QA on JayPC: export slides with PowerPoint COM (`$p.Slides.Item(i).Export(...)` from PowerShell); there is no LibreOffice.

## Most recent test: motor parameters vs MC Workbench (2026-11-23)

TEST_LOG entry "Motor parameters vs MC Workbench". Tool: `ESC_based/software/motor_params.py spin|stall|fit`, using the DIAG3 frame from firmware CHANGE 11 (Vbus and 100 ms means of Vq/Vd/Iq).

| | Workbench | Motor A (arm) | Motor B |
|---|---|---|---|
| Ke (Vrms/krpm) | 5.00 | 4.92 | 5.00 |
| Kt (mN·m/A) | 58.5 | 57.6 | 58.5 |
| Rs (Ω, stall) | 5.29 | 4.53 | 4.78 |
| J (kg·m², = Kt/G) | not used by the firmware | 8.98e-6 | 4.19e-6 |

- Ke/Kt use only the ≥ 2 rev/s spin data: below that, stick-slip and dead-time bias the slope low.
- Inertia and friction aren't used in torque mode.
- **Open:** the back-EMF Kt predicts 0.76 N/A at A's tip (r = 75.7 mm), but the scale calibration measured 1.058 N/A (39 % apart). Either the scale was pressed at about 54 mm, or the current scale (`AMPLIFICATION_GAIN` 9.14) reads about 28 % low. A multimeter phase-to-phase reading decides it: 9.0–9.6 Ω = current scale right; about 6.6 Ω = wrong.
- Ls is not measured (needs a 16 kHz capture).

## Hardware access and routine (from JayPC)

- **Pi:** `ssh jayrpi4@192.168.1.164`, repo at `~/1DOF-FOC-Force-Feedback`, scripts in `ESC_based/software/`. Sync it with `git pull` on the Pi.
- **Build/flash both ESCs:** `powershell -ExecutionPolicy Bypass -File ESC_based/STM32/flash_and_verify.ps1 -Yes` (`-BuildOnly` to just build). **Never regenerate from MC Workbench/CubeMX**: it reverts hand edits (16× PGA, CAN, coupling code).
- **After a flash or power cycle:** run `python3 zero_iq_check.py` on the Pi. "RUN bit never seen" with live telemetry means the ESC motor supply is off. Idle Iq of ±28 A before the first START is normal.
- **P2P operating point:** `python3 p2p_teleop.py --kf 50 --dob-g 80`. It starts with a ~35 s ripple calibration that swings arm A 1.25 turns, so keep its path clear. Guided runs: `--guide ab|free|taps|contact --surface ...`.
- **Hand-held tests:** the user runs them in their own terminal (`ssh -t ... "cd ... && python3 ..."`), since they can't see Claude's live output. Hands-off tests Claude may run directly.
- **Logging:** log every result. P2P results go in `Peer_to_Peer/testing.md`; everything else goes in `TEST_LOG.md`. Commit and push on the feature branch.
