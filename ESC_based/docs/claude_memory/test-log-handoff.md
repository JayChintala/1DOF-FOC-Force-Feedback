---
name: test-log-handoff
description: ESC_based/software/TEST_LOG.md is the running log and handoff for the sensorless force-feedback plan (Phases A–D); read its TL;DR and "0c. RESUME HERE" before any test
metadata:
  node_type: memory
  type: reference
  originSessionId: 9e805e0b-1b2f-421d-baab-c7c4142cc4a7
  modified: 2026-10-27T03:58:20.584Z
---

The user's "sensorless force-feedback improvement plan" (Phases A–D) is
executed on the Pi and logged in `ESC_based/software/TEST_LOG.md`. All
work lives in `ESC_based/software/`, not `software/`.

**State at the end of 2026-10-28:**

- C1–C3 done; holds fixed (contact spring); in-session leader cal
  adopted (`TELEOP_LEADER_CAL=1`).
- Tip force calibrated: 1.044 N/A. Measured Iq breaks down above about
  0.55 A at some rotor angles.
- Firmware **`b26c643` (CHANGES 1–7) is flashed** (runtime Tafter via
  SET_PARAM, DIAG2 frame).
- **Next:** bus_check → zero_iq_check → `tafter_sweep.py` at a bad
  angle. Details are in the UPDATE box at the top of TEST_LOG section 0c.
- Everything is committed and pushed. The user has been approving
  commits and pushes.

**Read TEST_LOG "Handoff → 0c. RESUME HERE"** for the operating-point
command, tools, rules and next options.

**Start every session** with `python3 bus_check.py` (passive). A reset
ESC needs `zero_iq_check.py` with the user's OK and both arms free.

**Each motor test:** confirm the shafts are clear and the user is
present. The user wants their feel report treated as the deciding
measure.

See [[flashed-telemetry-mismatch]], [[esc1-bus-dropouts-undervoltage]].
