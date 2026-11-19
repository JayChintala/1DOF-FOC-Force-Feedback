---
name: flashed-telemetry-mismatch
description: TELEM Iq is meaningless while the ESC is IDLE (reads ~±25 A); only trust it after START. Supersedes the 2026-09-22 0x010/0x011 layout note.
metadata:
  node_type: memory
  type: project
  originSessionId: 9e805e0b-1b2f-421d-baab-c7c4142cc4a7
  modified: 2026-10-20T13:56:18.051Z
---

Re-checked 2026-10-20 on a quiet bus, both boards on ESC_based firmware:
only TELEM (base+0x012, 8 B, 1000/s) and DBG (0x100+TELEM ID, 10/s) are on
the wire, layout `<fHH>` as in `software/can_interface.py`. µs-timestamp gaps
999..1006, none missing over 4 s. The old 0x010/0x011/0x014 frames are gone.

At rest, never STARTed since power-up: ESC1 Iq = **-24.41 A** (std 0.066),
ESC2 = **+25.28 A** (std 0.044), on a ~16.5 mA lattice at that fixed angle.

**Why:** `FOC_HighFrequencyTask` calls `FOC_CurrControllerM1()` and writes
`FOCVars.Iqd` every 16 kHz cycle even with PWM off, and TELEM sends
`MC_GetIqdMotor1_F()`. In IDLE that is a Park transform of uncalibrated
shunt readings (offsets are measured on START). Confirmed 2026-10-20: after
one START/STOP the idle reading drops to ~0 (A +0.02, B -0.005 A). The old "-7.94 A at rest" observation was very likely the same
artifact, not a gain bug.

**How to apply:** never read Iq as current (noise floor, offsets, observers)
unless the node is in RUN; check Iq ≈ 0 after START + SET_IQ 0 first. The
Iq lattice spacing is angle-dependent (it mixes phase ADC counts of 29.4 mA),
so don't expect a fixed 29.4 mA Iq lattice. Also: the first START after ESC power-up runs a 2 s, 0.8 A Id alignment that ends
with a one-frame encoder counter jump to raw 142 on both boards; teleop's 0.5 s
settle doesn't cover it. Results log: ESC_based/software/TEST_LOG.md.
See [[esc1-bus-dropouts-undervoltage]].
