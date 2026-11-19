---
name: user-runs-live-scripts
description: "User can't see plots or my live command output; positioning tests need a live text readout they run in their own terminal"
metadata:
  node_type: memory
  type: feedback
  originSessionId: 12aaaac4-6bb4-46f7-bb91-63794cc50b48
  modified: 2026-10-29T13:40:38.860Z
---

The user can't identify rotor angles from a graph ("i cant see the graph"), and my Bash output only reaches them after the command ends. For tests where they must position/hold something, give the script a live text guide (e.g. `tafter_sweep.py --window`) and have them run it in their VSCode terminal, then paste the table back.

**Why:** 2026-10-29 Tafter sweep: they couldn't find the "bad angle" until a live encoder readout was added.
**How to apply:** for hand-held hardware tests, print a live numeric target/distance and auto-start when held; ask the user to run it themselves. See [[test-log-handoff]].
