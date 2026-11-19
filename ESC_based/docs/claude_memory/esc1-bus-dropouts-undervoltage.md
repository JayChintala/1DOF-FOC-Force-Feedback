---
name: esc1-bus-dropouts-undervoltage
description: "ESC1 (node base 0x000) intermittently drops off the CAN bus and resets; the Pi logs continuous undervoltage events. Telemetry never goes stale, so scripts show plausible cached values for a dead node."
metadata:
  node_type: memory
  type: project
  originSessionId: 9e805e0b-1b2f-421d-baab-c7c4142cc4a7
  modified: 2026-10-27T03:58:23.121Z
---

Diagnosed 2026-09-22 on branch `tune/current-sensing-gain-16x`, after the
gs_usb -> Waveshare CAN FD HAT swap.

**The CAN link and command path are healthy.** `can0` is the HAT (clock
40 MHz, bitrate 1 Mbps, ERROR-ACTIVE, berr-counter tx 0 rx 0, zero bus-errors
and zero TX errors since boot). START + SET_IQ both work: ESC2 tracked a
0.2 A command (iq_readback ~0.20) and ESC1, when present, tracked 0.2 A and
spun Motor 1 through continuous revolutions.

**ESC1 is intermittent.** Over ~3 minutes of sniffing, node base 0x000 went
through three phases: streaming garbage (iq_readback ~ +28.3 A, encoder
counter sweeping 0..3999 randomly), then *completely absent* for >12 s (only
0x030/0x031/0x034 on the wire at 1 kHz), then back at a clean 1 kHz with sane
values. ESC2 (base 0x020) never dropped.

**The Pi is browning out**: 542 `hwmon1: Undervoltage detected!` events since
boot, recurring every 8-20 s at the time of diagnosis; `vcgencmd
get_throttled` = 0x50000 (under-voltage has occurred, soft-temp-limit has
occurred). The HAT is new load on the same 5 V rail.

**Why this looks like "CAN broke" when it hasn't:** `MotorCANInterface`
never invalidates telemetry. `iq_readback` / `enc_count_*` hold their last
value forever when a node stops transmitting, and `get_telemetry()` returns
those stale numbers with no staleness check in any test script. Meanwhile
SET_IQ still goes out on the wire and is ACKed by the *other* ESC, so there
are no TX errors and no bus-off - the frame is simply never received by the
node that reset.

**How to apply:** Before blaming CAN, run a per-ID presence check (count
frames per arbitration ID per second for ~10 s) and confirm BOTH node bases
are streaming at 1 kHz. Then check `dmesg | grep Undervoltage`. Add a
staleness guard on `*_time` in `can_interface.py` before trusting any
readback. See [[flashed-telemetry-mismatch]].

**Update 2026-10-26:** the Pi is no longer browning out (throttled 0x0).
Both ESCs dropped off together twice, while idle. can0 went ERROR-PASSIVE
with the tx error counter at 128 (no ACK). The user re-plugged the ESC
power each time, and the boards came back RESET (+28 A idle), needing
`zero_iq_check.py`. Suspect the ESC supply or connectors. If TELEM is
0/s, ask the user to check the ESC power.

Open sub-question: at 0.2 A Motor 1 spins freely but Motor 2 moves 1 encoder
count in 3 s, despite both reporting iq_readback ~0.2. Asymmetry not yet
explained; may relate to the 16x current-sense gain branch.
