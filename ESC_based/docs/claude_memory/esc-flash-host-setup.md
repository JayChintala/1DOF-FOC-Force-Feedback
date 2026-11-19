---
name: esc-flash-host-setup
description: "Which ST-LINK serial is ESC1 vs ESC2, and the Windows host (JayPC) build/flash + SSH-to-Pi setup"
metadata:
  node_type: memory
  type: reference
  originSessionId: b9e180a3-fcc6-4abd-bf1f-0da37315d6e0
  modified: 2026-11-19T01:19:08.325Z
---

Firmware builds and flashing happen on the Windows host **JayPC** (has the full
CubeIDE project, STM32CubeIDE 2.1.1, STM32CubeProgrammer v2.22.0, both ESC USBs).
The Pi (192.168.1.164, user jayrpi4) runs CAN tests. Passwordless SSH JayPC -> Pi
works (key `jayis@JayPC` added to the Pi's authorized_keys 2026-11-18).

`STM32_Programmer_CLI` is on JayPC's user PATH
(`C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin`).

**Board mapping (verified 2026-11-18 by resetting probe 0 and watching TELEM timestamps):**
- ESC1, CAN base 0x000, build `CAN_NODE_ID 0`: ST-LINK SN `066BFF525266555067073237`, COM3
- ESC2, CAN base 0x020, build `CAN_NODE_ID 1`: ST-LINK SN `066BFF515266555067014734`, COM6

CAN_NODE_ID is compile-time, so each board needs its own build; flashing the wrong
one gives two nodes the same ID. Flash: `STM32_Programmer_CLI -c port=SWD sn=<SN> -w <elf> -v -rst`.

**How to verify a reset from the Pi:** gap detection does NOT work (boards reboot in
<3 ms and the CAN HAT drops cause 3-7 ms gaps on both nodes, see [[can-hat-irq-drops]]).
Use the TELEM µs timestamp (bytes 6..7): consecutive steps are multiples of 1000 µs
within ±6 µs; a reset gives an off-grid step. `-halt` via the CLI does not keep the core halted.

Related: [[test-log-handoff]], [[esc1-bus-dropouts-undervoltage]].
