# CAN Cheat Sheet — 1DOF FOC Force-Feedback Project

Board: B-G431B-ESC1 (STM32G431CB), onboard TCAN330 transceiver
Bus: 1 Mbit/s classic CAN, standard 11-bit IDs
Node base: `CAN_NODE1_BASE = 0x000`

---

## CAN ID Map

| ID (hex) | Name | Direction | Payload | Format |
|---|---|---|---|---|
| `0x001` | START | Pi → MCU | none | — |
| `0x002` | STOP | Pi → MCU | none | — |
| `0x003` | SET_IQ | Pi → MCU | 4 bytes | float32, little-endian, Amps |
| `0x012` | TELEM | MCU → Pi | 8 bytes | see layout below — Iq + encoder + MCU clock, one frame |

**TELEM (`0x012`) payload**, little-endian, sent once per 1 kHz firmware tick:

| Bytes | Type | Meaning |
|---|---|---|
| 0..3 | float32 | Iq, Amps (raw, unfiltered) |
| 4..5 | uint16 | raw TIM4 encoder count, 0..3999 (wraps at `M1_PULSE_NBR`) |
| 6..7 | uint16 | MCU microsecond clock, free-running, wraps at 65536 |

**Retired IDs — do not reuse:** `0x010` (IQ_READBACK), `0x011` (IQ_MEAN), `0x013` (ELEC_ANGLE), `0x014` (ENC_COUNT). All four merged into `0x012`. Two motors at 1 kHz went from 6000 frames/s to 2000. The Iq EWMA, Iq boxcar mean and electrical angle are no longer transmitted at all — nothing on the Pi ever read them. TELEM deliberately took a *new* ID rather than reusing `0x010`, so that a board left on old firmware goes silent instead of decoding as plausible nonsense.

Note: RX filter on the MCU only accepts IDs `0x001`–`0x003` (range filter). Anything else sent to the board is silently dropped by hardware filtering, not by application code.

---

## Sending Commands (`cansend`)

Syntax: `cansend <interface> <ID>#<hex payload>`

```bash
# Start motor
cansend can0 001#

# Stop motor
cansend can0 002#

# Set torque/current command (SET_IQ) — needs float32 LE hex payload
cansend can0 003#0000003F   # 0.5 A
cansend can0 003#0000803F   # 1.0 A
cansend can0 003#00000040   # 2.0 A
```

`cansend` is silent on success — no output means it worked. It only prints on error (bad interface, malformed frame).

### Generating the hex payload for SET_IQ (Python)

```python
import struct

def iq_hex(amps: float) -> str:
    return struct.pack('<f', amps).hex()

print(iq_hex(0.5))   # 0000003f
print(iq_hex(1.0))   # 0000803f
print(iq_hex(-0.75)) # 0000403f... (negative works too, just pack it)
```

---

## Reading the Bus (`candump`)

```bash
# Watch everything
candump can0

# Watch only one ID (if your candump build supports id:mask filtering)
candump can0,012:7FF   # this node's telemetry only
candump can0,032:7FF   # ESC 2's telemetry (node base 0x020)
```

Typical output line:
```
can0  012   [8]  4F 09 B8 C2  9E 0F  10 27
```
- `012` = CAN ID (hex, no `0x` prefix)
- `[8]` = data length in bytes
- `4F 09 B8 C2` = raw payload bytes, **in the order transmitted** (little-endian for our multi-byte fields)

---

## Decoding Payloads

### SET_IQ and TELEM bytes 0..3 (float32, little-endian)

Bytes are LSB-first. To decode `4F 09 B8 C2` → reverse byte order → `C2 B8 09 4F` → interpret as IEEE754 float32.

Python:
```python
import struct

def decode_float(hex_bytes: str) -> float:
    # hex_bytes like "4F09B8C2" (no spaces) as seen on the wire
    b = bytes.fromhex(hex_bytes)
    return struct.unpack('<f', b)[0]

decode_float("4F09B8C2")  # -> -92.0 (example from bench test)
```

### A whole TELEM frame at once

```python
import struct

def decode_telem(hex_bytes: str):
    # hex_bytes like "4F09B8C29E0F1027" (no spaces) as seen on the wire
    iq, enc, us = struct.unpack('<fHH', bytes.fromhex(hex_bytes))
    return {"iq_A": iq, "enc_count": enc, "mcu_us": us}

decode_telem("0000803F9E0F1027")  # -> {'iq_A': 1.0, 'enc_count': 3998, 'mcu_us': 10000}
```

`mcu_us` wraps every 65.536 ms, so successive frames differ by ~1000. It is
the clock to difference when computing velocity: it is sampled on the MCU
alongside the encoder count, so it carries none of the bus or kernel jitter
that arrival times do.

---

## Known Behavior / Gotchas

- **-92A on the Iq field at boot / before calibration** — expected artifact, uncalibrated current-sense offset. Not a fault. Goes away once the motor has been started and current loop is active.
- **SET_IQ is silently dropped if motor isn't in RUN state.** Firmware checks `MC_GetSTMStateMotor1() == RUN` before applying the command. Always sequence: `START` → wait briefly → `SET_IQ`.
- **The encoder field is a raw counter and wraps at `M1_PULSE_NBR` (0..3999).** If tracking multi-revolution position on the Pi side, must detect wraparound and unwrap manually — the MCU does not do this for you.
- **No command timeout/watchdog currently implemented.** If the Pi stops sending SET_IQ (crash, disconnect), the MCU holds the last commanded Iq value indefinitely. Consider adding a timeout on the Pi side (resend STOP or zero-current periodically) until a firmware watchdog exists.
- **Transceiver requires `CAN_SHDN` pin LOW to be in normal mode.** This is set correctly in current firmware (`GPIO_PIN_RESET` at boot). If CAN mysteriously goes dead again after future `.ioc` regeneration, check this first — it's the classic regression point.
- **`StdFiltersNbr` must be ≥1** for any standard-ID RX filter to actually take effect at the FDCAN hardware level. Also a classic `.ioc` regeneration regression point.

---

## Quick Reference: Full Bench Test Sequence

```bash
# 1. Start
cansend can0 001#

# 2. Command torque
cansend can0 003#0000003F

# 3. Watch telemetry
candump can0

# 4. Stop
cansend can0 002#
```
