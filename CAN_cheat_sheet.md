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
| `0x010` | IQ_READBACK | MCU → Pi | 4 bytes | float32, little-endian, Amps |
| `0x013` | ELEC_ANGLE | MCU → Pi | 2 bytes | int16, DPP format (electrical angle) |
| `0x014` | ENC_COUNT | MCU → Pi | 4 bytes | uint32 (raw TIM4 counter, wraps at `M1_PULSE_NBR`) |

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
candump can0,014:7FF   # ENC_COUNT only
candump can0,010:7FF   # IQ_READBACK only
```

Typical output line:
```
can0  010   [4]  4F 09 B8 C2
```
- `010` = CAN ID (hex, no `0x` prefix)
- `[4]` = data length in bytes
- `4F 09 B8 C2` = raw payload bytes, **in the order transmitted** (little-endian for our multi-byte fields)

---

## Decoding Payloads

### IQ_READBACK / SET_IQ (float32, little-endian)

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

### ENC_COUNT (uint32)

```python
def decode_u32(hex_bytes: str) -> int:
    b = bytes.fromhex(hex_bytes)
    return struct.unpack('<I', b)[0]

decode_u32("DE010000")  # -> 478
```

### ELEC_ANGLE (int16, DPP format)

```python
def decode_i16(hex_bytes: str) -> int:
    b = bytes.fromhex(hex_bytes)
    return struct.unpack('<h', b)[0]

decode_i16("21D6")  # example
```

---

## Known Behavior / Gotchas

- **-92A on IQ_READBACK at boot / before calibration** — expected artifact, uncalibrated current-sense offset. Not a fault. Goes away once the motor has been started and current loop is active.
- **SET_IQ is silently dropped if motor isn't in RUN state.** Firmware checks `MC_GetSTMStateMotor1() == RUN` before applying the command. Always sequence: `START` → wait briefly → `SET_IQ`.
- **ENC_COUNT is a raw 16-bit-range counter, wraps at `M1_PULSE_NBR`.** If tracking multi-revolution position on the Pi side, must detect wraparound and unwrap manually — the MCU does not do this for you.
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
