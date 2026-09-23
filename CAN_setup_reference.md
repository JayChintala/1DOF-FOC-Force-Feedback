# CAN Setup Reference — 1DOF-FOC-Force-Feedback (B-G431B-ESC1)

Board has an onboard TCAN330 CAN transceiver (UM2516 §5.5). No external transceiver needed.

---

## Required `.ioc` Settings

### FDCAN1 Peripheral (Connectivity → FDCAN1)
| Setting | Value |
|---|---|
| Mode | Activated, Classic Frame Format, Normal mode |
| Nominal Prescaler | 10 |
| Nominal Sync Jump Width | 1 |
| Nominal Time Seg1 | 13 |
| Nominal Time Seg2 | 3 |
| Resulting bitrate | ~1 Mbps @ 170MHz FDCAN kernel clock (PCLK1) |
| **Std Filters Nbr** | **1** (must be ≥1, or no runtime filter config takes effect — hardware-level, not caught by any error) |
| Ext Filters Nbr | 0 |
| Tx Fifo Queue Mode | FIFO mode |

### FDCAN1 Pins
| Pin | Function |
|---|---|
| PA11 | FDCAN1_RX (AF9) |
| PB9 | FDCAN1_TX |

### FDCAN1 NVIC
- `FDCAN1_IT0_IRQn` enabled

### GPIO — CAN Transceiver Control
| Pin | Label | Mode | Output Level |
|---|---|---|---|
| **PC11** | `CAN_SHDN` | GPIO_Output | **LOW** — HIGH = shutdown mode (per TCAN330 datasheet, Table 6-5). Must be LOW for normal operation. |
| PC14 | `CAN_TERM` | GPIO_Output | HIGH (120Ω termination enabled, if this board is a bus endpoint) |
| PB13 | `CAN_TERM_ALT` | GPIO_Output | HIGH |

---

## Code Structure

### `Inc/can_driver.h`
```c
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

#define CAN_NODE_BLOCK_SIZE 0x20U
#define CAN_NODE1_BASE 0x000U
/* Motor 2: #define CAN_NODE2_BASE (CAN_NODE1_BASE + CAN_NODE_BLOCK_SIZE) */

#define CAN_ID_START(base)       ((base) + 0x001U)
#define CAN_ID_STOP(base)        ((base) + 0x002U)
#define CAN_ID_SET_IQ(base)      ((base) + 0x003U)
#define CAN_ID_TELEM(base)       ((base) + 0x012U)
/* 0x010 IQ_READBACK, 0x011 IQ_MEAN, 0x013 ELEC_ANGLE, 0x014 ENC_COUNT:
   all retired and merged into TELEM. Do not reuse those IDs. */

void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan);
void CAN_ProcessPendingMessages(void);
void CAN_SendTelemetry(void);

#endif
```

### `Src/can_driver.c`
- `CAN_Driver_Init()` — NVIC enable, standard-ID range filter (`0x001`–`0x003` → RX FIFO0), global filter set to reject non-matching frames, activate RX notification, start peripheral.
- `HAL_FDCAN_RxFifo0Callback()` — parses START/STOP/SET_IQ into a "latest wins" pending-command struct (ISR context — keep minimal, defer motor calls).
- `CAN_ProcessPendingMessages()` — called from non-ISR context, drains pending struct into `MC_StartMotor1()` / `MC_StopMotor1()` / `MC_SetCurrentReferenceMotor1_F()`. SET_IQ is dropped if motor isn't in RUN state (Pi must sequence START before SET_IQ).
- `CAN_SendTelemetry()` — transmits ONE 8-byte frame (`CAN_ID_TELEM`, base+`0x012`): Iq (`MC_GetIqdMotor1_F()`) as float32, raw encoder count (`__HAL_TIM_GET_COUNTER(&htim4)`, wraps at `M1_PULSE_NBR`) as uint16, and a uint16 microsecond timestamp from `DWT->CYCCNT`. The separate IQ_READBACK / IQ_MEAN / ELEC_ANGLE / ENC_COUNT frames are retired; the electrical angle and the conditioned-Iq variants are no longer sent, as nothing read them.
- `MicroClock_Init()` / `MicroClock_Now_u16()` — DWT cycle-counter microsecond timebase for that timestamp. Deliberately NOT a timer peripheral: DWT needs no `.ioc` entry, so Workbench regeneration cannot revert it.

### `Src/main.c`
- `#include "can_driver.h"`
- `CAN_Driver_Init(&hfdcan1);` in `USER CODE BEGIN 2`, after `MX_FDCAN1_Init()` and `MX_NVIC_Init()`.

### `Src/mc_app_hooks.c`
- `#include "can_driver.h"` in the includes block.
- In `MC_APP_PostMediumFrequencyHook_M1()`, inside `USER SECTION BEGIN/END PostMediumFrequencyHookM1`:
  ```c
  CAN_ProcessPendingMessages();
  CAN_SendTelemetry();
  ```

---

## Gotchas

- **`Std Filters Nbr = 0` silently blocks all RX** at the hardware level, even with correct runtime filter/callback code. No error is thrown; symptoms look like a broken interrupt chain (RXF0S stays 0, NVIC never pending, callback never fires).
- **`CAN_SHDN` HIGH = shutdown, not enable.** Wrong polarity here looks identical to a dead/missing transceiver (0V on CANH/CANL) but is a one-line GPIO fix, not a hardware fault.
- **`mc_app_hooks.c` include is unprotected.** Unlike CubeMX's `/* USER CODE BEGIN/END */` markers, this file's `/* USER SECTION */` markers only wrap function bodies — the `#include "can_driver.h"` line must be manually re-added after any regeneration of this specific file.
- **Sync fixes into the `.ioc`, not just the `.c` files**, or they'll be overwritten on next CubeMX/MC Workbench regeneration. Use a git branch to test regeneration before merging.

---

## Not Yet Implemented

- Command timeout/watchdog (firmware holds last Iq value indefinitely if Pi stops sending)
- Multi-board addressing (requires activating `CAN_NODE2_BASE` etc. and replacing hardcoded `CAN_NODE1_BASE` throughout `can_driver.c` with a per-build or per-board selector)
