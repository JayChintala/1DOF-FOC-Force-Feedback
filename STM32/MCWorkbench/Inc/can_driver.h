/*
 * can_driver.h
 *
 *  Created on: Jul 3, 2026
 *
 * Multi-node addressing: each ESC's IDs are offset by CAN_NODE_BASE,
 * computed from CAN_NODE_ID * CAN_NODE_STRIDE. Set CAN_NODE_ID per board
 * before flashing (recommended: via a separate CubeIDE build
 * configuration's preprocessor define, e.g. "Debug_ESC2" with
 * CAN_NODE_ID=1 -- see project notes). Defaulting to 0 here means ESC 1
 * needs no changes at all; only ESC 2 (and beyond) sets CAN_NODE_ID.
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>

#include "stm32g4xx_hal.h"

/* ---- Node addressing ---- */
#define CAN_NODE_STRIDE 0x020

#ifndef CAN_NODE_ID
#define CAN_NODE_ID 0 /* 0 = ESC 1 (default/unchanged), 1 = ESC 2, etc. */
#endif

#define CAN_NODE_BASE (CAN_NODE_ID * CAN_NODE_STRIDE)

/* ---- Per-command ID offsets (must match can_interface.py) ---- */
#define CAN_ID_START(base) ((base) + 0x001)
#define CAN_ID_STOP(base) ((base) + 0x002)
#define CAN_ID_SET_IQ(base) ((base) + 0x003)
#define CAN_ID_IQ_READBACK(base) ((base) + 0x010)
#define CAN_ID_ELEC_ANGLE(base) ((base) + 0x013)
#define CAN_ID_ENC_COUNT(base) ((base) + 0x014)

/* ---- Public API ---- */
void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan);
void CAN_ProcessPendingMessages(void);
void CAN_SendTelemetry(void);

#endif /* CAN_DRIVER_H */
