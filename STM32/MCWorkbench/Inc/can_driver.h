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
#define CAN_ID_IQ_MEAN(base) ((base) + 0x011)
#define CAN_ID_ENC_COUNT(base) ((base) + 0x014)

/* 0x013 (ELEC_ANGLE) is retired -- the electrical angle now travels in bytes
 * 4..5 of the IQ_MEAN frame. See the hard three-frame limit documented in
 * CAN_SendTelemetry(): the G4 has only 3 FDCAN Tx elements, so a fourth
 * frame is dropped on every cycle rather than delayed. Pack new signals into
 * the spare payload bytes; do not add an ID. */

/* ---- Public API ---- */
void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan);
void CAN_ProcessPendingMessages(void);
void CAN_SendTelemetry(void);

/* Tx-FIFO-full drop counters, incremented in CAN_SendTelemetry() when
 * HAL_FDCAN_AddMessageToTxFifoQ() fails for that message. Not currently
 * exposed via the MC register interface -- see MC_REG_SECTOR in
 * sync_registers.c for the pattern if that's added later. */
uint32_t CAN_GetIqTxDropCount(void);
uint32_t CAN_GetIqMeanTxDropCount(void);
uint32_t CAN_GetEncTxDropCount(void);

/* ---- Iq telemetry conditioning ----
 * Defined in the USER CODE blocks of mc_tasks_foc.c, because that is where
 * the 16 kHz HighFrequencyTask lives and a separate source file would have
 * to be added to the CubeIDE build. Declared here because can_driver.c is
 * the only consumer. Values are in raw s16A, matching FOCVars[].Iqd.q --
 * scale with scaleParams_M1.current to get Amps.
 *
 * NOTE: these live in USER CODE BEGIN/END blocks, so they survive a
 * MotorControl Workbench regeneration. */
void IqTelem_UpdateHF(int16_t iq_s16);
float IqTelem_GetEwma(void);
float IqTelem_GetMeanAndReset(void);

#endif /* CAN_DRIVER_H */
