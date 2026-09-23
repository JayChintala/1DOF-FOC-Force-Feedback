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
#define CAN_ID_TELEM(base) ((base) + 0x012)

/* TELEM payload, 8 bytes, little-endian -- everything the Pi reads, in one
 * frame per 1 kHz tick:
 *   [0..3] float32  Iq, Amps (MC_GetIqdMotor1_F().q -- raw, unfiltered)
 *   [4..5] uint16   raw TIM4 encoder count, 0..3999 (wraps at M1_PULSE_NBR)
 *   [6..7] uint16   MCU timestamp, microseconds, free-running, wraps at 65536
 *
 * 0x010 (IQ_READBACK), 0x011 (IQ_MEAN), 0x013 (ELEC_ANGLE) and 0x014
 * (ENC_COUNT) are ALL retired. Previously this node sent three frames per
 * tick; two motors at 1 kHz put 6000 frames/s on the wire and cost the Pi an
 * interrupt each. Iq and the encoder count are the only signals anything
 * actually reads, and together they fit one frame -- so one frame is what
 * gets sent.
 *
 * WHY A NEW ID (0x012) RATHER THAN REUSING 0x010: the old 0x010 also began
 * with a float32 Iq, so a node still running pre-merge firmware would look
 * valid to the new parser while bytes 4..7 (which used to be the Iq EWMA)
 * got decoded as an encoder count and a timestamp -- plausible-looking
 * garbage. A retired ID makes a half-flashed bus go silent instead, which is
 * a failure you notice. Do not recycle 0x010/0x011/0x013/0x014.
 *
 * Bytes 4..5 hold the encoder count as uint16 (not the uint32 it used to be)
 * because the counter only ever spans 0..3999; that is what buys the two
 * bytes the timestamp needs. Both fields wrap, and both are unwrapped on the
 * Pi -- see WrappingCounter in can_interface.py, which handles both.
 *
 * The three-Tx-element limit documented in CAN_SendTelemetry() is no longer
 * binding at one frame, but it has not gone away. If you add a signal, pack
 * it into a spare byte of an existing frame; do not add an ID. */

/* ---- Public API ---- */
void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan);
void CAN_ProcessPendingMessages(void);
void CAN_SendTelemetry(void);

/* Tx-FIFO-full drop counter, incremented in CAN_SendTelemetry() when
 * HAL_FDCAN_AddMessageToTxFifoQ() fails. Not currently exposed via the MC
 * register interface -- see MC_REG_SECTOR in sync_registers.c for the
 * pattern if that's added later. One counter, because there is now one
 * telemetry frame. */
uint32_t CAN_GetTelemTxDropCount(void);

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
