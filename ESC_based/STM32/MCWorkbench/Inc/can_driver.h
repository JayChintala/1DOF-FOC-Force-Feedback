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
 *
 * ESC_based: each node also listens to its PEER's TELEM frame, straight off
 * the bus, so the two ESCs can couple to each other without the Pi in the
 * loop. The coupling law itself lives in couple_ctrl.c; this file is the
 * transport.
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

/* The other ESC of the bilateral pair: 0 <-> 1. Override per build only if
 * the pair is ever renumbered. */
#ifndef CAN_PEER_NODE_ID
#define CAN_PEER_NODE_ID (CAN_NODE_ID ^ 1)
#endif

#define CAN_PEER_BASE (CAN_PEER_NODE_ID * CAN_NODE_STRIDE)

/* ---- Per-command ID offsets (must match ESC_based/software/can_interface.py) ---- */
#define CAN_ID_START(base) ((base) + 0x001)
#define CAN_ID_STOP(base) ((base) + 0x002)
#define CAN_ID_SET_IQ(base) ((base) + 0x003)
#define CAN_ID_COUPLE_GAINS(base) ((base) + 0x004)
#define CAN_ID_COUPLE_LOCAL(base) ((base) + 0x005)
#define CAN_ID_COUPLE_MODE(base) ((base) + 0x006)
#define CAN_ID_TELEM(base) ((base) + 0x012)

/* DBG lives 0x100 above TELEM (0x112 for ESC 1, 0x132 for ESC 2), outside
 * every node's 0x20-wide block. Lower IDs win CAN arbitration, so this
 * keeps the diagnostic stream strictly below all control traffic: a DBG
 * frame can never delay the peer TELEM frame the coupling law is waiting
 * for. Inside a node's own block that is impossible -- ESC 1's 0x013 would
 * outrank ESC 2's TELEM at 0x032. */
#define CAN_ID_DBG(base) (0x100 + CAN_ID_TELEM(base))

/* Standard-ID filter elements CAN_Driver_Init() configures: one range for
 * this node's commands, one exact match for the peer's TELEM. The FDCAN
 * only evaluates the first StdFiltersNbr elements, so FDCAN1.StdFiltersNbr
 * in the .ioc must be at least this -- see the backstop in
 * CAN_Driver_Init(). */
#define CAN_STD_FILTER_COUNT 2u

/* Commands, Pi -> MCU, all little-endian. Commands a node does not
 * recognise, or whose length is wrong, are ignored.
 *
 *   base+0x001 START          no payload
 *   base+0x002 STOP           no payload. Also cancels any coupling mode.
 *   base+0x003 SET_IQ         float32 Iq, Amps. Applied only in direct mode
 *                             (COUPLE_MODE off) and only in RUN. Must be
 *                             refreshed within COUPLE_DIRECT_TIMEOUT_US or
 *                             the current is zeroed -- see couple_ctrl.h.
 *   base+0x004 COUPLE_GAINS   float32 Kp [A/count], float32 Kd [A/(count/s)]
 *   base+0x005 COUPLE_LOCAL   float32 Kd_local [A/(count/s)], float32 Iq_max [A]
 *   base+0x006 COUPLE_MODE    uint8 mode: 0 off (direct SET_IQ), 1 hold,
 *                             2 peer. Sending a mode again re-zeroes it.
 *
 * TELEM payload, 8 bytes, little-endian -- everything the Pi reads, in one
 * frame per 1 kHz tick. The PEER ESC reads it too: bytes 4..7 are what
 * its coupling law runs on.
 *   [0..3] float32  Iq, Amps (MC_GetIqdMotor1_F().q -- raw, unfiltered)
 *   [4..5] uint16   raw TIM4 encoder count, 0..3999 (wraps at M1_PULSE_NBR)
 *   [6..7] uint16   MCU timestamp, microseconds, free-running, wraps at 65536
 *
 * DBG payload, 8 bytes, little-endian -- what this node's controller did on
 * the same tick:
 *   [0..1] int16    Iq commanded, mA (coupling output, or the direct SET_IQ)
 *   [2..3] int16    coupling error, counts, saturated at +-32767; 0 when
 *                   not engaged
 *   [4..5] uint16   MCU timestamp -- the SAME value as this tick's TELEM,
 *                   which is how the Pi pairs the two frames
 *   [6]    uint8    status bits, COUPLE_STATUS_* in couple_ctrl.h
 *   [7]    uint8    age of the peer TELEM the law used, 0.1 ms units;
 *                   255 = 25.5 ms or more, or never received
 * Sent every tick while a coupling mode is requested or engaged, on the
 * tick the status byte changes, and every CAN_DBG_IDLE_DIVIDER ticks
 * otherwise -- so the Pi sees RUN and trip state before it engages
 * anything, and a trip is logged on the tick it happened.
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
 * TELEM has no spare byte left, which is the only reason DBG is a second
 * ID rather than more fields. Two frames per tick still fit the three-
 * element Tx FIFO documented in SendTelemetry(); a third would not leave
 * room for a retransmission backlog. Pack anything new into DBG. */

#define CAN_DBG_IDLE_DIVIDER 100u /* 10 Hz DBG while nothing is coupled */

/* One sample of this node's encoder and clocks, taken together at the top
 * of the 1 kHz tick, so TELEM, DBG and the coupling law all describe the
 * same instant. */
typedef struct {
  uint32_t cyc; /* raw DWT->CYCCNT; differences valid up to ~25 s */
  uint32_t us;  /* extended microsecond clock, wraps at 2^32 us (~71 min) */
  uint16_t enc; /* raw TIM4 count, 0..3999 */
} CAN_TickSample_t;

/* The latest TELEM received from the peer node. */
typedef struct {
  uint32_t seq;        /* increments per received frame; 0 = none yet */
  uint32_t arrival_us; /* when it left the Rx FIFO, on THIS node's
                          CAN_TickSample_t.us clock. Exact only while the
                          frame is fresh -- latch it when seq changes. */
  uint16_t enc;        /* peer's raw TIM4 count */
  uint16_t ts_us;      /* peer's own uint16 microsecond timestamp */
} CAN_PeerSample_t;

/* ---- Public API ---- */
void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan);
void CAN_ProcessPendingMessages(void);

/* Samples the encoder, runs the coupling controller and sends TELEM (and
 * DBG). Called once per tick from MC_APP_PostMediumFrequencyHook_M1(),
 * right after CAN_ProcessPendingMessages(). */
void CAN_ControlTick(void);

/* Copies the latest peer TELEM into *out. Returns false until the first
 * peer frame arrives. Call only from the 1 kHz hook context: it relies on
 * that context outranking the FDCAN Rx interrupt. */
bool CAN_GetPeerSample(const CAN_TickSample_t* now, CAN_PeerSample_t* out);

/* Tx-FIFO-full drop counters, incremented when
 * HAL_FDCAN_AddMessageToTxFifoQ() fails. Not currently exposed via the MC
 * register interface -- see MC_REG_SECTOR in sync_registers.c for the
 * pattern if that's added later. */
uint32_t CAN_GetTelemTxDropCount(void);
uint32_t CAN_GetDbgTxDropCount(void);

/* ---- Iq telemetry conditioning ----
 * Defined in the USER CODE blocks of mc_tasks_foc.c, because that is where
 * the 16 kHz HighFrequencyTask lives. Declared here because can_driver.c is
 * the only consumer. Values are in raw s16A, matching FOCVars[].Iqd.q --
 * scale with scaleParams_M1.current to get Amps.
 *
 * NOTE: these live in USER CODE BEGIN/END blocks, so they survive a
 * MotorControl Workbench regeneration. */
void IqTelem_UpdateHF(int16_t iq_s16);
float IqTelem_GetEwma(void);
float IqTelem_GetMeanAndReset(void);

#endif /* CAN_DRIVER_H */
