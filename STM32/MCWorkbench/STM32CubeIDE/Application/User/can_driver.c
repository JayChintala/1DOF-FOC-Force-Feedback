/*
 * can_driver.c
 */
#include "can_driver.h"

#include <string.h>

#include "main.h"   /* for Error_Handler() */
#include "mc_api.h" /* for MC_StartMotor1, MC_StopMotor1, etc. */
#include "mc_parameters.h" /* for scaleParams_M1 (s16A -> Amps) */

static FDCAN_HandleTypeDef* s_hfdcan = NULL;

volatile uint32_t g_fdcanCallbackHits = 0;

extern TIM_HandleTypeDef htim4;

/* Latest-value command struct -- "latest wins" model fits your protocol:
 * you don't need historical Iq setpoints, just the newest one. */
typedef struct {
  volatile bool start_pending;
  volatile bool stop_pending;
  volatile bool iq_pending;
  volatile float iq_value;
} CAN_CmdState_t;

static volatile CAN_CmdState_t s_cmd = {0};

/* Incremented whenever HAL_FDCAN_AddMessageToTxFifoQ() fails for the
 * corresponding telemetry message (e.g. hardware Tx FIFO still full because
 * this node keeps losing arbitration to a lower-ID node on the bus).
 *
 * NOT wired into the MC register interface, so nothing reads them at runtime
 * -- which is why the ENC_COUNT overflow described in CAN_SendTelemetry()
 * went unnoticed. Read them over SWD, or follow the MC_REG_SECTOR pattern in
 * sync_registers.c to expose them. There is no separate angle counter: the
 * electrical angle rides in the IQ_MEAN frame. */
static volatile uint32_t s_iqTxDropCount = 0;
static volatile uint32_t s_iqMeanTxDropCount = 0;
static volatile uint32_t s_encTxDropCount = 0;

uint32_t CAN_GetIqTxDropCount(void) { return s_iqTxDropCount; }
uint32_t CAN_GetIqMeanTxDropCount(void) { return s_iqMeanTxDropCount; }
uint32_t CAN_GetEncTxDropCount(void) { return s_encTxDropCount; }

void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan) {
  s_hfdcan = hfdcan;

  /* Manually enable NVIC for FDCAN1_IT0 -- CubeMX codegen isn't emitting
     this despite .ioc having it correctly configured (verified in raw .ioc). */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  FDCAN_FilterTypeDef filt = {0};
  filt.IdType = FDCAN_STANDARD_ID;
  filt.FilterIndex = 0;
  filt.FilterType = FDCAN_FILTER_RANGE;
  filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filt.FilterID1 = CAN_ID_START(CAN_NODE_BASE); /* this node's START */
  filt.FilterID2 =
      CAN_ID_SET_IQ(CAN_NODE_BASE); /* this node's SET_IQ, inclusive range */

  HAL_StatusTypeDef st1 = HAL_FDCAN_ConfigFilter(s_hfdcan, &filt);
  HAL_StatusTypeDef st2 =
      HAL_FDCAN_ConfigGlobalFilter(s_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_StatusTypeDef st3 = HAL_FDCAN_ActivateNotification(
      s_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_StatusTypeDef st4 = HAL_FDCAN_Start(s_hfdcan);

  if (st1 != HAL_OK || st2 != HAL_OK || st3 != HAL_OK || st4 != HAL_OK) {
    /* One or more FDCAN setup steps failed. Trap here rather than limping
       on with a half-configured peripheral. */
    Error_Handler();
  }
}

/* Overrides the HAL weak callback -- fires in ISR context on new message */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan,
                               uint32_t RxFifo0ITs) {
  g_fdcanCallbackHits++;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) !=
      HAL_OK) {
    return;
  }

  /* Only this node's IDs pass the RX filter configured above, so no
     explicit node-base check is needed here -- the filter already
     guarantees rxHeader.Identifier belongs to CAN_NODE_BASE. */
  switch (rxHeader.Identifier) {
    case CAN_ID_START(CAN_NODE_BASE):
      s_cmd.start_pending = true;
      break;
    case CAN_ID_STOP(CAN_NODE_BASE):
      s_cmd.stop_pending = true;
      break;
    case CAN_ID_SET_IQ(CAN_NODE_BASE): {
      float iq;
      memcpy((void*)&iq, rxData,
             sizeof(float)); /* float LE, per your protocol */
      s_cmd.iq_value = iq;
      s_cmd.iq_pending = true;
      break;
    }
    default:
      break; /* shouldn't happen given filter, but stay defensive */
  }

  /* Re-arm notification if your MCU/HAL version requires it after each RX --
     check your HAL version's FDCAN interrupt handling; some auto-rearm, some
     don't. */
}

/* Called from MC_APP_PostMediumFrequencyHook_M1, NOT from ISR context */
void CAN_ProcessPendingMessages(void) {
  if (s_cmd.start_pending) {
    s_cmd.start_pending = false;
    MC_StartMotor1();
  }
  if (s_cmd.stop_pending) {
    s_cmd.stop_pending = false;
    MC_StopMotor1();
  }
  if (s_cmd.iq_pending) {
    s_cmd.iq_pending = false;
    if (MC_GetSTMStateMotor1() == RUN) {
      qd_f_t iqdRef;
      iqdRef.q = s_cmd.iq_value;
      iqdRef.d = 0.0f;
      MC_SetCurrentReferenceMotor1_F(iqdRef);
    }
    /* else: silently dropped -- Pi is responsible for sequencing START before
     * SET_IQ */
  }
}

void CAN_SendTelemetry(void) {
  FDCAN_TxHeaderTypeDef hdr = {0};
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

  qd_f_t iqd = MC_GetIqdMotor1_F(); /* .q = torque-producing current, .d = 0 in
                                       your control scheme */
  int16_t elAngle = MC_GetElAngledppMotor1(); /* electrical angle, DPP format */
  uint32_t encCount =
      __HAL_TIM_GET_COUNTER(&htim4); /* raw mechanical position from encoder */

  /* Conditioned Iq, accumulated by IqTelem_UpdateHF() at 16 kHz. Scaled here
   * with the same factor MCI_GetIqd_F() applies, so all three values below
   * are in Amps and overlay directly on one plot. Reading it from the live
   * scaleParams_M1 rather than recomputing from RSHUNT/AMPLIFICATION_GAIN
   * means a future gain change has exactly one place to edit.
   *
   * GetMeanAndReset must be called once per telemetry period and no more --
   * it clears the accumulator. */
  float const iqScale = scaleParams_M1.current;
  float const iqEwma = IqTelem_GetEwma() * iqScale;
  float const iqMean = IqTelem_GetMeanAndReset() * iqScale;

  /* HARD LIMIT: exactly three frames may be queued here, no more.
   *
   * The STM32G4 FDCAN message RAM has a fixed layout and SRAMCAN_TFQ_NBR is
   * hardcoded to 3 (stm32g4xx_hal_fdcan.c) -- three Tx elements, not
   * configurable. An element stays occupied until its frame has finished on
   * the wire, and an 8-byte standard frame at 1 Mbit/s takes ~115 us while
   * queuing all of them takes a few us. So a fourth offer always arrives
   * with TFQF still set and HAL_FDCAN_AddMessageToTxFifoQ returns HAL_ERROR
   * immediately (see its TFQF check). The fourth frame is not delayed, it is
   * dropped, every single 1 kHz cycle.
   *
   * That is exactly what a previous version of this function did: it queued
   * IQ_READBACK, IQ_MEAN, ELEC_ANGLE, ENC_COUNT and silently lost ENC_COUNT
   * on every cycle, taking position feedback on the Pi with it. Adding a
   * fourth frame was what tipped it over -- the original three fit exactly.
   *
   * ELEC_ANGLE is therefore folded into the IQ_MEAN frame rather than being
   * sent separately: it is only 2 bytes, and IQ_MEAN had 4 spare. The
   * standalone 0x013 ID is retired. If you ever need another signal, pack it
   * into the spare bytes below -- do not add a frame. */

  /* Bytes 0..3 raw (untouched, as before), 4..7 EWMA. Widening this frame is
   * backward compatible: can_interface.py matches on len(data) >= 4 and
   * slices [:4], so a consumer that only wants the raw value is unaffected. */
  uint8_t iqPayload[8];
  memcpy(&iqPayload[0], &iqd.q, sizeof(float));
  memcpy(&iqPayload[4], &iqEwma, sizeof(float));

  hdr.Identifier = CAN_ID_IQ_READBACK(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, iqPayload) != HAL_OK) {
    s_iqTxDropCount++;
  }

  /* Bytes 0..3 boxcar mean, 4..5 electrical angle (DPP). 2 bytes spare. */
  uint8_t meanPayload[6];
  memcpy(&meanPayload[0], &iqMean, sizeof(float));
  memcpy(&meanPayload[4], &elAngle, sizeof(int16_t));

  hdr.Identifier = CAN_ID_IQ_MEAN(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_6;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, meanPayload) != HAL_OK) {
    s_iqMeanTxDropCount++;
  }

  /* Last of the three, so it is the one that suffers if the invariant above
   * is ever violated again -- and it is the one that matters most. Keep it
   * last only while the count is 3. */
  hdr.Identifier = CAN_ID_ENC_COUNT(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_4;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, (uint8_t*)&encCount) !=
      HAL_OK) {
    s_encTxDropCount++;
  }
}

