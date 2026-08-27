/*
 * can_driver.c
 */
#include "can_driver.h"

#include <string.h>

#include "main.h"   /* for Error_Handler() */
#include "mc_api.h" /* for MC_StartMotor1, MC_StopMotor1, etc. */

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
 * this node keeps losing arbitration to a lower-ID node on the bus). Not
 * wired into the MC register interface yet (see MC_REG_SECTOR in
 * sync_registers.c for the pattern to follow) -- these accessors exist so
 * that can be added without touching this file again. */
static volatile uint32_t s_iqTxDropCount = 0;
static volatile uint32_t s_angleTxDropCount = 0;
static volatile uint32_t s_encTxDropCount = 0;

uint32_t CAN_GetIqTxDropCount(void) { return s_iqTxDropCount; }
uint32_t CAN_GetAngleTxDropCount(void) { return s_angleTxDropCount; }
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

  hdr.Identifier = CAN_ID_IQ_READBACK(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_4;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, (uint8_t*)&iqd.q) !=
      HAL_OK) {
    s_iqTxDropCount++;
  }

  hdr.Identifier = CAN_ID_ELEC_ANGLE(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_2;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, (uint8_t*)&elAngle) !=
      HAL_OK) {
    s_angleTxDropCount++;
  }

  hdr.Identifier = CAN_ID_ENC_COUNT(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_4;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, (uint8_t*)&encCount) !=
      HAL_OK) {
    s_encTxDropCount++;
  }
}
