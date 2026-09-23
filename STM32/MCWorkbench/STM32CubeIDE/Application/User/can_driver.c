/*
 * can_driver.c
 */
#include "can_driver.h"

#include <string.h>

#include "main.h"   /* for Error_Handler() */
#include "mc_api.h" /* for MC_StartMotor1, MC_StopMotor1, etc. */
#include "mc_parameters.h" /* for scaleParams_M1 (s16A -> Amps), used only
                              if the Iq sent below is switched to the
                              IqTelem EWMA -- see CAN_SendTelemetry() */

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
 * telemetry message (e.g. hardware Tx FIFO still full because this node
 * keeps losing arbitration to a lower-ID node on the bus).
 *
 * NOT wired into the MC register interface, so nothing reads it at runtime
 * -- which is why the ENC_COUNT overflow described in CAN_SendTelemetry()
 * went unnoticed for so long. Read it over SWD, or follow the MC_REG_SECTOR
 * pattern in sync_registers.c to expose it. One counter now, because there
 * is one telemetry frame. */
static volatile uint32_t s_telemTxDropCount = 0;

uint32_t CAN_GetTelemTxDropCount(void) { return s_telemTxDropCount; }

/* ---- Microsecond timebase for the telemetry timestamp ----
 *
 * The Pi used to derive dt from its own arrival times, which folds every
 * source of transport jitter -- Tx FIFO wait, arbitration, kernel IRQ
 * latency, the GIL -- straight into any velocity computed from consecutive
 * encoder samples. telemetry_rate_probe.py measured a p99 of 0.27 ms on a
 * filtered socket and far worse before that, against a 1 ms sample period:
 * the same jitter as the signal. Timestamping at the source makes dt a
 * property of the MCU's clock instead, and the transport can then be as
 * late as it likes without corrupting the derivative.
 *
 * DWT->CYCCNT is the timebase because it is free: a core-resident 32-bit
 * cycle counter on the Cortex-M4, no timer peripheral to allocate and -- the
 * reason that matters here -- no .ioc change, so MotorControl Workbench
 * regeneration cannot silently revert it (see the Gotchas in
 * CAN_setup_reference.md).
 *
 * Divisor comes from SystemCoreClock rather than a hardcoded 170 so that a
 * clock-tree change cannot quietly rescale every dt on the Pi. */
static uint32_t s_cyclesPerUs = 1u;

static void MicroClock_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  s_cyclesPerUs = SystemCoreClock / 1000000u;
  if (s_cyclesPerUs == 0u) {
    s_cyclesPerUs = 1u; /* never divide by zero if the clock is unset */
  }
}

/* Truncates to 16 bits on purpose. The field wraps every 65.536 ms, which is
 * 65 telemetry periods -- so consecutive samples are never ambiguous and the
 * Pi can unwrap it the same way it unwraps the encoder. */
static uint16_t MicroClock_Now_u16(void) {
  return (uint16_t)((DWT->CYCCNT / s_cyclesPerUs) & 0xFFFFu);
}

void CAN_Driver_Init(FDCAN_HandleTypeDef* hfdcan) {
  s_hfdcan = hfdcan;

  /* Before anything can be timestamped. Cheap and idempotent. */
  MicroClock_Init();

  /* PRIORITY 5, DELIBERATELY LOW -- do not raise this.
   *
   * This used to be (0, 0), the highest preemption priority on the chip,
   * which put it ABOVE the 16 kHz FOC current loop (ADC1_2_IRQn at
   * priority 2, main.c). Under NVIC_PRIORITYGROUP_3 a lower number wins, so
   * every arriving SET_IQ preempted TSK_HighFrequencyTask() mid-computation
   * to run a handler that does nothing more urgent than set a bool. Textbook
   * priority inversion: the hard-real-time task was interruptible by the
   * soft one.
   *
   * There is no latency argument for keeping it high. The callback does not
   * act on commands -- it latches them into s_cmd, and they are applied by
   * CAN_ProcessPendingMessages() from the 1 kHz medium-frequency hook. A
   * command therefore waits up to 1 ms to take effect no matter what this
   * number is, so ISR latency of tens of microseconds is invisible.
   *
   * 5 sits below the FOC loop (2), below TIM1_BRK (4) and below SysTick
   * (TICK_INT_PRIORITY, 4), and above nothing that matters.
   *
   * The Rx FIFO absorbs the added delay with room to spare -- see the
   * overflow budget in HAL_FDCAN_RxFifo0Callback().
   *
   * Set here as well as in HAL_FDCAN_MspInit() (and the .ioc that generates
   * it) because MspInit runs first, during HAL_FDCAN_Init(); this call is
   * what actually takes effect. Keep all three in agreement -- if Workbench
   * regenerates msp.c from a stale .ioc, this line is the backstop. */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
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

/* Overrides the HAL weak callback -- fires in ISR context on new message.
 *
 * DRAINS THE FIFO IN A LOOP. This is not defensive padding; reading a single
 * message here is incorrect. HAL_FDCAN_IRQHandler() clears the RF0N (new
 * message) flag BEFORE invoking this callback, and RF0N is a single bit, not
 * a count. So if two frames are already queued when this runs, reading one
 * leaves the other stranded: no new arrival means no new interrupt, and the
 * FIFO stays one message behind forever -- every command applied is the
 * PREVIOUS one, and a third arrival overflows. Draining to the fill level
 * makes the handler correct regardless of how long it was held off.
 *
 * OVERFLOW BUDGET, now that this runs below the FOC loop:
 *   Rx FIFO 0 holds 3 elements (SRAMCAN_RF0_NBR, fixed in the G4's message
 *   RAM layout). The longest this handler can be held off is one execution
 *   of the 16 kHz FOC ISR, which must itself fit inside 62.5 us. Filling 3
 *   slots needs 3 frames back to back; the shortest command frame (START,
 *   zero data bytes) occupies ~50 us of wire time at 1 Mbit/s and SET_IQ
 *   ~80 us, so 3 of them take >=150 us to arrive. 150 us of buffer against
 *   <62.5 us of worst-case starvation, and that is the adversarial case --
 *   the Pi actually sends commands at a few hundred Hz, milliseconds apart.
 *   Losing a frame to priority is not a realistic failure mode here.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan,
                               uint32_t RxFifo0ITs) {
  g_fdcanCallbackHits++;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) !=
        HAL_OK) {
      break;
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
  }

  /* No re-arm needed: FDCAN_IT_RX_FIFO0_NEW_MESSAGE stays enabled in IE
     across interrupts. HAL_FDCAN_IRQHandler() clears only the IR flag. */
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

  /* uint16, not the uint32 this used to be. TIM4's ARR is M1_PULSE_NBR
     (= 4*PPR - 1 = 3999), so the counter spans 0..3999 and the top two bytes
     were always zero -- they now carry the timestamp instead. */
  uint16_t encCount = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

  /* Sampled here, next to the values it describes, so it dates the payload
     and not the moment the frame won arbitration. */
  uint16_t tstamp_us = MicroClock_Now_u16();

  /* The 16 kHz conditioned-Iq accumulator (IqTelem_UpdateHF) is still
     running and must still be drained exactly once per telemetry period --
     GetMeanAndReset clears it, and left uncalled the sum grows without bound
     and loses float precision. Its result is no longer transmitted: nothing
     on the Pi ever read iq_mean or iq_ewma, and the merged frame has no room
     for them. The accumulator is kept rather than ripped out because
     IqTelem_GetEwma() is the obvious fix if raw Iq proves too noisy to
     teleoperate on -- swapping iqd.q below for IqTelem_GetEwma() *
     scaleParams_M1.current is then a one-line change on this side and none
     at all on the Pi's. */
  (void)IqTelem_GetMeanAndReset();

  /* ONE frame per tick. The three-Tx-element limit that used to dominate
   * this function (SRAMCAN_TFQ_NBR is hardcoded to 3 in
   * stm32g4xx_hal_fdcan.c, so a fourth frame offered in the same cycle is
   * dropped outright rather than delayed) no longer binds -- but the reason
   * it is gone is that the frame count came down, not that the limit did.
   *
   * Everything the Pi actually reads is Iq and the encoder count, and both
   * fit in 8 bytes alongside the timestamp. Sending them together is not
   * only cheaper (two nodes at 1 kHz: 6000 frames/s -> 2000, and one Pi-side
   * interrupt per tick instead of three) -- it also makes the two signals
   * share a single timestamp by construction. They previously arrived in
   * separate frames whose relative delay nothing controlled, so any
   * controller reading position and current together had a skew it could
   * neither measure nor bound. That skew is now structurally zero.
   *
   * Layout must stay in lockstep with CAN_ID_TELEM's comment in
   * can_driver.h and _handle_message() in can_interface.py. */
  uint8_t payload[8];
  memcpy(&payload[0], &iqd.q, sizeof(float));
  memcpy(&payload[4], &encCount, sizeof(uint16_t));
  memcpy(&payload[6], &tstamp_us, sizeof(uint16_t));

  hdr.Identifier = CAN_ID_TELEM(CAN_NODE_BASE);
  hdr.DataLength = FDCAN_DLC_BYTES_8;
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, payload) != HAL_OK) {
    s_telemTxDropCount++;
  }
}

