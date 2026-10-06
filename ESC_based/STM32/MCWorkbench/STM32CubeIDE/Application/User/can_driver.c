/*
 * can_driver.c
 *
 * Transport for the CAN protocol in can_driver.h. Every decision about the
 * Iq reference -- direct SET_IQ, or the coupling law -- lives in
 * couple_ctrl.c; this file only moves frames and latches commands.
 */
#include "can_driver.h"

#include <string.h>

#include "couple_ctrl.h"
#include "main.h"   /* for Error_Handler() */
#include "mc_api.h" /* for MC_StartMotor1, MC_StopMotor1, etc. */
#include "mc_parameters.h" /* for scaleParams_M1 (s16A -> Amps), used only
                              if the Iq sent below is switched to the
                              IqTelem EWMA -- see SendTelemetry() */

#if CAN_PEER_NODE_ID == CAN_NODE_ID
#error "CAN_PEER_NODE_ID must differ from CAN_NODE_ID"
#endif

static FDCAN_HandleTypeDef* s_hfdcan = NULL;

volatile uint32_t g_fdcanCallbackHits = 0;

extern TIM_HandleTypeDef htim4;

/* Latest-value command struct -- "latest wins" model fits your protocol:
 * you don't need historical setpoints, just the newest one.
 *
 * Multi-field commands are written by the Rx interrupt as clear-write-set:
 * pending=false, fields, pending=true. The reader (the 1 kHz hook) outranks
 * the Rx interrupt, so it can land between any two of those writes but can
 * never be interrupted BY them: it either sees pending=false and skips, or
 * sees pending=true with every field of one frame. Without the leading
 * clear, a second frame arriving before the first was consumed could be
 * read half old, half new -- a Kp from one frame and a Kd from the next. */
typedef struct {
  volatile bool start_pending;
  volatile bool stop_pending;
  volatile bool iq_pending;
  volatile float iq_value;
  volatile bool gains_pending;
  volatile float kp;
  volatile float kd;
  volatile bool local_pending;
  volatile float kd_local;
  volatile float iq_max;
  volatile bool mode_pending;
  volatile uint8_t mode;
} CAN_CmdState_t;

static volatile CAN_CmdState_t s_cmd = {0};

/* Latest peer TELEM, double-buffered. The Rx interrupt fills the buffer
 * that is NOT current, then flips s_peerIdx with one aligned store. The
 * reader (CAN_GetPeerSample(), 1 kHz hook) outranks the Rx interrupt, so it
 * can preempt a half-written buffer but only ever reads the other one --
 * and cannot itself be preempted mid-read by a flip. No lock, no torn
 * sample, no waiting in either context. */
typedef struct {
  uint32_t seq;
  uint32_t arrival_cyc; /* DWT->CYCCNT when the frame left the Rx FIFO */
  uint16_t enc;
  uint16_t ts_us;
} CAN_PeerRaw_t;

static volatile CAN_PeerRaw_t s_peerBuf[2];
static volatile uint32_t s_peerIdx = 0u;
static uint32_t s_peerSeq = 0u; /* Rx-interrupt-only */

/* Incremented whenever HAL_FDCAN_AddMessageToTxFifoQ() fails (e.g. hardware
 * Tx FIFO still full because this node keeps losing arbitration to a
 * lower-ID node on the bus).
 *
 * NOT wired into the MC register interface, so nothing reads it at runtime.
 * Read it over SWD, or follow the MC_REG_SECTOR pattern in sync_registers.c
 * to expose it. */
static volatile uint32_t s_telemTxDropCount = 0;
static volatile uint32_t s_dbgTxDropCount = 0;

uint32_t CAN_GetTelemTxDropCount(void) { return s_telemTxDropCount; }
uint32_t CAN_GetDbgTxDropCount(void) { return s_dbgTxDropCount; }

/* ---- Microsecond timebase ----
 *
 * The Pi used to derive dt from its own arrival times, which folds every
 * source of transport jitter -- Tx FIFO wait, arbitration, kernel IRQ
 * latency, the GIL -- straight into any velocity computed from consecutive
 * encoder samples. Timestamping at the source makes dt a property of the
 * MCU's clock instead, and the transport can then be as late as it likes
 * without corrupting the derivative. The peer ESC now relies on the same
 * property for its own velocity estimate of this shaft.
 *
 * DWT->CYCCNT is the timebase because it is free: a core-resident 32-bit
 * cycle counter on the Cortex-M4, no timer peripheral to allocate and -- the
 * reason that matters here -- no .ioc change, so MotorControl Workbench
 * regeneration cannot silently revert it (see the Gotchas in
 * CAN_setup_reference.md).
 *
 * BUG FIXED HERE: the original timestamp was (CYCCNT / cycles_per_us) &
 * 0xFFFF. CYCCNT wraps every 2^32 cycles -- 25.26 s at 170 MHz -- and at
 * that moment CYCCNT / 170 drops from 25264513 to 0, not to a multiple of
 * 65536. So every 25.26 s the uint16 timestamp jumped backwards by ~32 ms,
 * inside the half-range an unwrapper accepts as genuine, and any velocity
 * differenced across that sample came out with a bogus dt. The clock below
 * accumulates CYCCNT DELTAS instead (wrap-safe as long as it is sampled
 * more often than every 25 s -- it is sampled every millisecond), so the
 * truncated uint16 now wraps exactly every 65536 us, as the Pi assumes.
 *
 * Divisor comes from SystemCoreClock rather than a hardcoded 170 so that a
 * clock-tree change cannot quietly rescale every dt. */
static uint32_t s_cyclesPerUs = 1u;
static uint32_t s_clockLastCyc = 0u; /* SampleTick()-only state */
static uint32_t s_clockRemCyc = 0u;
static uint32_t s_clockUs = 0u;

static void MicroClock_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  s_cyclesPerUs = SystemCoreClock / 1000000u;
  if (s_cyclesPerUs == 0u) {
    s_cyclesPerUs = 1u; /* never divide by zero if the clock is unset */
  }
  s_clockLastCyc = 0u;
  s_clockRemCyc = 0u;
  s_clockUs = 0u;
}

/* Samples the encoder and both clocks back to back, and advances the
 * extended microsecond clock. Hook context only: it is the clock's single
 * writer. */
static void SampleTick(CAN_TickSample_t* s) {
  uint32_t const cyc = DWT->CYCCNT;
  uint16_t const enc = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);

  s_clockRemCyc += cyc - s_clockLastCyc; /* unsigned: wrap-safe */
  s_clockLastCyc = cyc;
  uint32_t const us = s_clockRemCyc / s_cyclesPerUs;
  s_clockRemCyc -= us * s_cyclesPerUs;
  s_clockUs += us;

  s->cyc = cyc;
  s->us = s_clockUs;
  s->enc = enc;
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
   * act on frames -- it latches them, and they are applied by the 1 kHz
   * medium-frequency hook. A frame therefore waits up to 1 ms to take
   * effect no matter what this number is, so ISR latency of tens of
   * microseconds is invisible.
   *
   * ESC_based relies on this ordering too: the hook (SysTick, priority 4)
   * must outrank this interrupt, because the lock-free handoffs above --
   * s_cmd and the peer double buffer -- are only correct when the reader
   * cannot be preempted by the writer.
   *
   * 5 sits below the FOC loop (2), below TIM1_BRK (4) and below SysTick
   * (TICK_INT_PRIORITY, 4), and above nothing that matters.
   *
   * Set here as well as in HAL_FDCAN_MspInit() (and the .ioc that generates
   * it) because MspInit runs first, during HAL_FDCAN_Init(); this call is
   * what actually takes effect. Keep all three in agreement -- if Workbench
   * regenerates msp.c from a stale .ioc, this line is the backstop. */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  /* The FDCAN evaluates only the first StdFiltersNbr filter elements, and
   * says nothing about the rest: with the .ioc's old value of 1, filter 1
   * below would be accepted by HAL_FDCAN_ConfigFilter() and then silently
   * never match -- no peer frames, ever, and PEER mode would just never
   * engage. FDCAN1.StdFiltersNbr is 2 in this project's .ioc; this is the
   * backstop if a regeneration from a stale .ioc puts 1 back. Re-running
   * HAL_FDCAN_Init() before HAL_FDCAN_Start() is legitimate: the handle is
   * READY, so it skips MspInit and just re-lays-out the message RAM. */
  if (s_hfdcan->Init.StdFiltersNbr < CAN_STD_FILTER_COUNT) {
    s_hfdcan->Init.StdFiltersNbr = CAN_STD_FILTER_COUNT;
    if (HAL_FDCAN_Init(s_hfdcan) != HAL_OK) {
      Error_Handler();
    }
  }

  /* Filter 0: this node's commands, START..COUPLE_MODE inclusive. */
  FDCAN_FilterTypeDef filt = {0};
  filt.IdType = FDCAN_STANDARD_ID;
  filt.FilterIndex = 0;
  filt.FilterType = FDCAN_FILTER_RANGE;
  filt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filt.FilterID1 = CAN_ID_START(CAN_NODE_BASE);
  filt.FilterID2 = CAN_ID_COUPLE_MODE(CAN_NODE_BASE);
  HAL_StatusTypeDef st1 = HAL_FDCAN_ConfigFilter(s_hfdcan, &filt);

  /* Filter 1: exactly the peer's TELEM. Nothing else the peer sends --
     its DBG, or the Pi's commands to it -- crosses into this node. */
  filt.FilterIndex = 1;
  filt.FilterType = FDCAN_FILTER_MASK;
  filt.FilterID1 = CAN_ID_TELEM(CAN_PEER_BASE);
  filt.FilterID2 = 0x7FF; /* all 11 ID bits must match */
  HAL_StatusTypeDef st2 = HAL_FDCAN_ConfigFilter(s_hfdcan, &filt);

  HAL_StatusTypeDef st3 =
      HAL_FDCAN_ConfigGlobalFilter(s_hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_StatusTypeDef st4 = HAL_FDCAN_ActivateNotification(
      s_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_StatusTypeDef st5 = HAL_FDCAN_Start(s_hfdcan);

  if (st1 != HAL_OK || st2 != HAL_OK || st3 != HAL_OK || st4 != HAL_OK ||
      st5 != HAL_OK) {
    /* One or more FDCAN setup steps failed. Trap here rather than limping
       on with a half-configured peripheral. */
    Error_Handler();
  }
}

static float FloatAt(const uint8_t* p) {
  float f;
  memcpy(&f, p, sizeof(float)); /* float LE, per the protocol */
  return f;
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
 * OVERFLOW BUDGET:
 *   Rx FIFO 0 holds 3 elements (SRAMCAN_RF0_NBR, fixed in the G4's message
 *   RAM layout). Peer TELEM now arrives continuously at 1 kHz on top of the
 *   Pi's commands. This handler can be held off by the FOC ISR (priority 2)
 *   and by the 1 kHz SysTick work (MF task plus hook, priority 4) -- tens
 *   of microseconds each, well under 100 us together. Filling 3 slots needs
 *   3 frames back to back: an 8-byte frame (TELEM, SET_IQ, COUPLE_*) is
 *   ~110 us of wire time at 1 Mbit/s and even a zero-byte START is ~50 us,
 *   so 3 frames take >=150 us to arrive, and that is the adversarial case.
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

    /* Only this node's command IDs and the peer's TELEM pass the filters
       configured above. Length is checked per frame: a short frame has no
       valid interpretation, and reading past its payload would latch
       stale bytes as a real command. */
    switch (rxHeader.Identifier) {
      case CAN_ID_START(CAN_NODE_BASE):
        s_cmd.start_pending = true;
        break;
      case CAN_ID_STOP(CAN_NODE_BASE):
        s_cmd.stop_pending = true;
        break;
      case CAN_ID_SET_IQ(CAN_NODE_BASE):
        if (rxHeader.DataLength != FDCAN_DLC_BYTES_4) break;
        s_cmd.iq_value = FloatAt(&rxData[0]);
        s_cmd.iq_pending = true;
        break;
      case CAN_ID_COUPLE_GAINS(CAN_NODE_BASE):
        if (rxHeader.DataLength != FDCAN_DLC_BYTES_8) break;
        s_cmd.gains_pending = false;
        s_cmd.kp = FloatAt(&rxData[0]);
        s_cmd.kd = FloatAt(&rxData[4]);
        s_cmd.gains_pending = true;
        break;
      case CAN_ID_COUPLE_LOCAL(CAN_NODE_BASE):
        if (rxHeader.DataLength != FDCAN_DLC_BYTES_8) break;
        s_cmd.local_pending = false;
        s_cmd.kd_local = FloatAt(&rxData[0]);
        s_cmd.iq_max = FloatAt(&rxData[4]);
        s_cmd.local_pending = true;
        break;
      case CAN_ID_COUPLE_MODE(CAN_NODE_BASE):
        if (rxHeader.DataLength == FDCAN_DLC_BYTES_0) break;
        s_cmd.mode_pending = false;
        s_cmd.mode = rxData[0];
        s_cmd.mode_pending = true;
        break;
      case CAN_ID_TELEM(CAN_PEER_BASE): {
        if (rxHeader.DataLength != FDCAN_DLC_BYTES_8) break;
        uint32_t const w = s_peerIdx ^ 1u;
        s_peerBuf[w].arrival_cyc = DWT->CYCCNT;
        s_peerBuf[w].enc = (uint16_t)(rxData[4] | (rxData[5] << 8));
        s_peerBuf[w].ts_us = (uint16_t)(rxData[6] | (rxData[7] << 8));
        s_peerBuf[w].seq = ++s_peerSeq;
        __DMB(); /* the sample is complete before it becomes current */
        s_peerIdx = w;
        break;
      }
      default:
        break; /* shouldn't happen given the filters, but stay defensive */
    }
  }

  /* No re-arm needed: FDCAN_IT_RX_FIFO0_NEW_MESSAGE stays enabled in IE
     across interrupts. HAL_FDCAN_IRQHandler() clears only the IR flag. */
}

bool CAN_GetPeerSample(const CAN_TickSample_t* now, CAN_PeerSample_t* out) {
  uint32_t const r = s_peerIdx;
  uint32_t const seq = s_peerBuf[r].seq;
  if (seq == 0u) {
    return false;
  }
  out->seq = seq;
  out->enc = s_peerBuf[r].enc;
  out->ts_us = s_peerBuf[r].ts_us;
  /* Cycle deltas are exact up to ~25 s, far longer than a sample stays
     fresh; couple_ctrl.c latches this only when seq changes. */
  out->arrival_us =
      now->us - (now->cyc - s_peerBuf[r].arrival_cyc) / s_cyclesPerUs;
  return true;
}

/* Called from MC_APP_PostMediumFrequencyHook_M1, NOT from ISR context.
 * Order matters only at the end: STOP is applied last so that it wins over
 * anything else that arrived in the same millisecond. */
void CAN_ProcessPendingMessages(void) {
  if (s_hfdcan == NULL) {
    return; /* hook can run before CAN_Driver_Init() -- see CAN_ControlTick() */
  }
  if (s_cmd.start_pending) {
    s_cmd.start_pending = false;
    MC_StartMotor1();
  }
  if (s_cmd.gains_pending) {
    float const kp = s_cmd.kp;
    float const kd = s_cmd.kd;
    s_cmd.gains_pending = false;
    (void)Couple_SetGains(kp, kd); /* invalid (negative, NaN) is ignored */
  }
  if (s_cmd.local_pending) {
    float const kd_local = s_cmd.kd_local;
    float const iq_max = s_cmd.iq_max;
    s_cmd.local_pending = false;
    (void)Couple_SetLocal(kd_local, iq_max);
  }
  if (s_cmd.mode_pending) {
    uint8_t const mode = s_cmd.mode;
    s_cmd.mode_pending = false;
    Couple_RequestMode(mode);
  }
  if (s_cmd.iq_pending) {
    float const iq = s_cmd.iq_value;
    s_cmd.iq_pending = false;
    /* Applied (or dropped, outside RUN or while a mode is requested) by
       Couple_Tick() later in this same tick. */
    Couple_SetDirectIq(iq);
  }
  if (s_cmd.stop_pending) {
    s_cmd.stop_pending = false;
    Couple_RequestMode(COUPLE_MODE_OFF);
    MC_StopMotor1();
  }
}

static void FillTxHeader(FDCAN_TxHeaderTypeDef* hdr, uint32_t id) {
  memset(hdr, 0, sizeof(*hdr));
  hdr->Identifier = id;
  hdr->IdType = FDCAN_STANDARD_ID;
  hdr->TxFrameType = FDCAN_DATA_FRAME;
  hdr->DataLength = FDCAN_DLC_BYTES_8;
  hdr->FDFormat = FDCAN_CLASSIC_CAN;
  hdr->BitRateSwitch = FDCAN_BRS_OFF;
  hdr->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
}

static void SendTelemetry(const CAN_TickSample_t* s) {
  FDCAN_TxHeaderTypeDef hdr;
  FillTxHeader(&hdr, CAN_ID_TELEM(CAN_NODE_BASE));

  qd_f_t iqd = MC_GetIqdMotor1_F(); /* .q = torque-producing current, .d = 0 in
                                       your control scheme */

  /* uint16: TIM4's ARR is M1_PULSE_NBR (= 4*PPR - 1 = 3999), so the counter
     spans 0..3999. Sampled at the top of the tick (SampleTick()), the same
     instant the coupling law used. */
  uint16_t const encCount = s->enc;
  uint16_t const tstamp_us = (uint16_t)s->us;

  /* The 16 kHz conditioned-Iq accumulator (IqTelem_UpdateHF) is still
     running and must still be drained exactly once per telemetry period --
     GetMeanAndReset clears it, and left uncalled the sum grows without bound
     and loses float precision. Its result is not transmitted. The
     accumulator is kept because IqTelem_GetEwma() is the obvious fix if raw
     Iq proves too noisy to teleoperate on -- swapping iqd.q below for
     IqTelem_GetEwma() * scaleParams_M1.current is then a one-line change on
     this side and none at all on the Pi's. */
  (void)IqTelem_GetMeanAndReset();

  /* SRAMCAN_TFQ_NBR is hardcoded to 3 in stm32g4xx_hal_fdcan.c, so a frame
   * offered while three are still queued is dropped outright rather than
   * delayed. TELEM and DBG make two per tick, which leaves one element of
   * slack for a frame still waiting from the previous tick.
   *
   * Layout must stay in lockstep with the TELEM comment in can_driver.h
   * and _handle_message() in can_interface.py. */
  uint8_t payload[8];
  memcpy(&payload[0], &iqd.q, sizeof(float));
  memcpy(&payload[4], &encCount, sizeof(uint16_t));
  memcpy(&payload[6], &tstamp_us, sizeof(uint16_t));

  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, payload) != HAL_OK) {
    s_telemTxDropCount++;
  }
}

static int16_t SaturateI16(int32_t v) {
  if (v > INT16_MAX) return INT16_MAX;
  if (v < INT16_MIN) return INT16_MIN;
  return (int16_t)v;
}

static void SendDebug(const CAN_TickSample_t* s) {
  static uint32_t idleTicks = 0u;
  static uint8_t lastFlags = 0u;

  Couple_Status_t st;
  Couple_GetStatus(&st);

  /* Every tick while coupling is requested or engaged, and on the very tick
     the status byte changes; 10 Hz otherwise. Without the change rule the
     tick a trip happened on would never be sent -- a trip drops coupling,
     which drops DBG to 10 Hz, so the trip code would first reach the log
     up to 100 ms after the event. */
  bool const active =
      (st.flags & (COUPLE_STATUS_ENGAGED | COUPLE_STATUS_REQUESTED)) != 0u;
  bool const changed = (st.flags != lastFlags);
  if (!active && !changed && (++idleTicks < CAN_DBG_IDLE_DIVIDER)) {
    return;
  }
  idleTicks = 0u;
  lastFlags = st.flags;

  /* mA, rounded half away from zero. Clamped as a float first: the direct
     path passes SET_IQ through unclamped (as the original firmware did),
     and converting an out-of-range float to an integer is undefined. */
  float ma = st.iq_cmd_a * 1000.0f;
  if (ma > (float)INT16_MAX) ma = (float)INT16_MAX;
  if (ma < (float)INT16_MIN) ma = (float)INT16_MIN;
  int16_t const iq_ma = SaturateI16((int32_t)(ma + (ma >= 0.0f ? 0.5f : -0.5f)));
  int16_t const err = SaturateI16(st.err_counts);
  uint16_t const tstamp_us = (uint16_t)s->us;
  uint32_t const age_01ms = st.peer_age_us / 100u;
  uint8_t const age = (age_01ms > 255u) ? 255u : (uint8_t)age_01ms;

  uint8_t payload[8];
  memcpy(&payload[0], &iq_ma, sizeof(int16_t));
  memcpy(&payload[2], &err, sizeof(int16_t));
  memcpy(&payload[4], &tstamp_us, sizeof(uint16_t));
  payload[6] = st.flags;
  payload[7] = age;

  FDCAN_TxHeaderTypeDef hdr;
  FillTxHeader(&hdr, CAN_ID_DBG(CAN_NODE_BASE));
  if (HAL_FDCAN_AddMessageToTxFifoQ(s_hfdcan, &hdr, payload) != HAL_OK) {
    s_dbgTxDropCount++;
  }
}

/* The scheduler starts running this hook inside MX_MotorControl_Init(),
 * which main() calls BEFORE MX_FDCAN1_Init() and CAN_Driver_Init(). Until
 * the driver is up there is no peripheral to send on and no clock to read,
 * so the first tick or two do nothing at all. */
void CAN_ControlTick(void) {
  if (s_hfdcan == NULL) {
    return;
  }
  CAN_TickSample_t s;
  SampleTick(&s);
  Couple_Tick(&s);
  SendTelemetry(&s); /* TELEM first: the peer's coupling law is waiting on it */
  SendDebug(&s);
}
