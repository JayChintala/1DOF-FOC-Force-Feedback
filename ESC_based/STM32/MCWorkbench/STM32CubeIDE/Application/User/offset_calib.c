/*
 * offset_calib.c
 *
 * CHANGE 5 -- see offset_calib.h and FIRMWARE_CHANGES.txt. Strong overrides
 * of two __weak functions in the R3_2 driver
 * (MCSDK_v6.4.2-Full/.../G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c, git-ignored and
 * left untouched). Both are reached only through the function pointers in
 * mc_config.c (pFctCurrReadingCalib, pFctGetPhaseCurrents), so the linker's
 * choice of the strong definition is the whole mechanism.
 *
 * CALIBRATION SEQUENCE (R3_2_CurrentReadingPolarization below):
 *   1. Shaft guard: if |speed| > OFFSET_CALIB_MAX_SPEED_01HZ, or a break is
 *      already latched, fall back to the stock all-off calibration (CH1..3
 *      disabled). With equal duties the windings are effectively shorted, so
 *      back-EMF would drive a real current into the offset average.
 *   2. Bootstrap precharge: low sides on for OFFSET_CALIB_PRECHARGE_MS, the
 *      same action and length as the CHARGE_BOOT_CAP state that follows, so
 *      the L6387E high sides switch from the first calibration period.
 *   3. Bridge on at 50 % on all three legs (zero voltage vector), ADC
 *      sampled at Half_PWMPeriod - 1 exactly as RUN's forced SECTOR_5.
 *   4. Three passes, NB_CONVERSIONS samples each, the bridge switching
 *      throughout: A+B (SECTOR_5: ADC1 ch3, ADC2 ch3), C on ADC2 ch18
 *      (SECTOR_2, OPAMP3 internal output), C on ADC1 ch12 (SECTOR_1, OPAMP3
 *      pin) -- ch12 last so OPAMP3 ends on its pin, as after stock
 *      calibration. The first OFFSET_CALIB_SETTLE_PERIODS conversions of each
 *      pass are dropped: after a read-path change the next conversion can
 *      still carry the previous JSQR channel, and the first periods after
 *      the bridge starts carry its turn-on transient.
 *
 * WHY THE SAMPLE-POINT CALLBACK IS REPLACED: the FOC current loop keeps
 * running while calibration blocks the MF task. FOC_CurrControllerM1() is
 * gated only on PWMState, which R3_2_SwitchOnPWM() sets, and the stock
 * polarization sample-point callback writes the PI output (CntPhA/B/C) to
 * CCR1..3 (r3_2_g4xx_pwm_curr_fdbk.c ~1294; its own comment at ~549 calls
 * them "wrongly written by FOC during calibration"). The polarization
 * current callbacks report 0 A, so with a non-zero Iq reference (0.05 A by
 * default at boot) the PI winds up to the voltage limit. Stock calibration
 * survives that only because CH1..3 are disabled. Here the legs are pinned
 * at 50 % regardless of what the FOC loop computes. The wound-up PI is
 * cleared by FOC_Clear() before RUN (mc_tasks_foc.c, CHARGE_BOOT_CAP).
 *
 * FAULTS: hardware OCP (COMP1/2/4 -> TIM1 BRK, AOE off) stays armed and
 * clears MOE by itself. The TIM1_BRK interrupt and the safety task share
 * SysTick's priority (4), so neither runs until this returns. The break flag
 * is therefore checked after the precharge and after every pass; on a break
 * the bridge is switched off, the remaining passes are skipped, the previous
 * offsets are kept, and OverCurrentFlag is raised so TSK_SafetyTask() faults
 * the motor in this same SysTick. The flag itself is left for the pending
 * BRK interrupt.
 *
 * Runs blocking in the MF task (SysTick), like the stock routine: about
 * 10 ms precharge + 3 x (16 + 128) PWM periods = ~37 ms.
 */

#include "offset_calib.h"

#if OFFSET_CALIB_SWITCHING

#include <stdlib.h>

#include "mc_config.h"          /* ENCODER_M1 (via mc_config_common.h) */
#include "pwm_common.h"         /* NB_CONVERSIONS, waitForPolarizationEnd() */
#include "r3_2_g4xx_pwm_curr_fdbk.h"

#define OFFSET_CALIB_SETTLE_PERIODS 16u /* dropped at the start of each pass */
#define OFFSET_CALIB_PRECHARGE_MS 10u   /* = M1_PWM_CHARGE_BOOT_CAP_MS (.ioc) */
#define OFFSET_CALIB_MAX_SPEED_01HZ 1   /* |avg mech speed|, 0.1 Hz units */

/* Same mask as the driver's private TIMxCCER_MASK_CH123. */
#define OFFSET_CALIB_CCER_CH123 ((uint32_t)(LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | \
                                            LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | \
                                            LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N))

static volatile uint32_t s_phaseCOffsetCh18 = 0u; /* sum during a pass, then the offset */
static volatile bool s_ch18Valid = false;
static volatile uint8_t s_settle = 0u;            /* conversions still to drop, ADC ISR */
static bool s_lastSwitched = false;

/* Replaces R3_2_SetADCSampPointPolarization() for the duration of the
 * calibration: selects the pass's read path (the TIM1 UP interrupt programs
 * JSQR and the OPAMP output from _Super.Sector) and pins all three legs at
 * 50 % -- see "WHY THE SAMPLE-POINT CALLBACK IS REPLACED" above. */
static uint16_t SetSampPoint50(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  uint32_t const half = pHandle->Half_PWMPeriod;

  pHandle->_Super.Sector = pHandle->PolarizationSector;

  LL_TIM_OC_SetCompareCH1(TIMx, half / 2u);
  LL_TIM_OC_SetCompareCH2(TIMx, half / 2u);
  LL_TIM_OC_SetCompareCH3(TIMx, half / 2u);
  LL_TIM_OC_SetCompareCH4(TIMx, half - 1u);

  /* Same late-update test as R3_2_WriteTIMRegisters(). */
  return (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET) ? MC_DURATION : MC_NO_ERROR;
}

/* As R3_2_HFCurrentsPolarizationAB(), plus the settle drop. */
static void HFPolarizationAB(PWMC_Handle_t *pHdl, ab_t *Iab)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  uint32_t const ADCDataReg1 = pHandle->pParams_str->ADCDataReg1[pHandle->PolarizationSector]->JDR1;
  uint32_t const ADCDataReg2 = pHandle->pParams_str->ADCDataReg2[pHandle->PolarizationSector]->JDR1;

  /* Disable ADC trigger source */
  LL_TIM_SetTriggerOutput(pHandle->pParams_str->TIMx, LL_TIM_TRGO_RESET);

  if (s_settle > 0u)
  {
    s_settle--;
  }
  else if (pHandle->PolarizationCounter < NB_CONVERSIONS)
  {
    pHandle->PhaseAOffset += ADCDataReg1;
    pHandle->PhaseBOffset += ADCDataReg2;
    pHandle->PolarizationCounter++;
  }
  else
  {
    /* Nothing to do */
  }

  Iab->a = 0;
  Iab->b = 0;
}

/* As R3_2_HFCurrentsPolarizationC(), plus the settle drop. Phase C is the
 * second conversion in both SECTOR_1 (ADC1 ch12) and SECTOR_2 (ADC2 ch18). */
static void HFPolarizationC(PWMC_Handle_t *pHdl, ab_t *Iab)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  uint32_t const ADCDataReg2 = pHandle->pParams_str->ADCDataReg2[pHandle->PolarizationSector]->JDR1;

  /* Disable ADC trigger source */
  LL_TIM_SetTriggerOutput(pHandle->pParams_str->TIMx, LL_TIM_TRGO_RESET);

  if (s_settle > 0u)
  {
    s_settle--;
  }
  else if (pHandle->PolarizationCounter < NB_CONVERSIONS)
  {
    if (SECTOR_2 == pHandle->PolarizationSector)
    {
      s_phaseCOffsetCh18 += ADCDataReg2;
    }
    else
    {
      pHandle->PhaseCOffset += ADCDataReg2;
    }
    pHandle->PolarizationCounter++;
  }
  else
  {
    /* Nothing to do */
  }

  Iab->a = 0;
  Iab->b = 0;
}

static bool BreakLatched(const TIM_TypeDef *TIMx)
{
  return (LL_TIM_IsActiveFlag_BRK(TIMx) != 0u);
}

/* Busy-waits ms milliseconds on DWT->CYCCNT. Enabling the counter is
 * idempotent and never resets it, so can_driver.c's microsecond clock is
 * undisturbed. */
static void DelayMs(uint32_t ms)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint32_t const start = DWT->CYCCNT;
  uint32_t const cycles = ms * (SystemCoreClock / 1000u);
  while ((DWT->CYCCNT - start) < cycles)
  {
    /* wait */
  }
}

/* One calibration pass on one read path. Returns false if a break latched. */
static bool RunPass(PWMC_R3_2_Handle_t *pHandle, uint8_t sector, PWMC_GetPhaseCurr_Cb_t cb)
{
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  __disable_irq();
  pHandle->PolarizationCounter = 0u;
  s_settle = OFFSET_CALIB_SETTLE_PERIODS;
  pHandle->PolarizationSector = sector;
  pHandle->_Super.Sector = sector;
  pHandle->_Super.pFctGetPhaseCurrents = cb;
  __enable_irq();

  waitForPolarizationEnd(TIMx,
                         &pHandle->_Super.SWerror,
                         pHandle->pParams_str->RepetitionCounter,
                         &pHandle->PolarizationCounter);

  return !BreakLatched(TIMx);
}

/* Overrides the __weak stock routine -- see the sequence at the top. */
void R3_2_CurrentReadingPolarization(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  PWMC_GetPhaseCurr_Cb_t const GetPhaseCurrCbSave = pHandle->_Super.pFctGetPhaseCurrents;
  PWMC_SetSampPointSectX_Cb_t const SetSampPointSectXCbSave = pHandle->_Super.pFctSetADCSampPointSectX;

  /* Kept so that an aborted calibration leaves the previous offsets in use. */
  uint32_t const prevA = pHandle->PhaseAOffset;
  uint32_t const prevB = pHandle->PhaseBOffset;
  uint32_t const prevC = pHandle->PhaseCOffset;
  uint32_t const prevCh18 = s_phaseCOffsetCh18;
  bool const prevCalibStatus = pHandle->_Super.offsetCalibStatus;

  int16_t const speed = SPD_GetAvrgMecSpeedUnit(&ENCODER_M1._Super);
  bool const switching = (abs((int)speed) <= OFFSET_CALIB_MAX_SPEED_01HZ) && !BreakLatched(TIMx);
  bool ok = true;

  pHandle->PhaseAOffset = 0u;
  pHandle->PhaseBOffset = 0u;
  pHandle->PhaseCOffset = 0u;
  s_phaseCOffsetCh18 = 0u;
  pHandle->PolarizationCounter = 0u;

  __disable_irq();
  pHandle->_Super.pFctSetADCSampPointSectX = &SetSampPoint50;
  pHandle->ADCTriggerEdge = (uint16_t)LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->PolarizationSector = SECTOR_5;
  pHandle->_Super.Sector = SECTOR_5;
  __enable_irq();

  if (switching)
  {
    /* Channels are normally enabled already; make sure, since the bridge
       must actually switch. CCR = 0: low sides on for the whole period. */
    LL_TIM_CC_EnableChannel(TIMx, OFFSET_CALIB_CCER_CH123);
    R3_2_TurnOnLowSides(pHdl, 0u);
    DelayMs(OFFSET_CALIB_PRECHARGE_MS);
    ok = !BreakLatched(TIMx);
  }
  else
  {
    /* Stock behaviour: inactive level on TIMx CHy and CHyN, all FETs off. */
    LL_TIM_CC_DisableChannel(TIMx, OFFSET_CALIB_CCER_CH123);
  }

  if (ok)
  {
    R3_2_SwitchOnPWM(pHdl); /* 50 % on all legs, MOE on */
    ok = RunPass(pHandle, SECTOR_5, &HFPolarizationAB);
  }
#if OFFSET_CALIB_CH18
  if (ok)
  {
    ok = RunPass(pHandle, SECTOR_2, &HFPolarizationC);
  }
#endif
  if (ok)
  {
    ok = RunPass(pHandle, SECTOR_1, &HFPolarizationC);
  }

  R3_2_SwitchOffPWM(pHdl);

  if (ok)
  {
    pHandle->PhaseAOffset /= NB_CONVERSIONS;
    pHandle->PhaseBOffset /= NB_CONVERSIONS;
    pHandle->PhaseCOffset /= NB_CONVERSIONS;
#if OFFSET_CALIB_CH18
    s_phaseCOffsetCh18 = s_phaseCOffsetCh18 / NB_CONVERSIONS;
    s_ch18Valid = true;
#endif
    pHandle->_Super.offsetCalibStatus = true;
  }
  else
  {
    /* Break during calibration: keep what was in use and let the safety
       task fault the motor in this same SysTick (see FAULTS above). */
    pHandle->PhaseAOffset = prevA;
    pHandle->PhaseBOffset = prevB;
    pHandle->PhaseCOffset = prevC;
    s_phaseCOffsetCh18 = prevCh18;
    pHandle->_Super.offsetCalibStatus = prevCalibStatus;
    pHandle->_Super.OverCurrentFlag = true;
  }
  s_lastSwitched = switching && ok;

  /* From here on identical to the stock epilogue. */
  __disable_irq();
  pHandle->_Super.pFctGetPhaseCurrents = GetPhaseCurrCbSave;
  pHandle->_Super.pFctSetADCSampPointSectX = SetSampPointSectXCbSave;
  __enable_irq();

  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetCompareCH1(TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2(TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3(TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  LL_TIM_CC_EnableChannel(TIMx, OFFSET_CALIB_CCER_CH123);

  pHandle->_Super.Sector = SECTOR_5;
  pHandle->_Super.BrakeActionLock = false;
}

/* Overrides the __weak stock routine. Verbatim copy of
 * r3_2_g4xx_pwm_curr_fdbk.c R3_2_GetPhaseCurrents() except the SECTOR_2/3
 * phase-C line, which subtracts the ch18 offset (OFFSET_CALIB_CH18). */
void R3_2_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *Iab)
{
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  uint8_t Sector;

  Sector = (uint8_t)pHandle->_Super.Sector;
  ADCDataReg1 = pHandle->pParams_str->ADCDataReg1[Sector]->JDR1;
  ADCDataReg2 = pHandle->pParams_str->ADCDataReg2[Sector]->JDR1;

  /* Disable ADC trigger source */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  switch (Sector)
  {
    case SECTOR_4:
    case SECTOR_5:
    {
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = (int32_t)(pHandle->PhaseAOffset) - (int32_t)(ADCDataReg1);

      /* Saturation of Ia */
      if (Aux < -INT16_MAX)
      {
        Iab->a = -INT16_MAX;
      }
      else  if (Aux > INT16_MAX)
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = (int16_t)Aux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = (int32_t)(pHandle->PhaseBOffset) - (int32_t)(ADCDataReg2);

      /* Saturation of Ib */
      if (Aux < -INT16_MAX)
      {
        Iab->b = -INT16_MAX;
      }
      else  if (Aux > INT16_MAX)
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = (int16_t)Aux;
      }
      break;
    }

    case SECTOR_6:
    case SECTOR_1:
    {
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = (int32_t)(pHandle->PhaseBOffset) - (int32_t)(ADCDataReg1);

      /* Saturation of Ib */
      if (Aux < -INT16_MAX)
      {
        Iab->b = -INT16_MAX;
      }
      else  if (Aux > INT16_MAX)
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = (int16_t)Aux;
      }

      /* Ia = -Ic -Ib */
      Aux = (int32_t)(ADCDataReg2) - (int32_t)(pHandle->PhaseCOffset); /* -Ic */
      Aux -= (int32_t)Iab->b;             /* Ia  */

      /* Saturation of Ia */
      if (Aux > INT16_MAX)
      {
        Iab->a = INT16_MAX;
      }
      else  if (Aux < -INT16_MAX)
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = (int16_t)Aux;
      }
      break;
    }

    case SECTOR_2:
    case SECTOR_3:
    {
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = (int32_t)(pHandle->PhaseAOffset) - (int32_t)(ADCDataReg1);

      /* Saturation of Ia */
      if (Aux < -INT16_MAX)
      {
        Iab->a = -INT16_MAX;
      }
      else  if (Aux > INT16_MAX)
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = (int16_t)Aux;
      }

      /* Ib = -Ic -Ia. Phase C is read on ADC2 ch18 (OPAMP3 internal output)
         in these sectors, so it gets the offset calibrated on that path. */
#if OFFSET_CALIB_CH18
      Aux = (int32_t)(ADCDataReg2)
          - (int32_t)(s_ch18Valid ? s_phaseCOffsetCh18 : pHandle->PhaseCOffset); /* -Ic */
#else
      Aux = (int32_t)(ADCDataReg2) - (int32_t)(pHandle->PhaseCOffset); /* -Ic */
#endif
      Aux -= (int32_t)Iab->a;             /* Ib */

      /* Saturation of Ib */
      if (Aux > INT16_MAX)
      {
        Iab->b = INT16_MAX;
      }
      else  if (Aux < -INT16_MAX)
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = (int16_t)Aux;
      }
      break;
    }

    default:
      break;
  }

  pHandle->_Super.Ia = Iab->a;
  pHandle->_Super.Ib = Iab->b;
  pHandle->_Super.Ic = -Iab->a - Iab->b;
}

uint16_t OffsetCalib_GetPhaseCCh18(void)
{
  return s_ch18Valid ? (uint16_t)s_phaseCOffsetCh18 : 0u;
}

bool OffsetCalib_LastUsedSwitching(void)
{
  return s_lastSwitched;
}

#else /* !OFFSET_CALIB_SWITCHING: stock calibration, nothing overridden */

uint16_t OffsetCalib_GetPhaseCCh18(void)
{
  return 0u;
}

bool OffsetCalib_LastUsedSwitching(void)
{
  return false;
}

#endif /* OFFSET_CALIB_SWITCHING */
