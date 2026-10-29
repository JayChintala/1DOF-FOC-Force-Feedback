/*
 * sampling_tafter.c
 *
 * CHANGE 7(a) -- see sampling_tafter.h and FIRMWARE_CHANGES.txt.
 *
 * Sampling_SetADCSampPointSectX() is a verbatim copy of the MCSDK
 * R3_2_SetADCSampPointSectX() (r3_2_g4xx_pwm_curr_fdbk.c ~1117-1184) and of
 * the static inline R3_2_WriteTIMRegisters() it calls (~1288-1310), except
 * that Tafter is read from s_tafter instead of pParams_str->Tafter. It runs
 * in the 16 kHz FOC interrupt; the offset calibration swaps this callback out
 * and restores it afterwards through the handle, so it composes with
 * offset_calib.c.
 */

#include "sampling_tafter.h"

#include "mc_config.h"             /* CircleLimitationM1, R3_2 handle type */
#include "mc_parameters.h"         /* R3_2_ParamsM1 */
#include "parameters_conversion.h" /* TW_AFTER, MAX_MODULE, SAMPLING_TIME */

#if SAMPLING_TAFTER_RUNTIME

/* Written by the 1 kHz hook (Sampling_SetTafter), read by the 16 kHz ISR:
   one aligned 16-bit store/load, so never torn. */
static volatile uint16_t s_tafter = TW_AFTER;

uint16_t Sampling_GetTafter(void)
{
  return s_tafter;
}

/* WHY THE CIRCLE LIMITATION FOLLOWS TAFTER: in the natural path the sample
 * lands Tafter after the highest-duty leg's turn-on (mirrored onto the
 * down-count when that is past the period centre) and must still fall inside
 * the MID leg's low-side window. The tightest case is two legs at equal
 * highest duty (sector boundary, SVPWM factor sqrt(3)/2), which needs
 *   Half - lowDuty > (Tafter + SAMPLING_TIME) / 2,
 * i.e. modulation m < (1 - (Tafter + Ts) / Half) / (sqrt(3)/2). The
 * Workbench-chosen 89 % (MAX_MODULE) satisfies this up to Tafter ~1190, so
 * the stock 807 leaves MaxModule unchanged; 1360 gives ~85 %, 2040 ~70.5 %.
 *
 * The three stores are made with interrupts off so that no PWM period sees
 * MaxVd above MaxModule (Circle_Limitation() would take the square root of a
 * negative number) or a Tafter that disagrees with its limit. */
void Sampling_SetTafter(uint16_t tafter)
{
  float const half = (float)PWM_Handle_M1.Half_PWMPeriod;
  float const ts = (float)SAMPLING_TIME + 1.0f;
  float m = 32767.0f * (1.0f - (((float)tafter + ts) / half)) / 0.8660254f;
  if (m > (float)MAX_MODULE)
  {
    m = (float)MAX_MODULE;
  }
  uint16_t const maxModule = (uint16_t)m;
  uint16_t const maxVd = (uint16_t)(((uint32_t)maxModule * 950u) / 1000u); /* as mc_config.c */

  uint32_t const primask = __get_PRIMASK();
  __disable_irq();
  s_tafter = tafter;
  CircleLimitationM1.MaxModule = maxModule;
  CircleLimitationM1.MaxVd = maxVd;
  __set_PRIMASK(primask);
}

/* Copy of R3_2_WriteTIMRegisters(). */
static inline uint16_t WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t SamplingPoint)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  uint16_t Aux;

  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t) pHandle->_Super.CntPhA);
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t) pHandle->_Super.CntPhB);
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t) pHandle->_Super.CntPhC);
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t) SamplingPoint);

  /* Limit for update event */
  if (((TIMx->CR2) & TIM_CR2_MMS_Msk) != LL_TIM_TRGO_RESET)
  {
    Aux = MC_DURATION;
  }
  else
  {
    Aux = MC_NO_ERROR;
  }
  return Aux;
}

/* Copy of R3_2_SetADCSampPointSectX() with Tafter from s_tafter. */
uint16_t Sampling_SetADCSampPointSectX(PWMC_Handle_t *pHdl)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl;
  uint16_t const Tafter = s_tafter; /* one read per period */
  uint16_t SamplingPoint;
  uint16_t DeltaDuty;

  /* Check if sampling the AB phases in the middle of the PWM period is possible */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHdl->lowDuty) > Tafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are
     * differences between offsets */

    /* Sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds to
     * sector 4 or 5 */
    pHandle->_Super.Sector = SECTOR_5;

    /* Set the sampling point in the middle of the PWM period */
    SamplingPoint = pHandle->Half_PWMPeriod - 1u;
  }
  else
  {
    /* In this case it is necessary to sample phases with maximum and variable complementary duty cycle */

    /* In every sector there is always one phase with maximum complementary duty, one with minimum
     * complementary duty and one with variable complementary duty. In this case, phases with variable
     * and maximum complementary duty are converted, always starting with the one with variable
     * complementary duty cycle */

    DeltaDuty = (uint16_t)(pHdl->lowDuty - pHdl->midDuty);

    /* Definition of crossing point */
    if (DeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHdl->lowDuty) * 2U))
    {
      SamplingPoint = pHdl->lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      SamplingPoint = pHdl->lowDuty + Tafter;

      if (SamplingPoint >= pHandle->Half_PWMPeriod)
      {
        /* ADC trigger edge must be changed from positive to negative */
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;
        SamplingPoint = (2U * pHandle->Half_PWMPeriod) - SamplingPoint - 1;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }
  return WriteTIMRegisters(&pHandle->_Super, SamplingPoint);
}

#else /* !SAMPLING_TAFTER_RUNTIME */

uint16_t Sampling_GetTafter(void)
{
  return R3_2_ParamsM1.Tafter;
}

#endif /* SAMPLING_TAFTER_RUNTIME */
