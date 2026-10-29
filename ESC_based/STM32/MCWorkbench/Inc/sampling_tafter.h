/*
 * sampling_tafter.h
 *
 * CHANGE 7(a) (FIRMWARE_CHANGES.txt): the current-sampling delay Tafter as a
 * RAM variable, settable at runtime over CAN (SET_PARAM, can_driver.c).
 *
 * R3_2_Params_t is declared "typedef const struct" in the MCSDK header, so
 * R3_2_ParamsM1.Tafter cannot be made writable. Instead
 * Sampling_SetADCSampPointSectX() is a copy of the MCSDK
 * R3_2_SetADCSampPointSectX() (r3_2_g4xx_pwm_curr_fdbk.c, not __weak) that
 * reads Tafter from here, installed as pFctSetADCSampPointSectX in
 * mc_config.c. Tbefore still comes from R3_2_ParamsM1.
 */

#ifndef SAMPLING_TAFTER_H
#define SAMPLING_TAFTER_H

#include <stdint.h>

#include "pwm_curr_fdbk.h"

/* 1 = runtime Tafter (this module + SET_PARAM); 0 = stock MCSDK sampling. */
#ifndef SAMPLING_TAFTER_RUNTIME
#define SAMPLING_TAFTER_RUNTIME 1
#endif

#define SAMPLING_TAFTER_MIN 400u  /* SET_PARAM accepted range, TIM1 counts */
#define SAMPLING_TAFTER_MAX 2040u

/* The Tafter in use, TIM1 counts (170 MHz). TW_AFTER (807) at boot, or
   R3_2_ParamsM1.Tafter when SAMPLING_TAFTER_RUNTIME is 0. */
uint16_t Sampling_GetTafter(void);

#if SAMPLING_TAFTER_RUNTIME
/* Sets Tafter and couples CircleLimitationM1.MaxModule/MaxVd to it (see
   sampling_tafter.c). Hook context only; the caller range-checks. */
void Sampling_SetTafter(uint16_t tafter);

/* pFctSetADCSampPointSectX for motor 1 (mc_config.c). */
uint16_t Sampling_SetADCSampPointSectX(PWMC_Handle_t *pHdl);
#endif

#endif /* SAMPLING_TAFTER_H */
