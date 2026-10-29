/*
 * offset_calib.h
 *
 * CHANGE 5 (FIRMWARE_CHANGES.txt): current-sense offset calibration with the
 * bridge SWITCHING at 50 % duty on all three legs, the same conditions RUN
 * samples under, plus a separate phase-C offset for the ADC2 ch18 read path
 * (sectors 2-3 read phase C through OPAMP3's internal output, while stock
 * MCSDK calibrates phase C only on ADC1 ch12, the OPAMP3 pin).
 *
 * Implemented as strong overrides of the __weak R3_2_CurrentReadingPolarization
 * and R3_2_GetPhaseCurrents in STM32CubeIDE/Application/User/offset_calib.c,
 * so the git-ignored MCSDK copy stays untouched.
 */

#ifndef OFFSET_CALIB_H
#define OFFSET_CALIB_H

#include <stdbool.h>
#include <stdint.h>

/* 1 = switching calibration and the overrides above; 0 = stock MCSDK
   calibration (nothing overridden, the accessors below return 0/false). */
#ifndef OFFSET_CALIB_SWITCHING
#define OFFSET_CALIB_SWITCHING 1
#endif

/* Sub-switch, only meaningful with OFFSET_CALIB_SWITCHING: 1 = calibrate and
   use a separate phase-C offset for the ADC2 ch18 path; 0 = sectors 2-3 use
   the ch12 PhaseCOffset as stock does. */
#ifndef OFFSET_CALIB_CH18
#define OFFSET_CALIB_CH18 1
#endif

/* The phase-C offset used on the ch18 path, left-aligned ADC units. 0 when
   OFFSET_CALIB_CH18 is off or no calibration has completed yet. */
uint16_t OffsetCalib_GetPhaseCCh18(void);

/* True if the most recent calibration ran with the bridge switching, false if
   it fell back to the stock all-off calibration (shaft moving, break active)
   or OFFSET_CALIB_SWITCHING is 0. */
bool OffsetCalib_LastUsedSwitching(void);

#endif /* OFFSET_CALIB_H */
