
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/
#define HW_DEAD_TIME_NS                      800 /*!< Dead-time inserted
                                                      by HW if low side signals
                                                      are not used */
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             0.09625565501973242 /*!< It expresses how
                                                      much the Vbus is attenuated
                                                      before being converted into
                                                      digital value */
#define NOMINAL_BUS_VOLTAGE_V                13U
/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT

#define RSHUNT                               0.003

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
/* 16x PGA gain (was 8x/4.57): keep in sync with the OPAMP PgaGain enum in
   main.c, DAC_OCP_Threshold in mc_parameters.c, and PID_TORQUE/FLUX_K*_DEFAULT
   in drive_parameters.h -- none of these are tied together by the compiler. */
#define AMPLIFICATION_GAIN                   9.14

/*** Noise parameters ***/
#define TNOISE_NS                            4000
#define TRISE_NS                             4000
#define MAX_TNTR_NS                          TRISE_NS

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 1.4 /*!< in Volts */
#define T0_C                                 25 /*!< in Celsius degrees */
#define dV_dT                                0.019 /*!< V/Celsius degrees */
#define T_MAX                                70 /*!< Sensor measured
                                                     temperature at maximum
                                                     power stage working
                                                     temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
