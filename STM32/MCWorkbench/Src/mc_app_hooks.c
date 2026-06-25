
#include "mc_api.h"
#include "mc_interface.h"
#include <string.h>
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup COMMON_MC
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

extern FDCAN_HandleTypeDef hfdcan1;

// Handle incoming commands
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
{
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);

    if (RxHeader.Identifier == 0x001)
    {
        MC_StartMotor1();
    }
    else if (RxHeader.Identifier == 0x002)
    {
        MC_StopMotor1();
    }
    else if (RxHeader.Identifier == 0x003)
    {
        qd_f_t Iqdref;
        memcpy(&Iqdref.q, RxData, 4);
        Iqdref.d = 0.0f;
        MC_SetCurrentReferenceMotor1_F(Iqdref);
    }
}

// Get Iq in Amperes
qd_f_t Iqd = MC_GetIqdMotor1_F();
float Iq = Iqd.q;

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[4];
memcpy(TxData, &Iq, 4);

TxHeader.Identifier = 0x010;
TxHeader.IdType = FDCAN_STANDARD_ID;
TxHeader.TxFrameType = FDCAN_DATA_FRAME;
TxHeader.DataLength = FDCAN_DLC_BYTES_4;
TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
TxHeader.MessageMarker = 0;

HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
