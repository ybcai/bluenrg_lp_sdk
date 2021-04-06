/**
  ******************************************************************************
  * @file    HAL/HAL_TimeBase_RTC_ALARM/Src/bluenrg_lp_it.c
  * @author  RF Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "HAL_TimeBase_RTC_ALARM_main.h"
#include "bluenrg_lp_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern RTC_HandleTypeDef        hRTC_Handle;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/


/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_IRQHandler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_IRQHandler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_IRQHandler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* BLUENRG_LP Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_bluenrg_lp.s).                    */
/******************************************************************************/

/**
  * @brief  This function handles External line interrupt request.
  * @param  None
  * @retval None
  */
void GPIOA_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BSP_PUSH1_GPIO_PORT, BSP_PUSH1_PIN);
}

/**
  * @brief  This function handles External line interrupt request.
  * @param  None
  * @retval None
  */
void GPIOB_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BSP_PUSH2_GPIO_PORT, BSP_PUSH2_PIN);
}

/**
* @brief  This function handles RTC ALARM interrupt request.
* @retval None
*/
void RTC_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hRTC_Handle);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
