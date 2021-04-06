/**
******************************************************************************
* @file    Examples_LL/RTC/RTC_Calendar_Init/Inc/RTC_Calendar_Init_main.h
* @author  RF Application Team
* @brief   Header for main.c module
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the 
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_rtc.h"
#include "bluenrg_lp_ll_gpio.h"
#include "bluenrg_lp_evb_config.h"
  
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */
  
/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
  
/* Exported types ------------------------------------------------------------*/
  
/* Exported constants --------------------------------------------------------*/
/* Define used to enable time-out management*/
#define USE_TIMEOUT       0
  
#define RTC_ERROR_NONE    0
#define RTC_ERROR_TIMEOUT 1
  
  /**
  * @brief Toggle periods for various blinking modes
  */
  
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW 500
#define LED_BLINK_ERROR 1000
  
/* Exported macro ------------------------------------------------------------*/
  
/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
  
/* Private defines -----------------------------------------------------------*/
  /**
  * @brief BSP_LED2 
  */
#define LED2_PIN                           LL_GPIO_PIN_8
#define LED2_GPIO_PORT                     GPIOB
#define LED2_GPIO_CLK_ENABLE()             LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/