/**
  ******************************************************************************
  * @file    TIM/TIM_OCToggle/Inc/TIM_OCToggle_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_OCToggle_main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal.h"

/* Private includes ----------------------------------------------------------*/
#include "bluenrg_lp_evb_config.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/*## Define Timer periode and pulse #######################################*/
/* ---------------------------------------------------------------------------
   TIM1 Configuration: Output Compare Toggle Mode:

  To get TIM1 counter clock at 1 MHz, the prescaler is computed as follows:
  Prescaler = (TIM1CLK / TIM1 counter clock) - 1
  Prescaler = (TIM_PERIPHCLK /1000000) - 1

  CC1 update rate = TIM1 counter clock / uhCCR1_Val
                  = 1 MHz/625 = 1600 Hz
  ==> So the TIM1 Channel 1 generates a periodic signal with a frequency equal
      to 800 Hz.

  CC2 update rate = TIM1 counter clock / uhCCR2_Val
                  = 1 MHz/1250 = 800 Hz
  ==> So the TIM1 Channel 2 generates a periodic signal with a frequency equal
      to 400 Hz.

  CC3 update rate = TIM1 counter clock / uhCCR3_Val
                  = 1 MHz/2500 = 400 Hz
  ==> So the TIM1 Channel 3 generates a periodic signal with a frequency equal
      to 200 Hz.

  CC4 update rate = TIM1 counter clock / uhCCR4_Val
                  = 1 MHz/5000 = 200 Hz
  ==> So the TIM1 Channel 4 generates a periodic signal with a frequency equal
      to 100 Hz.


  --------------------------------------------------------------------------- */
#define PRESCALER_VALUE (uint32_t)(( TIM_PERIPHCLK / 1000000) - 1)

#define PULSE1_VALUE 625
#define PULSE2_VALUE 1250
#define PULSE3_VALUE 2500
#define PULSE4_VALUE 5000


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
