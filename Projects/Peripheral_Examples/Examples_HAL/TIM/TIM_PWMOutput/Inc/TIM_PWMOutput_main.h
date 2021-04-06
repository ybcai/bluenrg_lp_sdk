/**
  ******************************************************************************
  * @file    TIM/TIM_PWMOutput/Inc/TIM_PWMOutput_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_PWMOutput_main.c module
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
  *******************************************************************************/

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

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Compute the prescaler value to have TIM1 counter clock equal to 1000000 Hz */
#define PRESCALER_VALUE     (uint32_t)((TIM_PERIPHCLK / 1000000) - 1)

/* -----------------------------------------------------------------------
TIM1 Configuration: generate 6 PWM signals with 6 different duty cycles.

    In this example TIM1 input clock (TIM1CLK) is set to APB0 clock (PCLK1),
    since APB0 prescaler is equal to 1.
      TIM1CLK = PCLK1
        => TIM1CLK = HCLK = 64 MHz

    To get TIM1 counter clock at 1 MHz, the prescaler is computed as follows:
       Prescaler = (TIM1CLK / TIM1 counter clock) - 1
       Prescaler = (64 MHz /1 MHz) - 1

    To get TIM1 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           = 40

    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%
    TIM1 Channel5 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 75%
    TIM1 Channel6 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 62.5%

  ----------------------------------------------------------------------- */

/* Initialize TIMx peripheral as follows:
   + Prescaler = (TIM_PERIPHCLK / 1000000) - 1
   + Period = (41 - 1)
   + ClockDivision = 0
   + Counter direction = Up
*/
#define  PERIOD_VALUE       (uint32_t)(41 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE*0.50)        /* Capture Compare 1 Value  */
#define  PULSE2_VALUE       (uint32_t)(PERIOD_VALUE*0.375)       /* Capture Compare 2 Value  */
#define  PULSE3_VALUE       (uint32_t)(PERIOD_VALUE*0.25)        /* Capture Compare 3 Value  */
#define  PULSE4_VALUE       (uint32_t)(PERIOD_VALUE*0.125)       /* Capture Compare 4 Value  */
#define  PULSE5_VALUE       (uint32_t)(PERIOD_VALUE*0.75)        /* Capture Compare 3 Value  */
#define  PULSE6_VALUE       (uint32_t)(PERIOD_VALUE*0.625)       /* Capture Compare 4 Value  */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
