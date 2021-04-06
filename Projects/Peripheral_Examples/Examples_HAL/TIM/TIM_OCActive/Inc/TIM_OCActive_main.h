/**
  ******************************************************************************
  * @file    TIM/TIM_OCActive/Inc/TIM_OCActive_main.h
  * @author  RF Application Team
  * @brief   Header for TIM_OCActive_main.c module
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

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/* Compute the prescaler value to have TIMx counter clock equal to 10 kHz */
#define  PRESCALER_VALUE  ((( TIM_PERIPHCLK ) / 10000) - 1)

/* The TIM1 CCR1 register value is equal to 10000: 
 * TIM1_CH1 delay = CCR1_Val/TIM1 counter clock  = 1s 
 * so the TIM1 Channel 1 generates a signal with a delay equal to 1s. 
 */
#define  PULSE1_VALUE       10000        /* Capture Compare 1 Value  */

/* The TIM1 CCR2 register value is equal to 5000:
 * TIM1_CH2 delay = CCR2_Val/TIM1 counter clock = 500 ms
 * so the TIM1 Channel 2 generates a signal with a delay equal to 500 ms.
 */
#define  PULSE2_VALUE       5000         /* Capture Compare 2 Value  */
 
/* The TIM1 CCR3 register value is equal to 2500:
 * TIM1_CH3 delay = CCR3_Val/TIM1 counter clock = 250 ms
 * so the TIM1 Channel 3 generates a signal with a delay equal to 250 ms.
 */ 
#define  PULSE3_VALUE       2500         /* Capture Compare 3 Value  */

/* The TIM1 CCR4 register value is equal to 1250:
 * TIM1_CH4 delay = CCR4_Val/TIM1 counter clock = 125 ms
 * so the TIM1 Channel 4 generates a signal with a delay equal to 125 ms.
 */ 
#define  PULSE4_VALUE       1250         /* Capture Compare 4 Value  */ 

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/