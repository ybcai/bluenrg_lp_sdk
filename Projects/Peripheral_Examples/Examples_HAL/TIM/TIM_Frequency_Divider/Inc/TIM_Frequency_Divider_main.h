/**
  ******************************************************************************
  * @file    TIM/TIM_Frequency_Divider/Inc/TIM_Frequency_Divider_main.h
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal.h"
#include "bluenrg_lp_evb_config.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Definition for TIMx clock resources */
#define TIMx                           TIM1
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM1_CLK_ENABLE

/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT         __HAL_RCC_GPIOA_CLK_ENABLE
#define TIMx_GPIO_PORT                 GPIOA
#define GPIO_PIN_CHANNEL1              GPIO_PIN_4
#define GPIO_PIN_CHANNEL2              GPIO_PIN_5
#define GPIO_PUPD_CHANNEL1             GPIO_PULLDOWN
#define GPIO_PUPD_CHANNEL2             GPIO_PULLDOWN
#define GPIO_AF_TIMx                   GPIO_AF4_TIM1

/* Exported functions ------------------------------------------------------- */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
