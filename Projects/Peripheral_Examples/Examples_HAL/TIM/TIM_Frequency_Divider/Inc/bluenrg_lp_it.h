/**
  ******************************************************************************
  * @file    TIM/TIM_Frequency_Divider/Inc/bluenrg_lp_it.h
  * @author  RF Application Team
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __BLUENRG_LP_IT_H
#define __BLUENRG_LP_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal.h"
#include "TIM_Frequency_Divider_main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_IRQHandler(void);
void HardFault_IRQHandler(void);
void SysTick_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLUENRG_LP_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
