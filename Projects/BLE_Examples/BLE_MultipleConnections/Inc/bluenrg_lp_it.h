/**
  ******************************************************************************
  * @file    bluenrg_lp_it.h 
  * @author  AMS - RF Application Team
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BlueNRG_LP_IT_H
#define BlueNRG_LP_IT_H

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_LP.h"

/* Exported defines ------------------------------------------------------------*/

#define DMA_IDLE        0
#define DMA_IN_PROGRESS 1

/* Exported constants --------------------------------------------------------*/
extern uint8_t command_in_progress;
extern uint8_t dma_state;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_IRQHandler(void);
void HardFault_IRQHandler(void);
void SVC_IRQHandler(void);
void PendSV_IRQHandler(void);
void SysTick_IRQHandler(void);


#endif /* BlueNRG_LP_IT_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
