/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Comm_Tx_IT_Init/Inc/USART_Comm_Tx_IT_Init_main.h
  * @author  RF Application Team
  * @brief   Header for USART_Comm_Tx_IT_Init_main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_usart.h"
#include "bluenrg_lp.h"
#include "bluenrg_lp_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void UserButton_Callback(void); 
void USART_TXEmpty_Callback(void); 
void USART_CharTransmitComplete_Callback(void); 
void Error_Callback(void); 

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_PIN         LL_GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT   GPIOA
#define USER_BUTTON_EXTI_IRQn   GPIOA_IRQn
#define USER_BUTTON_IRQHANDLER  GPIOA_IRQHandler
#define LED2_PIN                LL_GPIO_PIN_8
#define LED2_GPIO_PORT          GPIOB

/* Define used to enable time-out management*/
#define USE_TIMEOUT       0

/**
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/