/**
  ******************************************************************************
  * @file    Examples_LL/USART/USART_Comm_Rx_IT/Inc/USART_Comm_Rx_IT_main.h
  * @author  RF Application Team
  * @brief   Header for USART_Comm_Rx_IT_main.c module
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

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_gpio.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_usart.h"
#include "bluenrg_lp_ll_pwr.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Initialize configured peripherals */
#if !defined(USE_LL_UNITARY) && !defined(USE_LL_INIT) 
  /* default initialization functions */
  #define USE_LL_UNITARY
  /* remember to define USE_FULL_LL_DRIVER preprocessor variable when it is used the USE_LL_INIT function */
#endif

/* Private definitions covering GPIO clock and USART pins 
   depending on selected USART instance. */

/* USART1 instance is used. (TX on PA.09, RX on PA.08)*/
#define USARTx_INSTANCE               USART1
#define USARTx_CLK_ENABLE()           LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#define USARTx_IRQn                   USART1_IRQn
#define USARTx_IRQHandler             USART1_IRQHandler

#define USARTx_GPIO_CLK_ENABLE()      LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA)   /* Enable the peripheral clock of GPIOA */
#define USARTx_TX_PIN                 LL_GPIO_PIN_9
#define USARTx_TX_GPIO_PORT           GPIOA
#define USARTx_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_0)
#define USARTx_RX_PIN                 LL_GPIO_PIN_8
#define USARTx_RX_GPIO_PORT           GPIOA
#define USARTx_SET_RX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_0)

#define LED2_PIN                LL_GPIO_PIN_8
#define LED2_GPIO_PORT          GPIOB
#define LED2_GPIO_CLK_ENABLE()             LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
/**
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000


/**
  * @brief Key push-button
  */
#define USER_BUTTON_PIN                         LL_GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA) 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG)   
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_EXTI_RISING_TRIG_ENABLE()   LL_EXTI_SetTrigger(LL_EXTI_TRIGGER_RISING_EDGE, USER_BUTTON_EXTI_LINE)     
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/* IRQ Handler treatment functions */
void UserButton_Callback(void); 
void USART_CharReception_Callback(void); 
void Error_Callback(void); 

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
