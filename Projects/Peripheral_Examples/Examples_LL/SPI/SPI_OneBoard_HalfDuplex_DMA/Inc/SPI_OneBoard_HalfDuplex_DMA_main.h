/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_OneBoard_HalfDuplex_DMA/Inc/SPI_OneBoard_HalfDuplex_DMA_main.h
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
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_gpio.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_spi.h"
#include "bluenrg_lp_ll_pwr.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */
#include "bluenrg_lp_evb_config.h"


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/**
  * @brief LED2 
  */
#define LED2_PIN                           LL_GPIO_PIN_8
#define LED2_GPIO_PORT                     GPIOB
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

#if !defined( CONFIG_DATASIZE_16BIT ) & !defined( CONFIG_DATASIZE_8BIT )
  // default SPI CONFIG DATASIZE is 16 BIT
  #define CONFIG_DATASIZE_16BIT 1
#endif 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DMA1_TransmitComplete_Callback(void);
void DMA_ReceiveComplete_Callback(void);
void SPI_TransferError_Callback(void);
void UserButton_Callback(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
