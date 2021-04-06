/**
  ******************************************************************************
  * @file    Examples_LL/I2C/I2C_TwoBoards_MasterRx_SlaveTx_IT_Init/Inc/I2C_TwoBoards_MasterRx_SlaveTx_IT_Init_main.h
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

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp.h"
#include "bluenrg_lp_ll_i2c.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */
#include "bluenrg_lp_evb_config.h"
/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/**
  * @brief Toggle periods for various blinking modes
  */

#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/**
  * @brief Slave settings
  */
#define SLAVE_OWN_ADDRESS                       180 /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits OA1[1:7] */

/** Uncomment this line to use the board as slave, if not it is used as master */
//#define SLAVE_BOARD 
  
#define I2Cx                          I2C2
#define I2Cx_SCL_PORT                 GPIOA   
#define I2Cx_SCL_PIN                  LL_GPIO_PIN_13
#define I2Cx_SCL_AF                   LL_GPIO_AF_0  
#define I2Cx_SDA_PORT                 GPIOA
#define I2Cx_SDA_PIN                  LL_GPIO_PIN_14 
#define I2Cx_SDA_AF                   LL_GPIO_AF_0
#define I2Cx_IRQn                     I2C2_IRQn
#define I2Cx_IRQHandler               I2C2_IRQHandler
#define LL_I2Cx_EnableClock()         LL_APB1_EnableClock(LL_APB1_PERIPH_I2C2)

/* Definition for usage of internal pull up */
/* Enable this define, for an integration of this example inside
   an ecosystem board with external pull-up */
#define EXTERNAL_PULL_UP_AVAILABLE      0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* IRQ Handler treatment functions */
#ifdef SLAVE_BOARD
  void Slave_Ready_To_Transmit_Callback(void);
  void Slave_Complete_Callback(void);
#else /* MASTER_BOARD */
  void Master_Reception_Callback(void);
  void Master_Complete_Callback(void);
  void UserButton_Callback(void); 
#endif /* SLAVE_BOARD */
void Error_Callback(void);

/* Private defines -----------------------------------------------------------*/
#define LED2_PIN                           LL_GPIO_PIN_8
#define LED2_GPIO_PORT                     GPIOB
#define LED2_GPIO_CLK_ENABLE()             LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)

#define USER_BUTTON_PIN                         LL_GPIO_PIN_10
#define USER_BUTTON_GPIO_PORT                   GPIOA
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA) 
#define USER_BUTTON_SYSCFG_CLK_ENABLE()         LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG)   
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_PA10
#define USER_BUTTON_EXTI_IRQn                   GPIOA_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT(USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_EXTI_RISING_TRIG_ENABLE()   LL_EXTI_SetTrigger(LL_EXTI_TRIGGER_RISING_EDGE, USER_BUTTON_EXTI_LINE)   
#define USER_BUTTON_IRQHANDLER                  GPIOA_IRQHandler


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
