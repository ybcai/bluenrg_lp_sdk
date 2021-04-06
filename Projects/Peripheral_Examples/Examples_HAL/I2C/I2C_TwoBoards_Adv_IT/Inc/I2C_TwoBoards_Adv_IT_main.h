/**
  ******************************************************************************
  * @file    I2C/I2C_TwoBoards_Adv_IT/Inc/I2C_TwoBoards_Adv_IT_main.h 
  * @author  RF Application Team
  * @brief   Header for I2C_TwoBoards_Adv_IT_main.c module
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
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define I2C_ADDRESS 0x3E

/** Uncomment this line to use the board as master, if not it is used as slave */
//#define MASTER_BOARD

#define MASTER_REQ_READ    0x12
#define MASTER_REQ_WRITE   0x34

#define I2Cx                          I2C2
#define I2Cx_SCL_PORT                 GPIOA   
#define I2Cx_SCL_PIN                  GPIO_PIN_13
#define I2Cx_SCL_AF                   GPIO_AF0_I2C2  
#define I2Cx_SDA_PORT                 GPIOA
#define I2Cx_SDA_PIN                  GPIO_PIN_14 
#define I2Cx_SDA_AF                   GPIO_AF0_I2C2
#define __HAL_RCC_I2Cx_CLK_ENABLE     __HAL_RCC_I2C2_CLK_ENABLE
#define __HAL_RCC_I2Cx_CLK_DISABLE    __HAL_RCC_I2C2_CLK_DISABLE
#define __HAL_RCC_I2Cx_SCL_GPIO_CLK_ENABLE          __HAL_RCC_GPIOA_CLK_ENABLE
#define __HAL_RCC_I2Cx_SDA_GPIO_CLK_ENABLE          __HAL_RCC_GPIOA_CLK_ENABLE
#define I2Cx_IRQn                     I2C2_IRQn
#define I2Cx_IRQHandler               I2C2_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
