/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_FullDuplex_DMA_Slave_Init/Inc/SPI_FullDuplex_DMA_Slave_Init_main.h
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
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_spi.h"
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
  
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void DMA1_ReceiveComplete_Callback(void);
void DMA1_TransmitComplete_Callback(void);
void SPI_SLAVE_TransferError_Callback(void);

/* Private defines -----------------------------------------------------------*/
#define LED2_PIN LL_GPIO_PIN_8
#define LED2_GPIO_PORT GPIOB
#define LED2_GPIO_CLK_ENABLE()             LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)

#if !defined( CONFIG_DATASIZE_16BIT ) & !defined( CONFIG_DATASIZE_8BIT )
  // default SPI CONFIG DATASIZE is 16 BIT
  #define CONFIG_DATASIZE_16BIT 1
#endif 

#if !defined( USE_SPI1_PINS ) & !defined( USE_SPI2_PINS )
  // default SPI pins for this example
  #define USE_SPI1_PINS 1
#endif  

#if defined( USE_SPI2_PINS ) /* Slave SPI2 */ 
    /**SPI_SLAVE GPIO Configuration  
    SPI2:
    PA5/AF1    ------> SPI2_SCK
    PA7/AF1    ------> SPI2_MISO
    PA12/AF3   ------> SPI2_MOSI 
    */
#define GPIO_PORT_SLAVE                        GPIOA
#define GPIO_PIN_SPI_SLAVE_SCK                 LL_GPIO_PIN_5
#define GPIO_PIN_SPI_SLAVE_MISO                LL_GPIO_PIN_7
#define GPIO_PIN_SPI_SLAVE_MOSI                LL_GPIO_PIN_12
#define GPIO_AF_SPI_SLAVE_SCK                  LL_GPIO_AF_1
#define GPIO_AF_SPI_SLAVE_MISO                 LL_GPIO_AF_1
#define GPIO_AF_SPI_SLAVE_MOSI                 LL_GPIO_AF_3
#define SPI_SLAVE                              SPI2
#define LL_SPI_Slave_EnableClock()            LL_APB1_EnableClock(LL_APB1_PERIPH_SPI2)
#define SPI_SLAVE_IRQn                         SPI2_IRQn
#define SPI_SLAVE_IRQHandler                   SPI2_IRQHandler
#define LL_DMAMUX_REQ_SPI_SLAVE_TX             LL_DMAMUX_REQ_SPI2_TX
#define LL_DMAMUX_REQ_SPI_SLAVE_RX             LL_DMAMUX_REQ_SPI2_RX
#define LL_SPI_Slave_EnableClock()             LL_APB1_EnableClock(LL_APB1_PERIPH_SPI2)

#elif defined( USE_SPI1_PINS ) /* Slave SPI1 */
    /**SPI_SLAVE GPIO Configuration    
    SPI1:
    PA13/AF2    ------> SPI1_SCK
    PA14/AF2    ------> SPI1_MISO
    PA15/AF2    ------> SPI1_MOSI 
    */
#define GPIO_PORT_SLAVE                        GPIOA
#define GPIO_PIN_SPI_SLAVE_SCK                 LL_GPIO_PIN_13
#define GPIO_PIN_SPI_SLAVE_MISO                LL_GPIO_PIN_14
#define GPIO_PIN_SPI_SLAVE_MOSI                LL_GPIO_PIN_15
#define GPIO_AF_SPI_SLAVE_SCK                  LL_GPIO_AF_2
#define GPIO_AF_SPI_SLAVE_MISO                 LL_GPIO_AF_2
#define GPIO_AF_SPI_SLAVE_MOSI                 LL_GPIO_AF_2
#define SPI_SLAVE                              SPI1
#define LL_SPI_Slave_EnableClock()            LL_APB1_EnableClock(LL_APB1_PERIPH_SPI1)
#define SPI_SLAVE_IRQn                         SPI1_IRQn
#define SPI_SLAVE_IRQHandler                   SPI1_IRQHandler 
#define LL_DMAMUX_REQ_SPI_SLAVE_TX             LL_DMAMUX_REQ_SPI1_TX
#define LL_DMAMUX_REQ_SPI_SLAVE_RX             LL_DMAMUX_REQ_SPI1_RX
#define LL_SPI_Slave_EnableClock()             LL_APB1_EnableClock(LL_APB1_PERIPH_SPI1) 
  
#endif 

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
