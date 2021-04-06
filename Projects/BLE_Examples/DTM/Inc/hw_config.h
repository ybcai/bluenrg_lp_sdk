/**
  ******************************************************************************
  * @file    hw_config.h 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    July-2015
  * @brief   
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
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bluenrg_lp_hal_power_manager.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_iwdg.h"
#include "bluenrg_lp_ll_usart.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_gpio.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_flash.h"
#include "bluenrg_lp_ll_spi.h"
#include "bluenrg_lp_ll_exti.h"

#include "bluenrg_lp_evb_config.h"

/* Exported defines ------------------------------------------------------------*/
/* USART DMA Channel */
#define DMA_CH_UART_TX          LL_DMA_CHANNEL_1
#define DMA_CH_UART_RX          LL_DMA_CHANNEL_2

/* SPI DMA channels */
#define DMA_CH_SPI_TX           LL_DMA_CHANNEL_3
#define DMA_CH_SPI_RX           LL_DMA_CHANNEL_1

#ifndef STACK_VERSION
/* USART1 instance is used. (TX on PA.09, RX on PA.08) */
#define USARTx_INSTANCE               BSP_UART
#define USARTx_CLK_ENABLE()           BSP_UART_CLK_ENABLE()
#define USARTx_IRQn                   BSP_UART_IRQn

#define USARTx_GPIO_CLK_ENABLE()      BSP_UART_TX_GPIO_CLK_ENABLE()
#define USARTx_GPIO_AF                BSP_UART_TX_GPIO_AF_N
#define USARTx_GPIO_PORT              BSP_UART_TX_GPIO_PORT
#define USARTx_TX_PIN                 BSP_UART_TX_PIN
#define USARTx_SET_TX_GPIO_AF()       BSP_UART_TX_GPIO_AF()
  
#define USARTx_RX_PIN                 BSP_UART_RX_PIN
#define USARTx_SET_RX_GPIO_AF()       BSP_UART_RX_GPIO_AF()
#else
/* USART1 instance is used. (TX on PB.15, RX on PB.14) */
#define USARTx_INSTANCE               USART1
#define USARTx_CLK_ENABLE()           LL_APB1_EnableClock(LL_APB1_PERIPH_USART)
#define USARTx_IRQn                   USART1_IRQn
#define USARTx_IRQHandler             USART1_IRQHandler

#define USARTx_GPIO_CLK_ENABLE()      LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)
#define USARTx_GPIO_AF                LL_GPIO_AF_7
#define USARTx_GPIO_PORT              GPIOB
#define USARTx_TX_PIN                 LL_GPIO_PIN_15
#define USARTx_SET_TX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_15, LL_GPIO_AF_7)
#define USARTx_RX_PIN                 LL_GPIO_PIN_14
#define USARTx_SET_RX_GPIO_AF()       LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_14, LL_GPIO_AF_7)

#endif

/* SPI port */
#define SPIx_CLK_ENABLE()           BSP_SPI_CLK_ENABLE()
#define SPIx_INSTANCE               BSP_SPI
#define SPIx_GPIO_CLK_ENABLE()      BSP_SPI_MISO_GPIO_CLK_ENABLE()
#define SPIx_GPIO_PORT              BSP_SPI_MISO_GPIO_PORT
#define SPIx_CS_PIN                 BSP_SPI_CS_PIN
#define SPIx_SET_CS_GPIO_AF()       BSP_SPI_CS_GPIO_AF()
#define SPIx_MISO_PIN               BSP_SPI_MISO_PIN
#define SPIx_SET_MISO_GPIO_AF()     BSP_SPI_MISO_GPIO_AF()
#define SPIx_MOSI_PIN               BSP_SPI_MOSI_PIN
#define SPIx_SET_MOSI_GPIO_AF()     BSP_SPI_MOSI_GPIO_AF()
#define SPIx_CLK_PIN                BSP_SPI_CLK_PIN
#define SPIx_SET_CLK_GPIO_AF()      BSP_SPI_CLK_GPIO_AF()

#define SPIx_IRQ_PIN                BSP_SPI_IRQ_PIN
#define SPIx_IRQ_PORT               BSP_SPI_IRQ_GPIO_PORT
#define SPIx_IRQ_CLK_ENABLE()       BSP_SPI_IRQ_GPIO_CLK_ENABLE()

#define EXTI_CLK_ENABLE()           LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG)
#define EXTI_CS_PIN                 BSP_SPI_EXTI_CS_PIN
#define EXTI_CS_IRQn                BSP_SPI_EXTI_CS_IRQn

#ifdef UART_INTERFACE
#define IO_WAKEUP_PIN   0
#endif
#ifdef SPI_INTERFACE
#define IO_WAKEUP_PIN   BSP_SPI_WAKEUP_PIN
#endif

#ifdef DEBUG_DTM
#define DEBUG_GPIO_CLK_ENABLE()      LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB)   /* Enable the peripheral clock of GPIOB */
#define DEBUG_GPIO_PORT              GPIOB
#define DEBUG_TEST_1_PIN             LL_GPIO_PIN_4
#define DEBUG_TEST_2_PIN             LL_GPIO_PIN_5
#endif

/* Exported constants --------------------------------------------------------*/
#ifndef NO_DMA

#define DMA_RX_BUFFER_SIZE 1024

extern uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];

#endif
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef WATCHDOG
void WDG_Configuration(void);
#endif

void NVIC_Configuration(void);
void GPIO_Configuration(void);

#ifdef UART_INTERFACE
void UART_Configuration(void);
void UART_Cmd(FunctionalState state);
#endif

#ifdef SPI_INTERFACE
void SPI_Slave_Configuration(void);
#endif

#ifndef NO_DMA
void DMA_Configuration(void);
void DMA_Rearm(uint32_t dma_channel, uint32_t buffer, uint32_t size);
uint8_t DMAUart_SendData(uint32_t address, uint8_t size);
#endif

void NVIC_DisableRadioIrq(void);
void NVIC_EnableRadioIrq(void);

#endif /* HW_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
