/**
  ******************************************************************************
  * @file    bluenrg_lp_hal_gpio_ex.h
  * @author  RF Application Team
  * @brief   Header file of GPIO HAL Extended module.
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
#ifndef BLUENRG_LP_HAL_GPIO_EX_H
#define BLUENRG_LP_HAL_GPIO_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal_def.h"

/** @addtogroup BLUENRG_LP_HAL_Driver
  * @{
  */

/** @defgroup GPIOEx GPIOEx
  * @brief GPIO Extended HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Constants GPIOEx Exported Constants
  * @{
  */

/** @defgroup GPIOEx_Alternate_function_selection GPIOEx Alternate function selection
  * @{
  */


 /* The table below gives an overview of the different alternate functions per port.
  * For more details refer yourself to the product data sheet.
  *
  */

 /*     |   AF0        |   AF1       |   AF2    |   AF3    |   AF4    |   AF5    |   AF6    |   AF7    |
  *____________________________________________________________________________________________________
  *     |SYS_AF/I2C    |USART/SPI    |SPI/USART |SPI       | TIM      | ADC      |          |  RF      |
  *     |LPUART        |SYS_AF/PDM   |TIM/RTC   |LPUART    |          | SYS_AF   |          |  SYS_AF  |
  *     |USART/SPI     |LPUART/I2C/RF|I2C/PDM   |TIM       |          |          |          |  USART   |
  *     |              |             |SYS_AF    |PDM       |          |          |          |          |
  *____________________________________________________________________________________________________
  * PA0 |I2C1_SCL      |USART_CTS    |SPI2_MCK  |          |TIM1_CH3  |ADC_DTB0  |          |RF_DTB7   |
  * PA1 |I2C1_SDA      |SPI2_MISO    |USART_TX  |          |TIM1_CH4  |ADC_DTB1  |          |RF_DTB0   |
  * PA2 |SWDIO         |USART_CK     |TIM_BKIN  |SPI3_MCK  |TIM1_CH5  |SWDIO     |          |SWDIO     |
  * PA3 |SWCLK         |USART_RTS_DE |TIM_BKIN2 |SPI3_SCK  |TIM1_CH6  |SWCLK     |          |SWCLK     |
  * PA4 |LCO           |SPI2_NSS     |          |LPUART_TX |TIM1_CH1  |ADC_DTB2  |          |          |
  * PA5 |MCO           |SPI2_SCK     |          |LPUART_RX |TIM1_CH2  |ADC_DTB3  |          |          |
  * PA6 |LPUART_CTS    |SPI2_MOSI    |          |SPI2_NSS  |TIM1_CH1  |ADC_DTB4  |          |RF_DTB16  |
  * PA7 |LPUART_RTS_DE |SPI2_MISO    |          |SPI2_SCK  |TIM1_CH2  |ADC_DTB5  |          |RF_DTB17  |
  * PA8 |USART_RX      |SPI1_MOSI    |          |SPI3_MISO |TIM1_CH3  |ADC_DTB6  |          |RF_DTB5   |
  * PA9 |USART_TX      |SPI1_SCK     |RTC_OUT   |SPI3_NSS  |TIM1_CH4  |ADC_DTB7  |          |RF_DTB6   |
  * PA10|LCO           |SPI1_MISO    |          |SPI3_MCK  |TIM1_CH5  |ADC_DTB8  |          |RF_DTB3   |
  * PA11|MCO           |SPI1_NSS     |          |SPI3_MOSI |TIM1_CH6  |ADC_DTB9  |          |RF_DTB4   |
  * PA12|I2C1_SMBA     |SWDIO        |SPI1_NSS  |SPI2_MOSI |TIM1_CH1  |          |          |RF_DTB13  |
  * PA13|I2C2_SCL      |SWCLK        |SPI1_SCK  |SPI2_MISO |TIM1_ETR  |          |          |RF_DTB11  |
  * PA14|I2C2_SDA      |             |SPI1_MISO |          |TIM1_BKIN |          |          |RF_DTB10  |
  * PA15|I2C2_SMBA     |             |SPI1_MOSI |          |TIM1_BKIN2|          |          |RF_DTB12  |
  *____________________________________________________________________________________________________
  * PB0 |USART_RX      |LPUART_RTS_DE|          |TIM1_CH2N |          |          |          |RF_DTB13  |
  * PB1 |SPI1_NSS      |PDM_CLK      |          |TIM1_ETR  |          |          |          |RF_DTB11  |
  * PB2 |USART_RTS_D   |PDM_DATA     |          |TIM1_CH3  |          |          |          |RF_DTB9   |
  * PB3 |USART_CTS     |LPUART_TX    |          |TIM1_CH4  |          |          |          |RF_DTB8   |
  * PB4 |LPUART_TX     |SPI2_MISO    |          |PDM_DATA  |          |          |          |RF_DTB10  |
  * PB5 |LPUART_RX     |SPI2_MOSI    |          |PDM_CLK   |          |          |          |RF_DTB15  |
  * PB6 |I2C2_SCL      |SPI2_NSS     |          |LPUART_TX |TIM1_CH1  |ADC_DTB10 |          |RF_DTB2   |
  * PB7 |I2C2_SDA      |SPI2_SCK     |          |LPUART_RX |TIM1_CH2  |ADC_DTB11 |          |RF_DTB1   |
  * PB8 |USART_CK      |LPUART_RX    |          |TIM1_CH4  |TIM1_CH1N |ADC_DTB12 |          |RF_DTB14  |
  * PB9 |USART_TX      |LPUART_CTS   |SPI2_MCK  |TIM1_CH1N |TIM1_CH2N |ADC_DTB13 |          |RF_DTB18  |
  * PB10|SPI1_NSS      |SPI2_SCK     |I2C1_SDA  |TIM1_CH2  |TIM1_CH3N |ADC_DTB14 |          |RF_DTB15  |
  * PB11|SPI1_SCK      |SPI2_NSS     |I2C1_SCL  |TIM1_CH1  |TIM1_CH4N |ADC_DTB15 |          |          |
  * PB12|SPI1_SCK      |LCO          |PDM_DATA  |TIM1_BKIN |TIM1_CH3  |ADC_DTB16 |          |          |
  * PB13|SPI1_MISO     |I2C2_SCL     |PDM_CLK   |TIM1_BKIN2|TIM1_CH4  |ADC_DTB17 |          |          |
  * PB14|SPI1_MOSI     |I2C2_SDA     |TIM1_ETR  |TIM1_CH3N |TIM1_CH5  |ADC_DTB18 |          |          |
  * PB15|I2C1_SMBA     |TX_SEQUENCE  |MCO       |TIM1_CH4N |TIM1_CH6  |          |          |          |
  *____________________________________________________________________________________________________*/

/**
  * @brief   AF 0 selection
  */

#define GPIO_AF0_MCO           ((uint8_t)0x00)  /*!< MCO Alternate Function mapping                 */
#define GPIO_AF0_LCO           ((uint8_t)0x00)  /*!< LCO Alternate Function mapping                 */
#define GPIO_AF0_SWDIO         ((uint8_t)0x00)  /*!< SWDIO Alternate Function mapping               */
#define GPIO_AF0_SWCLK         ((uint8_t)0x00)  /*!< SWCLK Alternate Function mapping               */
#define GPIO_AF0_I2C1          ((uint8_t)0x00)  /*!< I2C1 Alternate Function mapping                */
#define GPIO_AF0_I2C2          ((uint8_t)0x00)  /*!< I2C2 Alternate Function mapping                */
#define GPIO_AF0_LPUART1       ((uint8_t)0x00)  /*!< LPUART Alternate Function mapping              */
#define GPIO_AF0_USART1        ((uint8_t)0x00)  /*!< USART Alternate Function mapping               */
#define GPIO_AF0_SPI1          ((uint8_t)0x00)  /*!< SPI1 Alternate Function mapping                */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_LCO           ((uint8_t)0x01)  /*!< LCO Alternate Function mapping                 */
#define GPIO_AF1_SWDIO         ((uint8_t)0x01)  /*!< SWDIO Alternate Function mapping               */
#define GPIO_AF1_SWCLK         ((uint8_t)0x01)  /*!< SWCLK Alternate Function mapping               */
#define GPIO_AF1_SPI1          ((uint8_t)0x01)  /*!< SPI1 Alternate Function mapping                */
#define GPIO_AF1_SPI2          ((uint8_t)0x01)  /*!< SPI2 Alternate Function mapping                */
#define GPIO_AF1_USART1        ((uint8_t)0x01)  /*!< USART Alternate Function mapping               */
#define GPIO_AF1_LPUART1       ((uint8_t)0x01)  /*!< LPUART Alternate Function mapping              */
#define GPIO_AF1_PDM           ((uint8_t)0x01)  /*!< PDM Alternate Function mapping                 */
#define GPIO_AF1_I2C2          ((uint8_t)0x01)  /*!< I2C2 Alternate Function mapping                */
#define GPIO_AF1_TX_SEQUENCE   ((uint8_t)0x01)  /*!< TX Sequence Alternate Function mapping         */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_MCO           ((uint8_t)0x02)  /*!< MCO Alternate Function mapping                 */
#define GPIO_AF2_SPI1          ((uint8_t)0x02)  /*!< SPI1 Alternate Function mapping                */
#define GPIO_AF2_SPI2          ((uint8_t)0x02)  /*!< SPI2 Alternate Function mapping                */
#define GPIO_AF2_TIM1          ((uint8_t)0x02)  /*!< TIM1 Alternate Function mapping                */
#define GPIO_AF2_USART1        ((uint8_t)0x02)  /*!< USART Alternate Function mapping               */
#define GPIO_AF2_RTC           ((uint8_t)0x02)  /*!< RTC Alternate Function mapping                 */
#define GPIO_AF2_I2C1          ((uint8_t)0x02)  /*!< I2C2 Alternate Function mapping                */
#define GPIO_AF2_PDM           ((uint8_t)0x02)  /*!< PDM Alternate Function mapping                 */
   
/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_SPI3           ((uint8_t)0x03)  /*!< SPI3 Alternate Function mapping               */
#define GPIO_AF3_SPI2           ((uint8_t)0x03)  /*!< SPI2 Alternate Function mapping               */
#define GPIO_AF3_TIM1           ((uint8_t)0x03)  /*!< TIM1 Alternate Function mapping               */
#define GPIO_AF3_PDM            ((uint8_t)0x03)  /*!< TIM1 Alternate Function mapping               */
#define GPIO_AF3_LPUART1        ((uint8_t)0x03)  /*!< TIM1 Alternate Function mapping               */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_TIM1           ((uint8_t)0x04)  /*!< TIM1 Alternate Function mapping               */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_ADC           ((uint8_t)0x05)  /*!< ADC Alternate Function mapping                 */
#define GPIO_AF5_SWDIO         ((uint8_t)0x05)  /*!< SWDIO Alternate Function mapping               */
#define GPIO_AF5_SWCLK         ((uint8_t)0x05)  /*!< SWCLK Alternate Function mapping               */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_RF_DTB        ((uint8_t)0x07)  /*!< RF DTB Alternate Function mapping              */
#define GPIO_AF7_SWDIO         ((uint8_t)0x07)  /*!< SWDIO Alternate Function mapping               */
#define GPIO_AF7_SWCLK         ((uint8_t)0x07)  /*!< SWCLK Alternate Function mapping               */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /*!< USART Alternate Function mapping               */

#define IS_GPIO_AF(AF)         ((AF) <= (uint8_t)0x07)


/**
  * @}
  */ 

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup GPIOEx_Exported_Macros GPIOEx Exported Macros
  * @{
  */

/** @defgroup GPIOEx_Get_Port_Index GPIOEx Get Port Index
* @{
  */

#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0uL : 1uL)

 /**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/ 
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* BLUENRG_LP_HAL_GPIO_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
