/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_IT_Slave/Src/bluenrg_lp_hal_msp.c
  * @author  RF Application Team
  * @brief   HAL MSP module.
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

/* Includes ------------------------------------------------------------------*/
#include "SPI_FullDuplex_IT_Slave_main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI_SLAVE)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI_SLAVE_CLK_ENABLE();
  
    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_SCK;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_SCK;
    HAL_GPIO_Init(GPIO_SLAVE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_MISO;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_MISO;
    HAL_GPIO_Init(GPIO_SLAVE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_SPI_SLAVE_MOSI;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF_SPI_SLAVE_MOSI;
    HAL_GPIO_Init(GPIO_SLAVE, &GPIO_InitStruct);
    
    /* SPI_SLAVE interrupt Init */
    HAL_NVIC_SetPriority(SPI_SLAVE_IRQn, 0);
    HAL_NVIC_EnableIRQ(SPI_SLAVE_IRQn);
  }
}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI_SLAVE)
  {
    /* Reset peripherals */
    __HAL_RCC_SPI_SLAVE_FORCE_RESET();
    __HAL_RCC_SPI_SLAVE_RELEASE_RESET();

    /* Peripheral clock disable */
    __HAL_RCC_SPI_SLAVE_CLK_DISABLE();
  
    HAL_GPIO_DeInit(GPIO_SLAVE, GPIO_PIN_SPI_SLAVE_SCK|GPIO_PIN_SPI_SLAVE_MISO|GPIO_PIN_SPI_SLAVE_MOSI);

    /* SPI_SLAVE interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI_SLAVE_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/