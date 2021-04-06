/**
******************************************************************************
* @file    I2S/I2S_Transmitter/Src/bluenrg_lp_hal_msp.c
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
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "I2S_Transmitter_main.h"

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
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  GPIO_InitTypeDef  GPIO_InitStruct;  
  
  static DMA_HandleTypeDef hdma_i2sTx_1;
  
  /* CODEC_I2S pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = AUDIO_OUT1_I2S_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = AUDIO_OUT1_I2S_SCK_AF;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_SD_AF;
  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_SD_PIN ;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_SD_GPIO_PORT, &GPIO_InitStruct);	
  
  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_WS_AF;
  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_WS_PIN;
  HAL_GPIO_Init(AUDIO_OUT1_I2S_WS_GPIO_PORT, &GPIO_InitStruct); 
  
  // uncomment these lines to use the MCK
//  GPIO_InitStruct.Alternate   = AUDIO_OUT1_I2S_MCK_AF;
//  GPIO_InitStruct.Pin         = AUDIO_OUT1_I2S_MCK_PIN; 
//  HAL_GPIO_Init(AUDIO_OUT1_I2S_MCK_GPIO_PORT, &GPIO_InitStruct);   
  
  /* Enable the I2S DMA clock */
  AUDIO_OUT1_I2S_DMAx_CLK_ENABLE(); 
  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {
    /* Configure the hdma_i2sTx handle parameters */   
    hdma_i2sTx_1.Init.Request             = DMA_REQUEST_SPI2_TX;   
    hdma_i2sTx_1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2sTx_1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sTx_1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sTx_1.Init.PeriphDataAlignment = AUDIO_OUT1_I2S_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sTx_1.Init.MemDataAlignment    = AUDIO_OUT1_I2S_DMAx_MEM_DATA_SIZE;
    hdma_i2sTx_1.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sTx_1.Init.Priority            = DMA_PRIORITY_HIGH;     
    hdma_i2sTx_1.Instance                 = AUDIO_OUT1_I2S_DMAx_STREAM;
    
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sTx_1);
    
    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx_1);
    
    /* Configure the DMA Stream */
    if (HAL_DMA_Init(&hdma_i2sTx_1) != HAL_OK)
    {
      Error_Handler();
    }
    
    /* DMA interrupt init */
    /* DMA_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(AUDIO_OUT1_I2S_DMAx_IRQ, 0);
    HAL_NVIC_EnableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ);
  }
}


/**
* @brief  Deinitializes I2S MSP.
* @param  hi2s  I2S handle 
* @retval HAL status
*/
static void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{     
  AUDIO_OUT1_I2S_CLK_DISABLE();  
  
  /* I2S pins de initialization: MCK, WS, SCK and SD pins -----------------------------*/
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SCK_GPIO_PORT, AUDIO_OUT1_I2S_SCK_PIN);  
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_SD_GPIO_PORT, AUDIO_OUT1_I2S_SD_PIN);    
  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_WS_GPIO_PORT, AUDIO_OUT1_I2S_WS_PIN);
  // uncomment this line to use the MCK
//  HAL_GPIO_DeInit(AUDIO_OUT1_I2S_MCK_GPIO_PORT, AUDIO_OUT1_I2S_MCK_PIN);
  
  if(hi2s->Instance == AUDIO_OUT1_I2S_INSTANCE)
  {     
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(hi2s->hdmatx); 
    
    /* I2S DMA IRQ Channel configuration */
    HAL_NVIC_DisableIRQ(AUDIO_OUT1_I2S_DMAx_IRQ); 
  }    
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/