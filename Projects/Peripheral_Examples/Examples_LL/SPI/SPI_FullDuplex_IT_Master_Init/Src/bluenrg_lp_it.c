/**
  ******************************************************************************
  * @file    Examples_LL/SPI/SPI_FullDuplex_IT_Master_Init/Src/bluenrg_lp_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
/* Includes ------------------------------------------------------------------*/
#include "SPI_FullDuplex_IT_Master_Init_main.h"
#include "bluenrg_lp_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_IRQHandler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_IRQHandler(void)
{  
  while (1)
  {
  }
}


/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_IRQHandler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_IRQHandler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_IRQHandler(void)
{
}

/******************************************************************************/
/* BLUENRG_LP Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_bluenrg_lp.s).                    */
/******************************************************************************/

/**
  * @brief  This function handles SPI_MASTER interrupt request.
  * @param  None
  * @retval None
  */
void SPI_MASTER_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_SPI_IsActiveFlag_RXNE(SPI_MASTER))
  {
    /* Call function Slave Reception Callback */
    SPI_MASTER_Rx_Callback();
  }
  /* Check TXE flag value in ISR register */
  if(LL_SPI_IsActiveFlag_TXE(SPI_MASTER))
  {
    /* Call function Slave Reception Callback */
    SPI_MASTER_Tx_Callback();
  }
  /* Check STOP flag value in ISR register */
  if(LL_SPI_IsActiveFlag_OVR(SPI_MASTER))
  {
    /* Call Error function */
    SPI_MASTER_TransferError_Callback();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
