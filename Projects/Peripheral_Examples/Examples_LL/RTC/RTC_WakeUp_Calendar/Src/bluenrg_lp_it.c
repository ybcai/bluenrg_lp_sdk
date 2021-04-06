/**
******************************************************************************
* @file    BlueNRGLP_it.c 
* @author  AMS RF Application Team
* @version V1.0.0
* @date    25-March-2019
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
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
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_it.h"
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_evb_button.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_ll_rtc.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  This function handles Hard Fault exception.
*/
void HardFault_IRQHandler(void)
{ 
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/******************************************************************************/
/*                 BLUENRG_LP Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (EXTI), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_bluenrg_lp.s).                                               */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/
void USART1_IRQHandler(void)
{  
}

/**
* @brief  This function handles DMA Handler.
*/
void DMA_IRQHandler(void)
{
}

/**
* @brief  This function handles GPIO interrupt request.
* @param  None
* @retval None
*/
void GPIOA_IRQHandler(void)
{
  if (BSP_PB_GetITPendingBit(BSP_PUSH1)) {
    BSP_PB_ClearITPendingBit(BSP_PUSH1);
  } 
}

/**
* @brief  This function handles RTC interrupt request.
* @param  None
* @retval None
*/
void RTC_IRQHandler(void)
{
  if(LL_RTC_IsActiveFlag_WUT(RTC)) {
    LL_RTC_ClearFlag_WUT(RTC);
  }
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
