/**
  ******************************************************************************
  * @file    RCC_HSEStartupTest_main.c
  * @author  RF Application Team
  * @brief   This example provides a way to measure HSE startup time
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "RCC_HSEStartupTest_main.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_ll_gpio.h"


uint32_t old_wakeup_time_mach, wakeup_time_mach, wakeup_time_sys, hse_ready_time_sys;

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static VTIMER_HandleType timerHandle;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Init();
void ModulesInit(void);
void ModulesTick(void);
void timeoutCB(void *param);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  WakeupSourceConfig_TypeDef wakeupIO = {0};  /* No Wakeup Source needed */
  PowerSaveLevels stopLevel;

  /* MCU Configuration--------------------------------------------------------*/

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  GPIO_Init();

  BSP_COM_Init(NULL);
  
  ModulesInit();
  
  /* Enable HSE Ready interrupt */
  LL_RCC_EnableIT_HSERDY();  
  NVIC_EnableIRQ(RCC_IRQn);
  
  timerHandle.callback = timeoutCB;
  HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_INTERVAL_MS);
  
  printf("HSE test app started\n");

  /* Infinite loop */
  while (1)
  {
    BSP_LED_On(BSP_LED1);
    ModulesTick();
    BSP_LED_Off(BSP_LED1);
    
    if(WAKEUP->CM0_WAKEUP_TIME != wakeup_time_mach)
    {
      //BSP_LED_On(BSP_LED1);
      //wakeup_time_mach = WAKEUP->CM0_WAKEUP_TIME;
      wakeup_time_sys = TIMER_MachineTimeToSysTime(wakeup_time_mach);
//      printf("%u %u ", wakeup_time_mach, wakeup_time_sys);
//      while(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy());
      //BSP_LED_Off(BSP_LED1);
    }
    
    /* Power Save Request */
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    
    //if(stopLevel == POWER_SAVE_LEVEL_STOP_WITH_TIMER)
    {
      /* System has gone to sleep (unless un interrupt has occurred in the meantime,
         but this is not the case) */
       //printf("%u -> %d\n", hse_ready_time_sys, hse_ready_time_sys - wakeup_time_sys);
       //while(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy());
    }
  }
}

void GPIO_Init()
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {
    .Pin = LL_GPIO_PIN_X|LL_GPIO_PIN_7,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Speed = LL_GPIO_SPEED_FREQ_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Pull = LL_GPIO_PULL_NO,
  };
  
  BSP_LED_Init(BSP_LED1); //Activity led
  //BSP_LED_On(BSP_LED1);
  
  /* GPIO for test signal */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOX);  
  LL_GPIO_Init(GPIOX, &GPIO_InitStruct);
}

void ModulesInit(void)
{  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  
  /* VTimer module Init */  
  HAL_VTIMER_Init(&VTIMER_InitStruct);
}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
}

void timeoutCB(void *param)
{
  HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_INTERVAL_MS);  
}

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Turn LED3 on: Transfer Error */
  BSP_LED_On(BSP_LED3);
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 

  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
