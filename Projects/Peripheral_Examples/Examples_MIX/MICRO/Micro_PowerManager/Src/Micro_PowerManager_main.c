
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : Micro_PowerSave_main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : 25-March-2019
* Description        : Code demonstrating Power Save with BlueNRG-LP
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bluenrg_lp_it.h"
#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_led.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_rtc.h"
#include "hal_miscutil.h"
#include "bluenrg_lp_hal_power_manager.h"
#include "bluenrg_lp_hal_vtimer.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples BlueNRG-LP Peripheral Examples
  * @{
  */


/** @addtogroup Micro_Examples Micro Examples
  * @{
  */

/** @addtogroup Micro_PowerManager  Micro Power Manager Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        1000

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us 

#define WAKEUP_TIMEOUT 5000


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
static VTIMER_HandleType timerHandle;

/* Private function prototypes -----------------------------------------------*/
void help(void);
void PrintNegotiatedLevel(uint8_t stopLevel);
void PrintWakeupSource(uint32_t wakeupSources);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Display the help functions.
  * @param  None
  * @retval None
  */
void help(void)
{
  printf("POWER MANAGER help commands:\r\n");
  printf("s:   SHUTDOWN LEVEL : the only wakeup source is a low pulse on the RSTN pad\r\n");
  printf("t:   POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on uart (PA8)/timeout 5 sec (VTIMER)/button PUSH1 (PA10)\r\n");
  printf("z:   POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on uart (PA8)/timeout 5 sec (RTC)/button PUSH1 (PA10)\r\n");
  printf("n:   POWER_SAVE_LEVEL_NOTIMER : wake on uart (PA8)/button PUSH1 (PA10)\r\n");
  printf("c:   POWER_SAVE_LEVEL_CPU_HALT : wake on button PUSH1 (PA10)\r\n");
  printf("l:   Toggle led LED1\r\n");
  printf("p:   Print Hello World message\r\n");
  printf("r:   Reset the BlueNRG-LP\r\n");
  printf("?:   Display this help menu\r\n");
  printf("\r\n> ");
}

/**
  * @brief  Display the Stop Level negotiated.
  * @param  stopLevel negotiated Stop level
  * @retval None
  */
void PrintNegotiatedLevel(uint8_t stopLevel)
{
  printf("Power save level negotiated: ");
  switch (stopLevel)
  { 
  case POWER_SAVE_LEVEL_RUNNING:
    printf ("RUNNING\r\n");
    break;
  case POWER_SAVE_LEVEL_CPU_HALT:
    printf ("CPU_HALT\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_WITH_TIMER:
    printf ("STOP_WITH_TIMER\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_NOTIMER:
    printf ("STOP_NOTIMER\r\n");
    break;
  }
}

/**
  * @brief  Display the Wakeup Source.
  * @param  wakeupSource Wakeup Sources
  * @retval None
  */
void PrintWakeupSource(uint32_t wakeupSources)
{
  printf("Wakeup Source : ");
  switch (wakeupSources)
  {
  case WAKEUP_RTC:
    printf("WAKEUP_RTC ");
    break;
  case WAKEUP_BLE_HOST_TIMER:
    printf("WAKEUP_BLE_HOST_TIMER ");
    break;
  case WAKEUP_BLE:
    printf("WAKEUP_BLE ");
    break;
  case WAKEUP_PA11:
    printf("WAKEUP_PA11 ");
    break;
  case WAKEUP_PA10:
    printf("WAKEUP_PA10 ");
    break;
  case WAKEUP_PA9:
    printf("WAKEUP_PA9 ");
    break;
  case WAKEUP_PA8:
    printf("WAKEUP_PA8 ");
    break;
  case WAKEUP_PB7:
    printf("WAKEUP_PB7 ");
    break;
  case WAKEUP_PB6:
    printf("WAKEUP_PB6 ");
    break;
  case WAKEUP_PB5:
    printf("WAKEUP_PB5 ");
    break;
  case WAKEUP_PB4:
    printf("WAKEUP_PB4 ");
    break;
  case WAKEUP_PB3:
    printf("WAKEUP_PB3 ");
    break;
  case WAKEUP_PB2:
    printf("WAKEUP_PB2 ");
    break;
  case WAKEUP_PB1:
    printf("WAKEUP_PB1 ");
    break;
  case WAKEUP_PB0:
    printf("WAKEUP_PB0 ");
    break;
  default:
    printf("0x%08x ", wakeupSources);
  }
  printf("\r\n");
}

void RTC_WakeupInit(void)
{
  /* Enable Peripheral Clock */
  LL_APB0_EnableClock(LL_APB0_PERIPH_RTC);
  for (volatile int i=0; i<0xFFFF; i++);
  
  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Init mode setup */
  LL_RTC_EnableInitMode(RTC);
  
  /* Wait till the Init mode is active */
  while(LL_RTC_IsActiveFlag_INIT(RTC) == RESET);
  
  /* Configure Hour Format */
  LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
  
  /* Output disabled */
  LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
  
  /* Output polarity */
  LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
  
  /* Set Synchronous prescaler factor */
  LL_RTC_SetSynchPrescaler(RTC, 31);
  
  /* Set Asynchronous prescaler factor */
  LL_RTC_SetAsynchPrescaler(RTC, 0);
  
  /* Exit Initialization mode */
  LL_RTC_DisableInitMode(RTC);

  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

void SetRTC_WakeupTimeout(uint32_t time)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);

  /* Disable Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* In case of interrupt mode is used, the interrupt source must disabled */
  LL_RTC_DisableIT_WUT(RTC);
  
  /* Wait till RTC WUTWF flag is set  */
  while(LL_RTC_IsActiveFlag_WUTW(RTC) == 0);
  
  /* Clear PWR wake up Flag */
  LL_PWR_ClearWakeupSource(LL_PWR_EWS_INT);
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  /* Configure the Wake-up Timer counter */
  LL_RTC_WAKEUP_SetAutoReload(RTC, time);
  
  /* Configure the clock source */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Configure the Interrupt in the RTC_CR register */
  LL_RTC_EnableIT_WUT(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* Configure NVIC for RTC */
  NVIC_SetPriority(RTC_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(RTC_IRQn);    
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);

}

void DisableRTC_WakeupTimeout(void)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);

  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);

}

/**
  * @brief  Timeout callback.
  * @param  Timer handle
  * @retval None
  */
void TimeoutCallback(void *timerHandle)
{
  /* Add app code to execute @ Stop timeout */
  printf("Vtimer Timeout!!!!!\r\n");
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t data, ret_val, tmp;
  PartInfoType partInfo; 
  WakeupSourceConfig_TypeDef wakeupIO;
  uint32_t wakeupSources;
  PowerSaveLevels stopLevel;
  HAL_VTIMER_InitType VTIMER_InitStruct;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_16M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init the UART peripheral */
  BSP_COM_Init(BSP_COM_RxDataUserCb);
  
  /* Init LED1 */
  BSP_LED_Init(BSP_LED1);
  
  /* Init BUTTON 1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* VTimer module Init */
  VTIMER_InitStruct.XTAL_StartupTime = HS_STARTUP_TIME;
  VTIMER_InitStruct.EnableInitialCalibration = INITIAL_CALIBRATION;
  VTIMER_InitStruct. PeriodicCalibrationInterval = CALIBRATION_INTERVAL;
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  timerHandle.callback = TimeoutCallback;

  /* RTC Wakeup Peripheral Init */
  RTC_WakeupInit();
    
  printf("Power Manager FW demo!\r\nDigit ? for help command\r\n");
  
  while(1) {
    /* To run the VTIMER state machine */
    HAL_VTIMER_Tick();
    
    if (BSP_COM_Read(&data)) {
      switch(data)
      {
      case 's':   
        { 
          /* SHUTDOWN LEVEL : the only wakeup source is a low pulse on the RSTN pad */
          printf("Enable Power Save Request : SHUTDOWN\r\n");
          while(BSP_COM_UARTBusy());
          if (HAL_PWR_MNGR_ShutdownRequest(TRUE) != SUCCESS)
            printf("ERORR during the SHUTDOWN Request!\r\n");
        }
        break;
      case 't':
        {
          /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART (PA8)/timeout 5 sec (VTIMER)/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_WITH_TIMER (VTIMER)\r\n");
          while(BSP_COM_UARTBusy());          
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
          wakeupIO.RTC_enable = 0;          
          ret_val = HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_TIMEOUT);
          if (ret_val != SUCCESS) {
            printf("HAL_VTIMER_StartTimerMs() error 0x%02x\r\n", ret_val);
            while(1);
          }
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
        }
        break;
      case 'z':
        {
          /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART (PA8)/timeout 5 sec (RTC)/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_WITH_TIMER (RTC)\r\n");
          while(BSP_COM_UARTBusy());          
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
          wakeupIO.RTC_enable = 1;          
          SetRTC_WakeupTimeout(WAKEUP_TIMEOUT);
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
          DisableRTC_WakeupTimeout();
        }
        break;
      case 'n':
        {
          /* POWER_SAVE_LEVEL_NOTIMER : wake on uart (PA8)/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_NOTIMER\r\n");
          while(BSP_COM_UARTBusy());
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
          wakeupIO.RTC_enable = 0;          
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
        }
        break;
      case 'c':
        {
          /* POWER_SAVE_LEVEL_CPU_HALT : wake on button PUSH1 (PA10) */
          printf("Enable Power Save Request : CPU_HALT\r\n");
          while(BSP_COM_UARTBusy());          
          __WFI();
          printf("Exit from WFI cortex instruction with interrupt\r\n");
          /* Empty the USART buffer */
          while(BSP_COM_Read(&tmp));
        }
        break;
      case 'l':
        {
          /* Toggle led LED1 */
          BSP_LED_Toggle(BSP_LED1);
        }
        break;
      case 'p':
        {
          HAL_GetPartInfo(&partInfo);
          printf("Hello World: BlueNRG-LP (%d.%d) is here!\r\n",
                 partInfo.die_major,
                 partInfo.die_cut);
        }
        break;
      case 'r':
        {
          /* Reset BlueNRG-LP */
          NVIC_SystemReset();
        }
        break;
      case '?':
        {
          /* Help command */
          help();
        }
        break;
      default:
        printf("UNKNWON COMMAND! Press ? for command list\r\n");
      }
    }
  }  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif


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
