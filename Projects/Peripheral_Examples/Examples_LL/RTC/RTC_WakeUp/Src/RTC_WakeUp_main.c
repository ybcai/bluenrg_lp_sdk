
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : RTC_WakeUp_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the RTC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RTC_WakeUp/RTC_WakeUp_main.c
 * @brief This example shows how to configure RTC peripheral to set the power save level stop with timer.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\RTC\\RTC_WakeUp\\MDK-ARM\\{STEVAL-IDB011V1}\\RTC_WakeUp.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\RTC\\RTC_WakeUp\\EWARM\\{STEVAL-IDB011V1}\\RTC_WakeUp.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1



* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  |   STEVAL-IDB011V1  |
-----------------------------------
|     A1     |      Not Used      |
|     A11    |      Not Used      |
|     A12    |      Not Used      |
|     A13    |      Not Used      |
|     A14    |      Not Used      |
|     A15    |      Not Used      |
|     A4     |      Not Used      |
|     A5     |      Not Used      |
|     A6     |      Not Used      |
|     A7     |      Not Used      |
|     A8     |      USART TX      |
|     A9     |      USART RX      |
|     B0     |       wakeup       |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |      Not Used      |
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200           | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |
-----------------------------------
|     DL1    |      Not Used      |
|     DL2    |      Not Used      |
|     DL3    |      Not Used      |
|     DL4    |      Not Used      |
|     U5     |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |
---------------------------------------
|      PUSH1     |      Not Used      |
|      PUSH2     |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This example shows how to configure GPIO, USART and RTC peripherals to set the power save level stop with timer. 
This example is based on the RTC LL API. 
The peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).


Example execution:
After startup from reset and system configuration, the  RTC Wakeup peripheral is initialized.
The application shows a state massage on USART peripheral and it is put in stop mode with timer enabled.
On first  character reception from USART Com port (ex: using HyperTerminal) the device is waked up. 
The device can be waked up also :
- pressing the User push-button (PUSH1)
- connecting PB0 to Vbat pin 
After 5 sec, if User doesn't interact with the application the device is waked up due to the RTC wake up timer is elapsed.

This example is applicable only for BlueNRG_LP cut 2.0 or above.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin (GPIO PA.09) to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin (GPIO PA.08) to PC COM port TX signal
- Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 

**/
   


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bluenrg_lp_it.h"
#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_led.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_rtc.h"
#include "bluenrg_lp_hal_power_manager.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples BlueNRG-LP Peripheral Examples
* @{
*/

/** @addtogroup RTC Examples
* @{
*/

/** @addtogroup RTC_WakeUp Example
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* the WUTF flag is set every (WUT[15:0] + 1) ck_wut cycles */
#define WAKEUP_TIMEOUT 4

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/  

/* Private function prototypes -----------------------------------------------*/
void PrintNegitatedLevel(uint8_t stopLevel);
void PrintWakeupSource(uint32_t wakeupSources);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Display the Stop Level negotiated.
* @param  stopLevel negotiated Stop level
* @retval None
*/
void PrintNegitatedLevel(uint8_t stopLevel)
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
  case WAKEUP_PA10:
    printf("WAKEUP_PA10 ");
    break;
  case WAKEUP_PA8:
    printf("WAKEUP_PA8 ");
    break;
  case WAKEUP_PB0:
    printf("WAKEUP_PB0 ");
    break;
  default:
    printf("(default) WAKEUP source 0x%08x ", wakeupSources);
  }
  printf("\r\n");
}

void RTC_WakeupInit(void)
{  
  /* Enable RTC peripheral Clocks */
  /* Enable RTC Clock */
  LL_APB0_EnableClock(LL_APB0_PERIPH_RTC);
  
  /* Force RTC peripheral reset */
  LL_APB0_ForceReset(LL_APB0_PERIPH_RTC);
  LL_APB0_ReleaseReset(LL_APB0_PERIPH_RTC);
  
  /* Check if RTC Reset Release flag interrupt occurred or not */
  while(LL_RCC_IsActiveFlag_RTCRSTREL() == 0)
  {
  }
  
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
  
  /* Set Asynchronous prescaler factor */
  LL_RTC_SetAsynchPrescaler(RTC, 0x7F);
  
  /* Set Synchronous prescaler factor */
  LL_RTC_SetSynchPrescaler(RTC, 0x00FF);
  
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
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
  uint8_t ret_val;
  WakeupSourceConfig_TypeDef wakeupIO;
  uint32_t wakeupSources;
  PowerSaveLevels stopLevel;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);
  
  /* Init BUTTON 1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
 
  /* RTC Wakeup Peripheral Init */
  RTC_WakeupInit();
  
  /* Pull UP/DOWN configuration to reduce the current leakage */
  LL_PWR_EnablePDA(LL_PWR_PUPD_IO2|LL_PWR_PUPD_IO3|LL_PWR_PUPD_IO4|
                   LL_PWR_PUPD_IO5|LL_PWR_PUPD_IO6|LL_PWR_PUPD_IO7|
                     LL_PWR_PUPD_IO10|LL_PWR_PUPD_IO11|LL_PWR_PUPD_IO12|
                       LL_PWR_PUPD_IO13|LL_PWR_PUPD_IO14|LL_PWR_PUPD_IO15);
  LL_PWR_EnablePDB(0xffff);
  
  while(1) {
    /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART (PA8)/timeout 5 sec, (RTC)/button PUSH1 (PA10) */
    printf("Enable Power Save Request : STOP_WITH_TIMER (RTC)\r\n");
    while(BSP_COM_UARTBusy());          
    wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10|WAKEUP_PB0;
    wakeupIO.IO_Mask_Low_polarity = WAKEUP_PA8;
    wakeupIO.RTC_enable = 1;          
    SetRTC_WakeupTimeout(WAKEUP_TIMEOUT);
    ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
    if (ret_val != SUCCESS)
      printf("Error during clock config 0x%2x\r\n", ret_val);
    PrintNegitatedLevel(stopLevel);
    if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
      wakeupSources = HAL_PWR_MNGR_WakeupSource();
      PrintWakeupSource(wakeupSources);
    }
    DisableRTC_WakeupTimeout();
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