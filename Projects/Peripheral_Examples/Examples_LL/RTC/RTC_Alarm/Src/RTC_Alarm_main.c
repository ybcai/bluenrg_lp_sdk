
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : RTC_Alarm_main.c
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
 * @file  RTC_Alarm/RTC_Alarm_main.c
 * @brief Configuration of the RTC LL API to configure and generate an alarm using the RTC peripheral. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\RTC\\RTC_Alarm\\MDK-ARM\\{STEVAL-IDB011V1}\\RTC_Alarm.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\RTC\\RTC_Alarm\\EWARM\\{STEVAL-IDB011V1}\\RTC_Alarm.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c RTC_Alarm - Release configuration


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
|     B0     |      Not Used      |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |        DL2         |
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |                 STEVAL-IDB011V1                |
---------------------------------------------------------------
|     DL1    |                    Not Used                    |
|     DL2    |   ON: RTC Alarm is generated; Blinking: error  |
|     DL3    |                    Not Used                    |
|     DL4    |                    Not Used                    |
|     U5     |                    Not Used                    |

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

Configuration of the RTC LL API to configure and generate an alarm using the RTC peripheral. The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

The RTC peripheral configuration is ensured by the Configure_RTC() function (configure of the needed RTC resources according to the used hardware CLOCK, PWR, RTC clock source and BackUp). 
You may update this function to change RTC configuration.

Configure_RTC_Alarm function is then called to initialize the alarm.

In this example, the Time is set to 23:59:50 and the Alarm must be generated on 00:00:00.

The current time and date are updated and displayed on USART peripheral.

- LED2 is turned ON when the RTC Alarm Interrupt is generated correctly.
- LED2 is toggling : This indicates that the system generates an error.

In this case, to monitor LED2 toggling frequency, an oscilloscope should be used.

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
#include "RTC_Alarm_main.h"
#include "bluenrg_lp_evb_config.h"
#include <string.h> 

/** @addtogroup BLUENRG_LP_LL_Examples
* @{
*/

/** @addtogroup RTC_Alarm
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Oscillator time-out values */
#define LSI_TIMEOUT_VALUE          ((uint32_t)2)     /* 2 ms */
#define LSE_TIMEOUT_VALUE          ((uint32_t)5000)  /* 5 s */
#define RTC_TIMEOUT_VALUE          ((uint32_t)1000)  /* 1 s */


#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[] = "hh:ms:ss";
uint8_t aShowDate[] = "dd/mm/aaaa";


uint8_t aShowTimePreviousValue[] = "hh:ms:ss";

/* Private function prototypes -----------------------------------------------*/
static void     LL_Init(void);
void     Configure_RTC(void);
void     Configure_RTC_Alarm(void);
uint32_t Enter_RTC_InitMode(void);
uint32_t Exit_RTC_InitMode(void);
uint32_t WaitForSynchro_RTC(void);
void     Show_RTC_Calendar(void);
void     LED_Init(void);
void     LED_On(void);
void     LED_Off(void);
void     LED_Blinking(uint32_t Period);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Main program
* @param  None
* @retval None
*/
int main(void)
{
  /* System initialization function */
  /* Configure the system clock to 32 MHz */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* BSP COM Init */
  /* needed to use printf over the USART */
  BSP_COM_Init(NULL);
    
  printf("\n\r\n\rRTC_Alarm example is started.\n\r");

  /* Initialize LED2 */
  LED_Init();
  LED_Off();
  
  /* Configure the RTC peripheral */
  Configure_RTC();
   
  /* Configure Alarm */
  /* Configure RTC Alarm */
  Configure_RTC_Alarm();
  
  /* Infinite loop */
  while (1)
  {
    /* Display the updated Time and Date */
    Show_RTC_Calendar();
    LL_mDelay(1000);
  }
}

/**
* @brief  Configure RTC.
* @note   Peripheral configuration is minimal configuration from reset values.
*         Thus, some useless LL unitary functions calls below are provided as
*         commented examples - setting is default configuration from reset.
* @param  None
* @retval None
*/
void Configure_RTC(void)
{  
  /* The low speed clocks to use for this example is the LSE */
#ifndef CONFIG_HW_LS_XTAL
#error "No Low Speed Crystal definition!!!"
#endif
  
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
  
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Enter in initialization mode */
  if (Enter_RTC_InitMode() != RTC_ERROR_NONE)
  {
    /* Initialization Error */
    LED_Blinking(LED_BLINK_ERROR);
  }
  
  /* Configure RTC */
  /* Configure RTC prescaler and RTC data registers */
  /* Set Hour Format */
  LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
  /* Set Asynch Prediv (value according to source clock) */
  LL_RTC_SetAsynchPrescaler(RTC, RTC_ASYNCH_PREDIV);
  /* Set Synch Prediv (value according to source clock) */
  LL_RTC_SetSynchPrescaler(RTC, RTC_SYNCH_PREDIV);
  
  /* Configure the Date */
  /* Note: __LL_RTC_CONVERT_BIN2BCD helper macro can be used if user wants to*/
  /*       provide directly the decimal value:                               */
  /*       LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_MONDAY,                    */
  /*                          __LL_RTC_CONVERT_BIN2BCD(31), (...))           */
  /* Set Date: Friday December 31th 2019 */
  LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_TUESDAY, 0x31, LL_RTC_MONTH_DECEMBER, 0x19);
  
  /* Configure the Time */
  /* Set Time: 23:59:50 */
  LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, 0x23, 0x59, 0x50);
  
  /* Set OutPut */
  /* Reset value is LL_RTC_ALARMOUT_DISABLE */
  //LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
  /* Set OutPutPolarity */
  /* Reset value is LL_RTC_OUTPUTPOLARITY_PIN_HIGH */
  //LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
  
  /* Exit of initialization mode */
  if (Exit_RTC_InitMode() != RTC_ERROR_NONE)
  {
    /* Initialization Error */
    LED_Blinking(LED_BLINK_ERROR);
  }
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

/**
* @brief  Configure the current time and date.
* @note   Peripheral configuration is minimal configuration from reset values.
*         Thus, some useless LL unitary functions calls below are provided as
*         commented examples - setting is default configuration from reset.
* @param  None
* @param  None
* @retval None
*/
void Configure_RTC_Alarm(void)
{
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Disable Alarm*/
  LL_RTC_ALMA_Disable(RTC);
  
  /* Configure the RTC Alarm peripheral */
  /* Set Alarm to 00:00:00
  RTC Alarm Generation: Alarm on Hours, Minutes and Seconds (ignore date/weekday)*/
  LL_RTC_ALMA_ConfigTime(RTC, LL_RTC_TIME_FORMAT_PM, 0x00, 0x00, 0x00);
  LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_DATEWEEKDAY);
  
  /* Note: following interfaces may be used but not needed as default values are used.*/
  //LL_RTC_ALMA_SetSubSecond(RTC, 0x00);
  //LL_RTC_ALMA_SetSubSecondMask(RTC, 0);
  //LL_RTC_ALMA_DisableWeekday(RTC);
  //LL_RTC_ALMA_SetDay(RTC, 0x01);
  
  /* Clear the Alarm interrupt pending bit */
  LL_RTC_ClearFlag_ALRA(RTC);
  
  /* Enable IT Alarm */
  LL_RTC_EnableIT_ALRA(RTC);
  
  /* Configure the NVIC for RTC Alarm */
  NVIC_SetPriority(RTC_IRQn, 0x0);
  NVIC_EnableIRQ(RTC_IRQn);
  
  /* Enable Alarm*/
  LL_RTC_ALMA_Enable(RTC);
  
  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

/**
* @brief  Enter in initialization mode
* @note In this mode, the calendar counter is stopped and its value can be updated
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Enter_RTC_InitMode(void)
{
  /* Set Initialization mode */
  LL_RTC_EnableInitMode(RTC);
  
#if (USE_TIMEOUT == 1)
  Timeout = RTC_TIMEOUT_VALUE;
#endif /* USE_TIMEOUT */
  
  /* Check if the Initialization mode is set */
  while (LL_RTC_IsActiveFlag_INIT(RTC) != 1)
  {
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      Timeout --;
    }
    if (Timeout == 0)
    {
      return RTC_ERROR_TIMEOUT;
    }
#endif /* USE_TIMEOUT */
  }
  
  return RTC_ERROR_NONE;
}

/**
* @brief  Exit Initialization mode
* @param  None
* @retval RTC_ERROR_NONE if no error
*/
uint32_t Exit_RTC_InitMode(void)
{
  LL_RTC_DisableInitMode(RTC);
  
  /* Wait for synchro */
  /* Note: Needed only if Shadow registers is enabled           */
  /*       LL_RTC_IsShadowRegBypassEnabled function can be used */
  return (WaitForSynchro_RTC());
}

/**
* @brief  Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are
*         synchronized with RTC APB clock.
* @param  None
* @retval RTC_ERROR_NONE if no error (RTC_ERROR_TIMEOUT will occur if RTC is
*         not synchronized)
*/
uint32_t WaitForSynchro_RTC(void)
{
  /* Clear RSF flag */
  LL_RTC_ClearFlag_RS(RTC);
  
#if (USE_TIMEOUT == 1)
  Timeout = RTC_TIMEOUT_VALUE;
#endif /* USE_TIMEOUT */
  
  /* Wait the registers to be synchronised */
  while (LL_RTC_IsActiveFlag_RS(RTC) != 1)
  {
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      Timeout --;
    }
    if (Timeout == 0)
    {
      return RTC_ERROR_TIMEOUT;
    }
#endif /* USE_TIMEOUT */
  }
  return RTC_ERROR_NONE;
}

/**
* @brief  Display the current time and date.
* @param  None
* @retval None
*/
void Show_RTC_Calendar(void)
{
  /* Note: need to convert in decimal value in using __LL_RTC_CONVERT_BCD2BIN helper macro */
  /* Display time Format : hh:mm:ss */
  sprintf((char *)aShowTime, "%.2d:%.2d:%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)));
  /* Display date Format : mm-dd-yy */
  sprintf((char *)aShowDate, "%.2d-%.2d-%.2d", __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)),
          __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)),
          2000 + __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)));
  
  // print on the UART the time and date
  BSP_COM_Write("\n\r", 2);
  BSP_COM_Write("\n\r", 2);
  BSP_COM_Write((uint8_t *)aShowTime, sizeof(aShowTime)+1);
  BSP_COM_Write("\n\r", 2);
  BSP_COM_Write((uint8_t *)aShowDate, sizeof(aShowTime)+1);
}

/**
* @brief  Initialize LED2.
* @param  None
* @retval None
*/
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LED2_GPIO_CLK_ENABLE();
  
  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
  
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Turn-on LED2.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(Period);
  }
}


static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
* @brief  Alarm callback
* @param  None
* @retval None
*/
void Alarm_Callback(void)
{
  /* Turn LED2 on: Alarm generation */
  LED_On();
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(char *file, uint32_t line)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
