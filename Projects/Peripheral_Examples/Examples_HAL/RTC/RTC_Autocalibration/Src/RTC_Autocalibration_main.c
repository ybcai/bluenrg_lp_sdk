
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : RTC_Autocalibration_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 19-April-2019
* Description        : Code demonstrating the basic RTC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  RTC_Autocalibration/RTC_Autocalibration_main.c
 * @brief Use of the LSE clock source autocalibration to get a precise RTC clock.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\RTC\\RTC_Autocalibration\\MDK-ARM\\{STEVAL-IDB011V1}\\RTC_Autocalibration.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\RTC\\RTC_Autocalibration\\EWARM\\{STEVAL-IDB011V1}\\RTC_Autocalibration.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c RTC_Autocalibration - Release configuration


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
|     A4     |        LCO         |
|     A5     |      Not Used      |
|     A6     |      TIM1 CH1      |
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
|     B9     |        DL3         |
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
|  LED name  |                    STEVAL-IDB011V1                   |
---------------------------------------------------------------------
|     DL1    |                       Not Used                       |
|     DL2    |   Blinking: toggled inside the RTC WakeUp interrupt  |
|     DL3    |                 Slow blinking: error                 |
|     DL4    |                       Not Used                       |
|     U5     |                       Not Used                       |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |     STEVAL-IDB011V1    |
-------------------------------------------
|      PUSH1     |        Not Used        |
|      PUSH2     |        Not Used        |
|      RESET     |    Reset BlueNRG-LP    |

@endtable

* \section Usage Usage

Use of the LSE clock source autocalibration to get a precise RTC clock. 
In the associated software, the system clock is set to 64 MHz.
As an application example, it demonstrates how to configure the TIM1 timer internally connected to LSE clock output, in order to adjust the RTC prescaler. 

The Low Speed External (LSE) clock is used as RTC clock source, it's depend on what is setted into the preprocessore define CONFIG_HW_LS_XTAL.
After reset, the RTC prescaler is set with the default LS frequency (32 kHz). 
The RTC WakeUp is configured to generate an interrupt each 1s.

LED2 is toggled inside the RTC WakeUp interrupt each 1s.

The inaccuracy of the LS clock causes the RTC WakeUp Interrupt to be inaccurate.
The RTC CK_SPRE signal can be monitored by LED2 which is toggled into the RTC Wakeup interrupt service routine.

When calibration is done the variable uwLsiFreq is visualized into the debugger to indicate the end of this operation. The LSE value is updated and displayed on the debugger in uwLsiFreq variable.

LED3 is On: This indicates that the system generates an error.

Connect the LCO pin (PA.04) to the TIM1 CH1 (PA.06).

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
#include "RTC_Autocalibration_main.h"
#include "bluenrg_lp_hal_rtc_ex.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

/* Captured Values */
uint32_t uwIC2Value1 = 0;
uint32_t uwIC2Value2 = 0;
uint32_t uwDiffCapture = 0;

/* Capture index */
uint16_t uhCaptureIndex = 0;

/* Frequency Value */
uint32_t uwFrequency = 0;


uint16_t tmpCCTIM_CHANNEL_1[2] = {0, 0};
__IO uint32_t uwLsFreq = 0;
     uint32_t uwCkApreFreq = 256;
__IO uint32_t uwCkSpreFreq = 1000;

__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;

__IO uint8_t RTCStatus = 0;
__IO uint8_t RTCStatusPrevious = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static uint32_t GetLSIFrequency(void);

/* Private user code ---------------------------------------------------------*/

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{ 
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* BSP COM Init */
  /* needed to use printf over the USART */
  BSP_COM_Init(NULL);
  
  /* Configure LED2 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_RTC_Init();

  /* Infinite loop */
  RTCStatus = 1;
  while (1)
  {
    if(RTCStatusPrevious != RTCStatus)
    {
      RTCStatusPrevious = RTCStatus;
      printf("RTCStatus=%d\n\r", RTCStatus);
    }
  }
}

/**
* @brief RTC Initialization Function
* @param None
* @retval None
*/
static void MX_RTC_Init(void)
{
/* The low speed clocks to use for this example is the LSE */
#ifndef CONFIG_HW_LS_XTAL
#error "No Low Speed Crystal definition!!!"
#endif
  
   /* RTC clock enable */
  __HAL_RCC_RTC_CLK_ENABLE();
  
  /* Force RTC peripheral reset */
  __HAL_RCC_RTC_FORCE_RESET();
  __HAL_RCC_RTC_RELEASE_RESET();
  
  /* Check if RTC Reset Release flag interrupt occurred or not */
  while(__HAL_RCC_GET_IT(RCC_IT_RTCRSTRELRDY) == 0)
  {
  }
  
  /* Get the Low Speed frequency:  TIM1 is used to measure the LSI frequency */
  uwLsFreq = GetLSIFrequency();
  
  /* Update the Calendar Configuration with the LSI exact value */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = (uwLsFreq / uwCkApreFreq ) - 1;;
  hrtc.Init.SynchPrediv = (uwLsFreq / (hrtc.Init.AsynchPrediv + 1)) - 1;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  
  /* Initialize RTC Only */
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Enable the WakeUp */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Clear flag Wake-Up */
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  
   /* Configure NVIC for RTC */
  NVIC_SetPriority(RTC_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(RTC_IRQn);  
}

/**
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM1_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
    /* The low speed clocks can be output on the LCO I/O */
#ifdef CONFIG_HW_LS_XTAL
  HAL_RCCEx_LSCOConfig(RCC_LSCO1,LL_RCC_LSCOSOURCE_LSE);
#else
#ifdef CONFIG_HW_LS_RO
  HAL_RCCEx_LSCOConfig(RCC_LSCO1,RCC_LSCOSOURCE_LSI);
#else
#warning "No Low Speed Crystal definition!!!"
#endif
#endif
}

/**
* @param  None
* @retval LSI Frequency
*/
static uint32_t GetLSIFrequency(void)
{
  /* Set TIMx instance */
  htim1.Instance = TIM1;
  /* Connect internally the TIM1 Input Capture of TIM_CHANNEL_1 to the LSI clock output */
    
  /* Start the TIM Input Capture measurement in interrupt mode */
  if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Wait until the TIM1 gets 2 LSI edges */
  while (uwCaptureNumber != 2)
  {
  }
  
  /* Disable TIM1 Capture/Compare channel Interrupt Request */
  HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
  
  /* Deinitialize the TIM1 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit(&htim1);
  
  return uwFrequency;
}

/**
* @brief  Input Capture callback in non blocking mode
* @param  htim : TIM IC handle
* @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if(uhCaptureIndex == 0)
    {
      /* Get the 1st Input Capture value */
      uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1)
    {
      /* Get the 2nd Input Capture value */
      uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 

      /* Capture computation */
      if (uwIC2Value2 > uwIC2Value1)
      {
        uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
      }
      else if (uwIC2Value2 < uwIC2Value1)
      {
        /* 0xFFFF is max TIM1_CCRx value */
        uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }

      /* Frequency computation: for this example TIMx (TIM1) is clocked by
         64 MHz Clk */ 
      uwFrequency = TIM_PERIPHCLK / uwDiffCapture;
      uhCaptureIndex = 0;
      uwCaptureNumber = 2;
    }
  }
}

/**
* @brief  RTC wakeup timer callback
* @param  hrtc :  RTC Handle Structure definition handle
* @retval None
*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Toggle LED2 */
  BSP_LED_Toggle(BSP_LED2);
  RTCStatus = (RTCStatus + 1 ) % 2;
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
     HAL_Delay(1000);
     BSP_LED_Toggle(BSP_LED3);
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
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
