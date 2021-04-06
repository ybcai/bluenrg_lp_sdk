
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : TIM_Frequency_Divider_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the TIM functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  TIM_Frequency_Divider/TIM_Frequency_Divider_main.c
 * @brief Configuration of the TIM peripheral to generate an output signal with a frequency equal at the half of the input signal.  
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\TIM\\TIM_Frequency_Divider\\MDK-ARM\\{STEVAL-IDB011V1}\\TIM_Frequency_Divider.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\TIM\\TIM_Frequency_Divider\\EWARM\\{STEVAL-IDB011V1}\\TIM_Frequency_Divider.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c TIM_Frequency_Divider - Release configuration


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
|     A11    |      TIM1 CH6      |
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
|     B14    |      TIM1 ETR      |
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
|  LED name  |      STEVAL-IDB011V1     |
-----------------------------------------
|     DL1    |         Not Used         |
|     DL2    |         Not Used         |
|     DL3    |         Not Used         |
|     DL4    |         Not Used         |
|     U5     |         Not Used         |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |      STEVAL-IDB011V1     |
---------------------------------------------
|      PUSH1     |         Not Used         |
|      PUSH2     |         Not Used         |
|      RESET     |     Reset BlueNRG-LP     |

@endtable

* \section Usage Usage

Configuration of the TIM peripheral to generate an output signal with a frequency equal at the half of the input signal.  
This example is based on the BLUENRG_LP TIM HAL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size). 

TIM1 is configured with the External clock source mode 2 enabled.
The configuration continues setting the channel 6 (TIM peripheral) in PWM (pulse width modulation) mode.
The output signal generated has the frequency equal to the half of the input signal.


Connect the following pins to an oscilloscope to monitor the different waveforms:
- TIM1_CH6 : PA11
- TIM1_ETR : PB14  

  
BlueNRG_LP-EVB Set-up
- Connect the external signal to use as refernce for the test to PB14 pin (TIM1_ETR).
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
#include "TIM_Frequency_Divider_main.h"

/** @addtogroup BLUENRG_LP_HAL_Examples
* @{
*/

/** @addtogroup TIM_Frequency_Divider
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* Private function prototypes -----------------------------------------------*/
static void MX_TIM1_Init(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* BSP COM Init */
  /* needed to use printf over the USART */
  BSP_COM_Init(NULL);
  
  /* Initialize all configured peripherals */
  MX_TIM1_Init();
  
  /* Start channel  in Output compare mode */
  if(HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_6) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  while (1)
  {
  }
}

/**
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0x00;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0x01;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockFilter = 0x0;                                   /* No filter */
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0x01;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  /* Initializes the TIM PWM Time Base */
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 
  The external Clock can be configured, if needed (the default clock is the
  internal clock from the APBx), using the following function:
  HAL_TIM_ConfigClockSource, the clock configuration should be done before
  any start function.
  */
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Initializes the TIM PWM  channel 6 */
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_TIM_MspPostInit(&htim1);
  
}   

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
static void Error_Handler(void)
{
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
