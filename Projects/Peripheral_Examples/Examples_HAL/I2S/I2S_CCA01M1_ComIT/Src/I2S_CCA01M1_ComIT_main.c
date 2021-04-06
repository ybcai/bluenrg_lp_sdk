
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : I2S_CCA01M1_ComIT_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the I2S functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  I2S_CCA01M1_ComIT/I2S_CCA01M1_ComIT_main.c
 * @brief The application is designed to play a pre-recorded audio frame, saved in the MCU flash memory.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\I2S\\I2S_CCA01M1_ComIT\\MDK-ARM\\{STEVAL-IDB011V1}\\I2S_CCA01M1_ComIT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\I2S\\I2S_CCA01M1_ComIT\\EWARM\\{STEVAL-IDB011V1}\\I2S_CCA01M1_ComIT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c I2S_CCA01M1_ComIT - Release configuration


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
|     A11    |        MCO         |
|     A12    |    OUT1_RST_PIN    |
|     A13    |      I2C2_SCL      |
|     A14    |      I2C2_SDA      |
|     A15    |    OUT1_PD_PIN     |
|     A4     |      I2S_WS        |
|     A5     |      I2S_CK        |
|     A6     |      I2S_SD        |
|     A7     |      Not Used      |
|     A8     |      Not Used      |
|     A9     |      Not Used      |
|     B0     |      Not Used      |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |      Not Used      |
|     B9     |      I2S_MCK       |
|     GND    |    CN6/6 CCA01M1   |
|     RST    |      Not Used      |
|    VBAT    |    CN6/4 CCA01M1   |
@endtable 


* \section LEDs_description LEDs description
@table
|  LED name  |    STEVAL-IDB011V1   |
-------------------------------------
|     DL1    |       Not Used       |
|     DL2    |       Not Used       |
|     DL3    |       Not Used       |
|     DL4    |       Not Used       |
|     U5     |       Not Used       |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |                      STEVAL-IDB011V1                     |
-----------------------------------------------------------------------------
|      PUSH1     |  Switch audio mode (setVolume/Mute/UnMute/Pause/Resume)  |
|      PUSH2     |                         Not Used                         |
|      RESET     |                     Reset BlueNRG-LP                     |

@endtable

* \section Usage Usage

The application is designed to play a pre-recorded audio frame, saved in the MCU flash memory, by means of the STA350BW device soldered on the X-NUCLEO-CCA01M1 expansion board. The application runs on BlueNRG_LP.

The application demonstrates both basic audio functionalities, like initialization/play/mute/unMute/pause/resume controls. 
It is possible to switch between several setup using the user button PUSH1 of the BlueNRG_LP.

In order to achieve these results, the application performs the following steps:
- Initialize communication peripheral, such as I2C and I2S buses
- initialize STA350BW audio device by means of I2C bus
- Start the playing of the audio frame from the BlueNRG_LP flash memory

In the firmware, audio-related parts are collected in the audio_application.c file, that makes use of the dedicated X-NUCLEO-CCA01M1 BSP layer. 

BlueNRG_LP-EVB Set-up with X-Nucleo-CCA01M1, connect the follow pins:
  I2C2_SCL:     PA13  -  CN10/3  PB8 
  I2C2_SDA:     PA14  -  CN10/5  PB9 
  OUT1_RST_PIN: PA12  -  CN10/33 PA10
  I2S_WS:       PA4   -  CN10/16 PB12
  I2S_CK:       PA5   -  CN10/30 PB13
  I2S_SD:       PA6   -  CN10/26 PB15
  DL2:          PB8   -  don't set! todo test
  I2S_MCK:      PB9   -  CN10/4  PC6
  3.3V          VBAT  -  CN6/4
  GND                 -  CN6/6  
for the power output stage of the X-Nucleo-CCA01M1 (5 V to 26 V) power source must be connected to the CN2 connector on the board:
  5V  power source    -  VS signal, pin 1 on the CN2 
  GND power source    -  ground connection on the CN2.2 


**/
   

/* Includes ------------------------------------------------------------------*/
#include "I2S_CCA01M1_ComIT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
EXTI_HandleTypeDef HEXTI_InitStructure;
I2S_HandleTypeDef hi2s;

/* Private function prototypes -----------------------------------------------*/
static void EXTI10_IRQHandler_Config(void);

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    Error_Handler();
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure External line 10 (connected to PA.10 pin) in interrupt mode */
  EXTI10_IRQHandler_Config();

  if(Init_AudioOut_Device() != 0)
  {
    Error_Handler(); 
  }
  
  /* Start Audio Streaming*/
  if(Start_AudioOut_Device() != 0)
  {
    Error_Handler(); 
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configures EXTI line 10 (connected to PA.10 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  EXTI_ConfigTypeDef EXTI_Config_InitStructure;
  
  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Configure PA.10 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  EXTI_Config_InitStructure.Line =    EXTI_LINE_PA10;
  EXTI_Config_InitStructure.Trigger = EXTI_TRIGGER_RISING_EDGE;
  EXTI_Config_InitStructure.Type =    EXTI_TYPE_EDGE;
   
  HAL_EXTI_SetConfigLine(&HEXTI_InitStructure, &EXTI_Config_InitStructure);
  HAL_EXTI_RegisterCallback(&HEXTI_InitStructure, HAL_EXTI_COMMON_CB_ID, Example_EXTI_Callback);
  HAL_EXTI_Cmd(&HEXTI_InitStructure , ENABLE);
  
  HAL_EXTI_ClearPending(&HEXTI_InitStructure);
  
  /* Enable and set line 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(GPIOA_IRQn, 0);
  HAL_NVIC_EnableIRQ(GPIOA_IRQn);
  
  /* Configure NVIC for SysTick_IRQn */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0);
}   
                                                 
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void Example_EXTI_Callback(uint32_t Line)
{
    Switch_Demo();
}
                                 
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    HAL_GPIO_TogglePin(AUDIO_OUT1_RST_GPIO_PORT, AUDIO_OUT1_RST_PIN);  
    HAL_Delay(100);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
