
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : ADC_BatterySensor_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the ADC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  ADC_BatterySensor/ADC_BatterySensor_main.c
 * @brief How to use an ADC peripheral to perform a single ADC conversion on the 
 *  internal temperature sensor and calculate the temperature in degrees Celsius. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\ADC\\ADC_BatterySensor\\MDK-ARM\\{STEVAL-IDB011V1}\\ADC_BatterySensor.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\ADC\\ADC_BatterySensor\\EWARM\\{STEVAL-IDB011V1}\\ADC_BatterySensor.eww</tt>
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
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |

@endtable 

@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
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

This example shows how to sample the supplied voltage using the ADC peripheral.
The data is sampled and then printed out by using the USART peripheral with a rate of 1 sample each 100 ms.
This example is driven by polling the End of conversion from Down Sampler.
The overrun from Down Sampler event is also monitored.
This example is based on the BLUENRG_LP ADC HAL API.

Example configuration:
The ADC is configured to sample the supplied voltage by using the Down Sampler internal path of the ADC.

Example execution:
The user must just load and then run the application.
The user can get the output data through a serial terminal program.

Connection needed:
None.

Other peripherals used:
  1 USART for output the data acquired

**/
   

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal.h"
#include "bluenrg_lp_evb_config.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define USER_SAMPLERATE       (ADC_SAMPLE_RATE_28)
#define USER_DATAWIDTH        (ADC_DS_DATA_WIDTH_12_BIT)
#define USER_RATIO            (ADC_DS_RATIO_128)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
ADC_ConfigChannelTypeDef xChannel;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t nVBattRawVal = 0;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  /* Parameters for ADC initialization */
  __HAL_RCC_ADCDIG_CLK_ENABLE();
  __HAL_RCC_ADCANA_CLK_ENABLE();
  
    /* Enable the ADC peripheral */
  HAL_ADC_StructInit(&hadc);
  hadc.Init.DataRatio = USER_RATIO;
  hadc.Init.DataWidth = USER_DATAWIDTH;
  hadc.Init.SampleRate = USER_SAMPLERATE;
  
  if (HAL_ADC_Init(&hadc) != HAL_OK) {
    Error_Handler();
  }
  
  /* Set the input channel */
  xChannel.ChannelType = ADC_CH_BATTERY_LEVEL_DETECTOR;
  xChannel.SequenceNumber = ADC_SEQ_POS_01;
  HAL_ADC_ConfigChannel(&hadc, &xChannel);
  
  /* Start ADC conversion */
  HAL_ADC_Start(&hadc);
  
  /* Infinite loop */
  while (1) {
    
    /* Check the ADC flag End Of Down Sampler conversion */
    if( HAL_ADC_PollForConversion(&hadc, 100) != HAL_OK) {
      
      /* Get the battery raw value from the Down Sampler */
      nVBattRawVal = HAL_ADC_GetValue(&hadc);
      
      /* Printout the output value */
      printf("Battery voltage %d\r\n", nVBattRawVal);
      
      /* Toggle the conversion/activity LED */
      BSP_LED_Toggle(BSP_LED1);
      
      /* Add 100 ms of delay between each measure */
      HAL_Delay(100);
      
      /* Start ADC conversion */
      HAL_ADC_Start(&hadc);
    }
    
    //    /* Check the ADC flag overrun of Down Sampler */
    //    if( LL_ADC_IsActiveFlag_OVRDS(ADC) == 1) {
    //      
    //      /* Clear the ADC flag overrun of Down Sampler */
    //      LL_ADC_ClearFlag_OVRDS(ADC);
    //      
    //      /* Turn on the LED2 is overrun occurs */
    //      BSP_LED_On(BSP_LED2);
    //    }
    
  }
}



/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
