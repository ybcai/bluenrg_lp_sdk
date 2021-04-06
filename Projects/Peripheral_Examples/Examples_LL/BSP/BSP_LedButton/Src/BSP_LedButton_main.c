
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BSP_LedButton_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the BSP functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  BSP_LedButton/BSP_LedButton_main.c
 * @brief This example describes how to configure the EXTI and use GPIOs to toggle 
 * the user LEDs available on the board when a user button is pressed.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\BSP\\BSP_LedButton\\MDK-ARM\\{STEVAL-IDB011V1}\\BSP_LedButton.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\BSP\\BSP_LedButton\\EWARM\\{STEVAL-IDB011V1}\\BSP_LedButton.eww</tt>
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
|     A6     |        U5          |
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
|     B8     |        DL2         |
|     B9     |        DL3         |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |
@endtable 


* \section LEDs_description LEDs description
@table
|  LED name  |            STEVAL-IDB011V1           |
-----------------------------------------------------
|     DL1    |               Not Used               |
|     DL2    |      Turn ON if PUSH1 is pressed     |
|     DL3    |   Toggle each time PUSH2 is pressed  |
|     DL4    |               Not Used               |
|     U5     |             Toggling LED             |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |            STEVAL-IDB011V1           |
---------------------------------------------------------
|      PUSH1     |    If it is pressed, DL2 stays ON    |
|      PUSH2     |   DL3 toggles each time was pressed  |
|      RESET     |           Reset BlueNRG-LP           |

@endtable

* \section Usage Usage

This example describes how to configure the EXTI and use GPIOs to toggle the user LEDs available on the board when a user button is pressed. 
This example is based on the BLUENRG_LP LL API. 
Peripheral initialization is done using LL initialization function to demonstrate BSP init usage.

PUSH1 uses the polling programming model to toggle a led connected to a specific GPIO pin.

In this example, PUSH2 is configured to generate an interrupt on each rising edge, in the interrupt routine a led connected to a specific GPIO pin is toggled.


**/
   
/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_ll_adc.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp_ll_usart.h"
#include "bluenrg_lp.h"
#include "bluenrg_lp_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */


/* Private includes ----------------------------------------------------------*/
#include "bluenrg_lp_evb_config.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

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
    while(1);
  }
    
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* Initialize the LEDs */
  BSP_LED_Init(BSP_LED_GREEN);
  BSP_LED_Init(BSP_LED_BLUE);
  BSP_LED_Init(BSP_LED_RED);
  
  /* Initialize the button BSP_PUSH1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_GPIO);
  
  /* Initialize the button BSP_PUSH2 as interrupt source */
  BSP_PB_Init(BSP_PUSH2, BUTTON_MODE_EXTI);
  
  /* Infinite loop */
  while (1) {
    /* If the button is pressed, then the LED turn ON */
    if(BSP_PB_GetState(BSP_PUSH1) == 1) {
      BSP_LED_On(BSP_LED_GREEN);
    }
    else {
      BSP_LED_Off(BSP_LED_GREEN);
    }
    
    /* Toggle the LED */
    LL_mDelay(100);
    BSP_LED_Toggle(BSP_LED_BLUE);
  }
  
}




/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/




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
