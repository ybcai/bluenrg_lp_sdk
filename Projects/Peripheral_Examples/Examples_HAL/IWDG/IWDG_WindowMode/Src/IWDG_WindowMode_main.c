
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : IWDG_WindowMode_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the IWDG functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file IWDG_WindowMode/IWDG_WindowMode_main.c
 * @brief How to periodically update the IWDG reload counter and simulate a software fault that generates an MCU IWDG reset after a preset laps of time.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\IWDG\\IWDG_WindowMode\\MDK-ARM\\{STEVAL-IDB011V1}\\IWDG_WindowMode.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\\Projects\\Peripheral_Examples\\Examples_HAL\\IWDG\\IWDG_WindowMode\\EWARM\\{STEVAL-IDB011V1}\\IWDG_WindowMode.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c IWDG_WindowMode - Release configuration


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
|  LED name  |           STEVAL-IDB011V1          |
---------------------------------------------------
|     DL1    |              Not Used              |
|     DL2    |  Blinking: application is running  |
|     DL3    |              On: error             |
|     DL4    |              Not Used              |
|     U5     |              Not Used              |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |    STEVAL-IDB011V1   |
-----------------------------------------
|      PUSH1     |   Activate the IWDG  |
|      PUSH2     |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |

@endtable

* \section Usage Usage

How to periodically update the IWDG reload counter and simulate a software fault that generates 
an MCU IWDG reset after a preset laps of time.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.

The IWDG time-out is set to 762 ms (the time-out may varies due to LSI frequency 
dispersion).

The Window option is enabled with a window register value set to 400 ms.
To prevent a reset, the down-counter must be reloaded when its value is:
 -lower than the window register value (400ms)
 -greater than 0x0
The IWDG counter is therefore refreshed each 450 ms in the main program infinite loop to 
prevent a IWDG reset (762 - 450 = 312 within the interval).
LED2 is also toggled each 450 ms indicating that the program is running. 
LED3 will turn on if any error occurs.

An EXTI Line is connected to a GPIO pin, configured to generate an interrupt
when the User push-button (PUSH1) is pressed.

Once the EXTI Line event occurs by pressing the User push-button (PUSH1), the refresh period is set to 200 ms.
That will make refresh being outside window value. As a result, when the IWDG counter is reloaded, the IWDG reset occurs.

In the ISR, a write to invalid address generates a Hard fault exception containing an infinite loop and preventing to return to main program (the IWDG counter is not refreshed).
As a result, when the IWDG counter falls to 0, the IWDG reset occurs.
If the IWDG reset is generated, after the system resumes from reset, the message "IWDGRST flag is set." is showed over the UART.

If the EXTI Line event does not occur, the IWDG counter is indefinitely refreshed
in the main program infinite loop, and there is no IWDG reset.
  
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
#include "IWDG_WindowMode_main.h"

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define IWDG_WINDOW (32000 * 400) / (16 * 1000)  /* 400 ms */
#define IWDG_RELOAD (32000 * 762) / (16 * 1000)  /* 762 ms */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

/* IWDG and TIM handlers declaration */
RCC_ClkInitTypeDef RCC_ClockFreq;

/* In while loop, time to wait before refresh */
__IO uint32_t WaitingDelay = 450;

/* Private function prototypes -----------------------------------------------*/
static void MX_IWDG_Init(void);

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
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure LED2 and LED3 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* BSP COM Init */
  BSP_COM_Init(NULL);
  
  /* Check if the system has resumed from IWDG reset */
  printf("Check if the system has resumed from IWDG reset:\n\r");
  if( (RAM_VR.ResetReason & RCC_FLAG_WDGRST) == RCC_FLAG_WDGRST)
  {
    /* IWDGRST flag is set */
    printf("YES, IWDG reset is generated.\n\r");
  }
  else
  {
    printf("NO.\n\r");
  }
  printf("\n\r");
  
  /* Clear reset flags anyway */
  __HAL_RCC_CLEAR_RESET_FLAGS();
  
  /* Initialize all configured peripherals */
  MX_IWDG_Init();
  
  /* Configure User push-button (PUSH1) */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* Infinite loop */
  while (1)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(BSP_LED2);
    
    HAL_Delay(WaitingDelay);
    
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
  }
}

/**
* @brief IWDG Initialization Function
* @param None
* @retval None
*/
static void MX_IWDG_Init(void)
{
  /* WDG clock enable */
  __HAL_RCC_WDG_CLK_ENABLE();
  
  /* Force WDG peripheral reset */
  __HAL_RCC_WDG_FORCE_RESET();
  __HAL_RCC_WDG_RELEASE_RESET();
  
  /* Check if WDG Reset Release flag interrupt occurred or not */
  while(__HAL_RCC_GET_IT(RCC_IT_WDGRSTRELRDY) == 0)
  {
  }
  
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = IWDG_WINDOW;
  hiwdg.Init.Reload = IWDG_RELOAD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  EXTI line detection callback to decreaase waiting delay. That will make
refresh being outside window value.
* @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
* @retval None
*/
void HAL_GPIO_EXTI_Callback(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BSP_PUSH1_PIN)
  {
    /* Switch off LED2 */
    BSP_LED_Off(BSP_LED2);
    /* waiting 200 ms to be above window value on next refresh */
    WaitingDelay = 1000;
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(BSP_LED3);
  
  /* Infinite loop */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
