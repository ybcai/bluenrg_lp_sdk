
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : UART_HyperTerminal_IT_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 27-May-2019
* Description        : Code demonstrating the use of a UART to transmit data (transmit/receive) 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  UART_HyperTerminal_IT/UART_HyperTerminal_IT_main.c
 * @brief Use of a UART to transmit data (transmit/receive)  between a board and an HyperTerminal PC application in Interrupt mode. This example  describes how to use the USART peripheral through the BLUENRG_LP UART HAL  and LL API, the LL API being used for performance improvement.
 *
 
* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\UART\\UART_HyperTerminal_IT\\MDK-ARM\\{STEVAL-IDB011V1}\\UART_HyperTerminal_IT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\UART\\UART_HyperTerminal_IT\\EWARM\\{STEVAL-IDB011V1}\\UART_HyperTerminal_IT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.


* \subsection Project_configurations Project configurations
- \c UART_HyperTerminal_IT - UART configuration


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
|  PIN name  |      STEVAL-IDB011V1     |
-----------------------------------------
|     A1     |         Not Used         |
|     A11    |         Not Used         |
|     A12    |         Not Used         |
|     A13    |         Not Used         |
|     A14    |         Not Used         |
|     A15    |         Not Used         |
|     A4     |         Not Used         |
|     A5     |         Not Used         |
|     A6     |         Not Used         |
|     A7     |         Not Used         |
|     A8     |         Not Used         |
|     A9     |         Not Used         |
|     B0     |         Not Used         |
|     B14    |         Not Used         |
|     B2     |         Not Used         |
|     B3     |         Not Used         |
|     B4     |         Not Used         |
|     B5     |         Not Used         |
|     B7     |         Not Used         |
|     B8     |         Not Used         |
|     B9     |         Not Used         |
|     GND    |         Not Used         |
|     RST    |         Not Used         |
|    VBAT    |         Not Used         |

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
|  LED name  |                            STEVAL-IDB011V1                           |
-------------------------------------------------------------------------------------
|     DL1    |                               Not Used                               |
|     DL2    |                               Not Used                               |
|     DL3    |  ON: error in transmission/reception process - Slow blinking: error  |
|     DL4    |                               Not Used                               |
|     U5     |                               Not Used                               |

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

Use of a UART to transmit data (transmit/receive) between a board and an HyperTerminal PC application in Interrupt mode. 
This example describes how to use the USART peripheral through the BLUENRG_LP UART HAL and LL API, the LL API being used for performance improvement.                 

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The UART peripheral configuration is ensured by the HAL_UART_Init() function.
This later is calling the HAL_UART_MspInit() function which core is implementing the configuration of the needed UART resources according to the used hardware.
You may update this function to change UART configuration.

The UART/Hyperterminal communication is then initiated.
Receive and Transmit functions which allow respectively the reception of Data from Hyperterminal and the transmission of a predefined data 
buffer, are implemented using LL USART API.

The Asynchronous communication aspect of the UART is clearly highlighted as the data buffers transmission/reception to/from Hyperterminal are done simultaneously.

For this example the TX buffer (aTxStartMessage) is predefined and the RX buffer (aRxBuffer) size is limited to 10 data by the mean of the RXBUFFERSIZE define in the UART_HyperTerminal_IT_main.c file.

In a first step the received data will be stored in the RX buffer and the TX buffer content will be displayed in the Hyperterminal interface.
In a second step the received data in the RX buffer will be sent back to Hyperterminal and displayed.

BlueNRG_LP Nucleo board's LEDs can be used to monitor the transfer status:
 - LED3 is ON when there is an error in transmission/reception process.  
 - LED3 toggles when there another error is detected.  

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration.

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
#include "UART_HyperTerminal_IT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
__IO uint8_t  ubTxComplete = 0;
__IO uint8_t  ubRxComplete = 0;

/* Buffer used for transmission */
uint8_t aTxStartMessage[] = "\n\r ****UART-Hyperterminal communication based on IT (Mixed HAL/LL usage) ****\n\r Enter 10 characters using keyboard :\n\r";
__IO uint32_t uwTxIndex = 0;
uint8_t ubSizeToSend = sizeof(aTxStartMessage);
uint8_t aTxEndMessage[] = "\n\r Example Finished\n\r";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO uint32_t uwRxIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

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

  /* Configure leds */
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /*-2- Configure UART peripheral for reception process (using LL) */  
  /* Any data received will be stored "aRxBuffer" buffer : the number max of 
     data received is RXBUFFERSIZE */
  /* Enable RXNE and Error interrupts */  
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);

  /*-3- Start the transmission process (using LL) **/  
  /* While the UART in reception process, user can transmit data from 
     "aTxStartMessage" buffer */
  /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
  LL_USART_TransmitData8(USART1, aTxStartMessage[uwTxIndex++]); 

  BSP_LED_On(BSP_LED1);
  
  /* Enable TXE interrupt */
  LL_USART_EnableIT_TXE(USART1); 

  /*-4- Wait for the end of the transfer */
  /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
     and is now based on LL API functions use. 
     Therefore, use of HAL IT based services is no more possible. */
  /*  Once TX transfer is completed, LED1 will turn On.
      Then, when RX transfer is completed, LED2 will turn On. */
  while (ubTxComplete == 0)
  {
  }
   
  while (ubRxComplete == 0)
  {
  }
  
  BSP_LED_Off(BSP_LED1);
  BSP_LED_On(BSP_LED2);


  /*-5- Send the received Buffer */
  /* Even if use of HAL IT based services is no more possible, use of HAL Polling based services
     (as Transmit in polling mode) is still possible. */
  if(HAL_UART_Transmit(&huart1, (uint8_t*)aRxBuffer, RXBUFFERSIZE, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
  
  /*-6- Send the End Message */  
  if(HAL_UART_Transmit(&huart1, (uint8_t*)aTxEndMessage, TXENDMESSAGESIZE, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief  Rx Transfer completed callback
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void UART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  aRxBuffer[uwRxIndex++] = LL_USART_ReceiveData8(USART1);

  /* Check if reception is completed (expected nb of bytes has been received) */
  if (uwRxIndex == RXBUFFERSIZE)
  {
    /* Set Reception complete boolean to 1 */
    ubRxComplete = 1;
  }
}

/**
  * @brief  Function called for achieving next TX Byte sending
  * @retval None
  */
void UART_TXEmpty_Callback(void)
{
  if(uwTxIndex == (ubSizeToSend - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART1);
  }

  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, aTxStartMessage[uwTxIndex++]);
}

/**
  * @brief  Function called at completion of last byte transmission
  * @retval None
  */
void UART_CharTransmitComplete_Callback(void)
{
  if(uwTxIndex == sizeof(aTxStartMessage))
  {
    uwTxIndex = 0;
    
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART1);
    
    /* Set Tx complete boolean to 1 */
    ubTxComplete = 1;
  }
}

/**
  * @brief  UART error callbacks
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void UART_Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* Turn LED3 on: Transfer error in reception/transmission process */
    BSP_LED_On(BSP_LED3);
  }
  else
  {
    /* Turn LED3 on: Transfer error in reception/transmission process */
    BSP_LED_On(BSP_LED3);
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Toggle LED3 for error */
  while(1)
  {
    BSP_LED_Toggle(BSP_LED3);
    HAL_Delay(1000);
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
