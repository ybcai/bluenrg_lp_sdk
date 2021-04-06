
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : SPI_FullDuplex_IT_Slave_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the SPI functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  SPI_FullDuplex_IT_Slave/SPI_FullDuplex_IT_Slave_main.c
 * @brief Data buffer transmission/reception between two boards via SPI using Interrupt mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\SPI\\SPI_FullDuplex_IT_Slave\\MDK-ARM\\{STEVAL-IDB011V1}\\SPI_FullDuplex_IT_Slave.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_HAL\\SPI\\SPI_FullDuplex_IT_Slave\\EWARM\\{STEVAL-IDB011V1}\\SPI_FullDuplex_IT_Slave.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c SPI_FullDuplex_IT_Slave - Release configuration


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
|     A12    |      SPI2 MOSI     |
|     A13    |      Not Used      |
|     A14    |      Not Used      |
|     A15    |      Not Used      |
|     A4     |      Not Used      |
|     A5     |      SPI2 SCK      |
|     A6     |      Not Used      |
|     A7     |      SPI2 MISO     |
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
|  LED name  |            STEVAL-IDB011V1           |
-----------------------------------------------------
|     DL1    |               Not Used               |
|     DL2    |   ON: reception process is complete  |
|     DL3    |               ON: error              |
|     DL4    |               Not Used               |
|     U5     |   blinking: due to SPI2 MOSI signal  |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |       STEVAL-IDB011V1      |
-----------------------------------------------
|      PUSH1     |          Not Used          |
|      PUSH2     |          Not Used          |
|      RESET     |      Reset BlueNRG-LP      |

@endtable

* \section Usage Usage

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The SPI peripheral configuration is ensured by the HAL_SPI_Init() function.
This later is calling the HAL_SPI_MspInit()function which core is implementing the configuration of the needed SPI resources according to the used hardware (CLOCK, GPIO and NVIC). 
You may update this function to change SPI configuration.

The SPI communication is then initiated.
The HAL_SPI_TransmitReceive_IT() function allows the reception and the transmission of a predefined data buffer at the same time (Full Duplex Mode).
If the Master board is used, the project SPI_FullDuplex_IT_Master must be used.
If the Slave board is used, the project SPI_FullDuplex_IT_Slave must be used.

For this example the aTxBuffer is predefined and the aRxBuffer size is same as aTxBuffer.

In a first step after the user press the User push-button (PUSH1), SPI Master starts the communication by sending aTxBuffer and receiving aRxBuffer through  HAL_SPI_TransmitReceive_IT(), at the same time SPI Slave transmits aTxBuffer and receives aRxBuffer through HAL_SPI_TransmitReceive_IT(). 
The callback functions (HAL_SPI_TxRxCpltCallback and HAL_SPI_ErrorCallbackand) update the variable wTransferState used in the main function to check the transfer status.
Finally, aRxBuffer and aTxBuffer are compared through Buffercmp() in order to check buffers correctness.  

BlueNRG_LP board's LEDs can be used to monitor the transfer status:
 - LED2 turns ON when the reception process is complete.
 - LED3 turns ON when there is an error in transmission/reception process.  

You need to perform a reset on Slave board, then perform it on Master board to have the correct behavior of this example.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button
 
BlueNRG_LP-EVB Set-up
- Connect Master board PA13 to Slave Board PA5
- Connect Master board PA14 to Slave Board PA7
- Connect Master board PA15 to Slave Board PA12
- Connect Master board GND  to Slave Board GND
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
#include "SPI_FullDuplex_IT_Slave_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
enum {
  TRANSFER_WAIT,
  TRANSFER_COMPLETE,
  TRANSFER_ERROR
};

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspiSlave;

#if defined( CONFIG_DATASIZE_16BIT )

/* Buffer used for transmission */
uint16_t aTxBuffer[] = {0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF,0xAAAA,0xFFFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer)/2;
/* Buffer used for reception */
uint16_t aRxBuffer[sizeof(aTxBuffer)/2];

#elif defined( CONFIG_DATASIZE_8BIT ) 

/* Buffer used for transmission */
uint8_t aTxBuffer[] = {0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF};
uint8_t ubNbDataToTransmit = sizeof(aTxBuffer);
/* Buffer used for reception */
uint8_t aRxBuffer[sizeof(aTxBuffer)];

#endif

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;

/* Private function prototypes -----------------------------------------------*/

static void MX_GPIO_Init(void);
static void MX_SPI_SLAVE_Init(void);

/* Private function prototypes -----------------------------------------------*/
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);

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
  
  /* BSP COM Init */
  /* needed to use printf over the USART */
  BSP_COM_Init(NULL);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI_SLAVE_Init();
  
  /* Configure LEDs */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  printf("Slave board\n\r");
  
#if defined( CONFIG_DATASIZE_16BIT ) 
  printf("SPI data width 16 bit\n\r");
#elif defined( CONFIG_DATASIZE_8BIT )
  printf("SPI data width 8 bit\n\r");
#endif 
  
  /*-1- Start the Full Duplex Communication process */  
  printf("Start the Full Duplex Communication process\n\r");  
  /* While the SPI in TransmitReceive process, user can transmit data through 
  "aTxBuffer" buffer & receive data through "aRxBuffer" */
  if(HAL_SPI_TransmitReceive_IT(&hspiSlave, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, ubNbDataToTransmit) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
  
  /*-2- Wait for the end of the transfer */  
  /*  Before starting a new communication transfer, you must wait the callback call 
  to get the transfer complete confirmation or an error detection.
  For simplicity reasons, this example is just waiting till the end of the 
  transfer, but application may perform other tasks while transfer operation
  is ongoing. */  
  while (wTransferState == TRANSFER_WAIT)
  {
  }
  switch(wTransferState)
  {
  case TRANSFER_COMPLETE :
    /*-3- Compare the sent and received buffers */
    printf("Compare the sent and received buffers:\n\r");
    if(Buffercmp((uint8_t*)aTxBuffer, (uint8_t*)aRxBuffer, sizeof(aTxBuffer)))
    {
      /* Processing Error */
      printf("Error\n\r");
      Error_Handler();     
    }
    else
    {
      printf("OK\n\r");
    }
    break;
  default : 
    Error_Handler();
    break;
  }
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief SPI_SLAVE Initialization Function
* @param None
* @retval None
*/
static void MX_SPI_SLAVE_Init(void)
{
  /* SPI_SLAVE parameter configuration*/
  hspiSlave.Instance = SPI_SLAVE;
  hspiSlave.Init.Mode = SPI_MODE_SLAVE;
  hspiSlave.Init.Direction = SPI_DIRECTION_2LINES;
#ifdef CONFIG_DATASIZE_16BIT 
  hspiSlave.Init.DataSize = SPI_DATASIZE_16BIT;
#else // CONFIG_DATASIZE_8BIT
  hspiSlave.Init.DataSize = SPI_DATASIZE_8BIT;
#endif 
  hspiSlave.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspiSlave.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspiSlave.Init.NSS = SPI_NSS_SOFT;
  hspiSlave.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspiSlave.Init.TIMode = SPI_TIMODE_DISABLE;
  hspiSlave.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspiSlave.Init.CRCPolynomial = 7;
  hspiSlave.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspiSlave.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspiSlave) != HAL_OK)
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
}

/**
* @brief  TxRx Transfer completed callback.
* @param  hspi: SPI handle
* @note   This example shows a simple way to report end of Interrupt TxRx transfer, and 
*         you can add your own implementation. 
* @retval None
*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED2 on: Transfer in transmission process is complete */
  printf("Transfer in transmission process is complete\n\r");
  /* Turn LED2 on: Transfer in reception process is complete */
  printf("Transfer in reception process is complete\n\r");
  BSP_LED_On(BSP_LED2);
  wTransferState = TRANSFER_COMPLETE;
}

/**
* @brief  SPI error callbacks.
* @param  hspi: SPI handle
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}

/**
* @brief  Compares two buffers.
* @param  pBuffer1, pBuffer2: buffers to be compared.
* @param  BufferLength: buffer's length
* @retval 0  : pBuffer1 identical to pBuffer2
*         >0 : pBuffer1 differs from pBuffer2
*/
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  /* Turn LED3 on */
  BSP_LED_On(BSP_LED3);
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
