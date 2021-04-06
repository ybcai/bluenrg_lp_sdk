/**
  @page UART_HyperTerminal_HwFlowControl_IT UART Hyperterminal IT example
  
  @verbatim
  ******************************************************************************
  * @file    UART/UART_HyperTerminal_HwFlowControl_IT/readme.txt 
  * @author  RF Application Team
  * @brief   Description of the UART Hyperterminal example.
  ******************************************************************************
  *
  * Copyright (c) 2018 STMicroelectronics. All rights reserved.
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                       opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  @endverbatim

@par Example Description

UART transmission (transmit/receive) in Interrupt with hardware flow
control mode between a board and an HyperTerminal PC application.

Board: BlueNRG_LP-EVB
  PA9   ------> USART1_TX  
  PA8   ------> USART1_RX  
  PB3   ------> USART1_nCTS  
  PB2   ------> USART1_nRTS 
   _________________________ 
  |           ______________|                       _______________
  |          |USART         |                      | HyperTerminal |
  |          |              |                      |               |
  |          |           TX |______________________|RX             |
  |          |         nCTS |______________________|nRTS           |
  |          |              |                      |               |
  |          |              |     RS232 Cable      |               |
  |          |              |                      |               |
  |          |           RX |______________________|TX             |  
  |          |         nRTS |______________________|nCTS           |        
  |          |              |                      |               |           
  |          |______________|                      |_______________|          
  |                         |                       
  |                         |                    
  |                         |                      
  |                         |                      
  |_BlueNRG_LP_Board________|                      

At the beginning of the main program the HAL_Init() function is called to reset
all the peripherals, initialize the Flash interface and the systick.

The UART peripheral configuration is ensured by the HAL_UART_Init() function.
This later is calling the HAL_UART_MspInit()function which core is implementing
the configuration of the needed UART resources according to the used hardware (CLOCK,
GPIO and NVIC). You may update this function to change UART configuration.

The UART/Hyperterminal communication is then initiated.
The HAL_UART_Receive_IT() and the HAL_UART_Transmit_IT() functions allow respectively
the reception of Data from Hyperterminal and the transmission of a predefined data
buffer  with hardware flow control.

The Asynchronous communication aspect of the UART is clearly highlighted as the
data buffers transmission/reception to/from Hyperterminal are done simultaneously.

For this example the TxBuffer (aTxStartMessage) is predefined and the RxBuffer (aRxBuffer)
size is limited to 10 data by the mean of the RXBUFFERSIZE define in the UART_HyperTerminal_HwFlowControl_IT_main.c file.

In a first step the received data will be stored in the RxBuffer buffer and the
TxBuffer buffer content will be displayed in the Hyperterminal interface.
In a second step the received data in the RxBuffer buffer will be sent back to
Hyperterminal and displayed.
The end of this two steps are monitored through the HAL_UART_GetState() function
result.

BlueNRG_LP-EVB board LEDs are used to monitor the transfer status:
	- LED2 turns ON if transmission/reception is complete and OK.
	- LED3 turns ON when when there is an error in transmission/reception process.

The UART is configured as follows:
	- BaudRate = 9600 baud
	- Word Length = 8 Bits ( 8bit data )
	- One Stop Bit
	- No parity
	- Hardware flow control enabled (RTS and CTS signals)
	- Reception and transmission are enabled in the time

@note USARTx/UARTx instance used and associated resources can be updated in "UART_HyperTerminal_HwFlowControl_IT_main.h"
file depending hardware configuration used.

@note When the parity is enabled, the computed parity is inserted at the MSB
position of the transmitted data.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - UART/UART_HyperTerminal_HwFlowControl_IT/Inc/bluenrg_lp_hal_conf.h                        HAL configuration file
  - UART/UART_HyperTerminal_HwFlowControl_IT/Inc/bluenrg_lp_it.h                              IT interrupt handlers header file
  - UART/UART_HyperTerminal_HwFlowControl_IT/Inc/UART_HyperTerminal_HwFlowControl_IT_main.h   Header for UART_HyperTerminal_HwFlowControl_IT_main.c module
  - UART/UART_HyperTerminal_HwFlowControl_IT/Src/bluenrg_lp_it.c                              IT interrupt handlers
  - UART/UART_HyperTerminal_HwFlowControl_IT/Src/UART_HyperTerminal_HwFlowControl_IT_main.c   Main program
  - UART/UART_HyperTerminal_HwFlowControl_IT/Src/bluenrg_lp_hal_msp.c                         HAL MSP module 


@par Hardware and Software environment

  - This example runs on BLUENRG_LP devices.
    
  - This example has been tested with STMicroelectronics BlueNRG_LP-EVB board and can be
    easily tailored to any other supported device and development board.    
      
  - BlueNRG_LP-EVB Set-up connection:
	  - USART1 TX (PA9) to RX pin of PC serial port 
	  - USART1 RX (PA9) to TX pin of PC serial port 
	  - USART1 nCTS (PB3) to nCTS pin of PC serial port 
	  - USART1 nRTS (PB2) to nRTS pin of PC serial port 


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
