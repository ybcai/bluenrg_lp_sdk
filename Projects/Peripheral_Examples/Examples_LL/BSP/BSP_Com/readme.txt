/**
  @page BSP_Com BSP example
  
  @verbatim
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    Examples_LL/BSP/BSP_Com/readme.txt 
  * @author  RF Application Team
  * @brief   Description of the BSP_Com example.
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

This example shows how to configure GPIO and USART peripherals to send characters 
asynchronously to an HyperTerminal (PC) in Polling mode. 
This example is based on BLUENRG_LP USART LL API. Peripheral initialization is 
done using LL unitary services functions for optimization purpose
(performance and size).


@par Directory contents 

  - BSP/BSP_Com/Inc/bluenrg_lp_it.h          Interrupt handlers header file
  - BSP/BSP_Com/Inc/BSP_Com_main.h           Header for BSP_Com_main.c module
  - BSP/BSP_Com/Inc/bluenrg_lp_assert.h      Template file to include assert_failed function
  - BSP/BSP_Com/Src/bluenrg_lp_it.c          Interrupt handlers
  - BSP/BSP_Com/Src/BSP_Com_main.c           Main program 


@par Hardware and Software environment

  - This example runs on BLUENRG_LP devices.
    
  - This example has been tested with BlueNRG_LP-EVB board and can be
    easily tailored to any other supported device and development board.
	
  - BlueNRG_LP-EVB Set-up
    Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
    - Connect BlueNRG_LP MCU board USART1 TX pin (GPIO PA.09) to PC COM port RX signal
    - Connect BlueNRG_LP MCU board USART1 RX pin (GPIO PA.08) to PC COM port TX signal
    - Connect BlueNRG_LP MCU board GND to PC COM port GND signal

  - Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
    - 115200 bauds
	- 8 bits data
	- 1 start bit
	- 1 stop bit
	- no parity
	- no HW flow control 


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
