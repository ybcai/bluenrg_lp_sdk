/**
  @page BSP_LedButton ADC example
  
  @verbatim
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    Examples_LL/BSP/BSP_TemperatureSensor_Init/readme.txt 
  * @author  RF Application Team
  * @brief   Description of the BSP_LedButton example.
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


This example describes how to configure the EXTI and use
GPIOs to toggle the user LEDs available on the board when
a user button is pressed. This example is based on the
BLUENRG_LP LL API. Peripheral initialization is done using LL
initialization function to demonstrate BSP init usage.

PUSH1 uses the polling programming model to toggle a led connected to a specific GPIO pin.

In this example, PUSH2 is configured to generate an interrupt on each rising edge,
in the interrupt routine a led connected to a specific GPIO pin is toggled.



@par Directory contents 

  - BSP/BSP_LedButton/Inc/bluenrg_lp_it.h          Interrupt handlers header file
  - BSP/BSP_LedButton/Inc/BSP_LedButton_main.h     Header for BSP_LedButton_main.c module
  - BSP/BSP_LedButton/Inc/bluenrg_lp_assert.h      Template file to include assert_failed function
  - BSP/BSP_LedButton/Src/bluenrg_lp_it.c          Interrupt handlers
  - BSP/BSP_LedButton/Src/BSP_LedButton_main.c     Main program  


@par Hardware and Software environment

  - This example runs on BLUENRG_LP devices.
    
  - This example has been tested with BlueNRG_LP-EVB board and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
