/**
  @page PDM_DigitalMicrophone PDM example

  @verbatim
  ******************** (C) COPYRIGHT 2020 STMicroelectronics *******************
  * @file    Examples_LL/PDM/ADC_DigitalMicrophone/readme.txt
  * @author  RF Application Team
  * @brief   Description of the PDM_DigitalMicrophone example. .TO BE USED only
  *          with BlueNRG-LP cut 2.0.
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

This example shows how to use the PDM and ADC peripheral to interface a digital
microphone. The DMA peripheral is used to transfer the data from the
ADC output register to the RAM memory. Then, the data are printed out by using
the USART peripheral.
This example is driven by interrupts from the DMA peripheral: Half Transfer and
Transfer Complete events.
The peripheral initialization is done using LL unitary service functions
for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample the output data coming from a digital microphone
connected to the Decimation Filter of the ADC peripheral.
The user can change the Decimation Filter setting and the output data rate.
The USART baudrate is set to 460800 bps for maintain high performance.

Example execution:
The user must connect the digital microphone according to the application
circuit suggested for the specific digital microphone. The input port is the
pin PB2. The BlueNRG-LP provides also a clock for the digital microphone from the
pin PB1. The specific clock frequency can be set by user.
The user can get the output data by logging the data through a serial terminal
and then process the audio stream acquired.

Connection needed:
PB2 must be connected to the output of the digital microphone.
PB1 must be connected to the input port to the digital microphone.

Other peripherals used:
  1 USART for output the data acquired
  1 DMA for transfer the output data from the ADC to the RAM memory

@par Directory contents

  - PDM/PDM_DigitalMicrophone/Inc/bluenrg_lp_it.h              Interrupt handlers header file
  - PDM/PDM_DigitalMicrophone/Inc/bluenrg_lp_assert.h          Template file to include assert_failed function
  - PDM/PDM_DigitalMicrophone/Src/bluenrg_lp_it.c              Interrupt handlers
  - PDM/PDM_DigitalMicrophone/Src/PDM_DigitalMicrophone_main.c  Main program


@par Hardware and Software environment

  - This example runs on BLUENRG_LP devices.

  - This example has been tested with STEVAL-IDB011V1 board and can be
    easily tailored to any other supported device and development board.


@par How to use it ?

In order to make the program work, you must do the following :
 - Open IAR/KEIL toolchain
 - Rebuild all files and load your image into target memory
 - Run the example as in the description

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
