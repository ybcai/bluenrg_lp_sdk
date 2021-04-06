/**
  @page ADC_BatterySensor ADC example

  @verbatim
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    Examples_LL/ADC/ADC_BatterySensor/readme.txt
  * @author  RF Application Team
  * @brief   Description of the ADC_BatterySensor example.
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

This example shows how to sample the supplied voltage using the ADC peripheral.
The data is sampled and then printed out by using the USART peripheral with a
rate of 1 sample each 100 ms.
This example is driven by polling the End of conversion from Down Sampler.
The overrun from Down Sampler event is also monitored.
This example is based on the BLUENRG_LP ADC LL API.
The peripheral initialization is done using LL unitary service functions
for optimization purposes (performance and size).

Example configuration:
The ADC is configured to sample the supplied voltage by using the Down Sampler
internal path of the ADC.
The USART baudrate is set to 115200 bps.

Example execution:
The user must just load and then run the application.
The user can get the output data through a serial terminal program.

Connection needed:
None.

Other peripherals used:
  1 USART for output the data acquired

@par Directory contents

  - ADC/ADC_BatterySensor/Inc/bluenrg_lp_it.h              Interrupt handlers header file
  - ADC/ADC_BatterySensor/Inc/bluenrg_lp_assert.h          Template file to include assert_failed function
  - ADC/ADC_BatterySensor/Src/bluenrg_lp_it.c              Interrupt handlers
  - ADC/ADC_BatterySensor/Src/ADC_BatterySensor_main.c  Main program


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
