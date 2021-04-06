/**
  @page Example_MIX DMA example

  @verbatim
  ******************** (C) COPYRIGHT 2018 STMicroelectronics *******************
  * @file    Examples_MIX/Template_MIX/Example_MIX/readme.txt
  * @author  RF Application Team
  * @brief   Description of the DMA example.
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


This project provides a reference template based on the BlueNRG_LP HAL and LL API that can be used
to build any firmware application.

This project is targeted to run on BLUENRG_LP device on BlueNRG_LP-EVB board from STMicroelectronics. 

This project MIX template provides:
 - Clock configuration 



@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note This example need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents

  - Template_MIX/Example_MIX/Src/system_bluenrg_lp.c      BLUENRG_LP system clock configuration file
  - Template_MIX/Example_MIX/Src/bluenrg_lp_it.c          Interrupt handlers
  - Template_MIX/Example_MIX/Src/Example_MIX_main.c                  Main program
  - Template_MIX/Example_MIX/Src/bluenrg_lp_hal_msp.c     HAL MSP module
  - Template_MIX/Example_MIX/Inc/bluenrg_lp_hal_conf.h    HAL Configuration file
  - Template_MIX/Example_MIX/Inc/bluenrg_lp_it.h          Interrupt handlers header file
  - Template_MIX/Example_MIX/Inc/Example_MIX_main.h                  Main program header file

@par Hardware and Software environment

  - This example runs on BLUENRG_LP Devices.

  - This example has been tested with STMicroelectronics BlueNRG_LP-EVB
    board and can be easily tailored to any other supported device
    and development board.

@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
