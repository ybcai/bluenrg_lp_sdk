/**
  @page I2S_CCA01M1_ComIT SPI Full Duplex IT example
  
  @verbatim
  ******************************************************************************
  * @file    I2S/I2S_CCA01M1_ComIT/readme.txt 
  * @author  RF Application Team
  * @brief   Description of the SPI Full Duplex IT example.
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
  
The application is designed to play a pre-recorded audio frame, saved in the MCU 
flash memory, by means of the STA350BW device soldered on the X-NUCLEO-CCA01M1 
expansion board. The application runs on BlueNRG_LP.

The application demonstrates both basic audio functionalities, 
like initialization/play/mute/unMute/pause/resume controls. 
It is possible to switch between several setup using the user button PUSH1 of the 
BlueNRG_LP.

In order to achieve these results, the application performs the following steps:
- Initialize communication peripheral, such as I2C and I2S buses
- initialize STA350BW audio device by means of I2C bus
- Start the playing of the audio frame from the BlueNRG_LP flash memory

In the firmware, audio-related parts are collected in the audio_application.c 
file, that makes use of the dedicated X-NUCLEO-CCA01M1 BSP layer. 

I2C2 configuration :
Timing = Fast-mode kHz, PRESC | 0h | SCLDEL | SDADEL | SCLH | SCLL 
                          1h  | 0h |    3h  |   2h   |  03h |  09h 
Own Address = 0x33
Addressing Mode = 7 bit
Dual Address Mode = DISABLE
Own Address 2 = 0;
General Call Mode = DISABLE
No Stretch Mode = DISABLE  

Audio Out configuration :
AudioFreq    = 32000
CPOL         = CPOL_LOW
DataFormat   = 16 bit
MCLKOutput   = enable
Mode         = MASTER TX
Standard     = PHILIPS

@note You need to perform a reset on Slave board, then perform it on Master board
      to have the correct behaviour of this example.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - I2S/I2S_CCA01M1_ComIT/Inc/bluenrg_lp_hal_conf.h           HAL configuration file
  - I2S/I2S_CCA01M1_ComIT/Inc/bluenrg_lp_it.h                 Interrupt handlers header file
  - I2S/I2S_CCA01M1_ComIT/Inc/I2S_CCA01M1_ComIT_main.h        Header for I2S_CCA01M1_ComIT_main.c module  
  - I2S/I2S_CCA01M1_ComIT/Inc/audio.h                         X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/audio_application.h             Application header file
  - I2S/I2S_CCA01M1_ComIT/Inc/BiquadCalculator.h              X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/BiquadPresets.h                 X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/cca01m1_audio.h                 X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/cca01m1_audio_ex.h              X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/cca01m1_conf.h                  X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/Fragment1.h                     X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/nucleo_errno.h                  X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/sta350bw.h                      X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Inc/sta350bw_reg.h                  X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/bluenrg_lp_it.c                 Interrupt handlers
  - I2S/I2S_CCA01M1_ComIT/Src/I2S_CCA01M1_ComIT_main.c        Main program
  - I2S/I2S_CCA01M1_ComIT/Src/bluenrg_lp_hal_msp.c            HAL MSP file
  - I2S/I2S_CCA01M1_ComIT/Src/audio_application.c             Audio application file
  - I2S/I2S_CCA01M1_ComIT/Src/BiquadCalculator.c              X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/BiquadPresets.c                 X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/cca01m1_audio.c                 X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/cca01m1_audio_ex.c              X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/sta350bw.c                      X-Nucleo CCA01M1 project file
  - I2S/I2S_CCA01M1_ComIT/Src/sta350bw_reg.c                  X-Nucleo CCA01M1 project file

@par Hardware and Software environment

  - This example runs on BLUENRG_LP devices.

  - This example has been tested with BlueNRG_LP-EVB board and can be
    easily tailored to any other supported device and development board.

  - BlueNRG_LP-EVB Set-up with X-Nucleo-CCA01M1:
      I2C2_SCL:     PA13 -> CN10/3 
      I2C2_SDA:     PA14 -> CN10/5 
      OUT1_RST_PIN: PA11 -> CN10/33 
      I2S_WS:       PA4  -> CN10/16
      I2S_CK:       PA5  -> CN10/30 
      I2S_SD:       PA6  -> CN10/26 
      I2S_MCK:      PB9  -> CN10/4 
      3.3V          VBAT -> CN6/4
      GND                -> CN6/6
  - X-Nucleo-CCA01M1 Set-up with external voltage generator:    
      5V  ext gen        -> VS signal, pin 1 on the CN2 
      GND ext gen        -> ground connection on the CN2.2 

@par How to use it ? 

In order to make the program work, you must do the following:
 - Open your preferred toolchain
 - Rebuild all files (master project) and load your image into target memory
    o Load the project in Master Board
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 