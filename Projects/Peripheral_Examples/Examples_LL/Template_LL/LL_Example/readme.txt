/**
  @page Templates_LL  Description of the Templates_LL example
  
  @verbatim
  ******************************************************************************
  * @file    Templates_LL/readme.txt 
  * @author  RF Application Team
  * @brief   Template to create a new LL example.
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

This project provides a reference template based on the BlueNRG_LP LL API that can be used
to build any firmware application.

This project is targeted to run on BLUENRG_LP device on BlueNRG_LP-EVB board from STMicroelectronics. 

This project LL template provides:
 - Clock configuration (file: Templates_LL_main.c)


@par Directory contents 

  - Templates_LL/Example_LL/Inc/bluenrg_lp_it.h          Interrupt handlers header file
  - Templates_LL/Example_LL/Inc/LL_Example_main.h        Header for Templates_LL_main.c module
  - Templates_LL/Example_LL/Inc/bluenrg_lp_assert.h      Template file to include assert_failed function
  - Templates_LL/Example_LL/Src/bluenrg_lp_it.c          Interrupt handlers
  - Templates_LL/Example_LL/Src/LL_Example_main.c        Main program 


@par Hardware and Software environment

  - This template runs on BLUENRG_LP devices.
    
  - This template has been tested with BlueNRG_LP-EVB and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
