/**
  @page Example_HAL GPIO EXTI example
  
  @verbatim
  ******************************************************************************
  * @file    Template_HAL/Example_HAL/readme.txt 
  * @author  RF Application Team
  * @brief   Description of the GPIO EXTI example.
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

This project provides a reference template based on the BlueNRG_LP HAL API that can be used
to build any firmware application.

This project is targeted to run on BLUENRG_LP device on BlueNRG_LP-EVB board from STMicroelectronics. 

This project LL template provides:
 - Clock configuration (file: Templates_HAL_main.c)


@par Directory contents 

  - Template_HAL/Example_HAL/Inc/bluenrg_lp_hal_conf.h    HAL configuration file
  - Template_HAL/Example_HAL/Inc/bluenrg_lp_it.h          Interrupt handlers header file
  - Template_HAL/Example_HAL/Inc/Example_HAL_main.h       Header for Example_HAL_main.c module  
  - Template_HAL/Example_HAL/Src/bluenrg_lp_it.c          Interrupt handlers
  - Template_HAL/Example_HAL/Src/bluenrg_lp_hal_msp.c     HAL MSP file
  - Template_HAL/Example_HAL/Src/Example_HAL_main.c       Main program 

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
