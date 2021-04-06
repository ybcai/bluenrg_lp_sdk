
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BSP_Sensors_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the BSP functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  BSP_Sensors/BSP_Sensors_main.c
 * @brief This example shows how to configure the inertial sensor and the pressure sensor.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\BSP\\BSP_Sensors\\MDK-ARM\\{STEVAL-IDB011V1}\\BSP_Sensors.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\BSP\\BSP_Sensors\\EWARM\\{STEVAL-IDB011V1}\\BSP_Sensors.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


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
|     A12    |      Not Used      |
|     A13    |      Not Used      |
|     A14    |      Not Used      |
|     A15    |      Not Used      |
|     A4     |      Not Used      |
|     A5     |      Not Used      |
|     A6     |      Not Used      |
|     A7     |      Not Used      |
|     A8     |      Not Used      |
|     A9     |      Not Used      |
|     B0     |      Not Used      |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |      Not Used      |
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |

@endtable 


* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB011V1  |
-----------------------------------
|     DL1    |      Not Used      |
|     DL2    |      Not Used      |
|     DL3    |      Not Used      |
|     DL4    |      Not Used      |
|     U5     |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |
---------------------------------------
|      PUSH1     |      Not Used      |
|      PUSH2     |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This example shows how to configure the inertial sensor and the pressure sensor.
This example is based on BLUENRG_LP USART LL API. 
Peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).

Example execution:
Uncommenting the line of "#define TEST_PRESSURE_SENSOR" or "#define TEST_INERTIAL_SENSOR" the test is enabled.
The test reads data from the sensor and shows the information on the COM port.

**/
   

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_ll_adc.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_exti.h"
#include "bluenrg_lp_ll_cortex.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_dma.h"
#include "bluenrg_lp.h"
#include "bluenrg_lp_ll_gpio.h"
#if defined(USE_FULL_ASSERT)
#include "bluenrg_lp_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
#include "bluenrg_lp_evb_config.h"
#include "bluenrg_lp_evb_config.h"

#include "lsm6dsox_reg.h"
#include "lps22hh_reg.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
#define TEST_INERTIAL_SENSOR
#define TEST_PRESSURE_SENSOR

/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  uint8_t tmp = 0, rst;
  (void)tmp;
  
#ifdef TEST_INERTIAL_SENSOR
  lsm6dsox_ctx_t inertialHandle;
  
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis1bit16_t data_raw_temperature2;
  float acceleration_mg[3];
  float angular_rate_mdps[3];
  float temperature_degC2;
#endif
  
#ifdef TEST_PRESSURE_SENSOR
  lps22hh_ctx_t pressureHandle;
  
  axis1bit32_t data_raw_pressure;
  axis1bit16_t data_raw_temperature;
  float pressure_hPa;
  float temperature_degC;
  
  lps22hh_reg_t reg;
#endif
  
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* LEDs init */
  BSP_LED_Init(BSP_LED_GREEN);
  BSP_LED_Init(BSP_LED_RED);
  
  /* BSP COM Init */
  BSP_COM_Init(NULL);
  
#ifdef TEST_INERTIAL_SENSOR
  /* Initialize the handle of the LSM6DSO driver */
  inertialHandle.write_reg = BSP_SPI_Write;
  inertialHandle.read_reg = BSP_SPI_Read;
  
  /* Inizialize the SPI */
  BSP_SPI_Init();
  
  /* Restore default configuration */
  lsm6dsox_reset_set(&inertialHandle, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&inertialHandle, &rst);
  } while (rst);
  
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&inertialHandle, PROPERTY_ENABLE);
  
  /* Set full scale */  
  lsm6dsox_xl_full_scale_set(&inertialHandle, LSM6DSOX_2g);
  lsm6dsox_gy_full_scale_set(&inertialHandle, LSM6DSOX_2000dps);
  
  /* Set Output Data Rate for Acc and Gyro */
  lsm6dsox_xl_data_rate_set(&inertialHandle, LSM6DSOX_XL_ODR_12Hz5);
  lsm6dsox_gy_data_rate_set(&inertialHandle, LSM6DSOX_GY_ODR_12Hz5);
  
#endif
  
#ifdef TEST_PRESSURE_SENSOR
  /* Initialize the handle of the LPS22HH driver */
  pressureHandle.write_reg = BSP_I2C_Write;
  pressureHandle.read_reg = BSP_I2C_Read;
  
  /* Inizialize the SPI */
  BSP_I2C_Init();
  
  /* Restore default configuration */
  lps22hh_reset_set(&pressureHandle, PROPERTY_ENABLE);
  do {
    lps22hh_reset_get(&pressureHandle, &rst);
  } while (rst);
  
  /*  Enable Block Data Update */
  lps22hh_block_data_update_set(&pressureHandle, PROPERTY_ENABLE);
  
  /* Set Output Data Rate */
  lps22hh_data_rate_set(&pressureHandle, LPS22HH_1_Hz_LOW_NOISE);
  
  /* Init data_raw_pressure */
  data_raw_pressure.i32bit = 0;
#endif
    
  /* Infinite loop */
  while (1) {
  
#ifdef TEST_PRESSURE_SENSOR
      /* Read output only if new value is available */
      lps22hh_read_reg(&pressureHandle, LPS22HH_STATUS, (uint8_t *)&reg, 1);
      if (reg.status.p_da) {
        lps22hh_pressure_raw_get(&pressureHandle, data_raw_pressure.u8bit);
        pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure.i32bit);
        printf("pressure [hPa]:\t%6.2f\r\n", pressure_hPa);
      }
      if (reg.status.t_da) {
        lps22hh_temperature_raw_get(&pressureHandle, data_raw_temperature.u8bit);
        temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);
        
        printf("temperature [degC]:\t%6.2f\r\n", temperature_degC);
      }
#endif
    
#ifdef TEST_INERTIAL_SENSOR
      /* Read output only if new value is available */
      lsm6dsox_xl_flag_data_ready_get(&inertialHandle, &tmp);
      if(tmp){
        /* Read acceleration field data */
        lsm6dsox_acceleration_raw_get(&inertialHandle, data_raw_acceleration.u8bit);
        acceleration_mg[0] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] = lsm6dsox_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
        
        printf("Acceleration [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      }
      
      lsm6dsox_gy_flag_data_ready_get(&inertialHandle, &tmp);
      if(tmp) {
        /* Read angular rate field data */
        lsm6dsox_angular_rate_raw_get(&inertialHandle, data_raw_angular_rate.u8bit);
        angular_rate_mdps[0] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = lsm6dsox_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
        
        printf("Angular rate [mdps]:\t%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      }
      
      lsm6dsox_temp_flag_data_ready_get(&inertialHandle, &tmp);
      if(tmp) { 
        /* Read temperature data */
        lsm6dsox_temperature_raw_get(&inertialHandle, data_raw_temperature2.u8bit);
        temperature_degC2 = lsm6dsox_from_lsb_to_celsius(data_raw_temperature2.i16bit);
        
        printf("Temperature [degC]:\t%6.2f\r\n", temperature_degC2);
      }
#endif
  }
}


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/


#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
