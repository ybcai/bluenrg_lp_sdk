
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : SensorDemo_BlueMS_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 20-March-2019
* Description        : Sensor Demo application for interacting with ST BLE Sensor app
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file SensorDemo_BlueMS_main.c
 * @brief This application contains an example which shows how implementing the Sensor Demo application
 * tailored for interacting with the ST BLE Sensor smartphone app (previously known as ST BlueMS).
 * The device sends periodically, to the ST BLE Sensor smartphone app, the data collected from the accelerometer sensor and environmental sensors: pressure and temperature sensor.
 * The usage is similar to the firmware example BLE_Examples/BLE_SensorDemo.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\MDK-ARM\\{STEVAL-IDB011V1}\\BLE_SensorDemo_BlueMSapp.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\EWARM\\{STEVAL-IDB011V1}\\BLE_SensorDemo_BlueMSapp.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c HigherApp_OTA - Release configuration for Higher Application with OTA Service
- \c LowerApp_OTA - Release configuration for Lower Application with OTA Service
- \c Release - Release configuration
- \c Use_OTA_ServiceManager - Release configuration for Application using OTA Service Manager


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
|            |       Release      |    HigherApp_OTA   |  Use_OTA_ServiceManager  |    LowerApp_OTA    |
--------------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V1  |      STEVAL-IDB011V1     |   STEVAL-IDB011V1  |
--------------------------------------------------------------------------------------------------------
|     A1     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A11    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A12    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A13    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A14    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A15    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A4     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A5     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A6     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A7     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A8     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     A9     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B0     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B14    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B2     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B3     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B4     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B5     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B7     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B8     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     B9     |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     GND    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|     RST    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |         Not Used         |      Not Used      |

@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |       Release      |                     HigherApp_OTA                    |                Use_OTA_ServiceManager                |                     LowerApp_OTA                     |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |                    STEVAL-IDB011V1                   |                    STEVAL-IDB011V1                   |                    STEVAL-IDB011V1                   |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |                       Not Used                       |                       Not Used                       |                       Not Used                       |
|     DL2    |      Not Used      |                       Not Used                       |                       Not Used                       |                       Not Used                       |
|     DL3    |      Error led     |   ON when OTA firmware upgrade is ongoing/Error led  |   ON when OTA firmware upgrade is ongoing/Error led  |   ON when OTA firmware upgrade is ongoing/Error led  |
|     DL4    |      Not Used      |                       Not Used                       |                       Not Used                       |                       Not Used                       |
|     U5     |    Activity led    |                     Activity led                     |                     Activity led                     |                     Activity led                     |

@endtable


* \section Buttons_description Buttons description
@table
|                |       Release      |    HigherApp_OTA   |     Use_OTA_ServiceManager     |    LowerApp_OTA    |
------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V1  |         STEVAL-IDB011V1        |   STEVAL-IDB011V1  |
------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |   Jump to OTA Service manager  |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |            Not Used            |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |        Reset BlueNRG-LP        |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This is a demonstration example of the Bluetooth LE Sensor Demo application version tailored for interacting with the ST BLE Sensor smarthphone application (previously known as ST BlueMS).
This  demo application allows to configure a BlueNRG-LP kit platform with a demo application able to interact with the ST BLE Sensor application and provide it a set of sensor data, which user can log to different cloud providers.
Emulated sensors values configurations for accelerometer sensor and/or temperature pressure sensors are also supported.

NOTEs:
     - In order to enable the sensor emulation configuration two possible preprocessor options are available: SENSOR_ACCELEROMETER_EMULATION (default configuration) for using emulated values for acceleration, SENSOR_PRESSURE_TEMPERATURE_EMULATION for using emulated values for environmental sensors.
     - OTA service support for lower or higher application is enabled, respectively,  through CONFIG_OTA_LOWER=1 or CONFIG_OTA_HIGHER=1(preprocessor, linker) options and files: OTA_btl.[ch] (refer to LowerApp_OTA and HigherApp_OTA IAR workspaces).
     - OTA service manager support is enabled, respectively, through CONFIG_OTA_USE_SERVICE_MANAGER(preprocessor, linker) options and files: OTA_btl.[ch] (refer to Use_OTA_ServiceManager IAR workspace).

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP SensorDemo with App \see SensorDemo_BlueMS_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bluenrg_lp_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "OTA_btl.h"
#include "bluenrg_lp_hal_power_manager.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "sensor.h"
#include "SensorDemo_config.h"
#include "OTA_btl.h"  
#include "gatt_db.h"
#include "bleplat.h"
#include "nvm_db.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BLE_SENSOR_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void ModulesInit(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
    while(1);
  }
  
  HAL_VTIMER_InitType VTIMER_InitStruct = {HS_STARTUP_TIME, INITIAL_CALIBRATION, CALIBRATION_INTERVAL};
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  
  BLEPLAT_Init();  
}

void ModulesTick(void)
{
  /* Timer tick */
  HAL_VTIMER_Tick();
  
  /* Bluetooth stack tick */
  BLE_STACK_Tick();
  
  /* NVM manager tick */
  NVMDB_Tick();
}

int main(void) 
{
  uint8_t ret;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();

  /* Configure I/O communication channel */
  BSP_COM_Init(BSP_COM_RxDataUserCb);

  ModulesInit();
  
  /* Application demo Led Init */
  BSP_LED_Init(BSP_LED1); //Activity led
  BSP_LED_Init(BSP_LED3); //Error led
  BSP_LED_On(BSP_LED1);
  BSP_LED_Off(BSP_LED3);
  
  PRINTF("BlueNRG-LP BLE Sensor Demo Application (version: %s)\r\n", BLE_SENSOR_VERSION_STRING); 
  
#if CONFIG_OTA_USE_SERVICE_MANAGER
  /* Initialize the button: to be done before Sensor_DeviceInit for avoiding to 
     overwrite pressure/temperature sensor IO configuration when using BUTTON_2 (IO5) */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_GPIO);
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  
  
  /* Sensor Device Init */
  ret = Sensor_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    BSP_LED_On(BSP_LED3);
    while(1);
  }

  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;

  while(1)
  {
    ModulesTick();
     
    /* Application Tick */
    APP_Tick();

    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

#if CONFIG_OTA_USE_SERVICE_MANAGER
    if (BSP_PB_GetState(USER_BUTTON) == SET)
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  }   
}

/****************** BlueNRG-LP Power Management Callback ********************************/

PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/* Hardware Error event. 
   This event is used to notify the Host that a hardware failure has occurred in the Controller. 
   Hardware_Code Values:
   - 0x01: Radio state error
   - 0x02: Timer overrun error
   - 0x03: Internal queue overflow error
   - 0x04: Late Radio ISR
   After this event with error code 0x01, 0x02 or 0x03, it is recommended to force a device reset. */

void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}


/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}

/***************************************************************************************/

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
