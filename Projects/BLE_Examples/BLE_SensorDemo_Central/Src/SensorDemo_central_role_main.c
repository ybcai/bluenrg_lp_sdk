
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : SensorDemo_central_role_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 19-March-2019
* Description        : Code which emulates ST Bluetooth LE Sensor Demo app available for smartphones (iOS and android)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file SensorDemo_central_role_main.c
 * @brief This application implements a basic version of the BlueNRG-LP Sensor Profile Central role which
 * emulates the ST BLE Sensor Demo app available for smartphones (iOS and android).
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SensorDemo_Central\\MDK-ARM\\{STEVAL-IDB011V1}\\BLE_SensorDemo_Central.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SensorDemo_Central\\EWARM\\{STEVAL-IDB011V1}\\BLE_SensorDemo_Central.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG Flasher utility and download the built binary image.

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


This application implements a basic version of the BlueNRG-LP Sensor Profile Central role which emulates the BlueNRG-LP Sensor Demo applications available for smartphones (iOS and android).

It configures a BlueNRG-LP device as a Sensor device, Central role which is able to find, connect and properly configure the free fall, acceleration and environment sensors characteristics provided by a BlueNRG-LP development platform  configured as a  Sensor device, Peripheral role. 

This application uses a new set of APIs allowing to perform the following operations on a BlueNRG-LP Master/Central device:

 - Master Configuration Functions
 - Master Device Discovery Functions
 - Master Device Connection Functions
 - Master Discovery Services, Characteristics Functions
 - Master Data Exchange Functions
 - Master Security Functions
 - Master Common Services Functions

These APIs are provided through a library  and they are fully documented on available doxygen documentation.

The following binary library is provided on Library\\BLE_Application\\Profile_Central\\library folder:
 - <b>libmaster_library_bluenrg1.a</b> for IAR, Keil and Atollic toolchains

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP SensorDemo Central \see SensorDemo_central_role_main.c for documentation.
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
#include "bluenrg_lp_hal_power_manager.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "gatt_db.h"
#include "master_basic_profile.h"
#include "master_config.h" 
#include "SensorDemo_config.h"
#include "bleplat.h"
#include "nvm_db.h"

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private variables ---------------------------------------------------------*/

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

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{    

  uint32_t timeTemperature = 0, currentTime;
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
  
  /* Master Init procedure */
  if (deviceInit() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the device init procedure\r\n");
  }
  /* Start Master Configure Scanning procedure */
  if (deviceScanConfiguration() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the Scanning procedure configuration\r\n");
  }  
  
  /* Start Master Configure Connection procedure */
  if (deviceConnectionConfiguration() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the Connection procedure configuration\r\n");
  } 
  
  /* Start Master Device Discovery procedure */
  if (deviceDiscovery() != BLE_STATUS_SUCCESS) {
    PRINTF("Error during the device discovery procedure\r\n");
  }

  PRINTF("Sensor Demo Central application\r\n");

  /* No Wakeup Source needed */
  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = 0;
  wakeupIO.RTC_enable = 0;

  /* Main loop */
  while(1) {
    
    ModulesTick();

    /* Master Tick */
    Master_Process(); 

    /* Power Save Request */
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel); 
      
    /* Start the connection procedure */
    if (masterContext.startDeviceDisc) {
      deviceConnection();
      masterContext.startDeviceDisc = FALSE;
   }
      
    /* Discovery characteristics of service to enable the notifications/indications */
    if (masterContext.findCharacOfService) {
      findCharcOfService();
      masterContext.findCharacOfService = FALSE;
      timeTemperature = HAL_VTIMER_GetCurrentSysTime();
    }

    /* Enable the notifications/indications */
    if (masterContext.enableNotif) {
      enableSensorNotifications();
      masterContext.enableNotif = FALSE;
      
      timeTemperature = HAL_VTIMER_GetCurrentSysTime();
    }
    /* Read the temperature sensor every 3 sec */
    currentTime = HAL_VTIMER_GetCurrentSysTime();  
    if ((HAL_VTIMER_DiffSysTimeMs(currentTime, timeTemperature)) >= READ_TEMPERATURE_TIMEOUT) {
      timeTemperature = HAL_VTIMER_GetCurrentSysTime();
      if (masterContext.connHandle != 0xFFFF)
      {
	readTemperature(); 
      }
    }
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
