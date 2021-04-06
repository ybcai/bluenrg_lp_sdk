
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_SerialPort_main.c
* Author             : RF Application Team
* Version            : 3.0.0
* Date               : 23-January-2020
* Description        : Code demonstrating the Bluetooth LE Serial Port application (previously named as Bluetooth LE Chat demo)
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_SerialPort_main.c
 * @brief This is a demo that shows how to implement a simple 2-way Bluetooth LE communication between two BlueNRG-LP devices emulating serial communication (previously named as Bluetooth LE Chat demo)
 * It also provides a reference example about how using the 
 * Bluetooth LE Over-The-Air (OTA) firmware upgrade capability with the Bluetooth LE Serial Port Demo.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SerialPort\\MDK-ARM\\{STEVAL-IDB011V1}\\BLE_SerialPort.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_SerialPort\\EWARM\\{STEVAL-IDB011V1}\\BLE_SerialPort.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Client - Client role configuration
- \c Server - Server role configuration
- \c Server_HigherApp_OTA - Server role configuration for Higher Application with OTA Service
- \c Server_LowerApp_OTA - Server role configuration for Lower Application with OTA Service
- \c Server_Use_OTA_ServiceManager - Server role configuration for Application using OTA Service Manager


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
|            |  Server_HigherApp_OTA  |   Server_Use_OTA_ServiceManager  |       Client       |   Server_LowerApp_OTA  |       Server       |
---------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |     STEVAL-IDB011V1    |          STEVAL-IDB011V1         |   STEVAL-IDB011V1  |     STEVAL-IDB011V1    |   STEVAL-IDB011V1  |
---------------------------------------------------------------------------------------------------------------------------------------------
|     A1     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A11    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A12    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A13    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A14    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A15    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A4     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A5     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A6     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A7     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A8     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     A9     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B0     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B14    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B2     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B3     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B4     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B5     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B7     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B8     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     B9     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     GND    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|     RST    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|    VBAT    |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |

@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
  The remote node will listen for RF messages and it will output them in the serial port.
  In other words everything typed in one node will be visible to the other node and viceversa.
@table
| Parameter name  | Value               | Unit      |
------------------------------------------------------
| Baudrate        | 115200 [default]    | bit/sec   |
| Data bits       | 8                   | bit       |
| Parity          | None                | bit       |
| Stop bits       | 1                   | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |            Server_HigherApp_OTA            |        Server_Use_OTA_ServiceManager       |       Client       |             Server_LowerApp_OTA            |       Server       |
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |               STEVAL-IDB011V1              |               STEVAL-IDB011V1              |   STEVAL-IDB011V1  |               STEVAL-IDB011V1              |   STEVAL-IDB011V1  |
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                  Not Used                  |                  Not Used                  |      Not Used      |                  Not Used                  |      Not Used      |
|     DL2    |                  Not Used                  |                  Not Used                  |      Not Used      |                  Not Used                  |      Not Used      |
|     DL3    |                  Not Used                  |                  Not Used                  |      Not Used      |                  Not Used                  |      Not Used      |
|     DL4    |                  Not Used                  |                  Not Used                  |      Not Used      |                  Not Used                  |      Not Used      |
|     U5     |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |      Not Used      |   ON when OTA firmware upgrade is ongoing  |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |  Server_HigherApp_OTA  |   Server_Use_OTA_ServiceManager  |       Client       |   Server_LowerApp_OTA  |       Server       |
-------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |     STEVAL-IDB011V1    |          STEVAL-IDB011V1         |   STEVAL-IDB011V1  |     STEVAL-IDB011V1    |   STEVAL-IDB011V1  |
-------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |        Not Used        |    Jump to OTA Service manager   |      Not Used      |        Not Used        |      Not Used      |
|      PUSH2     |        Not Used        |             Not Used             |      Not Used      |        Not Used        |      Not Used      |
|      RESET     |    Reset BlueNRG-LP    |         Reset BlueNRG-LP         |  Reset BlueNRG-LP  |    Reset BlueNRG-LP    |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This Serial Port demo has 2 roles:
 - The server that expose the Serial Port service. It is the slave.
 - The client that uses the Serial Port service. It is the master.

The Serial Port Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it will send notifications which will contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable caracteristic. When the client has data to be sent to the server, it will write a value into this characteristic.

The maximum length of the characteristic value is 20 bytes.

NOTES:
 - OTA service support for lower or higher application is enabled, respectively,  through CONFIG_OTA_LOWER=1 or CONFIG_OTA_HIGHER=1(preprocessor, linker) options and files: OTA_btl.[ch] (refer to Server_LowerApp_OTA and  Server_HigherApp_OTA IAR workspaces).
 - OTA service manager support is enabled, respectively,  through CONFIG_OTA_USE_SERVICE_MANAGER (preprocessor, linker) options and files: OTA_btl.[ch] (refer to Use_OTA_ServiceManager IAR workspace).    

**/
    
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP Serial Port demo \see BLE_SerialPort_main.c for documentation.
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
#include "app_state.h"
#include "serial_port.h"
#include "SerialPort_config.h"
#include "OTA_btl.h" 
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "clock.h"
#include "bleplat.h"
#include "nvm_db.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_SERIAL_PORT_VERSION_STRING "2.0.0" 


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

  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init Clock */
  Clock_Init();

  /* Configure I/O communication channel */
  BSP_COM_Init(Process_InputData);

  ModulesInit(); 

#if SERVER
  printf("BlueNRG-LP BLE Serial Port Server Application (version: %s)\r\n", BLE_SERIAL_PORT_VERSION_STRING);
#else
  printf("BlueNRG-LP BLE Serial Port Client Application (version: %s)\r\n", BLE_SERIAL_PORT_VERSION_STRING); 
#endif

  /* Init Serial port Device */
  ret = Serial_port_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Serial_port_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BLE Stack Initialized \n");
  
#if CONFIG_OTA_USE_SERVICE_MANAGER
  /* Initialize the button */
  BSP_PB_Init(USER_BUTTON, BUTTON_MODE_GPIO); 
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
   
  while(1) {
    
    ModulesTick();
    
    /* Application tick */
    APP_Tick();
    
#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
    /* Check if the OTA firmware upgrade session has been completed */
    if (OTA_Tick() == 1)
    {
      /* Jump to the new application */
      OTA_Jump_To_New_Application();
    }
#endif  /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

#if CONFIG_OTA_USE_SERVICE_MANAGER
    if (BSP_PB_GetState(USER_BUTTON) == SET) 
    {
      OTA_Jump_To_Service_Manager_Application();
    }
#endif /* CONFIG_OTA_USE_SERVICE_MANAGER */
  }
  
} /* end main() */


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

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
