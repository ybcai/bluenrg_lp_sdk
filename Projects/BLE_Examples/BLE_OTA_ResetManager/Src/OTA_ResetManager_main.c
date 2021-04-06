
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : OTA_ResetManager_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 18-March-2019
* Description        : Code demonstrating the Bluetooth LE OTA Reset Manager application
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  OTA_ResetManager_main.c
 * @brief This application implements the OTA Reset Manager which, at reset,  
 *        passes control to the latest valid Bluetooth LE application updated through the 
 *        Bluetooth LE Over-The-Air (OTA) Service.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ResetManager\\MDK-ARM\\{STEVAL-IDB011V1}\\BLE_OTA_ResetManager.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_OTA_ResetManager\\EWARM\\{STEVAL-IDB011V1}\\BLE_OTA_ResetManager.eww</tt>
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

  - The OTA Reset Manager is a basic application which is is stored at BlueNRG-LP FLASH base address (0x10040000) and 
    it allows to transfer of control towards the new upgraded application every time we reset.
  - The new application has to add the OTA service and related characteristics defined on 
    files OTA_btl.c.
  - At device reset, the reset manager will take care of jumping to the location of the last image 
    that was successfully loaded by the OTA bootloader.

NOTEs
  - Before downloading the OTA Reset Manager performs a device Mass Erase of the selected BlueNRG-LP device (use IAR, Project, Download, Erase Memory). Then, open the IAR project related to a Lower Application with OTA Service and download it on the selected device. At this stage, the BlueNRG-LP device is ready for performing OTA upgrades.
  - Refer to BLE_SerialPort and BLE_SensorDemo projects for related OTA update examples (Lower and Higher Applications with OTA service configurations).
  - On BlueNRG-LP, Bluetooth LE stack v3.0 or later, OTA FW upgrade supports the data length extended capability. User is requested to add the OTA_EXTENDED_PACKET_LEN=1 option and to use a Bluetooth LE stack configuration ioption supporting this capability (BLE_STACK_CONFIGURATION=BLE_OTA_BASIC_CONFIGURATION or BLE_STACK_CONFIGURATION=BLE_STACK_FULL_CONFIGURATION).

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
* BlueNRG-LP OTA Reset manager \see OTA_ResetManager_main.c for documentation.
*
*@{
*/

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
*/
/* Includes ------------------------------------------------------------------*/
#include "BlueNRG_LP.h"
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_api.h"
#include "OTA_btl.h"
#include "bluenrg_lp_ll_flash.h"

typedef  void (*pFunction)(void);

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05

/** @brief Get specific application tag value stored at vector table index 
* OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET
*/
#define TAG_VALUE(x)     (* ((volatile uint32_t*) ((x) + OTA_TAG_VECTOR_TABLE_ENTRY_OFFSET)))

/**
* @brief  It check if  flash storage area has to be erased or not
* @param  None.
* @retval Status: 1 (erase flash); 0 (don't erase flash).
*
* @note The API code could be subject to change in future releases.
*/
static uint8_t OTA_Check_Storage_Area(uint32_t start_address, uint32_t end_address)
{
  volatile uint32_t *address; 
  uint32_t i; 
  
  for(i=start_address;i<end_address; i = i +4)
  { 
    address = (volatile uint32_t *) i;
    if (*address != 0xFFFFFFFF)
      return 1; /* do flash erase */
  }
  
  return 0; /* no flash erase is required */
}

/**
* @brief  It erases the new flash storage area. 
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static void OTA_Erase_Storage_Area(uint16_t startPageNumber, uint16_t endPageNumber)
{
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, (startPageNumber), (endPageNumber-startPageNumber));
}

/**
* @brief  It defines the valid application address where to jump
*         by checking the OTA application validity tags for the lower and
*         higher applications
* @param  None.
* @retval None.
*
* @note The API code could be subject to change in future releases.
*/
static uint32_t OTA_Check_Application_Tags_Value(void)
{
  uint32_t appAddress = 0;
  if ( ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG))   || /* 1 */
      ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG))         || /* 2 */
        ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG))               || /* 4 */
          ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG))   || /* 8 */
            ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_VALID_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG)))           /* 9 */  
  {
    /* Jump to Lower Application */
    appAddress = APP_LOWER_ADDRESS;
    
    if (OTA_Check_Storage_Area(APP_HIGHER_ADDRESS,APP_HIGHER_ADDRESS_END))
    {
      /* Erase OLD Higher application storage area */
      OTA_Erase_Storage_Area(OTA_HIGHER_APPLICATION_PAGE_NUMBER_START, OTA_HIGHER_APPLICATION_PAGE_NUMBER_END); 
     
    }
  }
  else if ( ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_INVALID_OLD_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG))       || /* 3 */
           ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_INVALID_OLD_TAG)) || /* 6 */
             ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_VALID_TAG)))         /* 7 */     
  {
    /* Jump to Higher Application */
    appAddress = APP_HIGHER_ADDRESS;
    
    if (OTA_Check_Storage_Area(APP_LOWER_ADDRESS,APP_LOWER_ADDRESS_END))
    { 
      /* Erase OLD Lower application storage area */
      OTA_Erase_Storage_Area(OTA_LOWER_APPLICATION_PAGE_NUMBER_START, OTA_LOWER_APPLICATION_PAGE_NUMBER_END); 
    }
  }
  else if ((TAG_VALUE(APP_LOWER_ADDRESS) == OTA_IN_PROGRESS_TAG) && (TAG_VALUE(APP_HIGHER_ADDRESS) == OTA_IN_PROGRESS_TAG))   /* 5 */
  {
    /* 5: Is it possible? No. What to do?*/
  }
  return appAddress;
}

/**
* @brief  OTA Reset Manager main function
* @param  None.
* @retval None.
*
* @note The code could be subject to change in future releases.
*/
int main(void) 
{
  pFunction Jump_To_Application;
  uint32_t JumpAddress, appAddress;
  
  /* Identifies the valid application where to jump based on the OTA application validity tags values placed on
  reserved vector table entry: OTA_TAG_VECTOR_TABLE_ENTRY_INDEX */
  appAddress = OTA_Check_Application_Tags_Value();
  
  if (appAddress == 0) {
    /* This case indicates that no valid application is present and this normally should not happen */
    while (1);
  }
  
  /* Jump to user application */
  JumpAddress = *(__IO uint32_t*) (appAddress + 4);
  Jump_To_Application = (pFunction) JumpAddress;
  /* Initialize user application's Stack Pointer */
  __set_MSP(*(__IO uint32_t*) appAddress);
  Jump_To_Application();
  
  /* Infinite loop */
  while (1)
  {
  }
}
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
