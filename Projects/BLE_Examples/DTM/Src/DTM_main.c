
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : DTM_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 19-March-2019
* Description        : DTM application which configures a BlueNRG-LP device as a network coprocessor (UART conficuration) in order to be used with the BlueNRG GUI or other instruments as CBT
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file DTM_main.c
 * @brief This application configures a BlueNRG-LP device as a network coprocessor (UART) in order to be used with the BlueNRG GUI or other instruments as CBT. Full stack  modular configuration option is used.
 * 

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\DTM\\MDK-ARM\\{STEVAL-IDB011V1}\\DTM.uvprojx</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\DTM\\EWARM\\{STEVAL-IDB011V1}\\DTM.eww</tt> 
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c SPI - Network coprocessor configuration: SPI mode (no updater)
- \c SPI_WITH_UPDATER - Network coprocessor configuration: SPI mode (with updater)
- \c UART - Network coprocessor configuration: UART mode (no updater)
- \c UART_WITH_UPDATER - Network coprocessor configuration: UART mode (with updater)


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
|            |        UART        |         SPI        |   UART_WITH_UPDATER  |  SPI_WITH_UPDATER  |
----------------------------------------------------------------------------------------------------
|  PIN name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V1  |    STEVAL-IDB011V1   |   STEVAL-IDB011V1  |
----------------------------------------------------------------------------------------------------
|     A1     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A11    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A12    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A13    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A14    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A15    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A4     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A5     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A6     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A7     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A8     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     A9     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B0     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B14    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B2     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B3     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B4     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B5     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B7     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B8     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     B9     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     GND    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     RST    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|    VBAT    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |

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
|            |        UART        |         SPI        |   UART_WITH_UPDATER  |  SPI_WITH_UPDATER  |
----------------------------------------------------------------------------------------------------
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V1  |    STEVAL-IDB011V1   |   STEVAL-IDB011V1  |
----------------------------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|     U5     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |        UART        |         SPI        |   UART_WITH_UPDATER  |  SPI_WITH_UPDATER  |
--------------------------------------------------------------------------------------------------------
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V1  |    STEVAL-IDB011V1   |   STEVAL-IDB011V1  |
--------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |       Not Used       |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |   Reset BlueNRG-LP   |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

DTM (Direct Test Mode) application allows to configure a BlueNRG-LP as a network coprocessor and target Bluetooth LE technology evaluation and RF evaluation performances tests using the BlueNRG GUI or other instruments as CBT. 

The DTM project includes the following configurations:
  - UART: DTM with UART interface (no updater code)
  - UART_WITH_UPDATER: DTM including the DTM_Updater in the first page of the Flash memory. This image is necessary to use the Device Configuration Tool (GUI) and the DTM Firmware Updater (GUI).
  - UART_FOR_UPDATER: The DTM image with the first page empty (offset 0x2000). This image needs the DTM with Updater to run and it is  used for the DTM Firmware Updater (GUI).
  - SPI: DTM with SPI interface (no updater code)   
  - SPI_WITH_UPDATER: DTM including the DTM_Updater in the first page of the Flash memory. This image is necessary to use the Device Configuration Tool (GUI) and the DTM Firmware Updater (GUI).
  - SPI_FOR_UPDATER: The DTM image with the first page empty (offset 0x2000). This image needs the SPI wiht updater to run and it is  used for the DTM Firmware Updater (GUI).   

 The configuration DTM - UART_WITH_UPDATER has the following memory layout:
 
------------- 0x1007FFFF
             
- <b> DTM </b>
 
------------- 0x10042000

- <b> DTM Updater </b>
 
------------- 0x10040000

 The DTM Updater allows to access the memory flash through ACI_HAL commands. It can be activated in the following way:
 - Activation by using ACI_HAL_UPDATER_START
 - Activation by using IO3 pin (high level at start up).
   - Note: if the IO3 pin is used and is high at start up, this will cause the  DTM Updater starts. So, to avoid this, the support of the DTM Updater can be removed.

Any change of relevant pins must be reported also in DTM Updater firmware.
 

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP DTM application \see DTM_main.c for documentation.
 *
 *@{
 */
/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "bluenrg_lp_stack.h"
#include "DTM_boot.h"
#include "bluenrg_lp_hal_power_manager.h"
#include "transport_layer.h"
#include "hw_config.h"
#include "hal_miscutil.h" 
#include "DTM_cmd_db.h"
#include "bleplat.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "nvm_db.h"
#include "DTM_burst.h"
#include "aci_adv_nwk.h"   
#include "aci_l2cap_nwk.h"

#define RESET_REASON_WDG        ((uint8_t)0x05)
#define RESET_REASON_LOCKUP     ((uint8_t)0x06)
#define RESET_REASON_POR_BOR    ((uint8_t)0x07)
#define RESET_REASON_CRASH      ((uint8_t)0x08)

/* Add aci_blue_initialized_event() prototype */
void aci_blue_initialized_event(uint8_t Reason_Code);

/* Add aci_blue_crash_info_event() prototype */
void aci_blue_crash_info_event(uint8_t Crash_Type,
                               uint32_t SP,
                               uint32_t R0,
                               uint32_t R1,
                               uint32_t R2,
                               uint32_t R3,
                               uint32_t R12,
                               uint32_t LR,
                               uint32_t PC,
                               uint32_t xPSR,
                               uint8_t Debug_Data_Length,
                               uint8_t Debug_Data[]);

extern uint32_t irq_count;
extern uint16_t num_packets;
/*
 ******************************************************************************
 ******************************************************************************
 * 
 * The DTM project includes the following configurations:
 *
 * Configuration       | Description
 *  UART/SPI               |  The DTM with interface UART/SPI
 *  UART/SPI_WITH_UPDATER  |  The DTM including the DTM_Updater in the first page
 *                     |   of the Flash memory. This image is necessary to use
 *                     |   the IFR Tool (GUI) and the DTM Firmware Updater (GUI).
 *  UART_FOR_UPDATER   |  The DTM image with the first page empty (offset 0x200).
 *                     |   This image needs the DTM_Updater to run and it is 
 *                     |   used for the DTM Firmware Updater (GUI).
 *------------------------------------------------------------------------------
 * 
 * The configuration DTM - UART_WITH_UPDATER has the following memory layout:
 * 
 * ------------- 0x1007FFFF
 *             
 *      DTM
 * 
 * ------------- 0x10042000
 *  DTM Updater
 * ------------- 0x10040000
 *
 * The DTM Updater allows to access the memory flash through ACI_HAL commands.
 * The DTM Updater can be activated in the following way:
 *  1) Activation by using ACI_HAL_UPDATER_START
 *  2) Activation by using IO15 pin (high level at start up).
 *     Note: if the IO15 pin is used and is high at start up, this will cause the 
 *     DTM Updater starts. So, to avoid this, the support of the DTM Updater
 *     can be removed.
 * 
 * Any change of relevant pins must be reported
 *  also in DTM Updater firmware.
 *
 ****************************************************************************
 ****************************************************************************
*/
int main(void)
{
  crash_info_t crash_info;
  WakeupSourceConfig_TypeDef wakeupIO;
  PowerSaveLevels stopLevel;

  /* System Init */
  DTM_SystemInit();
  
  /* Stack Initialization */
  DTM_StackInit();
  
  aci_adv_nwk_init();
  aci_l2cap_nwk_init();
  
    
  /* Transport Layer Init */
  transport_layer_init();
  
  /* Get crash info */
  HAL_GetCrashInfo(&crash_info); 
  
  if(RAM_VR.Reserved[0] == 0x01){
    // Send a comman complete event for HCI_Reset
    uint8_t buffer_out[] = {0x04,0x0E,0x04,0x01,0x03,0x0C,0x00};
    RAM_VR.Reserved[0] = 0x00;
    send_event(buffer_out,7,-1);
  }
  
#ifdef LL_ONLY
  uint8_t Value = 1;
  aci_hal_write_config_data(0x2C, 1, &Value);
  
#else
  
  uint8_t reset_reason = 0x01;
  
  /* EVT_BLUE_INITIALIZED */  
  /* Check the reset reason */
  if(RAM_VR.ResetReason & RCC_CSR_WDGRSTF){
    reset_reason = RESET_REASON_WDG;
  }
  else if(RAM_VR.ResetReason & RCC_CSR_LOCKUPRSTF) {
    reset_reason = RESET_REASON_LOCKUP;
  }
  else if(RAM_VR.ResetReason & RCC_CSR_PORRSTF) {
    reset_reason = RESET_REASON_POR_BOR;
  }
  
  if((crash_info.signature&0xFFFF0000) == CRASH_SIGNATURE_BASE) {  
    reset_reason = RESET_REASON_CRASH;
  }

  aci_blue_initialized_event(reset_reason);

#endif

  if((crash_info.signature&0xFFFF0000) == CRASH_SIGNATURE_BASE) { 
    aci_blue_crash_info_event(crash_info.signature&0xFF,
                              crash_info.SP,
                              crash_info.R0,
                              crash_info.R1,
                              crash_info.R2,
                              crash_info.R3,
                              crash_info.R12,
                              crash_info.LR,
                              crash_info.PC,
                              crash_info.xPSR,
                              0,
                              NULL);
  }

  wakeupIO.IO_Mask_High_polarity = 0;
  wakeupIO.IO_Mask_Low_polarity = IO_WAKEUP_PIN;
  wakeupIO.RTC_enable = 0;

  while(1) {
    BURST_Tick();
    HAL_VTIMER_Tick();
    BLE_STACK_Tick();
    NVMDB_Tick();
    transport_layer_tick();
    if(num_packets != 0 &&  irq_count == num_packets)
    {
      uint32_t Number_Of_TX_Packets = 0;
      uint16_t Number_Of_RX_Packets;
      
      /* Reached number of tx test packets */
      hci_le_test_end(&Number_Of_RX_Packets);
      aci_hal_le_tx_test_packet_number(&Number_Of_TX_Packets);
      aci_hal_le_test_end_event(Number_Of_TX_Packets);
      irq_count = 1;
      num_packets = 0;
    }
    
    HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel); 
  }
}


