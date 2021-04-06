
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_Beacon_FreeRTOS_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 09-September-2019
* Description        : Code demostrating the Bluetooth LE Beacon application and the use of FreeRTOS with Bluetooth LE stack
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Beacon_FreeRTOS_main.c
 * @brief This is a Bluetooth LE beacon demo that shows how to configure a Bluetooth LE device 
 * in order to advertise specific manufacturing data and allow another Bluetooth LE device to
 * know if it is in the range of the BlueNRG-LP beacon device. It also shows how to
 * use FreeRTOS with ST Bluetooth LE stack v3.x.
 * 

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon_FreeRTOS\\EWARM\\{STEVAL-IDB011V1}\\BLE_Beacon.eww </tt>
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

The Beacon demo configures a BlueNRG-LP device in advertising mode (non-connectable mode) with specific manufacturing data.
It transmits advertisement packets at regular intervals which contain the following manufacturing data:
@table   
------------------------------------------------------------------------------------------------------------------------
| Data field              | Description                       | Notes                                                  |
------------------------------------------------------------------------------------------------------------------------
| Company identifier code | SIG company identifier (1)        | 0x004C (Apple, Inc.)				       |
| ID                      | Beacon ID                         | Fixed value                                            |
| Length                  | Length of the remaining payload   | NA                                                     |
| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |                                              
| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |                                       
| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |                                       
@endtable

     - (1): SIG company identifiers are available on https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers/
     - NA : Not Applicable;

The BTLE_StackTick() is called from a FreeRTOS task (BLETask).
A task randomly changes the Minor number in the advertising data, every 500 ms. A message is sent through UART each time
this is done.
Another task sends other messages through UART every 200 ms and generates a short pulse on LED3 (visible with a logic
analyzer or oscilloscope).
A low priority has been assigned to the BLETask in this example. In general, assigning an high priority to Bluetooth LE Task can give
better latency, especially if other tasks are CPU resource hungry. If some tasks require a lot of CPU time, it is recommended
to assign to those tasks a priority lower than the BLETask, otherwise Bluetooth LE operations may be slowed down. Only for tasks that
perform very short sporadic operations before waiting for an event, it is still reasonable to choose a priority higher than
the BLETask.

**/
   
/** @addtogroup BlueNRGLP_demonstrations_applications
 *  BlueNRG-LP Beacon FreeRTOS demo \see BLE_Beacon_FreeRTOS_main.c for documentation.
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
#include "Beacon_config.h"
#include "OTA_btl.h"
#include "bluenrg_lp_hal_power_manager.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "bleplat.h"
#include "nvm_db.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "freertos_ble.h"

/* Binary semaphore used to synchronize Stack Tick and radio ISR. */
SemaphoreHandle_t radioActivitySemaphoreHandle;
/* Mutex used to avoid that the BLE Stack Tick can be interrupted by an ACI
   function in another thread. */
SemaphoreHandle_t BLETickSemaphoreHandle;
/* Mutex used to access UART resource */
SemaphoreHandle_t UARTSemaphoreHandle;

#define BLE_BEACON_VERSION_STRING "1.0.0"

/* Set to 1 to enable the name AD data in extended advertising events (if
  extended advertising events are used).  */
#define DEVICE_NAME_IN_ADV 0

/* PHY used in extended advertising events. One between: LE_1M_PHY,
  LE_2M_PHY and LE_CODED_PHY.  */
#define EXT_ADV_PHY LE_CODED_PHY

/*-----------------------------------------------------------*/
/* Priorities at which the tasks are created.
   Assigning an high priority to BLE Task can give better latency, especially
   if other tasks are CPU resource hungry.
*/
#define TEST_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 2 )
#define	BLE_TASK_PRIORITY		            ( tskIDLE_PRIORITY + 1 )

/*-----------------------------------------------------------*/
/* Wait time of the test task (numbe rof ticks) */
#define TEST_PERIOD         			    ( 200 / portTICK_PERIOD_MS )
#define ADV_CHANGE_PERIOD         			( 500 / portTICK_PERIOD_MS )

/* Private macro -------------------------------------------------------------*/
#define DEBUG 1 

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) do{ xSemaphoreTake(UARTSemaphoreHandle, portMAX_DELAY);\
                      printf(__VA_ARGS__);                              \
                      xSemaphoreGive(UARTSemaphoreHandle); }while(0)
                        
#else
#define PRINTF(...)
#endif
                        
/* Private variables ---------------------------------------------------------*/


  /* Set AD Type Flags at beginning on Advertising packet  */
static uint8_t adv_data[] = {
  /* Advertising data: Flags AD Type */
  0x02, 
  0x01, 
  0x06, 
  /* Advertising data: manufacturer specific data */
  26, //len
  AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
  0x4C, 0x00, //Company identifier code
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x05, // Major number 
  0x00, 0x07, // Minor number 
  (uint8_t)-56,         // Tx power measured at 1 m of distance (in dBm)
#if DEVICE_NAME_IN_ADV
  15,       // Length of following AD data
  0x09,'E','x','t','e','n','d','e','d','B','e','a','c','o','n'
#endif
};

NO_INIT(uint32_t dyn_alloc_a[DYNAMIC_MEMORY_SIZE>>2]);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void createTasks( void );

void ModulesInit(void)
{
  uint8_t ret;
  BLE_STACK_InitTypeDef BLE_STACK_InitParams = BLE_STACK_INIT_PARAMETERS;
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA|LL_AHB_PERIPH_RNG);
  
  /* BlueNRG-LP stack init */
  ret = BLE_STACK_Init(&BLE_STACK_InitParams);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in BLE_STACK_Init() 0x%02x\r\n", ret);
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

void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t address[CONFIG_DATA_PUBADDR_LEN] = {0x66,0x77,0x88,0xE1,0x80,0x02};
  
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, address);
  
  /* Set the TX Power to 0 dBm */
  ret = aci_hal_set_tx_power_level(0,25);
  if(ret != 0) {
    PRINTF ("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
    while(1);
  }

  /* Init the GATT */
  ret = aci_gatt_srv_init();
  if (ret != 0) 
  {
    PRINTF ("Error in aci_gatt_srv_init() 0x%04xr\n", ret);
  }
  else
  {
    PRINTF ("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* Init the GAP */
  ret = aci_gap_init(0x01, 0x00, 0x08, PUBLIC_ADDR, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != 0)
  {
    PRINTF ("Error in aci_gap_init() 0x%04x\r\n", ret);
  }
  else
  {
    PRINTF ("aci_gap_init() --> SUCCESS\r\n");
  }
}


/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
  uint8_t ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters[2];
   
  /* Set advertising configuration for legacy advertising. */  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_LEGACY,
                                              160, 160,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_configuration() 0x%04x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, 30, adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_data() 0x%04x\r\n", ret);
    return;
  }
  
#if EXTENDED_ADV
  /* Set advertising configuration for extended advertising. */  
  ret = aci_gap_set_advertising_configuration(1, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_NONE,
                                              160, 160,
                                              ADV_CH_ALL,
                                              0,NULL, /* No peer address */
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              (EXT_ADV_PHY==LE_2M_PHY)?LE_1M_PHY:EXT_ADV_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              EXT_ADV_PHY, /* Secondary advertising PHY */
                                              1, /* SID */
                                              0 /* No scan request notifications */);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_configuration() 0x%04x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_data() 0x%04x\r\n", ret);
    return;
  }
  
#endif
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  Advertising_Set_Parameters[1].Advertising_Handle = 1; // This is the handle for the set containing extended events
  Advertising_Set_Parameters[1].Duration = 0;
  Advertising_Set_Parameters[1].Max_Extended_Advertising_Events = 0;
  
  
   /* Enable advertising */
#if EXTENDED_ADV  
  ret = aci_gap_set_advertising_enable(ENABLE, 2, Advertising_Set_Parameters);
#else
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters);
#endif
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF ("Error in aci_gap_set_advertising_enable() 0x%04x\r\n", ret);
    return;
  }
  else
    PRINTF ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");

}



int main(void) 
{
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);
  
  UARTSemaphoreHandle = xSemaphoreCreateMutex();
  
  BSP_LED_Init(BSP_LED1);
  BSP_LED_Init(BSP_LED3);
  
  BSP_LED_On(BSP_LED1);
  
  
   /* Create a binary semaphore to sync with radio interrupts */
  radioActivitySemaphoreHandle = xSemaphoreCreateBinary();
  /* Create a mutex semaphore to avoid calling aci functions while
    BTLE_StackTick() is running.*/
  BLETickSemaphoreHandle =  xSemaphoreCreateMutex();
  if(radioActivitySemaphoreHandle==NULL || BLETickSemaphoreHandle == NULL){
    while(1);
  }
    
  ModulesInit(); 
  
  PRINTF("BlueNRG-LP BLE Beacon with FreeRTOS Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 

  createTasks();

}


static void BLETask( void *pvParameters )
{
  /* To make sure no other BLE functions are called from other tasks. */
  xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
  
  /* Init the BlueNRG-LP device */
  Device_Init();
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
  
  /* BLE is initialized. Let other tasks call BLE functions. */
  xSemaphoreGive(BLETickSemaphoreHandle);
  
  while(1)
  {
    /* Take the semaphore to avoid that other ACI functions can interrupt the
       execution of BTLE_StackTick();   */
    xSemaphoreTake(BLETickSemaphoreHandle, portMAX_DELAY);
    ModulesTick();    
    xSemaphoreGive(BLETickSemaphoreHandle);
    if(BLE_STACK_SleepCheck() != POWER_SAVE_LEVEL_RUNNING)
    {
      xSemaphoreTake(radioActivitySemaphoreHandle, portMAX_DELAY);
    }
  }
}

/*-----------------------------------------------------------*/
/* Just a test task which makes a very short pulse on a GPIO. */
static void testTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, TEST_PERIOD );
    
    /* Only do a pulse. */
    BSP_LED_On(BSP_LED3);
    __NOP();__NOP();__NOP();__NOP();
    BSP_LED_Off(BSP_LED3);
    
    PRINTF("Test Task\r\n");
  }  
}



/***************************************************************************************/

/*-----------------------------------------------------------*/
/* Another task that changes the advertising data */
static void changeADVDataTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  uint8_t Random_Number[8];
  
  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();
  
  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, ADV_CHANGE_PERIOD );
    
    BLE_ACI_PROTECTED(hci_le_rand(Random_Number));
    
    adv_data[28] = Random_Number[0];
    
    /* In this case there is no need to disable advertising before updating buffer
       content, since only one byte is changed (atomic operation) and there is no
       risk to have inconsistent data. */
    
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, 30, adv_data));
    
    
#if EXTENDED_ADV
    BLE_ACI_PROTECTED(aci_gap_set_advertising_data(1, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data));
#endif
    
    PRINTF("ADV change %d\r\n", adv_data[28]);
    
  }  
}
/*-----------------------------------------------------------*/

void createTasks( void )
{
  
  xTaskCreate(BLETask,"BLEStack", 650, NULL, BLE_TASK_PRIORITY, NULL);
  
  xTaskCreate( testTask, "Test", 80, NULL, TEST_TASK_PRIORITY, NULL );
  
  xTaskCreate( changeADVDataTask, "ADV", 100, NULL, TEST_TASK_PRIORITY, NULL );
  
  /* Start the tasks and timer running. */
  vTaskStartScheduler();
  
  /* If all is well, the scheduler will now be running, and the following
  line will never be reached.  If the following line does execute, then
  there was insufficient FreeRTOS heap memory available for the idle and/or
  timer tasks	to be created.  See the memory management section on the
  FreeRTOS web site for more details. */
  for( ;; );
}
/*-----------------------------------------------------------*/


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

/***************************************************************************************/


void vApplicationMallocFailedHook( void )
{
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created.  It is also called by various parts of the
  demo application.  If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
  task.  It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()).  If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;
  
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
  /* This function will be called by each tick interrupt if
  configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
  added here, but the tick hook is called from an interrupt context, so
  code must not attempt to block, and only the interrupt safe FreeRTOS API
  functions can be used (those that end in FromISR()). */
}
/*-----------------------------------------------------------*/

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
  ex: PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
