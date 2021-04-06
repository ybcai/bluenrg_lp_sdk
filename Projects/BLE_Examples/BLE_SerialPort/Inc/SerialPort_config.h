#ifndef _SERIALPORT_CONFIG_H_
#define _SERIALPORT_CONFIG_H_

#include "bluenrg_lp_stack.h"
#include "stack_user_cfg.h"
#if defined (CONFIG_OTA_LOWER) || defined (CONFIG_OTA_HIGHER)
#include "OTA_btl.h"
#endif

/* This file contains all the information needed to init the BlueNRG-1 stack.
 * These constants and variables are used from the BlueNRG-1 stack to reserve RAM and FLASH 
 * according the application requests
 */

/* Default number of link */
#define MIN_NUM_LINK_CONF                1

/* Number of services */
#define NUM_APP_GATT_SERVICES_CONF  (1)       /* attributes for serial port service */

/* Number of characteristics attributes requests */
#define NUM_APP_GATT_CHAR_ATTRIBUTES_CONF (5) /* attributes for serial port service characteristics */

/* Set supported max value for ATT_MTU enabled by the application */
#if (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED == 1) && defined(CONFIG_SW_OTA_DATA_LENGTH_EXT) 
  #define MAX_ATT_MTU_CONF    (OTA_ATT_MTU_SIZE_CONF)   //* OTA Client & Server supported ATT_MTU */   
#else /* No data length extension support */
  #define MAX_ATT_MTU_CONF    (BLE_STACK_DEFAULT_ATT_MTU)             /* DEFAULT_ATT_MTU size = 23 bytes */ 
#endif   

#if defined (CONFIG_OTA_LOWER) || defined (CONFIG_OTA_HIGHER)

#define OTA_GATT_ATTRIBUTES_CONF     (1 + 9)  /* 10 for OTA Service & characteristics */

#else /* NO OTA Service is required */
  
#define OTA_GATT_ATTRIBUTES_CONF     (0)  /* OTA Service & characteristics */

#endif 

/* Number of links needed for the demo: 1
 * Only 1 the default
 */
#define NUM_LINKS                                   (MIN_NUM_LINK_CONF)
#define NUM_ADV_SETS_CONF                            (2U) 
#define NUM_AUX_SCAN_SLOTS_CONF                      (2U)
#define WHITE_LIST_SIZE_LOG2_CONF                    (0U)
#define L2CAP_MPS_CONF                               (247U)
#define NUM_L2CAP_COCS_CONF                          (0U)

/* Number of GATT attributes needed for the demo. */
#define NUM_GATT_ATTRIBUTES     (NUM_APP_GATT_SERVICES_CONF + NUM_APP_GATT_CHAR_ATTRIBUTES_CONF + OTA_GATT_ATTRIBUTES_CONF)

/* Additional number of memory blocks  to be added to the minimum */
#define OPT_MBLOCKS_CONF		(6) 

/* Set the number of memory block for packet allocation */
#define MBLOCKS_COUNT           (BLE_STACK_MBLOCKS_CALC(MAX_ATT_MTU_CONF, NUM_LINKS) + OPT_MBLOCKS_CONF)

#define NUM_OF_CONCURRENT_GATT_CLIENT_PROC              (1)

/* RAM reserved to manage all the data stack according the number of links,
 * number of services, number of attributes and attribute value length
 */
#define DYNAMIC_MEMORY_SIZE (BLE_STACK_TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_OF_CONCURRENT_GATT_CLIENT_PROC,MBLOCKS_COUNT,NUM_ADV_SETS_CONF,NUM_AUX_SCAN_SLOTS_CONF,WHITE_LIST_SIZE_LOG2_CONF,NUM_L2CAP_COCS_CONF))

/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH_CONF 0xFFFFFFFF

#define CALIBRATION_INTERVAL_CONF   10000

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION         TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION         FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 320 // 780 us


/* Radio Config Hot Table */
extern uint8_t hot_table_radio_config[];

/* This structure contains memory and low level hardware configuration data for the device */
#define BLE_STACK_INIT_PARAMETERS {                                             \
    .BLEStartRamAddress = (uint8_t*)dyn_alloc_a,                                \
    .TotalBufferSize = DYNAMIC_MEMORY_SIZE,                                     \
    .NumAttrRecords = NUM_GATT_ATTRIBUTES,                                      \
    .MaxNumOfClientProcs = NUM_OF_CONCURRENT_GATT_CLIENT_PROC,                  \
    .NumOfLinks = NUM_LINKS,                                                    \
    .NumBlockCount = MBLOCKS_COUNT,                                             \
    .ATT_MTU = MAX_ATT_MTU_CONF,                                                \
    .MaxConnEventLength = MAX_CONN_EVENT_LENGTH_CONF,                           \
    .SleepClockAccuracy = SLEEP_CLOCK_ACCURACY,                                 \
    .HotAnaConfigTable = hot_table_radio_config,                                \
    .NumOfAdvDataSet = NUM_ADV_SETS_CONF,                                       \
    .NumOfAuxScanSlots = NUM_AUX_SCAN_SLOTS_CONF,                               \
    .WhiteListSizeLog2 = WHITE_LIST_SIZE_LOG2_CONF,                             \
    .L2CAP_MPS = L2CAP_MPS_CONF,                                                \
    .L2CAP_NumChannels = NUM_L2CAP_COCS_CONF                                    \
}

#endif // _SERIALPORT_CONFIG_H_
