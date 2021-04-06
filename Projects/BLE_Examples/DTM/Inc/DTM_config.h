/**
  ******************************************************************************
  * @file    DTM_config.h 
  * @author  VMA RF Application Team
  * @version V1.1.0
  * @date    April-2019
  * @brief   
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

#ifndef _DTM_CONFIG_H_
#define _DTM_CONFIG_H_
#include "BlueNRG_LP.h"
#include "bluenrg_lp_stack.h"
#include "stack_user_cfg.h"

/** This file contains all the information needed to init the BlueNRG-1 stack. 
 * These constants and variables are used from the BlueNRG-1,2 stack to reserve RAM and FLASH 
 * according the application requests.
 * NOTE: the current selected configuration is tuned for supporting up to 8 connections,
 * 8 services, 68 attributes. 
 * MAX ATT_MTU size is set to 220 bytes which is the max allowed value for fitting with the 
 * overall DTM RAM memory requirements for all the supported configurations (SPI and UARTs),  and
 * use the extended packey length for OTA client FW upgrade procedure.
 * This implies to reduce the PREPARE_WRITE_LIST_SIZE value. 
 * User can modify, tune the configuration options according to his specific application requirements. 
 */


/* MAX number of links for DTM */
#define DTM_NUM_LINK_CONF                               (CONFIG_NUM_MAX_LINKS)
/* MAX number of GAP and GATT attributes for DTM */
#define DTM_NUM_GATT_ATTRIBUTES_CONF                    (100)

/* Number of links needed for the demo: 1
 * Only 1 the default
 */
#define NUM_LINKS               (DTM_NUM_LINK_CONF)

#define NUM_AUX_SCAN_SLOTS_CONF                 (4U)   
#define WHITE_LIST_SIZE_LOG2_CONF               (4U)
#define L2CAP_MPS_CONF                          (247U)
#define NUM_L2CAP_COCS_CONF                     (2U)

/* Number of GATT attributes needed for the DTM */
#define NUM_GATT_ATTRIBUTES     (DTM_NUM_GATT_ATTRIBUTES_CONF)

/* Set supported max value for ATT_MTU */
#define MAX_ATT_MTU_CONF            (247) 

#define OPT_MBLOCKS_CONF                                (30)

#define ACI_ATT_QUEUED_WRITE_SIZE_CONF                  (512)
#define ACI_GATT_WR_BUFFER_SIZE_CONF                    (3072 + ACI_ATT_QUEUED_WRITE_SIZE_CONF)
#define NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF         (8)

/* Set the number of memory block for packet allocation */
#define MBLOCKS_COUNT           (BLE_STACK_MBLOCKS_CALC(MAX_ATT_MTU_CONF, NUM_LINKS) + OPT_MBLOCKS_CONF)

/* RAM reserved to manage all the data stack according the number of links,
 * number of services, number of attributes and attribute value length
 */
#define DYNAMIC_MEMORY_SIZE (BLE_STACK_TOTAL_BUFFER_SIZE(NUM_LINKS,NUM_GATT_ATTRIBUTES,NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF,MBLOCKS_COUNT,NUM_ADV_SETS_CONF,NUM_AUX_SCAN_SLOTS_CONF,WHITE_LIST_SIZE_LOG2_CONF,NUM_L2CAP_COCS_CONF))

/* Maximum duration of the connection event */
#define MAX_CONN_EVENT_LENGTH_CONF 0xFFFFFFFF

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

#define CALIBRATION_INTERVAL_CONF   10000


/* Radio Config Hot Table */
extern uint8_t hot_table_radio_config[];

/* This structure contains memory and low level hardware configuration data for the device */
#define BLE_STACK_INIT_PARAMETERS {                                             \
    .BLEStartRamAddress = (uint8_t*)dyn_alloc_a,                                \
    .TotalBufferSize = DYNAMIC_MEMORY_SIZE,                                     \
    .NumAttrRecords = NUM_GATT_ATTRIBUTES,                                      \
    .MaxNumOfClientProcs = NUM_OF_CONCURRENT_GATT_CLIENT_PROC_CONF,             \
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

#endif // _DTM_CONFIG_H_
