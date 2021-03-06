
#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "profile.h"
#include "bluenrg_lp_hal_vtimer.h"
#include "bluenrg_lp_evb_com.h"
#include "procedures.h"

#define ADV_INTERVAL_MIN    ((uint16_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint16_t)(100/0.625))     // 100 ms

#define DEBUG 2

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#if DEBUG > 1
#include <stdio.h>
#define PRINTF_DBG2(...) printf(__VA_ARGS__)
#else
#define PRINTF_DBG2(...)
#endif

#define PRINT_ADDDRESS(a)   PRINTF("0x%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0])


typedef struct Blacklist_Entry_t_s {
  /** Address type. 
  * Values:
  - 0x00: Public Device Address
  - 0x01: Random Device Address
  */
  uint8_t address_type;
  /** Public Device Address or Random Device Address of the device
    to be added to the white list.
  */
  uint8_t address[6];
  /** Counts how many times the device satisfied the blacklist condition, i.e. connection fails to be established. */
  uint8_t blacklist_hit; 
} Blacklist_Entry_t;
Blacklist_Entry_t last_conn_failed_device;
#define MAX_BLACKLIST_HIT 5


uint8_t DeviceInBlacklist(uint8_t address_type, uint8_t address[6]);

static uint8_t procedure = 0xFF;

char name[] = LOCAL_NAME;
#define NAME_LENGTH (sizeof(name)-1)

#define USE_SCAN_RESP_DATA 1

#if MAX_NUM_MASTERS

#if USE_SCAN_RESP_DATA
static uint8_t scan_resp_data[19] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH + 1, AD_TYPE_COMPLETE_LOCAL_NAME};
#define UUID_SERVICE_DATA_LEN   0
#else
#define UUID_SERVICE_DATA_LEN   19
#endif

static uint8_t adv_data[5+NAME_LENGTH+UUID_SERVICE_DATA_LEN] = {
  0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
  NAME_LENGTH+1, AD_TYPE_COMPLETE_LOCAL_NAME};

#define NON_DISCOVERABLE_ADV_DATA_LEN   3               // Do not send name and UUID if not discoverable
#define DISCOVERABLE_ADV_DATA_LEN       sizeof(adv_data)

#endif /* MAX_NUM_MASTERS */

tBleStatus ConfigureWhiteList(void)
{
  Bonded_Device_Entry_t bonded_device;
  uint8_t num_returned_devices;
  uint8_t num_bonded_devices = 0;
  tBleStatus ret;
  
  PRINTF("Bonded devices:\n");
  
  for(int i = 0;; i++) {  
    aci_gap_get_bonded_devices(i, 1, &num_returned_devices, &bonded_device);
    
    if(num_returned_devices){
      PRINTF("Type %d, ", bonded_device.Address_Type);
      PRINT_ADDDRESS(bonded_device.Address);
      PRINTF("\n");
    }
    else {
      break;
    }    
    num_bonded_devices += num_returned_devices;    
  }
  
  if(num_bonded_devices){
    
#if MAX_NUM_MASTERS    
    if(APP_FLAG(ADVERTISING)){
      // Stop advertising to update white list
      ret = StopAdvertising();
      if(ret)
        return ret;      
    }
#endif
#if MAX_NUM_SLAVES
    if(APP_FLAG(SCANNING)){
      // Stop advertising to update white list
      StopScan();
    }
#endif
   
    ret = aci_gap_configure_white_and_resolving_list(0x01);
    if(ret == BLE_STATUS_SUCCESS)
    {
      PRINTF("Whitelist configured.\n");
    }    
    else
    {
      PRINTF("Error while configuring whitelist: 0x%02X\n", ret);
    }
#if MAX_NUM_MASTERS
    if(APP_FLAG(ADVERTISING)){
      // Restart advertising
      ret = StartAdvertising();
      if(ret)
        return ret;
    }
#endif
#if MAX_NUM_SLAVES
    if(APP_FLAG(SCANNING)){
      // Restart advertising
      ret = StartAutoConnection();
      if(ret)
        return ret;
    }
#endif
    
  }
  else {
    PRINTF("No bonded devices.\n");
  }
  
  return BLE_STATUS_SUCCESS;
}


tBleStatus StartAutoConnection(void)
{
  tBleStatus ret;
    
  ret = aci_gap_start_procedure(GAP_AUTO_CONNECTION_ESTABLISHMENT_PROC, LE_1M_PHY_BIT, 0, 0);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting connection: 0x%02X\n", ret);
    procedure = 0xFF;
    
    return ret;
  }
  else {
    PRINTF("Connecting...\n");
    procedure = GAP_AUTO_CONNECTION_ESTABLISHMENT_PROC;
  }
  
  return BLE_STATUS_SUCCESS;
}

void StartGeneralConnectionEstablishment(void)
{
  tBleStatus ret;
  
  ret = aci_gap_start_procedure(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC, LE_1M_PHY_BIT, 0, 0);
  
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINTF("Error while starting scanning: 0x%02X\n", ret);
    procedure = 0xFF;
  }
  else {
    PRINTF("Scanning...\n");
    procedure = GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC;
  }  
}

void StopScan(void)
{
  PRINTF("Stop Scanning.\n");
  aci_gap_terminate_proc(procedure);
}

#if MAX_NUM_MASTERS
tBleStatus ConfigureAdvertising(uint8_t discoverable)
{
  tBleStatus ret;
  
  if(discoverable){
    adv_data[2] = FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED;    
    // Add name to advertising data
    Osal_MemCpy(adv_data+5,name,NAME_LENGTH);
#if !USE_SCAN_RESP_DATA    
    // Add service UUID to advertising data
    adv_data[5+NAME_LENGTH] = 18;
    adv_data[5+NAME_LENGTH+1] = AD_TYPE_128_BIT_UUID_SERVICE_DATA;
    Osal_MemCpy(adv_data+5+NAME_LENGTH+2,SerialPort_service_uuid,sizeof(SerialPort_service_uuid));
    adv_data[5+NAME_LENGTH+2+16] = PROFILE_DATA_COLLECTOR; // Service data, to identify different kind of devices (0: slave-only device, 1: master slave)
#else
    scan_resp_data[0] = 18;
    scan_resp_data[1] = AD_TYPE_128_BIT_UUID_SERVICE_DATA;
    Osal_MemCpy(scan_resp_data+2,SerialPort_service_uuid,sizeof(SerialPort_service_uuid));
    scan_resp_data[18] = PROFILE_DATA_COLLECTOR; // Service data, to identify different kind of devices (0: slave-only device, 1: master slave)
#endif
    
    ret = aci_gap_set_advertising_configuration(0x00, // Advertising handle
                                                0x02, // General discoverable mode
                                                0x0013, // Connectable, Scannable, Legacy
                                                ADV_INTERVAL_MIN,
                                                ADV_INTERVAL_MAX,
                                                ADV_CH_ALL,
                                                0, NULL, // No peer address
                                                ADV_NO_WHITE_LIST_USE,
                                                127, // No preference for TX power
                                                LE_1M_PHY, // Primary_Advertising_PHY (not used for legacy adv)
                                                0, // Secondary_Advertising_Max_Skip (not used for legacy adv)
                                                LE_1M_PHY, //  Secondary_Advertising_PHY (not used for legacy adv)
                                                0, // Advertising_SID (not used for legacy adv)
                                                0); // No scan request notification
    PRINTF("Advertising configuration (discoverable) 0x%02X\n", ret);
    if(ret)
      return ret;
    
#if USE_SCAN_RESP_DATA
    ret = aci_gap_set_scan_response_data(0x00, sizeof(scan_resp_data), scan_resp_data);
    if(ret)
      return ret;
#endif
     
    ret = aci_gap_set_advertising_data(0x00, // Advertising handle
                                        0x03, // Complete data
                                        DISCOVERABLE_ADV_DATA_LEN, adv_data);
    PRINTF("aci_gap_set_advertising_data 0x%02X\n", ret);
    return ret;
  }
  else {
    adv_data[2] = FLAG_BIT_BR_EDR_NOT_SUPPORTED;
    
    ret = aci_gap_set_advertising_configuration(0x00, // Advertising handle
                                                0x00, // Not discoverable
                                                0x0013, // Connectable, Scannable, Legacy
                                                ADV_INTERVAL_MIN,
                                                ADV_INTERVAL_MAX,
                                                ADV_CH_ALL,
                                                0, NULL, // No peer address
                                                ADV_WHITE_LIST_FOR_ALL,
                                                127, // No preference for TX power
                                                LE_1M_PHY, // Primary_Advertising_PHY (not used for legacy adv)
                                                0, // Secondary_Advertising_Max_Skip (not used for legacy adv)
                                                LE_1M_PHY, //  Secondary_Advertising_PHY (not used for legacy adv)
                                                0, // Advertising_SID (not used for legacy adv)
                                                0); // No scan request notification
    PRINTF("Advertising configuration (non-discoverable) 0x%02X\n", ret);
    if(ret)
      return ret;
    
    ret = aci_gap_set_advertising_data(0x00, // Advertising handle
                                        0x03, // Complete data
                                        NON_DISCOVERABLE_ADV_DATA_LEN, adv_data);
    PRINTF("aci_gap_set_advertising_data 0x%02X\n", ret);
    return ret;
  }
}

tBleStatus StartAdvertising()
{
  tBleStatus ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters = {
    .Advertising_Handle = 0,
    .Duration = 0,
    .Max_Extended_Advertising_Events = 0,
  };
  
  ret = aci_gap_set_advertising_enable(ENABLE,1,&Advertising_Set_Parameters);
  
  PRINTF("Enable advertising 0x%02X\n", ret);
  return ret;
}

tBleStatus StopAdvertising(void)
{
  tBleStatus ret;
  Advertising_Set_Parameters_t Advertising_Set_Parameters = {
    .Advertising_Handle = 0,
    .Duration = 0,
    .Max_Extended_Advertising_Events = 0,
  };
  
  ret = aci_gap_set_advertising_enable(DISABLE,1,&Advertising_Set_Parameters);
  PRINTF("Disable advertising 0x%02X\n", ret);
  return ret;
}
#endif

void BlacklistReset(void)
{
  last_conn_failed_device.blacklist_hit = 0;  
}

void BlacklistHit(uint8_t address_type, uint8_t address[])
{
  if(last_conn_failed_device.address_type == address_type && memcmp(last_conn_failed_device.address, address, 6) == 0){
    // This is the same device to which the connection has been failed.
    last_conn_failed_device.blacklist_hit++;
    PRINTF_DBG2("DEVICE IN BLACKLIST, COUNT: %d\n", last_conn_failed_device.blacklist_hit);
    if(last_conn_failed_device.blacklist_hit>=MAX_BLACKLIST_HIT){
      aci_gap_remove_bonded_device(address_type, address);
    }
  }
  else {
    // New device
    PRINTF_DBG2("DEVICE ADDED TO BLACKLIST\n");
    last_conn_failed_device.address_type = address_type;
    Osal_MemCpy(last_conn_failed_device.address, address, 6);
    last_conn_failed_device.blacklist_hit=1;
  }
}

/*******************************************************************************
* Function Name  : DeviceInBlacklist
* Description    : Checks if device is in blacklist
* Input          : address_type The address type (0: PUBLIC_ADDRESS, 1: RANDOM_ADDRESS)
*                  address      The address
* Return         : TRUE if device is in blacklist, FALSE otherwise.
*******************************************************************************/
uint8_t DeviceInBlacklist(uint8_t address_type, uint8_t address[6])
{
  if(last_conn_failed_device.blacklist_hit > MAX_BLACKLIST_HIT &&
     address_type == last_conn_failed_device.address_type &&
     memcmp(address, last_conn_failed_device.address, 6) == 0){
       // This device is blacklisted. 
       return TRUE;
  }
  
  return FALSE;  
}
