/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : profile.c
* Author             : AMS - RF  Application team
* Description        : This file define the procedure to connect and send data.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

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

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define ADV_INTERVAL_MIN    ((uint16_t)(100/0.625))     // 100 ms
#define ADV_INTERVAL_MAX    ((uint16_t)(100/0.625))     // 100 ms
#define SCAN_INTERVAL       ((uint16_t)(100/0.625))     // 100 ms
#define SCAN_WINDOW         ((uint16_t)(100/0.625))     // 100 ms
#define CONN_INTERVAL_MIN   ((uint16_t)(100/1.25))      // 100 ms
#define CONN_INTERVAL_MAX   ((uint16_t)(100/1.25))      // 100 ms
#define SUPERVISION_TIMEOUT ((uint16_t)(1000/10))       // 1000 ms
#define CE_LENGTH           ((uint16_t)(20/0.625))      // 20 ms

#define DEBOUNCE_TIMEOUT_MS         300
#define WRITE_INTERVAL_MS           1000
#define RESTART_ADV_TIMEOUT_MS      1000  // TODO: this delay should not be needed
#define RESTART_SCAN_TIMEOUT_MS     1000  // TODO: this delay should not be needed

#define DEBUG         2

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

#define MAX_NUM_SLAVES 3
#define MAX_NUM_MASTERS 2

#define PRINT_ADDDRESS(a)   PRINTF("0x%02X%02X%02X%02X%02X%02X", a[5], a[4], a[3], a[2], a[1], a[0])

uint8_t button_timer_expired = TRUE;

#if CLIENT


typedef enum {
  STATE_NORMAL,
  STATE_PAIRING,
}DeviceStateTypeDef;

typedef enum {
  SUBSTATE_INIT,
  SUBSTATE_WAITING_CONN,
  SUBSTATE_CONNECTED,
  SUBSTATE_WAITING_PAIRING,
}DeviceSubstateTypeDef;

struct {
  DeviceStateTypeDef state;
  DeviceSubstateTypeDef substate;
}device;

#endif

#if DEBUG
#define STATE_TRANSITION(STATE,SUBSTATE)   do { device.state = STATE; device.substate = SUBSTATE; printf("STATE (%d,%d)\n",STATE,SUBSTATE);}while(0)
#else
#define STATE_TRANSITION(STATE,SUBSTATE)   do { device.state = STATE; device.substate = SUBSTATE;}while(0)
#endif

/* Private variables ---------------------------------------------------------*/

const char name[] = "CentralNode";

#if MAX_NUM_MASTERS
static uint8_t adv_data[5 + sizeof(name)] = {0x02, AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED, sizeof(name)+1, AD_TYPE_COMPLETE_LOCAL_NAME};
#endif

volatile int app_flags = 0;

static uint8_t debounce_timeout_occurred = TRUE;
static VTIMER_HandleType debounce_timer;
static VTIMER_HandleType write_timer;

#if RESTART_ADV_TIMEOUT_MS && MAX_NUM_MASTERS
static VTIMER_HandleType restart_adv_timer;
#endif

#if RESTART_SCAN_TIMEOUT_MS
static VTIMER_HandleType restart_scan_timer;
#endif


typedef enum {
  Slave_State_NotBonded = 0,      // Not bonded
  Slave_State_Bonded_NotConnected,// Bonded and not connected
  Slave_State_Bonded_Connecting,  // Bond is being restore (i.e. encrypting link)
  Slave_State_Bonded_Connected,   // Connected and bond restored (i.e. links is encrypted)
}SlaveStatus;


// Type of the structure used to store the state related to each server/slave
typedef struct _slave {
  uint8_t  address_type;
  uint8_t  address[6];
  uint16_t conn_handle;
  MasterState state;
  MasterState resume_state;
  MasterPairingState pairing_state;
  uint8_t is_in_pairing_mode;
  uint16_t Serial Port_serv_start_handle;
  uint16_t Serial Port_serv_end_handle;
  uint16_t tx_handle;
  uint16_t rx_handle;
}slave_device;

slave_device slaves[MAX_NUM_SLAVES];

// Type of the structure used to store the state related to each master
typedef struct _master {
  uint8_t  address_type;
  uint8_t  address[6];
  uint16_t conn_handle;
}master_device;

#if MAX_NUM_MASTERS
master_device masters[MAX_NUM_MASTERS];
#endif

uint8_t num_connected_slaves = 0;
uint8_t num_connected_masters = 0;

void SlaveInit(uint8_t slave_index);
void MasterInit(uint8_t master_index);


/* Private function prototypes -----------------------------------------------*/
void StopRadioActivity(void);
void DebounceTimeoutCB(void *param);
void SendDataCB(void *param);
#if RESTART_ADV_TIMEOUT_MS
void RestartAdvTimeoutCB(void *param);
#endif
#if RESTART_SCAN_TIMEOUT_MS
void RestartScanTimeoutCB(void *param);
#endif

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : address_is_set.
* Description    : Check if address is set, i.e. if it is different
*                  from 0x000000000000
* Input          : the address.
* Return         : TRUE if addres is set, FALSE otherwise.
*******************************************************************************/
uint8_t address_is_set(uint8_t address[6])
{
  int i;
  
  for(i = 0; i < 6; i++){
    if(address[i] != 0)
      break;
  }
  if(i == 6)
    return FALSE;
  else  
    return TRUE;
}


/*******************************************************************************
* Function Name  : DeviceInit.
* Description    : Init profile and sevices for the device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t DeviceCommonInit(uint8_t role)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  
  uint8_t addr_len;
  uint8_t address[6];

  /* Set the TX power to -2 dBm */
  aci_hal_set_tx_power_level(0, 25);
  
  /* Since we need to transfer notifications of 244 bytes in a single packet, the LL payload must be
   244 bytes for application data + 3 bytes for ATT header + 4 bytes for L2CAP header. */
   ret = hci_le_write_suggested_default_data_length(251, 2120);
   PRINTF("hci_le_write_suggested_default_data_length(): 0x%02x\r\n", ret);

  /* GATT Init */
  ret = aci_gatt_srv_init();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gatt_srv_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_srv_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00, strlen(name), STATIC_RANDOM_ADDR, &service_handle,  
                     &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gap_init() --> SUCCESS\r\n");
  }
  
  aci_hal_read_config_data(0x80, &addr_len, address);
  PRINTF("Static random address: ");
  PRINT_ADDDRESS(address);
  PRINTF("\r\n");

  /* Set the device name */
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle,
                                   0, strlen(name), (uint8_t *)name);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in Gatt Update characteristic value 0x%02x\r\n", ret);
    return ret;
  } else {
    PRINTF("aci_gatt_update_char_value() --> SUCCESS\r\n");
  }
  
  aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
  
  ServerInit();
  
  
  aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_NOT_REQUIRED, 0, 0, 7, 16, DONOT_USE_FIXED_PIN_FOR_PAIRING, 0);
  
  for(int i = 0; i < MAX_NUM_SLAVES; i++)
    SlaveInit(i);
  
  for(int i = 0; i < MAX_NUM_MASTERS; i++)
    MasterInit(i);
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED, SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, ACTIVE_SCAN, SCAN_INTERVAL, SCAN_WINDOW);
  
  PRINTF("Scan configuration %02X\n", ret);
    
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, CONN_INTERVAL_MIN, CONN_INTERVAL_MAX, 0, SUPERVISION_TIMEOUT, CE_LENGTH, CE_LENGTH);
  
  PRINTF("Connection configuration %02X\n", ret);
  
#if MAX_NUM_MASTERS
  
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
  
  PRINTF("Advertising configuration %02X\n", ret);
  
  memcpy(adv_data + 5, name, sizeof(name));
  ret = aci_gap_set_advertising_data(0x00, // Advertising handle
                                     0x03, // Complete data
                                     sizeof(adv_data), adv_data);
  
  PRINTF("Set advertising data %02X\n", ret);
  
  Advertising_Set_Parameters.Advertising_Handle = 0;
  Advertising_Set_Parameters.Duration = 0;
  Advertising_Set_Parameters.Max_Extended_Advertising_Events = 0;
  
  ret = aci_gap_set_advertising_enable(ENABLE, 1, &Advertising_Set_Parameters);
  
  PRINTF("Advertising enable %02X\n", ret);
  
  APP_FLAG_SET(ADVERTISING);
#endif  
  
  debounce_timer.callback = DebounceTimeoutCB;
  write_timer.callback = SendDataCB;
  
#if RESTART_ADV_TIMEOUT_MS && MAX_NUM_MASTERS
  restart_adv_timer.callback = RestartAdvTimeoutCB;
#endif
  
#if RESTART_SCAN_TIMEOUT_MS
  restart_scan_timer.callback = RestartScanTimeoutCB;
#endif
  
  HAL_VTIMER_StartTimerMs(&write_timer, WRITE_INTERVAL_MS);
  
  STATE_TRANSITION(STATE_NORMAL, SUBSTATE_INIT);
  
  return BLE_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name  : App_SleepMode_Check.
* Description    : Check if the device can go to sleep. See sleep.h
* Input          : Requested sleep mdoe.
* Return         : Allowed sleep mode
*******************************************************************************/
#if SLEEP_ENABLED
PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level)
{
  if(BSP_COM_TxFifoNotEmpty() || BSP_COM_UARTBusy())
    return POWER_SAVE_LEVEL_RUNNING;
  
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}
#endif


/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick(void)
{  
  if(debounce_timeout_occurred && BSP_PB_GetState(BSP_PUSH1)==1){
    
    APP_FLAG_SET(BUTTON_PRESSED);
    
    BSP_LED_On(BSP_LED3);
    
    debounce_timeout_occurred = FALSE;
    HAL_VTIMER_StartTimerMs(&debounce_timer, DEBOUNCE_TIMEOUT_MS);        
  }
  
  DeviceStateMachine();
  
  PerSlaveStateMachine();
  
}/* end APP_Tick() */

void DebounceTimeoutCB(void *param)
{
  debounce_timeout_occurred = TRUE;
}

void SendDataCB(void *param)
{
  tBleStatus ret;  
  static uint32_t counter = 0;  
  uint8_t data_sent = FALSE;
  uint8_t data[CHARACTERISTIC_LEN] = {0};
  
  HOST_TO_LE_32(data,counter);
  
  for(int i = 0; i < MAX_NUM_SLAVES; i++){    
    if(address_is_set(slaves[i].address) && slaves[i].rx_handle){
      data_sent = TRUE;
      ret = aci_gatt_write_without_resp(slaves[i].conn_handle, slaves[i].rx_handle+1, sizeof(data), data);
      if(ret == BLE_STATUS_SUCCESS)
        PRINTF("Data sent to slave %d (%d) 0x%04X\n", i, counter, slaves[i].conn_handle);
      else
        PRINTF("Error sending data to slave %d: 0x%02X (handle 0x%04X)\n", i, ret, slaves[i].conn_handle);
    }
  }
  
  if(data_sent)  
    counter++;
  
  HAL_VTIMER_StartTimerMs(&write_timer, WRITE_INTERVAL_MS);
}

#if RESTART_ADV_TIMEOUT_MS
void RestartAdvTimeoutCB(void *param)
{  
  Advertising_Set_Parameters_t Advertising_Set_Parameters;
  tBleStatus ret;
  
  // Start again advertising
  Advertising_Set_Parameters.Advertising_Handle = 0;
  Advertising_Set_Parameters.Duration = 0;
  Advertising_Set_Parameters.Max_Extended_Advertising_Events = 0;         
  ret = aci_gap_set_advertising_enable(ENABLE, 1, &Advertising_Set_Parameters);
  
  PRINTF("Advertising enable %02X\n", ret);
  
  APP_FLAG_SET(ADVERTISING);  
}
#endif

#if RESTART_SCAN_TIMEOUT_MS
void RestartScanTimeoutCB(void *param)
{  
  
  if(StartAutoConnection()==BLE_STATUS_SUCCESS){
    APP_FLAG_SET(SCANNING);
  }
  else{
    // Retry later
    HAL_VTIMER_StartTimerMs(&restart_scan_timer, RESTART_SCAN_TIMEOUT_MS);
  }
}
#endif

/* ***************** BlueNRG-LP Stack Callbacks ********************************/


void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{  
  PRINTF_DBG2("hci_le_connection_complete_event, handle: 0x%04X, Status %d\r\n", Connection_Handle, Status);
   
   if(Status == 0){
     
     client_hci_le_connection_complete_event(Connection_Handle, Role, Peer_Address_Type, Peer_Address);
     
     server_hci_le_connection_complete_event(Connection_Handle, Role, Peer_Address_Type, Peer_Address);
     
   }   
   else if(Status == BLE_ERROR_UNKNOWN_CONNECTION_ID){
     PRINTF_DBG2("Connection canceled.\r\n");
   }

}/* end hci_le_connection_complete_event() */


void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{
  
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  int i;
  
  PRINTF("hci_disconnection_complete_event, Status 0x%02X, Handle 0x%04X, Reason 0x%02X\n", Status, Connection_Handle, Reason);
  
  if(Status != 0){
    return;
  }
    
  i = get_slave_index_from_conn_handle(Connection_Handle);
  
  if(i >= 0){
    
    PRINTF("Disconnected from slave ");
    PRINT_ADDDRESS(slaves[i].address);
    PRINTF("\n");
    
    if(device.state == STATE_PAIRING && slaves[i].is_in_pairing_mode){
      // The device that was trying to pair with is disconnected
      STATE_TRANSITION(STATE_NORMAL, SUBSTATE_INIT);
    }
    else if(device.state == STATE_NORMAL && device.substate == SUBSTATE_WAITING_CONN){
      // One of the devices disconnected
      // STATE_TRANSITION called also in  aci_gap_pairing_complete_event, but it may
      // happen that the event is not raised if the device has been disconected before pairing is complete
      STATE_TRANSITION(STATE_NORMAL, SUBSTATE_INIT);
    }
    else {
      PRINTF("No change. STATE (%d,%d)\n",device.state, device.substate);
    }
    
    if(Reason == BLE_ERROR_CONNECTION_FAILED_TO_ESTABLISH){
      
      BlacklistHit(slaves[i].address_type, slaves[i].address);
    }
    else {
      // Reset blacklist status
      BlacklistReset();
    }
    
    SlaveInit(i);
    
    num_connected_slaves--;
    PRINTF("Connected slaves: %d\n", num_connected_slaves);
    
    return;
  }
  
#if MAX_NUM_MASTERS
  
  i = get_master_index_from_conn_handle(Connection_Handle);
  
  if(i >= 0){ // If we arrive here, this condition should always be true
    
    tBleStatus ret;    
    Advertising_Set_Parameters_t Advertising_Set_Parameters;
    
    PRINTF("Disconnected from master ");
    PRINT_ADDDRESS(masters[i].address);
    PRINTF(", status 0x%02X, reason 0x%02X\r\n", Status, Reason);
    
    MasterInit(i);
    
    num_connected_masters--;    
    
    PRINTF("Connected masters: %d\n", num_connected_masters);
    
    // Start again advertising
    Advertising_Set_Parameters.Advertising_Handle = 0;
    Advertising_Set_Parameters.Duration = 0;
    Advertising_Set_Parameters.Max_Extended_Advertising_Events = 0;         
    ret = aci_gap_set_advertising_enable(ENABLE, 1, &Advertising_Set_Parameters);
    
    PRINTF("Advertising enable %02X\n", ret);
    
    APP_FLAG_SET(ADVERTISING);
  }
  else {
    // We should not arrive here
    PRINTF("Disconnection: unexpected handle\n");
  }
#endif
  
}/* end hci_disconnection_complete_event() */

void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  int slave_index;
  
  STATE_TRANSITION(STATE_NORMAL, SUBSTATE_INIT);
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);  
  if(slave_index < 0) // This should not happen
    return;
  
  slaves[slave_index].is_in_pairing_mode = FALSE;
  
  if(Status != 0){
    PRINTF("Pairing failed. Status %02X, Reason: %02X\r\n", Status, Reason);
    slaves[slave_index].state = DONE;
    slaves[slave_index].resume_state = IDLE;
    slaves[slave_index].pairing_state = PAIRING_IDLE;
    
    aci_gap_remove_bonded_device(slaves[slave_index].address_type, slaves[slave_index].address);
    
    aci_gap_terminate(slaves[slave_index].conn_handle, BLE_ERROR_TERMINATED_REMOTE_USER);
    
    return;
  }
  
  slaves[slave_index].state = slaves[slave_index].resume_state;
  slaves[slave_index].resume_state = IDLE;
  slaves[slave_index].pairing_state = PAIRING_DONE;
  
  PRINTF("Pairing complete (slave %d).\r\n", slave_index);
}

void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                 uint16_t Attribute_Handle,
                                 uint8_t Attribute_Value_Length,
                                 uint8_t Attribute_Value[])
{ 
  int i;
  
  i = get_slave_index_from_conn_handle(Connection_Handle);  

  if(i < 0){ // This should not happen
    return;
  }
 
  if(Attribute_Handle == slaves[i].tx_handle + 1)
  {
    uint32_t counter;
    
    counter = LE_TO_HOST_32(Attribute_Value);
    
    PRINTF("Notification from slave %d: %d (%d bytes)\n", i, counter, Attribute_Value_Length);
    
  }
  
}
                                                                                              
void aci_att_clt_read_by_group_type_resp_event(uint16_t Connection_Handle,
                                           uint8_t Attribute_Data_Length,
                                           uint8_t Data_Length,
                                           uint8_t Attribute_Data_List[])
{
  int slave_index;
  
  //PRINTF_DBG2("aci_att_clt_read_by_group_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);  
  if(slave_index < 0) // This should not happen
    return;
  
  switch(slaves[slave_index].state){
    
  case DISCOVERING_SERVICES:
    if(Attribute_Data_Length == 20){ // Only 128bit UUIDs
      for(int i = 0; i < Data_Length; i += Attribute_Data_Length){
        if(memcmp(&Attribute_Data_List[i+4],Serial Port_service_uuid,16) == 0){
          memcpy(&slaves[slave_index].Serial Port_serv_start_handle, &Attribute_Data_List[i], 2);
          memcpy(&slaves[slave_index].Serial Port_serv_end_handle, &Attribute_Data_List[i+2], 2);
          PRINTF("Slave %d, Serial Port service handles: 0x%04X 0x%04X\r\n", slave_index, slaves[slave_index].Serial Port_serv_start_handle, slaves[slave_index].Serial Port_serv_end_handle);
        }
      }
    }
    break;
  default:
	break;
  }
  
}

void print_uuid(uint8_t *uuid)
{
  for(int i = 0; i < 16; i++)
    PRINTF("%02X",uuid[i]);
}

void aci_att_clt_read_by_type_resp_event(uint16_t Connection_Handle,
                                     uint8_t Handle_Value_Pair_Length,
                                     uint8_t Data_Length,
                                     uint8_t Handle_Value_Pair_Data[])
{
  int slave_index;
  uint16_t handle;
  
  //PRINTF_DBG2("aci_att_clt_read_by_type_resp_event, Connection Handle: 0x%04X\r\n", Connection_Handle);
  
  slave_index = get_slave_index_from_conn_handle(Connection_Handle);
  if(slave_index < 0) // This should not happen
    return;
  
  switch(slaves[slave_index].state){
  case DISCOVERING_CHAT_CHAR:
    for(int i = 0; i < Data_Length; i += Handle_Value_Pair_Length){
      if(Handle_Value_Pair_Length == 21){ // 128-bit UUID
        handle = LE_TO_HOST_16(&Handle_Value_Pair_Data[i]);
        print_uuid(&Handle_Value_Pair_Data[i+5]);
        if(memcmp(&Handle_Value_Pair_Data[i+5], Serial Port_TX_char_uuid, 16) == 0){
          slaves[slave_index].tx_handle = handle;
          PRINTF("TX Char handle for slave %d: 0x%04X\r\n", slave_index, handle);
        }
        else if(memcmp(&Handle_Value_Pair_Data[i+5], Serial Port_RX_char_uuid, 16) == 0){
          slaves[slave_index].rx_handle = handle;
           PRINTF("RX Char Handle for slave %d: 0x%04X\r\n", slave_index, handle);
        }
      }
    }
    break;    
  }
}

#define INSUFFICIENT_ENCRYPTION 0x0F

void aci_gatt_clt_error_resp_event(uint16_t Connection_Handle,
                               uint8_t Req_Opcode,
                               uint16_t Attribute_Handle,
                               uint8_t Error_Code)
{
  int i;
  
  //PRINTF_DBG2("aci_gatt_clt_error_resp_event %04X %02X %04X %02X\n", Connection_Handle, Req_Opcode, Attribute_Handle, Error_Code);
  
  //PRINTF_DBG2("aci_gatt_clt_error_resp_event.\r\n");
  
  i = get_slave_index_from_conn_handle(Connection_Handle);  
  if(i < 0) // This should not happen
    return;
  
  if(Error_Code == INSUFFICIENT_ENCRYPTION){
    // Start pairing
    StartPairing(i, Connection_Handle, slaves[i].state - 1); // After pairing go one state back
    return;
  }  
}

void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  int i;
  
  i = get_slave_index_from_conn_handle(Connection_Handle);
  
  if(i < 0) // This should not happen
    return;
  
  if(Error_Code != BLE_STATUS_SUCCESS){
    PRINTF_DBG2("Procedure terminated with error 0x%02X (0x%04X).\r\n", Error_Code, slaves[i].conn_handle);
    slaves[i].state = DONE;
    return;
  }
  
  switch(slaves[i].state){    
  case EXCHANGING_CONFIG:
    PRINTF("Configuration exchanged (0x%04X).\r\n", slaves[i].conn_handle);    
    slaves[i].state = START_SERVICE_DISCOVERY;
    break;    
  case DISCOVERING_SERVICES:
    PRINTF_DBG2("Discovering services ended (0x%04X).\r\n", slaves[i].conn_handle);
    if(slaves[i].Serial Port_serv_start_handle != 0)
      slaves[i].state = START_CHAT_CHAR_DISCOVERY;
    else
      slaves[i].state = DONE;
    break;    
  case DISCOVERING_CHAT_CHAR:
    PRINTF_DBG2("Discovering Serial Port Service characteristics ended (0x%04X).\r\n", slaves[i].conn_handle);
    if(slaves[i].tx_handle != 0)
      slaves[i].state = ENABLE_TX_CHAR_NOTIFICATIONS;
    else 
      slaves[i].state = DONE;
    break;
  case ENABLING_TX_CHAR_NOTIFICATIONS:
    PRINTF("Notifications for TX Charac enabled (0x%04X).\r\n", slaves[i].conn_handle);
    slaves[i].state = DONE;
    break;
  default:
	break;
  }
}

void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
}

void hci_le_advertising_report_event(uint8_t Num_Reports,
                                     Advertising_Report_t Advertising_Report[])
{
  uint8_t AD_len, AD_type;
  uint8_t i = 0;
  tBleStatus ret;
  
  while(i < Advertising_Report[0].Data_Length){
    AD_len = Advertising_Report[0].Data[i];
    AD_type = Advertising_Report[0].Data[i+1];    
    if(AD_type == AD_TYPE_128_BIT_UUID_SERVICE_DATA){
      // Search for Serial Port service UUID
      if(memcmp(&Advertising_Report[0].Data[i+2], Serial Port_service_uuid, sizeof(Serial Port_service_uuid))==0 && Advertising_Report[0].Data[i+18] == 0){
        // Device found!
        aci_gap_terminate_proc(GAP_GENERAL_CONNECTION_ESTABLISHMENT_PROC);
        ret = aci_gap_create_connection(LE_1M_PHY_BIT, Advertising_Report[0].Address_Type, Advertising_Report[0].Address);        
        PRINTF("aci_gap_create_connection %02X\r\n", ret);
        return;
      }
    }
    i += AD_len+1;
  }
  
}

void hci_le_extended_advertising_report_event(uint8_t Num_Reports,
                                              Extended_Advertising_Report_t Extended_Advertising_Report[])
{
  Advertising_Report_t Advertising_Report;
  Advertising_Report.Address_Type = Extended_Advertising_Report[0].Address_Type;
  memcpy(Advertising_Report.Address, Extended_Advertising_Report[0].Address, 6);
  Advertising_Report.Data_Length = Extended_Advertising_Report[0].Data_Length;
  Advertising_Report.Data = Extended_Advertising_Report[0].Data;
  Advertising_Report.RSSI = Extended_Advertising_Report[0].RSSI;
  hci_le_advertising_report_event(1, &Advertising_Report);
}

void aci_gap_bond_lost_event(void)
{
  //tBleStatus ret;
  
  PRINTF("aci_gap_bond_lost_event\r\n");
  
//  for(int slave_index = 0; slave_index < MAX_NUM_SLAVES; slave_index++){
//    if(slaves[slave_index].pairing_state == PAIRING){
//      ret = aci_gap_send_pairing_req(slaves[slave_index].conn_handle, 1);
//      PRINTF("Force rebond %02X\r\n", ret);
//    }
//  }
}

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  PRINTF("hci_le_data_length_change_event handle: 0x%04X, MaxTxOctets: %d, MaxTxTime: %d, MaxRxOctets: %d, MaxRxTime: %d.\r\n", Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t RX_MTU)
{
  PRINTF("aci_att_exchange_mtu_resp_event, handle: 0x%04X, RX_MTU: %d \r\n", Connection_Handle, RX_MTU);
}
