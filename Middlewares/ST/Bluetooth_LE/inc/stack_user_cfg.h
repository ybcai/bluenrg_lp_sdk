/**
  ******************************************************************************
  * @file    stack_user_cfg.h
  * @author  AMS - RF Application team
  * @version V1.0.0
  * @date    08 April 2019
  * @brief   BLE stack modular configuration options header file
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
  * <h2><center>&copy; COPYRIGHT STMicroelectronics</center></h2>
  ******************************************************************************
* \section BLE_Config BLE stack configuration options  

    - In order to configure the BLE stack v3.0 or later the following options are available:

       - BLE stack full configuration: all available modules are included. 
         - User is requested to add the following preprocessor option on project configuration: 

           BLE_STACK_FULL_CONF

       - BLE stack basic configuration: no Controller Privacy, no LE Secure Connections, no Master GAP role,
         no Data Length Extension, no LE 2M PHY, no LE CODED PHY, no Extended Advertising
         - User is requested to add the following preprocessor option on project configuration: 

           BLE_STACK_BASIC_CONF 

       - BLE stack configuration with Slave and Data Length Extension:

           BLE_STACK_SLAVE_DLE_CONF 

       - BLE stack configuration with Slave, Data Length Extension, LE 2M PHY and LE CODED PHY.

           BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF  
	   

**/
#ifndef _STACK_USER_CFG_H_
#define _STACK_USER_CFG_H_

#include <bluenrg_lp_api.h>
#include <system_util.h>
#include <stdint.h>

/* --------------------- BLE stack configuration options --------------------------------------------------- */


/** @defgroup BLE_Stack_Config BLE Stack Modular Configuration
 *  @brief Constants to select modular configuration. 
 *  @li BLE_STACK_BASIC_CONF: only basic features are enabled (no Controller Privacy, no LE Secure Connections,
    no Master GAP role, no Data Length Extension)
 *  @li BLE_STACK_FULL_CONF: all features are enabled
 *  @li BLE_STACK_SLAVE_DLE_CONF : Slave + Data Length Extension enabled (if available)
 *  @li BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF : Slave + Data Length Extension enabled  + PHY 2M/Coded PHY (if available)
 *
 *  Uncomment one of the lines or define the configuration in your toolchain compiler preprocessor.
 *  @note It is mandatory to define one (and only one) of the options.
 *  @{
 */

/* #define BLE_STACK_BASIC_CONF */
/* #define BLE_STACK_FULL_CONF */
/* #define BLE_STACK_SLAVE_DLE_CONF */
/* #define BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF */

/**
 * @}
 */

/* --------------------- BLE stack configuration options and associated module inclusion -------------------- */

/**
 * @brief Full BLE stack configuration
 */
#if defined BLE_STACK_FULL_CONF 

/* Controller Privacy module: ENABLED */
#define CONTROLLER_PRIVACY_ENABLED               (1U)

/* LE Secure Connections module: ENABLED */
#define SECURE_CONNECTIONS_ENABLED               (1U)

/* Master GAP role module: ENABLED */
#define CONTROLLER_MASTER_ENABLED                (1U)


#ifdef CONFIG_DEVICE_BLUENRG_LP

/* Data length extension module: ENABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED  (1U)

/* LE 2M and LE Coded PHYs module: ENABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED           (1U)

 /* Extended advertising and scanning: ENABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED           (1U)

/* L2CAP COS feature: ENABLED */
#define L2CAP_COS_ENABLED                         (1U)

#endif


#ifdef CONFIG_DEVICE_BLUENRG_2

/* Data length extension module: ENABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED  (1U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED           (0U)

 /* Extended advertising and scanning: ENABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED           (1U)

/* L2CAP COS feature: DISABLED */
#define L2CAP_COS_ENABLED                         (0U)

#endif


#ifdef CONFIG_DEVICE_BLUENRG_1

/* Data length extension module: DISABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED  (0U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED           (0U)

 /* Extended advertising and scanning: ENABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED           (1U)

/* L2CAP COS feature: DISABLED */
#define L2CAP_COS_ENABLED                         (0U)

#endif


/**
 * @brief Basic BLE stack configuration
 */
#elif defined BLE_STACK_BASIC_CONF

/* Controller Privacy module: DISABLED */
#define CONTROLLER_PRIVACY_ENABLED               (0U)

/* LE Secure Connections module: DISABLED */
#define SECURE_CONNECTIONS_ENABLED               (0U)

/* Master GAP role module: DISABLED */
#define CONTROLLER_MASTER_ENABLED                (0U)

/* Data length extension module: DISABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED (0U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED          (0U)

 /* Extended advertising and scanning: DISABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED          (0U)

/* L2CAP COS feature: DISABLED */
#define L2CAP_COS_ENABLED                        (0U)


/**
 * @brief BLE stack configuration: only slave with Data Length Extension
 */
#elif defined BLE_STACK_SLAVE_DLE_CONF

/* Controller Privacy module: DISABLED */
#define CONTROLLER_PRIVACY_ENABLED               (0U)

/* LE Secure Connections module: DISABLED */
#define SECURE_CONNECTIONS_ENABLED               (0U)

/* Master GAP role module: DISABLED */
#define CONTROLLER_MASTER_ENABLED                (0U)

/* L2C COS feature: DISABLED */
#define L2C_COS_ENABLED                          (0U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED          (0U)

 /* Extended advertising and scanning: DISABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED          (0U)

/* L2CAP COS feature: DISABLED */
#define L2CAP_COS_ENABLED                        (0U)


#if defined (CONFIG_DEVICE_BLUENRG_LP) || defined (CONFIG_DEVICE_BLUENRG_2)

/* Data length extension module: ENABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED (1U)

#elif defined  CONFIG_DEVICE_BLUENRG_1 /* BlueNRG-1 device doesn't support Data length extension */

 /* Data length extension module: DISABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED (0U)

#endif

/**
 * @brief BLE stack configuration: only slave with Data Length Extension + LE 2M/Coded PHY (for connections only)
 */
#elif defined BLE_STACK_SLAVE_DLE_LE_2M_CODED_CONF

/* Controller Privacy module: DISABLED */
#define CONTROLLER_PRIVACY_ENABLED               (0U)

/* LE Secure Connections module: DISABLED */
#define SECURE_CONNECTIONS_ENABLED               (0U)

/* Master GAP role module: DISABLED */
#define CONTROLLER_MASTER_ENABLED                (0U)

/* L2CAP COS feature: DISABLED */
#define L2CAP_COS_ENABLED                        (0U)

 /* Extended advertising and scanning: DISABLED */
#define CONTROLLER_EXT_ADV_SCAN_ENABLED          (0U)

#if defined (CONFIG_DEVICE_BLUENRG_LP)

/* Data length extension module: ENABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED (1U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED          (1U)

#else

 /* Data length extension module: DISABLED */
#define CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED (0U)

/* LE 2M and LE Coded PHYs module: DISABLED */
#define CONTROLLER_2M_CODED_PHY_ENABLED          (0U)

#endif



#else
#error "No BLE_STACK_*_CONF has been defined."
#endif


/* --------------------- BLE stack prototypes -------------------- */
/* CONTROLLER PRIVACY */
tBleStatus hci_le_remove_device_from_resolving_list_api(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6]);
tBleStatus hci_le_add_device_to_resolving_list_api(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Peer_IRK[16], uint8_t Local_IRK[16]);
tBleStatus hci_le_set_resolvable_private_address_timeout_api(uint16_t RpaTimeout);
tBleStatus hci_le_set_address_resolution_enable_api(uint8_t AddressResolutionEnable);
tBleStatus hci_le_read_peer_resolvable_address_api(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Peer_Resolvable_Address[6]);
tBleStatus hci_le_read_local_resolvable_address_api(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Local_Resolvable_Address[6]);
tBleStatus hci_le_read_resolving_list_size_api(uint8_t *resolving_List_Size);
tBleStatus hci_le_clear_resolving_list_api(void);
tBleStatus hci_le_set_privacy_mode_api(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Privacy_Mode);

uint32_t PRIV_controller_privacy_csr_ucfg(void);
uint32_t data_length_extension_csr_ucfg(void);
uint32_t phy_upd_csr_ucfg(void);
uint32_t secure_connections_csr_ucfg(void);
uint32_t master_csr_ucfg(void);
uint32_t l2c_cos_csr_ucfg(void);
void LL_priv_u8OfflineProcessing_ucfg(void);
void       LL_priv_u8OfflineProcessing(void);
/* PRIVACY related routines */
void LL_priv_vOwnPrivateAddress_ucfg(uint32_t *Own, uint32_t* Peer,uint8_t is_ISR);
void LL_priv_vOwnPrivateAddress(uint32_t *Own, uint32_t* Peer,uint8_t is_ISR);
void LL_priv_vPeerPrivateAddress_ucfg(uint32_t *Peer, uint8_t is_ISR);
void LL_priv_vPeerPrivateAddress(uint32_t *Peer, uint8_t is_ISR);
uint8_t LL_priv_u32ProcessAdvPacket_ucfg(uint32_t* ReceivedTransmitAddress, uint8_t PduType, uint32_t* PeerIDAddress,
    uint8_t AdvertisingEventproperties, uint8_t filterPolicy, uint32_t* peer_addr, uint16_t advertisinghandle);
uint8_t LL_priv_u32ProcessAdvPacket(uint32_t* ReceivedTransmitAddress, uint8_t PduType, uint32_t* PeerIDAddress,
    uint8_t AdvertisingEventproperties, uint8_t filterPolicy, uint32_t* peer_addr, uint16_t advertisinghandle);
uint8_t LL_priv_u32ProcessScanPacketTx_ucfg(uint32_t* ReceivedTransmitAddress, uint32_t* PeerIDaddress, uint8_t isScanner, uint8_t* initPeerAddr,
    uint8_t initPeerAddrType, uint8_t LocalFilterPolicy, uint8_t resolvedirect,uint8_t* DatabaseEntry);
uint8_t LL_priv_u32ProcessScanPacketTx(uint32_t* ReceivedTransmitAddress, uint32_t* PeerIDaddress, uint8_t isScanner, uint8_t* initPeerAddr,
    uint8_t initPeerAddrType, uint8_t LocalFilterPolicy, uint8_t resolvedirect,uint8_t* DatabaseEntry);
uint8_t LL_priv_u32ProcessScanPacketRx_ucfg(uint32_t* ReceivedReceiveAddress, uint8_t LocalFilterPolicy, uint32_t* own_addr_used, uint8_t PointerToDataBase, uint8_t resolvedirect);
uint8_t LL_priv_u32ProcessScanPacketRx(uint32_t* ReceivedReceiveAddress, uint8_t LocalFilterPolicy, uint32_t* own_addr_used, uint8_t PointerToDataBase, uint8_t resolvedirect);
void LL_priv_SynchPeerIDList_And_WhiteIDList_ucfg(void);
void LL_priv_SynchPeerIDList_And_WhiteIDList(void);

void       PRIV_vGeneratePrivateAddress(uint8_t *IRK, uint32_t *PrivateAddress,uint32_t Hash,uint8_t is_ISR);
void       PRIV_vGenerate_hci_le_enhanced_connection_complete_event(uint8_t role, uint32_t *peer_addr, uint8_t status, uint16_t connHandle,
					                                                          uint16_t interval, uint16_t latency, uint32_t supertimeout, uint16_t CONN_Sca);
BOOL       PRIV_vGenerate_hci_le_direct_advertising_report_event(uint8_t scanFilterPolicy, BOOL adv_direct_add_rpa, uint8_t scantype, 
                                                                 uint8_t peer_addr_type, uint8_t* p1, uint8_t rssi, uint8_t length);
void LL_priv_Init_ResolvedPart_ucfg(void);
void LL_priv_Init_ResolvedPart(void);
uint8_t hci_le_check_own_address_type_max_value_ucfg(void);
tBleStatus GAP_enable_controller_privacy(uint8_t *gapRole, uint8_t *numServiceRec);
tBleStatus LL_Encrypt_Plain_Data(uint8_t *key, uint8_t *textData, uint8_t *textDataOut, BOOL inISR);

/*Extended advertiser related routines*/
void LL_eadv_start_extended(void *pointer);
void LL_eadv_start_extended_ucfg(void *pointer);

/*Extended scanner related routines*/
tBleStatus LL_Set_Extented_Scan_Enable(void *pExtScanEnable);
void LL_escan_ScanStopCancel(void);
void LL_escan_ScanStart(void);
BOOL LL_escan_EauxIsr(void *pointer);
//void SCAN_INIT(void);
void LL_escan_ExtendedAdvReceiveProcess(void *linkpointer, uint8_t phyindex,
                                                            BOOL *ReportTobeSend, BOOL *bypass, uint8_t *bypassauxslot,
                                                            uint8_t *SID_Flag, uint8_t *SID, uint8_t *TxPower,
                                                            void *EADVEventPointer_p, BOOL AnonymousFlag, BOOL
                                                            ReceiveAddressPresentFlag, uint32_t *Temp);

void LL_escan_push_hci_le_extended_advertising_report_event_ucfg(void* pointer,
                                                                 uint8_t len,uint8_t offset,
                                                                 uint8_t event_type);
tBleStatus LL_Set_Extented_Scan_Enable_ucfg(void *pExtScanEnable);
void LL_escan_ScanStopCancel_ucfg(void);
void LL_escan_ScanStart_ucfg(void);
BOOL LL_escan_EauxIsr_ucfg(void *pointer);
void SCAN_INIT_ucfg(void);
void LL_escan_extended_adv_receive_process_ucfg(void *linkpointer, 
                                           uint8_t phyindex,
										   BOOL anonymous_flag, 
										   BOOL receive_address_present_flag, 
										   uint32_t *received_receive_address);
void LL_escan_extended_adv_receive_process(void *linkpointer, 
                                           uint8_t phyindex,
										   BOOL anonymous_flag, 
										   BOOL receive_address_present_flag, 
										   uint32_t *received_receive_address);

void LL_escan_push_hci_le_extended_advertising_report_event(void* pointer,
                                                            uint8_t len,uint8_t offset,
                                                            uint8_t event_type);
tBleStatus LL_eadv_max_supported_data_check_ucfg(uint16_t Data_Length,
						void *linkpointer);
tBleStatus LL_eadv_max_supported_data_check(uint16_t Data_Length,
						void *linkpointer);
/* DATA LENGTH EXTENSION */
tBleStatus hci_le_set_data_length_api(uint16_t Connection_Handle, uint16_t TxOctets, uint16_t TxTime);
tBleStatus hci_le_read_suggested_default_data_length_api(uint16_t *SuggestedMaxTxOctets, uint16_t *SuggestedMaxTxTime);
tBleStatus hci_le_write_suggested_default_data_length_api(uint16_t SuggestedMaxTxOctets, uint16_t SuggestedMaxTxTime);
tBleStatus hci_le_read_maximum_data_length_api(uint16_t *supportedMaxTxOctets, uint16_t *supportedMaxTxTime,
                                               uint16_t *supportedMaxRxOctets, uint16_t *supportedMaxRxTime);

void       LL_cpe_init_length_update(void);
void       Data_Len_Update_Offline_Processing(void *params, uint32_t ctrl_flds);
void       LL_conn_upd_max_tx_time_coded(void *params);
void       LL_conn_upd_data_length_change_event(void *params);
void       LL_phy_upd_compute_data_PDU_length_params(void *params);
tBleStatus ll_write_supported_data_ucfg(uint16_t Supported_Max_Tx_Octets, uint16_t Supported_Max_Tx_Time,
                                        uint16_t Supported_Max_Rx_Octets, uint16_t Supported_Max_Rx_Time);
tBleStatus ll_write_supported_data(uint16_t Supported_Max_Tx_Octets, uint16_t Supported_Max_Tx_Time,
                                   uint16_t Supported_Max_Rx_Octets, uint16_t Supported_Max_Rx_Time);


/* CONTROLLER MASTER */
tBleStatus aci_gap_set_scan_configuration_api(uint8_t duplicate_filtering,
                                              uint8_t scanning_filter_policy,
                                              uint8_t phy,
                                              uint8_t scan_type,
                                              uint16_t scan_interval,
                                              uint16_t scan_window);

tBleStatus aci_gap_set_connection_configuration_api(uint8_t phy,
                                             uint16_t conn_interval_min,
                                             uint16_t conn_interval_max,
                                             uint16_t conn_latency,
                                             uint16_t supervision_timeout,
                                             uint16_t minimum_ce_length,
                                             uint16_t maximum_ce_length);

tBleStatus aci_gap_create_connection_api(uint8_t Initiating_PHY,
                                         uint8_t Peer_Address_Type,
                                         uint8_t Peer_Address[6]);

tBleStatus aci_gap_start_procedure_api(uint8_t procedure_code,
                                       uint8_t phys,
                                       uint16_t duration,
                                       uint16_t period);

tBleStatus aci_gap_discover_name_api(uint8_t PHYs,
                                     uint8_t Peer_Address_Type,
                                     uint8_t Peer_Address[6]);

tBleStatus aci_gap_terminate_proc_api(uint8_t Procedure_Code);

tBleStatus aci_gap_start_connection_update_api(uint16_t Connection_Handle,
                                               uint16_t Conn_Interval_Min,
                                               uint16_t Conn_Interval_Max,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint16_t Minimum_CE_Length,
                                               uint16_t Maximum_CE_Length);

tBleStatus aci_gap_send_pairing_req_api(uint16_t Connection_Handle, uint8_t Force_Rebond);

tBleStatus aci_l2cap_connection_parameter_update_resp_api(uint16_t Connection_Handle,
                                                          uint16_t Conn_Interval_Min,
                                                          uint16_t Conn_Interval_Max,
                                                          uint16_t Slave_latency,
                                                          uint16_t Timeout_Multiplier,
                                                          uint16_t Minimum_CE_Length,
                                                          uint16_t Maximum_CE_Length,
                                                          uint8_t  Identifier,
                                                          uint8_t  Accept);

void GAP_master_connection_complete_handler_ucfg(uint8_t status, uint16_t connectionHandle);
void GAP_master_connection_complete_handler(uint8_t status, uint16_t connectionHandle);
BOOL GAP_parse_connectable_advertising_report_ucfg(uint8_t *adv_buf, BOOL extended);
BOOL GAP_parse_connectable_advertising_report(uint8_t *adv_buf, BOOL extended);

BOOL GAP_parse_advertising_report_ucfg(uint8_t *adv_buf, BOOL extended);
BOOL GAP_parse_advertising_report(uint8_t *adv_buf, BOOL extended);

tBleStatus GAP_enable_disable_scan_ucfg(BOOL enable, uint8_t duplicate_filtering);
tBleStatus GAP_enable_disable_scan_ext(BOOL enable, uint8_t duplicate_filtering);
tBleStatus GAP_enable_disable_scan_legacy(BOOL enable, uint8_t duplicate_filtering);
tBleStatus GAP_create_connection(uint8_t *peer_address, uint8_t peer_address_type, uint8_t own_address_type, uint8_t initiator_filter_policy, uint8_t phys);
tBleStatus GAP_set_scan_parameters(uint8_t own_address_type, uint8_t phys);
tBleStatus GAP_discover_peer_name_ucfg(void);
tBleStatus GAP_add_device_to_white_and_resolving_list_ucfg(uint8_t lists, uint8_t addr_type,uint8_t addr[6]);
tBleStatus GAP_add_device_to_white_and_resolving_list_full(uint8_t lists, uint8_t addr_type, uint8_t addr[6]);
tBleStatus GAP_clear_white_and_resolving_list_ucfg(uint8_t lists);
tBleStatus GAP_clear_white_and_resolving_list_full(uint8_t lists);

tBleStatus hci_le_create_connection_api(uint16_t LE_Scan_Interval,
                                        uint16_t LE_Scan_Window,
                                        uint8_t  Initiator_Filter_Policy,
                                        uint8_t  Peer_Address_Type,
                                        uint8_t  Peer_Address[6],
                                        uint8_t  Own_Address_Type,
                                        uint16_t Conn_Interval_Min,
                                        uint16_t Conn_Interval_Max,
                                        uint16_t Conn_Latency,
                                        uint16_t Supervision_Timeout,
                                        uint16_t Minimum_CE_Length,
                                        uint16_t Maximum_CE_Length);

tBleStatus hci_le_create_connection_cancel_api(void);

tBleStatus hci_le_connection_update_api(uint16_t Connection_Handle,
                                        uint16_t Conn_Interval_Min,
                                        uint16_t Conn_Interval_Max,
                                        uint16_t Conn_Latency,
                                        uint16_t Supervision_Timeout,
                                        uint16_t Minimum_CE_Length,
                                        uint16_t Maximum_CE_Length);

tBleStatus hci_le_set_scan_parameters_api(uint8_t  LE_Scan_Type,
                                          uint16_t LE_Scan_Interval,
                                          uint16_t LE_Scan_Window,
                                          uint8_t  Own_Address_Type,
                                          uint8_t  Scanning_Filter_Policy);

tBleStatus hci_le_set_scan_enable_api(uint8_t LE_Scan_Enable, uint8_t Filter_Duplicates);

tBleStatus hci_le_start_encryption_api(uint16_t Connection_Handle, uint8_t Random_Number[8],
                                       uint16_t Encrypted_Diversifier, uint8_t Long_Term_Key[16]);

tBleStatus hci_le_set_host_channel_classification_api(uint8_t LE_Channel_Map[5]);

void GAP_DiscProcTimeoutcb(uint8_t timer_id);
uint32_t   cancel_connect_master(uint8_t slaveno);
void       isr_event_handler_scan_data(void *params);
tBleStatus LL_Start_Encryption(uint16_t connHandle, uint8_t *randNum, uint8_t *ediv, uint8_t *ltk);
tBleStatus GAP_check_and_set_role(uint8_t *gapRole, uint8_t role);
tBleStatus GAP_discover_peer_name(void);
void       full_state_func_call_init(void);

/* SMP_MASTER */
tBleStatus smp_MI_Start_Encryption(void *params);
void       smp_Execute_Actions_wrt_Current_State_MI_excerpt(void *params);
tBleStatus smp_Process_Rx_Packet_wrt_Current_State_MI_excerpt(void *params);
void       smp_Execute_Actions_wrt_Current_State_sc_MI_excerpt_phase2as2(void *params);


/* SECURE CONNECTION */
tBleStatus aci_gap_passkey_input_api(uint16_t Connection_Handle, uint8_t Input_Type);
tBleStatus aci_gap_numeric_comparison_value_confirm_yesno_api(uint16_t Connection_Handle, uint8_t Confirm_Yes_No);  
tBleStatus hci_le_read_local_p256_public_key_api(void);
tBleStatus hci_le_generate_dhkey_api(uint8_t Remote_P256_Public_Key[64]);

void       PKA_Offline_Processing(void);
void       smp_sc_init(BOOL useDebugKey);
tBleStatus smp_sc_continue_on_exclusive_sc_pairing(void *params);
void       smp_sc_check_for_ecc_pk_generation_request(void);
tBleStatus smp_sc_generate_new_local_oob_data(void);
void       smp_sc_update_statistics_on_pairing_complete(void *params);
void       smp_sc_hci_le_read_local_p256_public_key_complete_evt_hndl(uint8_t status, uint8_t local_p256_public_key[64]);
void       smp_sc_hci_le_generate_dhkey_complete_evt_hndl(uint8_t status, uint8_t DHKey[32]);
tBleStatus smp_sc_MI_PairingResponse_Receive_sc_excerpt(void *params);
void       smp_sc_MI_Pairing_Phase_2AS1_Start_sc_excerpt(void *params);
void       smp_sc_SR_Pairing_Phase_2AS1_Start_sc_excerpt(void *params);
void       smp_sc_MI_PairingConfirm_Send_sc_excerpt(void *params);
tBleStatus smp_sc_MI_PairingRandom_Receive_sc_excerpt1(void *params);
void       smp_sc_MI_PairingRandom_Receive_sc_excerpt2(void *params);
void       smp_sc_SR_PairingConfirm_Receive_sc_excerpt(void *params);
void       smp_sc_SR_PairingRandom_Receive_sc_excerpt(void *params);
BOOL       smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase_1_to_2(void *params);
void       smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase2as2(void *params);
tBleStatus smp_Process_Rx_Packet_wrt_Current_State_sc_excerpt(void *params);
tBleStatus smp_Process_Rx_Packet_Exception_Cases_sc_excerpt(void *params);


/* L2CAP Connection Oriented Services/Channels */
tBleStatus aci_l2cap_cfc_connection_req_api(uint16_t Connection_Handle,
                                            uint16_t SPSM,
                                            uint16_t CID,
                                            uint16_t MTU,
                                            uint16_t MPS,
                                            uint8_t CFC_Policy,
                                            uint16_t RX_SDU_Buffer_Size,
                                            void * RX_SDU_Buffer);

tBleStatus aci_l2cap_cfc_connection_resp_api(uint16_t Connection_Handle,
                                             uint8_t Identifier,
                                             uint16_t CID,
                                             uint16_t MTU,
                                             uint16_t MPS,
                                             uint16_t Result,
                                             uint8_t CFC_Policy,
                                             uint16_t RX_SDU_Buffer_Size,
                                             void * RX_SDU_Buffer);

tBleStatus aci_l2cap_send_flow_control_credits_api(uint16_t Connection_Handle,
                                                   uint16_t CID,
                                                   uint16_t RX_Credits,
                                                   uint8_t CFC_Policy,
                                                   uint16_t *RX_Credit_Balance);

tBleStatus aci_l2cap_disconnect_api(uint16_t Connection_Handle,
                                    uint16_t CID);

tBleStatus aci_l2cap_transmit_sdu_data_api(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t SDU_Length,
                                           uint8_t SDU_Data[]);

tBleStatus aci_l2cap_extract_sdu_data_api(uint16_t Connection_Handle,
                                          uint16_t CID,
                                          uint16_t SDU_Data_Buffer_Size,
                                          void * SDU_Data_Buffer,
                                          uint16_t *SDU_Length);

tBleStatus L2C_cos_cfc_init(void);
void       L2C_cos_process_q(void);
tBleStatus L2C_cos_process_cfc_mode_command(void *params);
BOOL       L2C_cos_is_pdu_cframe_cfc_command_opcode(uint8_t opCode);
tBleStatus L2C_cos_le_frame_data_hndl(void *params);
void       L2C_cos_reset_any_pending_channel(uint16_t connection_handle);
void       L2C_cos_physical_link_disconnection_hndl(uint16_t connection_handle);


/* PHY UPDATE */
tBleStatus hci_le_read_phy_api(uint16_t Connection_Handle, uint8_t *TX_PHY, uint8_t *RX_PHY);
tBleStatus hci_le_set_default_phy_api(uint8_t ALL_PHYS, uint8_t TX_PHYS, uint8_t RX_PHYS);
tBleStatus hci_le_set_phy_api(uint16_t Connection_Handle, uint8_t ALL_PHYS, uint8_t TX_PHYS, uint8_t RX_PHYS, uint16_t PHY_options);
tBleStatus hci_le_enhanced_receiver_test_api(uint8_t RX_Frequency, uint8_t Phy, uint8_t Modulation_iIndex);
tBleStatus hci_le_enhanced_transmitter_test_api(uint8_t TX_Frequency, uint8_t Length_Of_Test_Data, uint8_t Packet_Payload, uint8_t Phy);

void       LL_cpe_init_phy_update(void);
void       LL_phy_upd_pending(uint8_t conn_idx);
tBleStatus LL_phy_update_init(void);
tBleStatus LL_phy_update_init_per_st_data(uint8_t conn_idx);

/* Advertising extension LL */
tBleStatus hci_le_extended_create_connection_api(uint8_t Initiating_Filter_Policy, uint8_t Own_Address_Type, uint8_t Peer_Address_Type, uint8_t Peer_Address[6], uint8_t Initiating_PHYs, Extended_Create_Connection_Parameters_t Extended_Create_Connection_Parameters[]);
tBleStatus hci_le_set_extended_scan_parameters_api(uint8_t Own_Address_Type, uint8_t Scanning_Filter_Policy, uint8_t Scanning_PHYs, Extended_Scan_Parameters_t Extended_Scan_Parameters[]);
tBleStatus hci_le_set_extended_scan_enable_api(uint8_t Enable, uint8_t Filter_Duplicates, uint16_t Duration, uint16_t Period);
tBleStatus hci_le_set_extended_advertising_parameters_api(uint8_t Advertising_Handle, uint16_t Advertising_Event_Properties, uint8_t Primary_Advertising_Interval_Min[3], uint8_t Primary_Advertising_Interval_Max[3],
                                                          uint8_t Primary_Advertising_Channel_Map, uint8_t Own_Address_Type, uint8_t Peer_Address_Type, uint8_t Peer_Address[6],
                                                          uint8_t Advertising_Filter_Policy, int8_t Advertising_Tx_Power, uint8_t Primary_Advertising_PHY, uint8_t Secondary_Advertising_Max_Skip,
                                                          uint8_t Secondary_Advertising_PHY, uint8_t Advertising_SID, uint8_t Scan_Request_Notification_Enable, int8_t *Selected_Tx_Power);
tBleStatus hci_le_set_advertising_set_random_address_api(uint8_t Advertising_Handle, uint8_t Advertising_Random_Address[6]);
tBleStatus hci_le_set_extended_advertising_enable_api(uint8_t Enable, uint8_t Number_of_Sets, Advertising_Set_Parameters_t Advertising_Set_Parameters[]);
tBleStatus hci_le_read_maximum_advertising_data_length_api(uint16_t *Maximum_Advertising_Data_Length);
tBleStatus hci_le_read_number_of_supported_advertising_sets_api(uint8_t *Num_Supported_Advertising_Sets);
tBleStatus hci_le_remove_advertising_set_api(uint8_t Advertising_Handle);
tBleStatus hci_le_clear_advertising_sets_api(void);
tBleStatus hci_le_set_periodic_advertising_parameters_api(uint8_t Advertising_Handle, uint16_t Periodic_Advertising_Interval_Min, uint16_t Periodic_Advertising_Interval_Max, uint16_t Periodic_Advertising_Properties);
tBleStatus hci_le_set_periodic_advertising_data_api(uint8_t Advertising_Handle, uint8_t Operation, uint8_t Advertising_Data_Length, uint8_t Advertising_Data[]);
tBleStatus hci_le_set_periodic_advertising_enable_api(uint8_t Enable, uint8_t Advertising_Handle);
uint32_t ext_adv_scan_enabled_ucfg(void);


/* GAP Advertising extension */
tBleStatus GAP_create_connection_ext(uint8_t *peer_address, uint8_t peer_address_type, uint8_t own_address_type, uint8_t initiator_filter_policy, uint8_t phys);
tBleStatus GAP_set_scan_parameters_ext(uint8_t own_address_type, uint8_t phys);
tBleStatus GAP_generate_extended_adv_rpa(uint8_t Advertising_Handle);
void GAP_extended_slave_connect_establish_check(void);
BOOL GAP_extended_lim_disc_timeout_handler(void);
void GAP_hci_le_advertising_set_terminated_evt_hndl(uint8_t status, uint8_t Advertising_Handle);
void Get_Random_Addr(uint8_t *randBdAddr);
void GAP_get_random_address(uint8_t *randBdAddr);

tBleStatus GAP_set_extended_advertising_configuration(uint8_t Advertising_Handle,
                                                      uint8_t Discoverability_Mode,
                                                      uint16_t Advertising_Event_Properties,
                                                      uint32_t Primary_Advertising_Interval_Min,
                                                      uint32_t Primary_Advertising_Interval_Max,
                                                      uint8_t Primary_Advertising_Channel_Map,
                                                      uint8_t Peer_Address_Type,
                                                      uint8_t Peer_Address[6],
                                                      uint8_t Advertising_Filter_Policy,
                                                      int8_t  Advertising_Tx_Power,
                                                      uint8_t Primary_Advertising_PHY,
                                                      uint8_t Secondary_Advertising_Max_Skip,
                                                      uint8_t Secondary_Advertising_PHY,
                                                      uint8_t Advertising_SID,
                                                      uint8_t Scan_Request_Notification_Enable);

tBleStatus GAP_set_advertising_configuration_ucfg(uint8_t Advertising_Handle,
                                                  uint8_t Discoverability_Mode,
                                                  uint16_t Advertising_Event_Properties,
                                                  uint32_t Primary_Advertising_Interval_Min,
                                                  uint32_t Primary_Advertising_Interval_Max,
                                                  uint8_t Primary_Advertising_Channel_Map,
                                                  uint8_t Peer_Address_Type,
                                                  uint8_t Peer_Address[6],
                                                  uint8_t Advertising_Filter_Policy,
                                                  int8_t Advertising_Tx_Power,
                                                  uint8_t Primary_Advertising_PHY,
                                                  uint8_t Secondary_Advertising_Max_Skip,
                                                  uint8_t Secondary_Advertising_PHY,
                                                  uint8_t Advertising_SID,
                                                  uint8_t Scan_Request_Notification_Enable);

tBleStatus GAP_set_scan_response_data_ucfg(uint8_t Advertising_Handle,
                                           uint16_t Scan_Response_Data_Length,
                                           uint8_t *Scan_Response_Data);

tBleStatus GAP_set_extended_scan_response_data(uint8_t Advertising_Handle,
                                               uint16_t Scan_Response_Data_Length,
                                               uint8_t* Scan_Response_Data_Ptr);

tBleStatus GAP_set_advertising_data_ucfg(uint8_t Advertising_Handle,
                                         uint8_t Operation,
                                         uint16_t Advertising_Data_Length,
                                         uint8_t *Advertising_Data);

tBleStatus GAP_set_extended_advertising_data(uint8_t Advertising_Handle,
                                             uint8_t Operation,
                                             uint16_t Advertising_Data_Length,
                                             uint8_t* Advertising_Data_Ptr);

tBleStatus GAP_set_extended_advertising_enable(uint8_t Enable,
                                               uint8_t Num_Of_Sets,
                                               Advertising_Set_Parameters_t* Advertising_Set_Parameters);

tBleStatus GAP_set_advertising_enable_ucfg(uint8_t Enable,
                                           uint8_t Num_Of_Sets,
                                           Advertising_Set_Parameters_t* Advertising_Set_Parameters);

tBleStatus GAP_suspend_resume_active_advertising_sets_extended(uint8_t resume);

tBleStatus GAP_set_controller_random_address_extended(uint8_t random_address[6]);

void GAP_hci_le_advertising_set_terminated_evt_hndl_ucfg(uint8_t status, uint8_t Advertising_Handle);

#endif /* _STACK_USER_CFG_H_ */

