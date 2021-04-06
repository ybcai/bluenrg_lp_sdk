/**
  ******************************************************************************
  * @file    stack_user_cfg.c
  * @author  AMS - RF Application team
  * @version V1.0.0
  * @date    19 February 2018
  * @brief   BLE stack modular configuration options file
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

  - NOTE: default configuration is BLE_STACK_BASIC_CONF
**/

#include "stack_user_cfg.h"

/* ---------------------------------------------------------------------------------------------------------- */
#ifdef CONTROLLER_EXT_ADV_SCAN_ENABLED
#if (CONTROLLER_EXT_ADV_SCAN_ENABLED == 1)

 tBleStatus hci_le_set_extended_advertising_parameters(uint8_t Advertising_Handle,
                                                      uint16_t Advertising_Event_Properties,
                                                      uint8_t Primary_Advertising_Interval_Min[3],
                                                      uint8_t Primary_Advertising_Interval_Max[3],
                                                      uint8_t Primary_Advertising_Channel_Map,
                                                      uint8_t Own_Address_Type,
                                                      uint8_t Peer_Address_Type,
                                                      uint8_t Peer_Address[6],
                                                      uint8_t Advertising_Filter_Policy,
                                                      int8_t Advertising_Tx_Power,
                                                      uint8_t Primary_Advertising_PHY,
                                                      uint8_t Secondary_Advertising_Max_Skip,
                                                      uint8_t Secondary_Advertising_PHY,
                                                      uint8_t Advertising_SID,
                                                      uint8_t Scan_Request_Notification_Enable,
                                                      int8_t *Selected_Tx_Power)
 {
    return  hci_le_set_extended_advertising_parameters_api(Advertising_Handle,
                                                           Advertising_Event_Properties,
                                                           Primary_Advertising_Interval_Min,
                                                           Primary_Advertising_Interval_Max,
                                                           Primary_Advertising_Channel_Map,
                                                           Own_Address_Type,
                                                           Peer_Address_Type,
                                                           Peer_Address,
                                                           Advertising_Filter_Policy,
                                                           Advertising_Tx_Power,
                                                           Primary_Advertising_PHY,
                                                           Secondary_Advertising_Max_Skip,
                                                           Secondary_Advertising_PHY,
                                                           Advertising_SID,
                                                           Scan_Request_Notification_Enable,
                                                           Selected_Tx_Power);
 }

tBleStatus hci_le_set_advertising_set_random_address(uint8_t Advertising_Handle,
                                                     uint8_t Advertising_Random_Address[6])
{
  return hci_le_set_advertising_set_random_address_api(Advertising_Handle,
                                                       Advertising_Random_Address);

}


tBleStatus hci_le_set_extended_advertising_enable(uint8_t Enable,
                                                  uint8_t Number_of_Sets,
                                                  Advertising_Set_Parameters_t Advertising_Set_Parameters[])
{
  return hci_le_set_extended_advertising_enable_api(Enable,
                                                    Number_of_Sets,
                                                    Advertising_Set_Parameters);
}


tBleStatus hci_le_read_maximum_advertising_data_length(uint16_t *Maximum_Advertising_Data_Length)
{
  return hci_le_read_maximum_advertising_data_length_api(Maximum_Advertising_Data_Length);
}

tBleStatus hci_le_read_number_of_supported_advertising_sets(uint8_t *Num_Supported_Advertising_Sets)
{
  return hci_le_read_number_of_supported_advertising_sets_api(Num_Supported_Advertising_Sets);
}

tBleStatus hci_le_remove_advertising_set(uint8_t Advertising_Handle)
{
  return hci_le_remove_advertising_set_api(Advertising_Handle);
}

tBleStatus hci_le_clear_advertising_sets(void)
{
  return hci_le_clear_advertising_sets_api();
}

tBleStatus hci_le_set_periodic_advertising_parameters(uint8_t Advertising_Handle,
                                                      uint16_t Periodic_Advertising_Interval_Min,
                                                      uint16_t Periodic_Advertising_Interval_Max,
                                                      uint16_t Periodic_Advertising_Properties)
{
  return hci_le_set_periodic_advertising_parameters_api(Advertising_Handle,
                                                        Periodic_Advertising_Interval_Min,
                                                        Periodic_Advertising_Interval_Max,
                                                        Periodic_Advertising_Properties);
}

tBleStatus hci_le_set_periodic_advertising_data(uint8_t Advertising_Handle,
                                                uint8_t Operation,
                                                uint8_t Advertising_Data_Length,
                                                uint8_t Advertising_Data[])
{
  
  return hci_le_set_periodic_advertising_data_api(Advertising_Handle,
                                                  Operation,
                                                  Advertising_Data_Length,
                                                  Advertising_Data);
}

tBleStatus hci_le_set_periodic_advertising_enable(uint8_t Enable,
                                                  uint8_t Advertising_Handle)
{
  return hci_le_set_periodic_advertising_enable_api(Enable,
                                                    Advertising_Handle);
}


tBleStatus hci_le_extended_create_connection(uint8_t Initiating_Filter_Policy,
                                             uint8_t Own_Address_Type,
                                             uint8_t Peer_Address_Type,
                                             uint8_t Peer_Address[6],
                                             uint8_t Initiating_PHYs,
                                             Extended_Create_Connection_Parameters_t Extended_Create_Connection_Parameters[])
{
  return hci_le_extended_create_connection_api(Initiating_Filter_Policy,
                                               Own_Address_Type,
                                               Peer_Address_Type,
                                               Peer_Address,
                                               Initiating_PHYs,
                                               Extended_Create_Connection_Parameters);
}

tBleStatus hci_le_set_extended_scan_parameters(uint8_t Own_Address_Type,
                                               uint8_t Scanning_Filter_Policy,
                                               uint8_t Scanning_PHYs,
                                               Extended_Scan_Parameters_t Extended_Scan_Parameters[])
{
     return hci_le_set_extended_scan_parameters_api(Own_Address_Type,
                                                    Scanning_Filter_Policy,
                                                    Scanning_PHYs,
                                                    Extended_Scan_Parameters);
}

uint32_t ext_adv_scan_enabled_ucfg(void)
{
  return 1;
}


tBleStatus hci_le_set_extended_scan_enable(uint8_t Enable, uint8_t Filter_Duplicates, uint16_t Duration, uint16_t Period)
{
  return hci_le_set_extended_scan_enable_api(Enable, Filter_Duplicates, Duration, Period);
}


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
                                                 uint8_t Scan_Request_Notification_Enable)
{
    return GAP_set_extended_advertising_configuration(Advertising_Handle,
                                                                 Discoverability_Mode,
                                                                 Advertising_Event_Properties,
                                                                 Primary_Advertising_Interval_Min,
                                                                 Primary_Advertising_Interval_Max,
                                                                 Primary_Advertising_Channel_Map,
                                                                 Peer_Address_Type,
                                                                 Peer_Address,
                                                                 Advertising_Filter_Policy,
                                                                 Advertising_Tx_Power,
                                                                 Primary_Advertising_PHY,
                                                                 Secondary_Advertising_Max_Skip,
                                                                 Secondary_Advertising_PHY,
                                                                 Advertising_SID,
                                                                 Scan_Request_Notification_Enable);
}


tBleStatus GAP_set_scan_response_data_ucfg(uint8_t Advertising_Handle,
                                           uint16_t Scan_Response_Data_Length,
                                           uint8_t* Scan_Response_Data)
{
    return GAP_set_extended_scan_response_data(Advertising_Handle,
                                               Scan_Response_Data_Length,
                                               Scan_Response_Data);
}

    
tBleStatus GAP_set_advertising_data_ucfg(uint8_t Advertising_Handle,
                                    uint8_t Operation, 
                                    uint16_t Advertising_Data_Length, 
                                    uint8_t* Advertising_Data)
{
    return GAP_set_extended_advertising_data(Advertising_Handle,
                                             Operation,
                                             Advertising_Data_Length,
                                             Advertising_Data);
                                                 
}


tBleStatus GAP_set_advertising_enable_ucfg(uint8_t Enable,
                                           uint8_t Num_Of_Sets,
                                           Advertising_Set_Parameters_t* Advertising_Set_Parameters)
{
    return GAP_set_extended_advertising_enable(Enable,
                                               Num_Of_Sets,
                                               Advertising_Set_Parameters);
}


void GAP_slave_connection_complete_handler_ucfg(uint16_t connectionHandle)
{
    ;
}


void GAP_hci_le_advertising_set_terminated_evt_hndl_ucfg(uint8_t status, uint8_t Advertising_Handle)
{
    GAP_hci_le_advertising_set_terminated_evt_hndl(status, Advertising_Handle);
}


tBleStatus GAP_create_connection(uint8_t *peer_address, uint8_t peer_address_type, uint8_t own_address_type, uint8_t initiator_filter_policy, uint8_t phys)
{
  return GAP_create_connection_ext(peer_address, peer_address_type, own_address_type, initiator_filter_policy, phys);
}


tBleStatus GAP_set_scan_parameters(uint8_t own_address_type, uint8_t phys)
{
  return GAP_set_scan_parameters_ext(own_address_type, phys);
}


tBleStatus GAP_suspend_resume_active_advertising_sets_ucfg(BOOL resume)
{
  return GAP_suspend_resume_active_advertising_sets_extended(resume);
}


tBleStatus GAP_set_controller_random_address_ucfg(uint8_t random_address[6])
{
  return GAP_set_controller_random_address_extended(random_address);
}


void LL_eadv_start_extended_ucfg(void *pointer)
{
    LL_eadv_start_extended(pointer);
}

tBleStatus LL_Set_Extented_Scan_Enable_ucfg(void *pExtScanEnable)
{
    return(LL_Set_Extented_Scan_Enable(pExtScanEnable));
}
 
void LL_escan_ScanStopCancel_ucfg(void)
{
    LL_escan_ScanStopCancel();
}

void LL_escan_ScanStart_ucfg(void)
{
   LL_escan_ScanStart();
}
 
BOOL LL_escan_EauxIsr_ucfg(void *pointer)
{
    return(LL_escan_EauxIsr(pointer));
}

void SCAN_INIT_ucfg(void)
{
    //SCAN_INIT();
}
 
void LL_escan_extended_adv_receive_process_ucfg(void *linkpointer, 
                                           uint8_t phyindex,
										   BOOL anonymous_flag, 
										   BOOL receive_address_present_flag, 
										   uint32_t *received_receive_address)
{
    LL_escan_extended_adv_receive_process(linkpointer,phyindex,anonymous_flag,
                                       receive_address_present_flag,received_receive_address);
}

void LL_escan_push_hci_le_extended_advertising_report_event_ucfg(void* pointer,
                                                                 uint8_t len,uint8_t offset,
                                                                 uint8_t event_type)
{
    LL_escan_push_hci_le_extended_advertising_report_event(pointer,
                                                            len,offset,
                                                            event_type);
}
tBleStatus LL_eadv_max_supported_data_check_ucfg(uint16_t Data_Length,
						void *linkpointer)
{
    return( LL_eadv_max_supported_data_check( Data_Length,linkpointer));
}
#endif

#endif

/* ---------------------------------------------------------------------------------------------------------- */

#ifdef CONTROLLER_PRIVACY_ENABLED
#if (CONTROLLER_PRIVACY_ENABLED == 1)

/* APIs definitions */

tBleStatus hci_le_remove_device_from_resolving_list(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6])
{
    return hci_le_remove_device_from_resolving_list_api(Peer_Identity_Address_Type, Peer_Identity_Address);
}

tBleStatus hci_le_add_device_to_resolving_list(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Peer_IRK[16], uint8_t Local_IRK[16])
{
    return hci_le_add_device_to_resolving_list_api(Peer_Identity_Address_Type, Peer_Identity_Address, Peer_IRK, Local_IRK);
}

tBleStatus hci_le_set_resolvable_private_address_timeout(uint16_t RpaTimeout)
{
    return hci_le_set_resolvable_private_address_timeout_api(RpaTimeout);
}

tBleStatus hci_le_set_address_resolution_enable(uint8_t AddressResolutionEnable)
{
    return hci_le_set_address_resolution_enable_api(AddressResolutionEnable);
}

tBleStatus hci_le_read_peer_resolvable_address(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Peer_Resolvable_Address[6])
{
    return hci_le_read_peer_resolvable_address_api(Peer_Identity_Address_Type, Peer_Identity_Address, Peer_Resolvable_Address);
}

tBleStatus hci_le_read_local_resolvable_address(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Local_Resolvable_Address[6])
{
    return hci_le_read_local_resolvable_address_api(Peer_Identity_Address_Type, Peer_Identity_Address, Local_Resolvable_Address);
}

tBleStatus hci_le_read_resolving_list_size(uint8_t *resolving_List_Size)
{
    return hci_le_read_resolving_list_size_api(resolving_List_Size);
}

tBleStatus hci_le_clear_resolving_list(void)
{
    return hci_le_clear_resolving_list_api();
}

tBleStatus hci_le_set_privacy_mode(uint8_t Peer_Identity_Address_Type, uint8_t Peer_Identity_Address[6], uint8_t Privacy_Mode)
{
    return hci_le_set_privacy_mode_api(Peer_Identity_Address_Type, Peer_Identity_Address, Privacy_Mode);
}

/* Internal core function definitions */
tBleStatus GAP_enable_controller_privacy_ucfg(uint8_t *gapRole, uint8_t *numServiceRec)
{
    return GAP_enable_controller_privacy(gapRole, numServiceRec);
}

tBleStatus GAP_add_device_to_white_and_resolving_list_ucfg(uint8_t lists,
                                                           uint8_t addr_type,
                                                           uint8_t addr[6])
{
  return GAP_add_device_to_white_and_resolving_list_full(lists, addr_type, addr);
}

tBleStatus GAP_clear_white_and_resolving_list_ucfg(uint8_t lists)
{
    return  GAP_clear_white_and_resolving_list_full(lists);
}

void LL_priv_u8OfflineProcessing_ucfg(void)
{
    LL_priv_u8OfflineProcessing();
}

uint32_t PRIV_controller_privacy_csr_ucfg(void)
{
    return 1U;
}

void LL_priv_vOwnPrivateAddress_ucfg(uint32_t *Own, uint32_t* Peer,uint8_t is_ISR)
{
    LL_priv_vOwnPrivateAddress(Own, Peer,is_ISR);
}

void LL_priv_vPeerPrivateAddress_ucfg(uint32_t *Peer,uint8_t is_ISR)
{
    LL_priv_vPeerPrivateAddress(Peer, is_ISR);
}

uint8_t LL_priv_u32ProcessAdvPacket_ucfg(uint32_t* ReceivedTransmitAddress, uint8_t PduType, uint32_t* PeerIDAddress,
    uint8_t AdvertisingEventproperties, uint8_t filterPolicy, uint32_t* peer_addr, uint16_t advertisinghandle)
{
    return (LL_priv_u32ProcessAdvPacket(ReceivedTransmitAddress,PduType,PeerIDAddress,AdvertisingEventproperties,
                                filterPolicy,peer_addr,advertisinghandle));
}

uint8_t LL_priv_u32ProcessScanPacketTx_ucfg(uint32_t* ReceivedTransmitAddress, uint32_t* PeerIDaddress, uint8_t isScanner, uint8_t* initPeerAddr,
    uint8_t initPeerAddrType, uint8_t LocalFilterPolicy, uint8_t resolvedirect,uint8_t* DatabaseEntry)
{
    return(LL_priv_u32ProcessScanPacketTx(ReceivedTransmitAddress,PeerIDaddress,isScanner,initPeerAddr,
                                   initPeerAddrType,LocalFilterPolicy,resolvedirect,DatabaseEntry));
}

uint8_t LL_priv_u32ProcessScanPacketRx_ucfg(uint32_t* ReceivedReceiveAddress, uint8_t LocalFilterPolicy, uint32_t* own_addr_used, uint8_t PointerToDataBase, uint8_t resolvedirect)
{
    return (LL_priv_u32ProcessScanPacketRx(ReceivedReceiveAddress,LocalFilterPolicy,own_addr_used,PointerToDataBase,resolvedirect));
}
void LL_priv_SynchPeerIDList_And_WhiteIDList_ucfg(void)
{
    LL_priv_SynchPeerIDList_And_WhiteIDList();
}

void LL_priv_Init_ResolvedPart_ucfg(void)
{
    LL_priv_Init_ResolvedPart();
}
uint8_t hci_le_check_own_address_type_max_value_ucfg(void)
{
    return 0x03;
}
#endif
#else
#warning CONTROLLER_PRIVACY_ENABLED is not defined
#endif

/* ---------------------------------------------------------------------------------------------------------- */
#ifdef CONTROLLER_MASTER_ENABLED
#if (CONTROLLER_MASTER_ENABLED == 1) 

tBleStatus hci_le_create_connection(uint16_t LE_Scan_Interval,
                                    uint16_t LE_Scan_Window,
                                    uint8_t Initiator_Filter_Policy,
                                    uint8_t Peer_Address_Type,
                                    uint8_t Peer_Address[6],
                                    uint8_t Own_Address_Type,
                                    uint16_t Conn_Interval_Min,
                                    uint16_t Conn_Interval_Max,
                                    uint16_t Conn_Latency,
                                    uint16_t Supervision_Timeout,
                                    uint16_t Minimum_CE_Length,
                                    uint16_t Maximum_CE_Length)
{
    return hci_le_create_connection_api(LE_Scan_Interval,
                                        LE_Scan_Window,
                                        Initiator_Filter_Policy,
                                        Peer_Address_Type,
                                        Peer_Address,
                                        Own_Address_Type,
                                        Conn_Interval_Min,
                                        Conn_Interval_Max,
                                        Conn_Latency,
                                        Supervision_Timeout,
                                        Minimum_CE_Length,
                                        Maximum_CE_Length);
}


tBleStatus hci_le_set_scan_enable(uint8_t LE_Scan_Enable, uint8_t Filter_Duplicates)
{
    return hci_le_set_scan_enable_api(LE_Scan_Enable, Filter_Duplicates);
}


tBleStatus hci_le_set_scan_parameters(uint8_t LE_Scan_Type,
                                      uint16_t LE_Scan_Interval,
                                      uint16_t LE_Scan_Window,
                                      uint8_t Own_Address_Type,
                                      uint8_t Scanning_Filter_Policy)
{
    return hci_le_set_scan_parameters_api(LE_Scan_Type,
                                          LE_Scan_Interval,
                                          LE_Scan_Window,
                                          Own_Address_Type,
                                          Scanning_Filter_Policy);
}


tBleStatus hci_le_create_connection_cancel(void)
{
    return hci_le_create_connection_cancel_api();
}


tBleStatus hci_le_connection_update(uint16_t Connection_Handle,
                                    uint16_t Conn_Interval_Min,
                                    uint16_t Conn_Interval_Max,
                                    uint16_t Conn_Latency,
                                    uint16_t Supervision_Timeout,
                                    uint16_t Minimum_CE_Length,
                                    uint16_t Maximum_CE_Length)
{
    return hci_le_connection_update_api(Connection_Handle,
                                        Conn_Interval_Min,
                                        Conn_Interval_Max,
                                        Conn_Latency,
                                        Supervision_Timeout,
                                        Minimum_CE_Length,
                                        Maximum_CE_Length);
}


tBleStatus hci_le_start_encryption(uint16_t Connection_Handle,
                                   uint8_t Random_Number[8],
                                   uint16_t Encrypted_Diversifier,
                                   uint8_t Long_Term_Key[16])
{
    return hci_le_start_encryption_api(Connection_Handle,
                                       Random_Number,
                                       Encrypted_Diversifier,
                                       Long_Term_Key);
}


tBleStatus hci_le_set_host_channel_classification(uint8_t LE_Channel_Map[5])
{
    return hci_le_set_host_channel_classification_api(LE_Channel_Map);
}


tBleStatus aci_gap_set_scan_configuration(uint8_t duplicate_filtering,
                                          uint8_t scanning_filter_policy,
                                          uint8_t phy,
                                          uint8_t scan_type,
                                          uint16_t scan_interval,
                                          uint16_t scan_window)
{
  return aci_gap_set_scan_configuration_api(duplicate_filtering, scanning_filter_policy, phy, scan_type, scan_interval, scan_window);
}

tBleStatus aci_gap_set_connection_configuration(uint8_t phy,
                                                uint16_t conn_interval_min,
                                                uint16_t conn_interval_max,
                                                uint16_t conn_latency,
                                                uint16_t supervision_timeout,
                                                uint16_t minimum_ce_length,
                                                uint16_t maximum_ce_length)
{
  return aci_gap_set_connection_configuration_api(phy, conn_interval_min, conn_interval_max, conn_latency,
                                                  supervision_timeout, minimum_ce_length, maximum_ce_length);
}

tBleStatus aci_gap_create_connection(uint8_t Initiating_PHY,
                                     uint8_t Peer_Address_Type,
                                     uint8_t Peer_Address[6])
{
  return aci_gap_create_connection_api(Initiating_PHY, Peer_Address_Type, Peer_Address);
}

tBleStatus aci_gap_start_procedure(uint8_t procedure_code, uint8_t phys, uint16_t duration, uint16_t period)
{
  return aci_gap_start_procedure_api(procedure_code, phys, duration, period);
}

tBleStatus aci_gap_discover_name(uint8_t PHYs,
                                 uint8_t Peer_Address_Type,
                                 uint8_t Peer_Address[6])
{
  return aci_gap_discover_name_api(PHYs, Peer_Address_Type, Peer_Address);
}

tBleStatus aci_gap_terminate_proc(uint8_t Procedure_Code)
{
  return aci_gap_terminate_proc_api(Procedure_Code);
}

tBleStatus aci_gap_start_connection_update(uint16_t Connection_Handle,
                                           uint16_t Conn_Interval_Min,
                                           uint16_t Conn_Interval_Max,
                                           uint16_t Conn_Latency,
                                           uint16_t Supervision_Timeout,
                                           uint16_t Minimum_CE_Length,
                                           uint16_t Maximum_CE_Length)
{
    return aci_gap_start_connection_update_api(Connection_Handle,
                                               Conn_Interval_Min,
                                               Conn_Interval_Max,
                                               Conn_Latency,
                                               Supervision_Timeout,
                                               Minimum_CE_Length,
                                               Maximum_CE_Length);
}


#if (CONTROLLER_EXT_ADV_SCAN_ENABLED == 1)
tBleStatus GAP_enable_disable_scan_ucfg(BOOL enable, uint8_t duplicate_filtering)
{
  return GAP_enable_disable_scan_ext(enable, duplicate_filtering);
}
#else
tBleStatus GAP_enable_disable_scan_ucfg(BOOL enable, uint8_t duplicate_filtering)
{
  return GAP_enable_disable_scan_legacy(enable, duplicate_filtering);
}
#endif

tBleStatus aci_gap_send_pairing_req(uint16_t Connection_Handle, uint8_t Force_Rebond)
{
    return aci_gap_send_pairing_req_api(Connection_Handle, Force_Rebond);
}

tBleStatus aci_l2cap_connection_parameter_update_resp(uint16_t Connection_Handle,
                                                      uint16_t Conn_Interval_Min,
                                                      uint16_t Conn_Interval_Max,
                                                      uint16_t Slave_latency,
                                                      uint16_t Timeout_Multiplier,
                                                      uint16_t Minimum_CE_Length,
                                                      uint16_t Maximum_CE_Length,
                                                      uint8_t Identifier,
                                                      uint8_t Accept)

{
    return aci_l2cap_connection_parameter_update_resp_api(Connection_Handle,
                                                          Conn_Interval_Min,
                                                          Conn_Interval_Max,
                                                          Slave_latency,
                                                          Timeout_Multiplier,
                                                          Minimum_CE_Length,
                                                          Maximum_CE_Length,
                                                          Identifier,
                                                          Accept);
}

void GAP_master_connection_complete_handler_ucfg(uint8_t status, uint16_t connectionHandle)
{
    GAP_master_connection_complete_handler(status, connectionHandle);
}

BOOL GAP_parse_connectable_advertising_report_ucfg(uint8_t *adv_buf, BOOL extended)
{
    return GAP_parse_connectable_advertising_report(adv_buf, extended);
}

BOOL GAP_parse_advertising_report_ucfg(uint8_t *adv_buf, BOOL extended)
{
    return GAP_parse_advertising_report(adv_buf, extended);
}


/* Internal core function definitions */
void GAP_DiscProcTimeoutcb_ucfg(int8_t timer_id)
{
    GAP_DiscProcTimeoutcb(timer_id);
}

tBleStatus LL_Start_Encryption_ucfg(uint16_t connHandle, uint8_t *randNum, uint8_t *ediv, uint8_t *ltk)
{
    return LL_Start_Encryption(connHandle, randNum, ediv, ltk);
}

tBleStatus GAP_check_and_set_role_ucfg(uint8_t *gapRole, uint8_t role)
{
    return GAP_check_and_set_role(gapRole, role);
}

tBleStatus GAP_discover_peer_name_ucfg(void)
{
    return GAP_discover_peer_name();
}

uint32_t master_csr_ucfg(void)
{
  return 1U;
}

/* SMP_MASTER functionalities */
tBleStatus smp_MI_Start_Encryption_ucfg(void *params)
{
    return smp_MI_Start_Encryption(params);
}
void       smp_Execute_Actions_wrt_Current_State_MI_excerpt_ucfg(void *params)
{
    smp_Execute_Actions_wrt_Current_State_MI_excerpt(params);
}
tBleStatus smp_Process_Rx_Packet_wrt_Current_State_MI_excerpt_ucfg(void *params)
{
    return smp_Process_Rx_Packet_wrt_Current_State_MI_excerpt(params);
}
#endif
#else
#warning CONTROLLER_MASTER_ENABLED is not defined
#endif


/* ---------------------------------------------------------------------------------------------------------- */
#if ((defined(CONTROLLER_MASTER_ENABLED) && (CONTROLLER_MASTER_ENABLED == 1)) || \
     (defined(SECURE_CONNECTIONS_ENABLED) && (SECURE_CONNECTIONS_ENABLED == 1)))
void smp_Execute_Actions_wrt_Current_State_sc_MI_excerpt_phase2as2_ucfg(void *params)
{
    smp_Execute_Actions_wrt_Current_State_sc_MI_excerpt_phase2as2(params);
}
#endif
    
/* ---------------------------------------------------------------------------------------------------------- */

#ifdef CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED
#if (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED == 1)

/* APIs definitions */
tBleStatus hci_le_set_data_length(uint16_t Connection_Handle, uint16_t TxOctets, uint16_t TxTime)
{
    return hci_le_set_data_length_api(Connection_Handle, TxOctets, TxTime);
}

tBleStatus hci_le_read_suggested_default_data_length(uint16_t *SuggestedMaxTxOctets, uint16_t *SuggestedMaxTxTime)
{
    return hci_le_read_suggested_default_data_length_api(SuggestedMaxTxOctets, SuggestedMaxTxTime);
}

tBleStatus hci_le_write_suggested_default_data_length(uint16_t SuggestedMaxTxOctets, uint16_t SuggestedMaxTxTime)
{
    return hci_le_write_suggested_default_data_length_api(SuggestedMaxTxOctets, SuggestedMaxTxTime);
}

tBleStatus hci_le_read_maximum_data_length(uint16_t *supportedMaxTxOctets, uint16_t *supportedMaxTxTime,
                                           uint16_t *supportedMaxRxOctets, uint16_t *supportedMaxRxTime)
{
    return hci_le_read_maximum_data_length_api(supportedMaxTxOctets, supportedMaxTxTime,
                                               supportedMaxRxOctets, supportedMaxRxTime);
}

/* Internal core function definitions */
void LL_cpe_init_length_update_ucfg(void)
{
    LL_cpe_init_length_update();
}

void Data_Len_Update_Offline_Processing_ucfg(void* params, uint32_t ctrl_flds)
{
    Data_Len_Update_Offline_Processing(params, ctrl_flds);
}

tBleStatus ll_write_supported_data_ucfg(uint16_t Supported_Max_Tx_Octets, uint16_t Supported_Max_Tx_Time,
                                        uint16_t Supported_Max_Rx_Octets, uint16_t Supported_Max_Rx_Time)
{
    return ll_write_supported_data(Supported_Max_Tx_Octets, Supported_Max_Tx_Time,
                                   Supported_Max_Rx_Octets, Supported_Max_Rx_Time);
}

#if (defined(CONTROLLER_2M_CODED_PHY_ENABLED) && (CONTROLLER_2M_CODED_PHY_ENABLED == 1))
void LL_conn_upd_max_tx_time_coded_ucfg(void *params)
{
    LL_conn_upd_max_tx_time_coded(params);
}

void LL_conn_upd_data_length_change_event_ucfg(void *params)
{
    LL_conn_upd_data_length_change_event(params);
}

void LL_phy_upd_compute_data_PDU_length_params_ucfg(void *params)
{
    LL_phy_upd_compute_data_PDU_length_params(params);
}
#endif

uint32_t data_length_extension_csr_ucfg(void)
{
  return 1U;
}
#endif
#else
#warning CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED is not defined
#endif


/* ---------------------------------------------------------------------------------------------------------- */
#ifdef SECURE_CONNECTIONS_ENABLED
#if (SECURE_CONNECTIONS_ENABLED == 1)
/*
* *****************************************************************************
*                         BLE Stack API functions
* *****************************************************************************
*/
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack API functions "_api" wrapping implementation
 */
tBleStatus aci_gap_passkey_input(uint16_t Connection_Handle, uint8_t Input_Type)
{
    return aci_gap_passkey_input_api(Connection_Handle, Input_Type);
}

tBleStatus aci_gap_numeric_comparison_value_confirm_yesno(uint16_t Connection_Handle, uint8_t Confirm_Yes_No)     
{
    return aci_gap_numeric_comparison_value_confirm_yesno_api(Connection_Handle, Confirm_Yes_No);
}

tBleStatus hci_le_read_local_p256_public_key(void)
{
    return hci_le_read_local_p256_public_key_api();
}

tBleStatus hci_le_generate_dhkey(uint8_t Remote_P256_Public_Key[64])
{
    return hci_le_generate_dhkey_api(Remote_P256_Public_Key);
}

/*
* *****************************************************************************
*                      BLE Stack INTERNAL core functions
* *****************************************************************************
*/
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack INTERNAL core functions' prototype declaration
 */
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack INTERNAL core functions' "ucfg" wrapping implementation
 */
/* ----------------------------------------------- from hci.c ----- */
void PKA_Offline_Processing_ucfg(void)
{
    PKA_Offline_Processing();
}

/* ----------------------------------------------- from smp_scv42_caf.h ----- */
void        smp_sc_init_ucfg(BOOL useDebugKey)
{
    smp_sc_init(useDebugKey);
}
tBleStatus  smp_sc_continue_on_exclusive_sc_pairing_ucfg(void *params)
{
    return smp_sc_continue_on_exclusive_sc_pairing(params);
}
void        smp_sc_check_for_ecc_pk_generation_request_ucfg(void)
{
    smp_sc_check_for_ecc_pk_generation_request();
}
tBleStatus  smp_sc_generate_new_local_oob_data_ucfg(void)
{
    return smp_sc_generate_new_local_oob_data();
}
void        smp_sc_update_statistics_on_pairing_complete_ucfg(void *params)
{
    smp_sc_update_statistics_on_pairing_complete(params);
}

/* ----------------------------------------------- from smp_scv42_sap.h ----- */
uint32_t secure_connections_csr_ucfg(void)
{
  return 1U;
}

void        smp_sc_hci_le_read_local_p256_public_key_complete_evt_hndl_ucfg(uint8_t status, uint8_t local_p256_public_key[64])
{
    smp_sc_hci_le_read_local_p256_public_key_complete_evt_hndl(status, local_p256_public_key);
}
void        smp_sc_hci_le_generate_dhkey_complete_evt_hndl_ucfg(uint8_t status, uint8_t DHKey[32])
{
    smp_sc_hci_le_generate_dhkey_complete_evt_hndl(status, DHKey);
}

/* ------------------------------------------------- from securitymgr.h ----- */
tBleStatus smp_sc_MI_PairingResponse_Receive_sc_excerpt_ucfg(void *params)
{
    return smp_sc_MI_PairingResponse_Receive_sc_excerpt(params);
}
void       smp_sc_MI_Pairing_Phase_2AS1_Start_sc_excerpt_ucfg(void *params)
{
    smp_sc_MI_Pairing_Phase_2AS1_Start_sc_excerpt(params);
}
void       smp_sc_SR_Pairing_Phase_2AS1_Start_sc_excerpt_ucfg(void *params)
{
    smp_sc_SR_Pairing_Phase_2AS1_Start_sc_excerpt(params);
}

void       smp_sc_MI_PairingConfirm_Send_sc_excerpt_ucfg(void *params)
{
    smp_sc_MI_PairingConfirm_Send_sc_excerpt(params);
}
tBleStatus smp_sc_MI_PairingRandom_Receive_sc_excerpt1_ucfg(void *params)
{
    return smp_sc_MI_PairingRandom_Receive_sc_excerpt1(params);
}
void       smp_sc_MI_PairingRandom_Receive_sc_excerpt2_ucfg(void *params)
{
    smp_sc_MI_PairingRandom_Receive_sc_excerpt2(params);
}

void       smp_sc_SR_PairingConfirm_Receive_sc_excerpt_ucfg(void *params)
{
    smp_sc_SR_PairingConfirm_Receive_sc_excerpt(params);
}
void       smp_sc_SR_PairingRandom_Receive_sc_excerpt_ucfg(void *params)
{
    smp_sc_SR_PairingRandom_Receive_sc_excerpt(params);
}

BOOL       smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase_1_to_2_ucfg(void *params)
{
    return smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase_1_to_2(params);
}
void       smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase2as2_ucfg(void *params)
{
    smp_Execute_Actions_wrt_Current_State_sc_excerpt_phase2as2(params);
}
tBleStatus smp_Process_Rx_Packet_wrt_Current_State_sc_excerpt_ucfg(void *params)
{
    return smp_Process_Rx_Packet_wrt_Current_State_sc_excerpt(params);
}
tBleStatus smp_Process_Rx_Packet_Exception_Cases_sc_excerpt_ucfg(void *params)
{
    return smp_Process_Rx_Packet_Exception_Cases_sc_excerpt(params);
}
#endif
#else
#warning "SECURE_CONNECTIONS_ENABLED is not defined"
#endif

/* ---------------------------------------------------------------------------------------------------------- */
#ifdef CONTROLLER_2M_CODED_PHY_ENABLED
#if (CONTROLLER_2M_CODED_PHY_ENABLED == 1)

/* APIs definitions */
tBleStatus hci_le_read_phy(uint16_t Connection_Handle, uint8_t *TX_PHY, uint8_t *RX_PHY)
{
    return hci_le_read_phy_api(Connection_Handle, TX_PHY, RX_PHY);
}

tBleStatus hci_le_set_default_phy(uint8_t ALL_PHYS, uint8_t TX_PHYS, uint8_t RX_PHYS)
{
    return hci_le_set_default_phy_api(ALL_PHYS, TX_PHYS, RX_PHYS);
}

tBleStatus hci_le_set_phy(uint16_t Connection_Handle, uint8_t ALL_PHYS, uint8_t TX_PHYS, uint8_t RX_PHYS, uint16_t PHY_options)
{
    return hci_le_set_phy_api(Connection_Handle, ALL_PHYS, TX_PHYS, RX_PHYS, PHY_options);
}

tBleStatus hci_le_enhanced_receiver_test(uint8_t RX_Frequency, uint8_t Phy, uint8_t Modulation_iIndex)
{
    return hci_le_enhanced_receiver_test_api(RX_Frequency, Phy, Modulation_iIndex);
}

tBleStatus hci_le_enhanced_transmitter_test(uint8_t TX_Frequency, uint8_t Length_Of_Test_Data, uint8_t Packet_Payload, uint8_t Phy)
{
    return hci_le_enhanced_transmitter_test_api(TX_Frequency, Length_Of_Test_Data, Packet_Payload, Phy);
}

/* Internal core function definitions */
void LL_cpe_init_phy_update_ucfg(void)
{
    LL_cpe_init_phy_update();
}

void LL_phy_upd_pending_ucfg(uint8_t conn_idx)
{
    LL_phy_upd_pending(conn_idx);
}

tBleStatus LL_phy_update_init_ucfg(void)
{
    return LL_phy_update_init();
}

tBleStatus LL_phy_update_init_per_st_data_ucfg(uint8_t conn_idx)
{
    return LL_phy_update_init_per_st_data(conn_idx);
}

uint32_t phy_upd_csr_ucfg(void)
{
    return 1U;
}
                                                                 
#if (!defined(CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED) || \
     (defined(CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED) && (CONTROLLER_DATA_LENGTH_EXTENSION_ENABLED == 0))) 

void LL_conn_upd_max_tx_time_coded_ucfg(void *params)
{
    LL_conn_upd_max_tx_time_coded(params);
}

void LL_conn_upd_data_length_change_event_ucfg(void *params)
{
    LL_conn_upd_data_length_change_event(params);
}
#endif

#endif
#else
#warning "CONTROLLER_2M_CODED_PHY_ENABLED is not defined"
#endif



/* ---------------------------------------------------------------------------------------------------------- */
#ifdef L2CAP_COS_ENABLED
#if (L2CAP_COS_ENABLED == 1)
/*
* *****************************************************************************
*                         BLE Stack API functions
* *****************************************************************************
*/
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack API functions "_api" wrapping implementation
 */
/* ------------------------------------------------ from bluenrg1_api.h ----- */
tBleStatus aci_l2cap_cfc_connection_req(uint16_t Connection_Handle,
                                        uint16_t SPSM,
                                        uint16_t CID,
                                        uint16_t MTU,
                                        uint16_t MPS,
                                        uint8_t CFC_Policy,
                                        uint16_t RX_SDU_Buffer_Size,
                                        void * RX_SDU_Buffer)
{
    return aci_l2cap_cfc_connection_req_api(Connection_Handle,
                                            SPSM,
                                            CID,
                                            MTU,
                                            MPS,
                                            CFC_Policy,
                                            RX_SDU_Buffer_Size,
                                            RX_SDU_Buffer);
}

tBleStatus aci_l2cap_cfc_connection_resp(uint16_t Connection_Handle,
                                         uint8_t Identifier,
                                         uint16_t CID,
                                         uint16_t MTU,
                                         uint16_t MPS,
                                         uint16_t Result,
                                         uint8_t CFC_Policy,
                                         uint16_t RX_SDU_Buffer_Size,
                                         void * RX_SDU_Buffer)
{
    return aci_l2cap_cfc_connection_resp_api(Connection_Handle,
                                             Identifier,
                                             CID,
                                             MTU,
                                             MPS,
                                             Result,
                                             CFC_Policy,
                                             RX_SDU_Buffer_Size,
                                             RX_SDU_Buffer);
}

tBleStatus aci_l2cap_send_flow_control_credits(uint16_t Connection_Handle,
                                               uint16_t CID,
                                               uint16_t RX_Credits,
                                               uint8_t  CFC_Policy,
                                               uint16_t *RX_Credit_Balance)
{
    return aci_l2cap_send_flow_control_credits_api(Connection_Handle,
                                                   CID,
                                                   RX_Credits,
                                                   CFC_Policy,
                                                   RX_Credit_Balance);
}

tBleStatus aci_l2cap_disconnect(uint16_t Connection_Handle,
                                uint16_t CID)
{
    return aci_l2cap_disconnect_api(Connection_Handle,
                                    CID);
}

tBleStatus aci_l2cap_transmit_sdu_data(uint16_t Connection_Handle,
                                       uint16_t CID,
                                       uint16_t SDU_Length,
                                       uint8_t SDU_Data[])
{
    return aci_l2cap_transmit_sdu_data_api(Connection_Handle, CID, SDU_Length, SDU_Data);
}

tBleStatus aci_l2cap_extract_sdu_data(uint16_t Connection_Handle,
                                      uint16_t CID,
                                      uint16_t SDU_Data_Buffer_Size,
                                      void * SDU_Data_Buffer,
                                      uint16_t *SDU_Length)
{
    return aci_l2cap_extract_sdu_data_api(Connection_Handle,
                                          CID,
                                          SDU_Data_Buffer_Size,
                                          SDU_Data_Buffer,
                                          SDU_Length);
}


/*
* *****************************************************************************
*                      BLE Stack INTERNAL core functions
* *****************************************************************************
*/
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack INTERNAL core functions' prototype declaration
 */
/* = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =
 * BLE Stack INTERNAL core functions' "ucfg" wrapping implementation
 */
uint32_t l2c_cos_csr_ucfg(void)
{
  return 1U;
}

tBleStatus L2C_cos_cfc_init_ucfg(void)
{
    return L2C_cos_cfc_init();
}

void L2C_cos_process_q_ucfg(void)
{
    L2C_cos_process_q();
}

tBleStatus L2C_cos_process_cfc_mode_command_ucfg(void *params)
{
    return L2C_cos_process_cfc_mode_command(params);
}

BOOL L2C_cos_is_pdu_cframe_cfc_command_opcode_ucfg(uint8_t opCode)
{
    return L2C_cos_is_pdu_cframe_cfc_command_opcode(opCode);
}

tBleStatus L2C_cos_le_frame_data_hndl_ucfg(void *params)
{
    return L2C_cos_le_frame_data_hndl(params);
}

void  L2C_cos_reset_any_pending_channel_ucfg(uint16_t connection_handle)
{
    L2C_cos_reset_any_pending_channel(connection_handle);
}

void  L2C_cos_physical_link_disconnection_hndl_ucfg(uint16_t connection_handle)
{
    L2C_cos_physical_link_disconnection_hndl(connection_handle);
}

#endif
#endif
