#include "stack_user_cfg.h"

#define ERR_UNKNOWN_HCI_COMMAND (0x01)

#ifdef CONTROLLER_EXT_ADV_SCAN_ENABLED
#if (CONTROLLER_EXT_ADV_SCAN_ENABLED == 0)

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
    return  ERR_UNKNOWN_HCI_COMMAND;
}

tBleStatus hci_le_set_advertising_set_random_address(uint8_t Advertising_Handle,
                                                     uint8_t Advertising_Random_Address[6])
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}



tBleStatus hci_le_set_extended_advertising_enable(uint8_t Enable,
                                                  uint8_t Number_of_Sets,
                                                  Advertising_Set_Parameters_t Advertising_Set_Parameters[])
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_read_maximum_advertising_data_length(uint16_t *Maximum_Advertising_Data_Length)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}

tBleStatus hci_le_read_number_of_supported_advertising_sets(uint8_t *Num_Supported_Advertising_Sets)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_remove_advertising_set(uint8_t Advertising_Handle)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_clear_advertising_sets(void)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_set_periodic_advertising_parameters(uint8_t Advertising_Handle,
                                                      uint16_t Periodic_Advertising_Interval_Min,
                                                      uint16_t Periodic_Advertising_Interval_Max,
                                                      uint16_t Periodic_Advertising_Properties)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_set_periodic_advertising_data(uint8_t Advertising_Handle,
                                                uint8_t Operation,
                                                uint8_t Advertising_Data_Length,
                                                uint8_t Advertising_Data[])
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_set_periodic_advertising_enable(uint8_t Enable,
                                                  uint8_t Advertising_Handle)
{
    return  ERR_UNKNOWN_HCI_COMMAND;
}


tBleStatus hci_le_extended_create_connection(uint8_t Initiating_Filter_Policy,
                                             uint8_t Own_Address_Type,
                                             uint8_t Peer_Address_Type,
                                             uint8_t Peer_Address[6],
                                             uint8_t Initiating_PHYs,
                                             Extended_Create_Connection_Parameters_t Extended_Create_Connection_Parameters[])
{
     return  ERR_UNKNOWN_HCI_COMMAND; 
}

tBleStatus hci_le_set_extended_scan_parameters(uint8_t Own_Address_Type,
                                               uint8_t Scanning_Filter_Policy,
                                               uint8_t Scanning_PHYs,
                                               Extended_Scan_Parameters_t Extended_Scan_Parameters[])
{
     return  ERR_UNKNOWN_HCI_COMMAND;   
}


tBleStatus hci_le_set_extended_scan_enable(uint8_t Enable, uint8_t Filter_Duplicates, uint16_t Duration, uint16_t Period)
{
     return  ERR_UNKNOWN_HCI_COMMAND;   
}


#endif
#endif
