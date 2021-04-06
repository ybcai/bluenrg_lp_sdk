/**
  ******************************************************************************
  * @file    bluenrg_lp_ll_radio.h
  * @author  RF Application Team
  * @brief   This file contains all the functions prototypes for the radio firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLUENRG_LP_LL_RADIO_H
#define BLUENRG_LP_LL_RADIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp.h"
#include "system_BlueNRG_LP.h"

/** @addtogroup BLUENRG_LP_LL_Driver
  * @{
  */

/** @addtogroup RADIO
  * @{
  */


/** @defgroup RADIO_Exported_Macros RADIO Exported Macros
  * @{
  */

#define BLUEGLOB_BASE               (_MEMORY_RAM_BEGIN_ + 0xC0U)
#define blueglob                    ((GLOBALSTATMACH_TypeDef*) BLUEGLOB_BASE)
#define bluedata                    ((STATMACH_TypeDef*) (BLUEGLOB_BASE+sizeof(GLOBALSTATMACH_TypeDef)))
#define BlueTransStruct             TXRXPACK_TypeDef

#define BLUE_DATA_PTR_CAST(PTR) (((uint32_t)(uintptr_t)(PTR)))
#define BLUE_STRUCT_PTR_CAST(PTR) (((uint32_t)(uintptr_t)(PTR)))

/**
  * @}
  */


/** @defgroup RADIO_Exported_Constants RADIO Exported Constants
  * @{
  */

/** @defgroup GlobalStatmach_Masks GlobalStatmach Masks
  * @{
  */

#define GLOBAL_BYTE4_CURSTMACHNUM_Msk                                      (0x7F)
#define GLOBAL_BYTE4_ACTIVE_Msk                                            (0x80)

#define GLOBAL_BYTE15_TXDELAYEND_Msk                                       (0x3F)
#define GLOBAL_BYTE15_TIMECAPTURESEL_Msk                                   (0x40)
#define GLOBAL_BYTE15_TIMECAPTURE_Msk                                      (0x80)

#define GLOBAL_BYTE20_AUTOTXRXSKIPEN_Msk                                   (0x01)
#define GLOBAL_BYTE20_CHKFLAGAUTOCLEARENA_Msk                              (0x04)
#define GLOBAL_BYTE20_NBANTENNAE_Msk                                       (0x38)
#define GLOBAL_BYTE20_FUNCDEBUGMODE_Msk                                    (0xC0)

#define GLOBAL_BYTE21_INTSEQERROR_Msk                                      (0x3F)

#define GLOBAL_BYTE22_INTADDPOINTERROR_Msk                                 (0x10)
#define GLOBAL_BYTE22_INTALLTABLEREADYERROR_Msk                            (0x20)
#define GLOBAL_BYTE22_INTTXDATAREADYERROR_Msk                              (0x40)
#define GLOBAL_BYTE22_INTNOACTIVELERROR_Msk                                (0x80)

#define GLOBAL_BYTE23_INTRCVLENGTHERROR_Msk                                (0x02)
#define GLOBAL_BYTE23_INTSEMATIMEOUTERROR_Msk                              (0x04)
#define GLOBAL_BYTE23_INTSEMAWASPREEMPT_Msk                                (0x08)
#define GLOBAL_BYTE23_INTSEQDONE_Msk                                       (0x10)
#define GLOBAL_BYTE23_INTTXRXSKIP_Msk                                      (0x20)
#define GLOBAL_BYTE23_INTACTIVE2ERR_Msk                                    (0x40)
#define GLOBAL_BYTE23_INTCONFIGERROR_Msk                                   (0x80)

/**
  * @}
  */


/** @defgroup Statmach_Masks Statmach Masks
  * @{
  */

#define STATEMACH_BYTE0_UCHAN_Msk                                          (0x3F)
#define STATEMACH_BYTE0_RADIOCOMLISTENA_Msk                                (0x40)
#define STATEMACH_BYTE0_TXMODE_Msk                                         (0x80)

#define STATEMACH_BYTE1_REMAP_CHAN_Msk                                     (0x3F)
#define STATEMACH_BYTE1_SN_Msk                                             (0x40)
#define STATEMACH_BYTE1_NESN_Msk                                           (0x80)

#define STATEMACH_BYTE2_SEMAPRIO_Msk                                       (0x07)
#define STATEMACH_BYTE2_SEMAPREEMPT_Msk                                    (0x08)
#define STATEMACH_BYTE2_BUFFOVERFLOW_Msk                                   (0x10)
#define STATEMACH_BYTE2_ENCRYPTON_Msk                                      (0x20)
#define STATEMACH_BYTE2_TXENC_Msk                                          (0x40)
#define STATEMACH_BYTE2_RCVENC_Msk                                         (0x80)

#define STATEMACH_BYTE3_TXPHY_Msk                                          (0x07)
#define STATEMACH_BYTE3_RXPHY_Msk                                          (0x70)

#define STATEMACH_BYTE34_PREAMBLEREP_Msk                                   (0x0F)
#define STATEMACH_BYTE34_ENAPREAMBLEREP_Msk                                (0x10)
#define STATEMACH_BYTE34_DISABLECRC_Msk                                    (0x20)
#define STATEMACH_BYTE34_MSBFIRST_Msk                                      (0x40)
#define STATEMACH_BYTE34_RXMICDBG_Msk                                      (0x80)

#define STATEMACH_BYTE35_INTTXERROR_Msk                                    (0x1F)
#define STATEMACH_BYTE35_INTENCERROR_Msk                                   (0x20)
#define STATEMACH_BYTE35_INTRXOVERFLOWERROR_Msk                            (0x40)
#define STATEMACH_BYTE35_RXDEBUGCRC_Msk                                    (0x80)

/**
  * @}
  */

/** @defgroup TxRxPack_Masks TxRxPack Masks
  * @{
  */

#define TXRXPACK_BYTE4_CALREQ_Msk                                          (0x01)
#define TXRXPACK_BYTE4_CHANALGO2SEL_Msk                                    (0x02)
#define TXRXPACK_BYTE4_KEEPSEMAREQ_Msk                                     (0x04)
#define TXRXPACK_BYTE4_SUPPENA_Msk                                         (0x08)
#define TXRXPACK_BYTE4_CRCINITSEL_Msk                                      (0x10)
#define TXRXPACK_BYTE4_ADVERTISE_Msk                                       (0x20)
#define TXRXPACK_BYTE4_SN_EN_Msk                                           (0x40)
#define TXRXPACK_BYTE4_INCCHAN_Msk                                         (0x80)

#define TXRXPACK_BYTE5_NEXTTXMODE_Msk                                      (0x01)
#define TXRXPACK_BYTE5_ALLTABLEREADY_Msk                                   (0x02)
#define TXRXPACK_BYTE5_TXDATAREADY_Msk                                     (0x04)
#define TXRXPACK_BYTE5_DATAREADYSEL_Msk                                    (0x08)
#define TXRXPACK_BYTE5_DISABLEWHITENING_Msk                                (0x10)
#define TXRXPACK_BYTE5_TESTPACKET_Msk                                      (0x20)

#define TXRXPACK_BYTE14_TIMER2_19_16_Msk                                   (0x0F)
#define TXRXPACK_BYTE14_TIMER2EN_Msk                                       (0x10)
#define TXRXPACK_BYTE14_TRIGRCV_Msk                                        (0x40)
#define TXRXPACK_BYTE14_TRIGDONE_Msk                                       (0x80)

#define TXRXPACK_BYTE15_INTTXOK_Msk                                        (0x01)
#define TXRXPACK_BYTE15_INTDONE_Msk                                        (0x02)
#define TXRXPACK_BYTE15_INTRCVTIMEOUT_Msk                                  (0x04)
#define TXRXPACK_BYTE15_INTRCVNOMD_Msk                                     (0x08)
#define TXRXPACK_BYTE15_INTRCVCMD_Msk                                      (0x10)
#define TXRXPACK_BYTE15_INTRCVTRIG_Msk                                     (0x20)
#define TXRXPACK_BYTE15_INTRCVCRCERR_Msk                                   (0x40)
#define TXRXPACK_BYTE15_INTRCVOK_Msk                                       (0x80)
#define TXRXPACK_BYTE15_INT_EN_Msk                                         (0xFF)

/**
  * @}
  */   


#define MAX_LL_PACKET_LENGTH    255 /* Maximum Link Layer Packet Length (user_payload + MIC)*/
#define MAX_OUTPUT_RF_POWER     0x1F
    
#define HEADER_LENGTH           2
#define MAX_PACKET_LENGTH       (MAX_LL_PACKET_LENGTH+HEADER_LENGTH)
#define MIC_FIELD_LENGTH        4
#define SUCCESS_0               0
#define INVALID_PARAMETER_C0    0xC0
#define RADIO_BUSY_C4           0xC4
#define NULL_0                  0
#define BLUE_IDLE_0             0  
#define TIMESTAMP_POSITION_ACCESSADDRESS    0x40 
#define TIMESTAMP_POSITION_LASTBIT          0x80   
#define BACK_TO_BACK_TIME       150 /* BLE IFS equal to 150 micro second */

#define STATEMACHINE_COUNT   8

/** @defgroup PHY PHY selection
* @{
*/

#define PHY_1M 0x00
#define PHY_2M 0x01
#define PHY_CODED_S_8 0x04
#define PHY_CODED_S_2 0x06

/**
  * @}
  */


/** @defgroup ActionTag_BitMask ActionTag BitMask
  * @{
  */

/* This bit activates the radio frequency PLL calibration.
 * 0: Radio frequency calibration disabled.
 * 1: Radio frequency calibration enabled.
*/
#define PLL_TRIG                    0x01
    
/* This bit determines if the action is an RX action or a TX action.
 * 1: TX
 * 0: RX
*/
#define TXRX                        0x02
    
/* The bit determines if the action (RX or TX) is going to be executed based on the back-to-back time or based on the WakeupTime.
 * 0: Based on the back-to-back time (default 150 µs).
 * 1: Based on the WakeupTime.
*/
#define TIMER_WAKEUP                0x04

/* The bit determines if whitening is disabled or not.
 *0: The whitening is enabled in the transmit block and in the receive block.
 *1: The whitening is disabled in the transmit block and in the receive block.
*/
#define WHITENING_DISABLE           0x10
    
/* It determines if the WakeupTime field of the ActionPacket is considered as absolute time or relative time to the current.
 * 0: Absolute
 * 1: Relative
*/
#define RELATIVE                    0x20
    
/* This bit sets where the position of time stamp is taken, the beginning of the packet or the end of it. RX only.
 * 0: End of the Packet
 * 1: Beginning of the packet
 */
#define TIMESTAMP_POSITION          0x40

/* This bit activates automatic channel increment. The API RADIO_SetChannel sets the value of the increment.
 * 0: No increment
 * 1: Automatic increment
*/
#define INC_CHAN                    0x80


/**
  * @}
  */

/**
  * @}
  */

/** @defgroup RADIO_Exported_Types RADIO Exported Types
  * @{
  */
   
/**
  * @brief Radio Global State Machine description
  */

typedef struct {
    volatile uint32_t RADIOCONFIGPTR;
    volatile uint8_t BYTE4;
    volatile uint8_t WAKEUPINITDELAY;
    volatile uint8_t TIMER12INITDELAYCAL;
    volatile uint8_t TIMER2INITDELAYNOCAL;
    volatile uint8_t TRANSMITCALDELAYCHK;
    volatile uint8_t TRANSMITNOCALDELAYCHK;
    volatile uint8_t RECEIVECALDELAYCHK;
    volatile uint8_t RECEIVENOCALDELAYCHK;
    volatile uint8_t CONFIGENDDURATION;
    volatile uint8_t TXDATAREADYCHECK;
    volatile uint8_t TXDELAYSTART;
    volatile uint8_t BYTE15;
    volatile uint8_t TXREADYTIMEOUT;
    volatile uint8_t RCVTIMEOUT[3];
    volatile uint8_t BYTE20;
    volatile uint8_t BYTE21;
    volatile uint8_t BYTE22;
    volatile uint8_t BYTE23;
    volatile uint32_t RESERVED; 
} GLOBALSTATMACH_TypeDef;

/**
  * @brief Radio Link State Machine description
  */

typedef struct {
    volatile uint8_t BYTE0;
    volatile uint8_t BYTE1;
    volatile uint8_t BYTE2;
    volatile uint8_t BYTE3;
    volatile uint32_t TXPOINT;
    volatile uint32_t RCVPOINT;
    volatile uint32_t TXPOINTPREV;
    volatile uint32_t RCVPOINTPREV;
    volatile uint32_t TXPOINTNEXT;
    volatile uint8_t PCNTTX[5];
    volatile uint8_t PCNTRCV[5];
    volatile uint8_t BYTE34;
    volatile uint8_t BYTE35;
    volatile uint32_t ACCADDR;
    volatile uint8_t CRCINIT[3];
    volatile uint8_t MAXRECEIVEDLENGTH;
    volatile uint8_t PAPOWER;
    volatile uint8_t HOPINCR;
    volatile uint8_t USEDCHANNELFLAGS[5];
    volatile uint8_t RESERVED;
    volatile uint16_t CONNEVENTCOUNTER;
    volatile uint16_t PAEVENTCOUNTER;
    volatile uint8_t ENCRYPTIV[8];
    volatile uint8_t ENCRYPTK[16];
} STATMACH_TypeDef;

/**
  * @brief Radio TxRxPack description
  */

typedef struct {
    volatile uint32_t NEXTPTR;
    volatile uint8_t  BYTE4;
    volatile uint8_t  BYTE5;
    volatile uint16_t RESERVED;
    volatile uint32_t DATAPTR;
    volatile uint8_t TIMER2[2];
    volatile uint8_t BYTE14;
    volatile uint8_t BYTE15;
    volatile uint32_t SUPPLEMENTPTR;
} TXRXPACK_TypeDef;
   

typedef enum {
  STATE_MACHINE_0 = 0,
  STATE_MACHINE_1,
  STATE_MACHINE_2,
  STATE_MACHINE_3,
  STATE_MACHINE_4,
  STATE_MACHINE_5,
  STATE_MACHINE_6,
  STATE_MACHINE_7,
} StateMachine_t; 

typedef struct ActionPacket ActionPacket;


typedef struct {
    uint32_t back2backTime;
    uint8_t tone_start_stop_flag;
    ActionPacket* current_action_packet;    
}RadioGlobalParameters_t;

struct ActionPacket
{
  uint8_t StateMachineNo ;      /* This parameter indicates the state machine number for this action. From 0 to 7. */
  uint8_t ActionTag;            /* The configuration of the current action. 
                                 * Action Tag: PLL_TRIG, TXRX, TIMER_WAKEUP, WHITENING_DISABLE, INC_CHAN, TIMESTAMP_POSITION, RELATIVE */
  uint8_t MaxReceiveLength;     /* Maximum size of received payload. Applicable only for RX actions. */
  uint32_t WakeupTime;          /* Contains the wakeup time in microsecond if it is relative.
                                 * If it is absolute it must be expressed in System Time (STU). */
  uint8_t *data;                /* Pointer to the array with the data to send (header, length and data field), for TX.
                                 * Pointer to the array where the data received are copied, for RX.
                                 * In case of RX, the array must have the max size MAX_PACKET_LENGTH. */
  uint32_t status;              /* The Status Register with the information on the action. */
  uint32_t timestamp_receive;   /* This field contains the timestamp when a packet is received.
                                 * Intended to be used in the dataRoutine() callback routine. RX only. */
  int32_t rssi;                 /* The rssi of the packet was received with. RX only. */
  BlueTransStruct trans_packet; /* This is a linked list, which is going to be used by Hardware.
                                  * User does not need to do anything. */
  ActionPacket *next_true;      /* Pointer to next ActionPacket if condRoutine() returns TRUE */
  ActionPacket *next_false;     /* Pointer to next ActionPacket if condRoutine() returns FALSE */
  uint8_t (*condRoutine)(ActionPacket*);        /* User callback that decide the next ActionPacket to use.
                                                 * It is time critical. Routine must end within 45 us. */
  uint8_t (*dataRoutine)(ActionPacket*, ActionPacket*); /* User callback for managing data. */
  uint8_t trans_config;                                 /* This is for configuring the device for TX or RX. User does not need to do anything. */
};

/**
  * @}
  */


/** @defgroup RADIO_Exported_Functions RADIO Exported Functions
  * @{
  */

void RADIO_Init(void);
uint8_t RADIO_GetStatus(uint32_t *time);
void RADIO_SetChannelMap(uint8_t StateMachineNo,uint8_t *chan_remap);
void RADIO_SetChannel(uint8_t StateMachineNo, uint8_t channel,uint8_t channel_increment); 
void RADIO_SetTxAttributes(uint8_t StateMachineNo, uint32_t NetworkID, uint32_t crc_init);
void RADIO_SetMaxReceivedLength(uint8_t StateMachineNo, uint8_t MaxReceivedLength);
void RADIO_SetBackToBackTime(uint32_t back_to_back_time);  
void RADIO_SetPhy(uint8_t StateMachineNo, uint8_t phy);
void RADIO_SetTxPower(uint8_t PowerLevel);    
void RADIO_IRQHandler(void);
uint8_t RADIO_StopActivity(void);
void RADIO_SetGlobalReceiveTimeout(uint32_t ReceiveTimeout);
void RADIO_SetReservedArea(ActionPacket *p); 
uint8_t RADIO_MakeActionPacketPending(ActionPacket *p);
uint8_t RADIO_StartTone(uint8_t RF_channel, uint8_t powerLevel, uint8_t freq_offset);
uint8_t RADIO_StopTone(void);
void RADIO_SetEncryptionCount(uint8_t StateMachineNo, uint8_t *count_tx, uint8_t *count_rcv);    
void RADIO_SetEncryptionAttributes(uint8_t StateMachineNo, uint8_t *enc_iv, uint8_t *enc_key); 
void RADIO_SetEncryptFlags(uint8_t StateMachineNo, FunctionalState EncryptFlagTx, FunctionalState EncryptFlagRcv);
void RADIO_EncryptPlainData(uint8_t *Key, uint8_t *plainData, uint8_t *cypherData);
void RADIO_SetDefaultPreambleLen(uint8_t StateMachineNo);
void RADIO_SetPreambleRep(uint8_t StateMachineNo, uint8_t PreaLen);
void RADIO_DisableCRC(uint8_t StateMachineNo, FunctionalState hwCRC);

/**
  * @}
  */

/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /*BLUENRG_LP_LL_RADIO_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
