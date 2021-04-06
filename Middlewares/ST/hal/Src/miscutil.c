/**
******************************************************************************
* @file    miscutil.c 
* @author  AMS - RF Application Team
* @version V1.0.0
* @date    3-April-2019
* @brief   Miscellaneous utilities for radio HW
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
* <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
******************************************************************************
*/ 
/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_ll_utils.h"
#include "bluenrg_lp_ll_bus.h"
#include "system_BlueNRG_LP.h"
#include "miscutil.h"
#include "hal_miscutil.h"
#include "bleplat.h"

/** @addtogroup BlueNRG_LP_Miscellaneous_Utilities
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** Minimum supported TX power in dBm. */
#define MIN_TX_POWER    (-40)
/** Maximum supported TX power in dBm. */
#define MAX_TX_POWER    6

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

/**
 * @brief Get Device ID, Version and Revision numbers
 */
void BLEPLAT_get_part_info(uint8_t *device_id, uint8_t *major_cut, uint8_t *minor_cut)
{
   PartInfoType partInfo;
   
   /* get partInfo */
   HAL_GetPartInfo(&partInfo);
  
  /* Set device ID */
  *device_id  = partInfo.die_id;
  
  /* Set major cut  */
  *major_cut = partInfo.die_major; 
 
  /* Set minor cut */
  *minor_cut = partInfo.die_cut;
}

/* Expected TX output power (dBm) for each PA level when SMPS voltage is 1.4V */
const int8_t normal_pa_level_table[32] = { -54, -21, -20, -19, -17, -16, -15, -14, -13,
  -12, -11, -10, -9, -8, -7, -6, -6, -4, -3, -3, -2, -2, -1, -1, 0, 0, 1, 2, 3, 4,
  5, 6};

/* Expected TX output power (dBm) for each PA level when SMPS voltage is 1.9V
   (high power mode). */
const int8_t high_power_pa_level_table[32] = { -54, -19, -18, -17, -16, -15, -14, -13, -12,
  -11, -10, -9, -8, -7, -6, -5, -4, -3, -3, -2, -1, 0, 1, 2, 3, 8, 8, 8, 8, 8,
  8, 8};

uint8_t BLEPLAT_DBmToPALevel(int8_t TX_dBm, uint8_t high_power)
{
  uint8_t i;
  const int8_t *pa_level_table = high_power?high_power_pa_level_table:normal_pa_level_table;
  
  for(i = 0; i < sizeof(normal_pa_level_table); i++)
  {
    if(pa_level_table[i] > TX_dBm)
      break;
  }
  if(i > 0)
  {
    i--;
  }
  
  return i;  
}

int8_t BLEPLAT_PALevelToDBm(uint8_t PA_Level, uint8_t high_power)
{
  const int8_t *pa_level_table = high_power?high_power_pa_level_table:normal_pa_level_table;
  
  if(PA_Level >= sizeof(normal_pa_level_table))
    return 127;
  return pa_level_table[PA_Level];
}

void BLEPLAT_ReadTransmitPower(int8_t *Min_Tx_Power, int8_t *Max_Tx_Power)
{
  *Min_Tx_Power = MIN_TX_POWER;
  *Max_Tx_Power = MAX_TX_POWER;    
}

void BLEPLAT_SetHighPower(uint8_t enable)
{
  HAL_SetHighPower((FunctionalState)enable);
}

void BLEPLAT_RadioControllerReset(void)
{
  LL_APB2_ForceReset(LL_APB2_PERIPH_MRBLE);
  LL_APB2_ReleaseReset(LL_APB2_PERIPH_MRBLE);
  MrBleBiasTrimConfig(FALSE); // Restore configuration, lost after controller reset.  
}

/** 
 *@
} */ /* End of group BlueNRG_LP_Miscellaneous_Utilities */
