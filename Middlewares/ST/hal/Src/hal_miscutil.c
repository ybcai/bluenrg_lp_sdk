/**
******************************************************************************
* @file    hal_miscutils.c 
* @author  AMG - RF Application Team
* @version V1.1.0
* @date    3-April-2018
* @brief   Miscellaneous utilities for interfacing to  HW
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
#include "system_BlueNRG_LP.h"
#include "hal_miscutil.h"
#include "bluenrg_lp_ll_pwr.h"

NO_INIT_SECTION(crash_info_t CrashInfoRam, ".crash_info_ram_vr");

/** @addtogroup BlueNRG_LP_Miscellaneous_Utilities
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void HAL_GetPartInfo(PartInfoType *partInfo)
{ 
  partInfo->die_id        =  DIE_ID_BLUENRG_LP;
  partInfo->die_major     =  LL_SYSCFG_GetDeviceVersion(); 
  partInfo->die_cut       =  LL_SYSCFG_GetDeviceRevision(); 
  partInfo->jtag_id_code  =  LL_SYSCFG_GetDeviceJTAG_ID();
  partInfo->flash_size    =  (LL_GetFlashSize() + 1) * 4;
  partInfo->ram_size      =  (LL_GetRAMSize() + 1) * 16 * 1024;
}

/**
 * @brief Get Crash Information utility
 */
void HAL_GetCrashInfo(crash_info_t *crashInfo)
{
  *crashInfo = CrashInfoRam;
  /* Reset crash info value */
  CrashInfoRam.signature = 0;
}
void HAL_CrashHandler(uint32_t msp, uint32_t signature)
{
  volatile uint32_t * crash_info = (volatile uint32_t *)&CrashInfoRam;
  register uint32_t reg_content;
  /* Init to zero the crash_info RAM locations */
  for (reg_content=0; reg_content<NMB_OF_EXCEP_RAM_WORD; reg_content++) {
    crash_info[reg_content] = 0;
  }
  /* Store Crash Signature */
  CrashInfoRam.signature = signature;
  /* Store SP register */
  CrashInfoRam.SP = msp;
  for (reg_content=2; reg_content<NMB_OF_EXCEP_RAM_WORD; reg_content++) {
    uint32_t *ptr = ((uint32_t *)msp)+(reg_content-2);
    if ((ptr >= ((uint32_t *)  _MEMORY_RAM_BEGIN_)) && 
        (ptr <= ((uint32_t *) _MEMORY_RAM_END_)))
      crash_info[reg_content] = *ptr;
  }
  NVIC_SystemReset();
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

void HAL_SetHighPower(FunctionalState state)
{
  if(state != DISABLE)
  {
    MODIFY_REG(RRM->LDO_ANA_ENG, 0, RRM_LDO_ANA_ENG_RFD_LDO_TRANSFO_BYPASS_Msk);
    LL_PWR_SetSMPSOutputLevel(LL_PWR_SMPS_OUTLVL_1V9);
  }
  else
  {
    MODIFY_REG(RRM->LDO_ANA_ENG, RRM_LDO_ANA_ENG_RFD_LDO_TRANSFO_BYPASS_Msk, 0);
    LL_PWR_SetSMPSOutputLevel(LL_PWR_SMPS_OUTLVL_1V4);
  }
}



/** 
 *@
} */ /* End of group BlueNRG_LP_Miscellaneous_Utilities */
