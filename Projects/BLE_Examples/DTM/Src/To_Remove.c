/**************************************************************
**
**
** This file shall be removed and change the stack function call 
** with the correct low level driver function API
**
**
****************************************************************/

#include "bluenrg_lp_ll_flash.h"
#include "bluenrg_lp_ll_pka.h"


void FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  LL_FLASH_Program(FLASH, Address, Data);
}

void FLASH_ErasePage(uint16_t PageNumber)
{
  LL_FLASH_Erase(FLASH, LL_FLASH_TYPE_ERASE_PAGES, PageNumber, 1);
}

ErrorStatus PKA_SetData(uint8_t dataType, uint32_t* srcData)
{
  return LL_PKA_SetData(dataType, srcData);
}

void PKA_StartProcessing(void)
{
  LL_PKA_Start(PKA);
}

void PKA_Reset(void)
{
   LL_PKA_SWReset(PKA);
}

FlagStatus PKA_GetProcessStatus(void)
{
  return (FlagStatus)LL_PKA_Ready(PKA);
}

ErrorStatus PKA_VerifyProcess(void)
{
  return LL_PKA_VerifyProcess();
}

ErrorStatus PKA_GetData(uint8_t dataType, uint8_t* dataTarget)
{
  return LL_PKA_GetData(dataType, dataTarget);
}
