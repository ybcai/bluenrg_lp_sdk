
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : Micro/Hello_World/Micro_Hello_World_main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : 25-March-2019
* Description        : Code demonstrating Hello World with BlueNRG-LP
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "bluenrg_lp_it.h"
#include "bluenrg_lp_evb_com.h"
#include "hal_miscutil.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples BlueNRG-LP LL Peripheral Examples
  * @{
  */


/** @addtogroup Micro_Examples Micro Examples
  * @{
  */

/** @addtogroup Micro_HelloWorld  Micro Hello World Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t counter = 0; 
  PartInfoType partInfo;
  crash_info_t crashInfo;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) 
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);
  
  HAL_GetPartInfo(&partInfo);
  
  HAL_GetCrashInfo(&crashInfo);
  if ((crashInfo.signature & 0xFFFF0000) == CRASH_SIGNATURE_BASE) {
    printf("Application crash detected\r\n");
    printf("Crash Info signature = 0x%08lx\r\n", crashInfo.signature);
    printf("Crash Info SP        = 0x%08lx\r\n", crashInfo.SP);
    printf("Crash Info R0        = 0x%08lx\r\n", crashInfo.R0);
    printf("Crash Info R1        = 0x%08lx\r\n", crashInfo.R1);
    printf("Crash Info R2        = 0x%08lx\r\n", crashInfo.R2);
    printf("Crash Info R3        = 0x%08lx\r\n", crashInfo.R3);
    printf("Crash Info R12       = 0x%08lx\r\n", crashInfo.R12);
    printf("Crash Info LR        = 0x%08lx\r\n", crashInfo.LR);
    printf("Crash Info PC        = 0x%08lx\r\n", crashInfo.PC);
    printf("Crash Info xPSR      = 0x%08lx\r\n", crashInfo.xPSR);
  }
  /* infinite loop */
  while(1) 
  {
    if (counter == 0 ) {
      printf("Hello World: BlueNRG-LP (%d.%d) is here!\r\n",
             partInfo.die_major,
             partInfo.die_cut);
    }
    counter = (counter +1) & (0xFFFFF);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
