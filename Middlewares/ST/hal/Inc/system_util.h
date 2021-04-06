/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name         : system_util.h
* Author             : AMS - VMA RF Application team 
* Version            : V1.0.0
* Date               : 14-September-2015
* Description        : System macros and typedefs.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup bluenrg_lp_system
  * @{
  */  

#ifndef __SYSTEM_UTIL_H__
#define __SYSTEM_UTIL_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include "compiler.h"
#include "stdint.h"
 
/** @addtogroup Exported types
  * @{
  */

typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;
/**@brief Macro that checks if STATE is a FlagStatus / ITStatus */
#define IS_FLAGSTATUS(STATE) (((STATE) == RESET) || ((STATE) == SET))
#define IS_ITSTATUS(STATE)   IS_FLAGSTATUS(STATE)

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
/** @brief Macro that checks if STATE is a FunctionalState */
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  SUCCESS = 0,
  ERROR = !SUCCESS
} ErrorStatus;
/** @brief Macro that checks if STATE is a ErrorStatus */
#define IS_ERROR_STATE(STATE) (((STATE) == SUCCESS) || ((STATE) == ERROR))

/** @brief Macro that returns a 16-bit value from a buffer where the value is stored in Little Endian Format */
#define LE_TO_HOST_16(ptr)   (uint16_t) ( ((uint16_t)*((uint8_t *)(ptr))) | \
                                           ((uint16_t)*((uint8_t *)(ptr) + 1) << 8) )

/** @brief Macro that stores a 16-bit value into a buffer in Little Endian Format (2 bytes) */
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) ((val)>>8) ) )

/** @brief Macro that returns a 32-bit value from a buffer where the value is stored in Little Endian Format */
#define LE_TO_HOST_32(ptr)   (uint32_t) ( ((uint32_t)*((uint8_t *)(ptr))) | \
                                           ((uint32_t)*((uint8_t *)(ptr) + 1) << 8)  | \
                                           ((uint32_t)*((uint8_t *)(ptr) + 2) << 16) | \
                                           ((uint32_t)*((uint8_t *)(ptr) + 3) << 24) )

/** @brief Macro that stores a 32-bit value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) ((val)>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) ((val)>>16) ) , \
                                   ((buf)[3] =  (uint8_t) ((val)>>24) ) ) 

/** @brief Booelan definition */
typedef uint8_t BOOL;

/**@brief TRUE, FALSE definition */
#define TRUE  ((BOOL)1U)
#define FALSE ((BOOL)0U)

/**
  * @}
  */

/** @addtogroup BlueNRG_LP_System_Exported_Macros
  * @{
  */

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         (REG)

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), ((READ_REG(REG) & (~(CLEARMASK))) | (SETMASK)))

#define MODIFY_REG_FIELD(REG, FIELD_NAME, VAL)  MODIFY_REG(REG, FIELD_NAME##_Msk, ((VAL)<<(FIELD_NAME##_Pos))&(FIELD_NAME##_Msk))

/**
  * @}
  */


#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_UTIL_H__ */

/**
 * @}
 */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
