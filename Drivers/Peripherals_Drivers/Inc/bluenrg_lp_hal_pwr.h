/**
  ******************************************************************************
  * @file    bluenrg_lp_hal_pwr.h
  * @author  RF Application Team
  * @brief   Header file of PWR HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics. 
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
#ifndef BLUENRG_LP_HAL_PWR_H
#define BLUENRG_LP_HAL_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal_def.h"

/* Include low level driver */
#include "bluenrg_lp_ll_pwr.h"
#include "bluenrg_lp_ll_exti.h"

/** @addtogroup BLUENRG_LP_HAL_Driver
  * @{
  */

/** @defgroup PWR PWR
  * @brief PWR HAL module driver
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup PWR_Exported_Types PWR Exported Types
  * @{
  */

/**
  * @brief  PWR PVD configuration structure definition
  */
typedef struct
{
  uint32_t PVDLevel;       /*!< PVDLevel: Specifies the PVD detection level.
                                This parameter can be a value of @ref PWR_PVD_detection_level. */

  uint32_t Mode;           /*!< Mode: Specifies the operating mode for the selected pins.
                                This parameter can be a value of @ref PWR_PVD_Mode. */
}PWR_PVDTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWR_Exported_Constants PWR Exported Constants
  * @{
  */

/** @defgroup PWR_PVD_detection_level  Power Voltage Detector Level selection
  * @note     Refer datasheet for selection voltage value
  * @{
  */
#define PWR_PVDLEVEL_0                  LL_PWR_PVDLEVEL_0   /*!< PVD threshold around 2.0 V */
#define PWR_PVDLEVEL_1                  LL_PWR_PVDLEVEL_1   /*!< PVD threshold around 2.2 V */
#define PWR_PVDLEVEL_2                  LL_PWR_PVDLEVEL_2   /*!< PVD threshold around 2.4 V */
#define PWR_PVDLEVEL_3                  LL_PWR_PVDLEVEL_3   /*!< PVD threshold around 2.5 V */
#define PWR_PVDLEVEL_4                  LL_PWR_PVDLEVEL_4   /*!< PVD threshold around 2.6 V */
#define PWR_PVDLEVEL_5                  LL_PWR_PVDLEVEL_5   /*!< PVD threshold around 2.8 V */
#define PWR_PVDLEVEL_6                  LL_PWR_PVDLEVEL_6   /*!< PVD threshold around 2.9 V */
#define PWR_PVDLEVEL_7                  LL_PWR_PVDLEVEL_7   /*!< External input analog voltage (compared internally to VREFINT) */
/**
  * @}
  */

/** @defgroup PWR_PVD_Mode  PWR PVD interrupt and event mode
  * @{
  */
#define PWR_PVD_MODE_NORMAL                 (0x00000000U)                  /*!< Basic mode is used */
#define PWR_PVD_MODE_IT                     (SYSCFG_PWRC_IER_PVD_IE)        /*!< Interrupt Mode detection */
/**
  * @}
  */

/** @defgroup PWR_Low_Power_Mode_Selection  PWR Low Power Mode Selection
  * @{
  */
#define PWR_MODE_DEEPSTOP                   LL_PWR_MODE_DEEPSTOP
#define PWR_MODE_SHUTDOWN                   LL_PWR_MODE_SHUTDOWN
/**
  * @}
  */

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/ 
/* Exported macros -----------------------------------------------------------*/
/** @defgroup PWR_Exported_Macros  PWR Exported Macros
  * @{
  */
/** @brief  Check whether or not a specific PWR flag is set.
  * @param __FLAG__ specifies the flag to check.
  *           This parameter can be one of the following values:
  *
  *            /--------------------------------SR1-------------------------------/
  *            @arg @ref PWR_FLAG_WPB0  Wake Up Flag 0. Indicates that a wakeup event
  *                                     was received from the PB0 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB1  Wake Up Flag 1. Indicates that a wakeup event
  *                                     was received from the PB1 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB2  Wake Up Flag 2. Indicates that a wakeup event
  *                                     was received from the PB2 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB3  Wake Up Flag 3. Indicates that a wakeup event
  *                                     was received from the PB3 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB4  Wake Up Flag 4. Indicates that a wakeup event
  *                                     was received from the PB4 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB5  Wake Up Flag 5. Indicates that a wakeup event
  *                                     was received from the PB5 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB6  Wake Up Flag 6. Indicates that a wakeup event
  *                                     was received from the PB6 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB7  Wake Up Flag 7. Indicates that a wakeup event
  *                                     was received from the PB7 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA8  Wake Up Flag 8. Indicates that a wakeup event
  *                                     was received from the PA8 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA9  Wake Up Flag 9. Indicates that a wakeup event
  *                                     was received from the PA9 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA10 Wake Up Flag 10. Indicates that a wakeup event
  *                                     was received from the PA10 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA11 Wake Up Flag 11. Indicates that a wakeup event
  *                                     was received from the PA11 I/O wakeup pin.
  *
  *            @arg @ref PWR_FLAG_WBLE        BLE WakeUp Flag
  *            @arg @ref PWR_FLAG_WBLEHCPU    BLE_Host CPU WakeUp Flag
  *            @arg @ref PWR_FLAG_RTC         Internal Wakeup Flag (RTC)
  *
  *            /--------------------------------SR2-------------------------------/
  *            @arg @ref PWR_FLAG_SMPSBYP  SMPS Bypass Flag
  *            @arg @ref PWR_FLAG_SMPSENR  SMPS Run mode Flag
  *            @arg @ref PWR_FLAG_SMPSRDY  SMPS Ready Flag
  *
  *            @arg @ref PWR_FLAG_REGLPS    Low Power regulator ready Flag
  *            @arg @ref PWR_FLAG_REGMS     Main regulator ready Flag
  *
  *            @arg @ref PWR_FLAG_PVDO   Power Voltage Detector Output. Indicates whether VDD voltage is
  *                                      below or above the selected PVD threshold.
  *
  *            @arg @ref PWR_FLAG_IOBOOTVAL0  PA8  input value latched at POR
  *            @arg @ref PWR_FLAG_IOBOOTVAL1  PA9  input value latched at POR
  *            @arg @ref PWR_FLAG_IOBOOTVAL2  PA10 input value latched at POR
  *            @arg @ref PWR_FLAG_IOBOOTVAL3  PA11 input value latched at POR
  *
  *           /----------------------------EXTSRR--------------------------/
  *            @arg @ref PWR_FLAG_DEEPSTOPF              System DEEPTSTOP Flag
  *            @arg @ref PWR_FLAG_RFPHASEF               Critical radio system phase Flag
  *
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */  
#define __HAL_PWR_GET_FLAG(__FLAG__)  ( ((((uint8_t)(__FLAG__)) >> 4U) == 1U)  ?\
                                      (PWR->SR1 & (1U << ((__FLAG__) & 15U))) :\
                                      ((((((uint8_t)(__FLAG__)) >> 4U) == 2U)) ?\
                                      (PWR->SR2 & (1U << ((__FLAG__) & 15U))) :\
                                      (PWR->EXTSRR & (1U << ((__FLAG__) & 15U))) ) )


/** @brief  Clear a specific PWR flag.
  * @param __FLAG__ specifies the flag to clear.
  *          This parameter can be one of the following values:
  *
  *            /--------------------------------SR1-------------------------------/
  *            @arg @ref PWR_FLAG_WPB0  Wake Up Flag 0. Indicates that a wakeup event
  *                                     was received from the PB0 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB1  Wake Up Flag 1. Indicates that a wakeup event
  *                                     was received from the PB1 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB2  Wake Up Flag 2. Indicates that a wakeup event
  *                                     was received from the PB2 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB3  Wake Up Flag 3. Indicates that a wakeup event
  *                                     was received from the PB3 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB4  Wake Up Flag 4. Indicates that a wakeup event
  *                                     was received from the PB4 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB5  Wake Up Flag 5. Indicates that a wakeup event
  *                                     was received from the PB5 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB6  Wake Up Flag 6. Indicates that a wakeup event
  *                                     was received from the PB6 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPB7  Wake Up Flag 7. Indicates that a wakeup event
  *                                     was received from the PB7 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA8  Wake Up Flag 8. Indicates that a wakeup event
  *                                     was received from the PA8 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA9  Wake Up Flag 9. Indicates that a wakeup event
  *                                     was received from the PA9 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA10 Wake Up Flag 10. Indicates that a wakeup event
  *                                     was received from the PA10 I/O wakeup pin.
  *            @arg @ref PWR_FLAG_WPA11 Wake Up Flag 11. Indicates that a wakeup event
  *                                     was received from the PA11 I/O wakeup pin.
  *
  *            @arg @ref PWR_FLAG_WBLE        BLE WakeUp Flag
  *            @arg @ref PWR_FLAG_WBLEHCPUF   BLE_Host CPU WakeUp Flag
  *
  *           /----------------------------EXTSRR--------------------------/
  *            @arg @ref PWR_FLAG_DEEPSTOPF              System DEEPTSTOP Flag
  *            @arg @ref PWR_FLAG_RFPHASEF               Critical radio system phase Flag
  *
  * @retval None   
  */
#define __HAL_PWR_CLEAR_FLAG(__FLAG__)   ( ((((uint8_t)(__FLAG__)) >> 4U) == 1U) ?\
                                           SET_BIT(PWR->SR, (1U << ((__FLAG__) & 15U))) :\
                                           ((((uint8_t)(__FLAG__)) >> 4U) == 2U) ? \
                                           SET_BIT(PWR->EXTSRR, (1U << ((__FLAG__) & 15U))) )
                                       
/**
  * @brief Enable the PVD Line.
  * @retval None
  */
#define __HAL_PWR_PVD_ENABLE_IT()   SET_BIT(SYSCFG->PWRC_IER, SYSCFG_PWRC_IER_PVD_IE)

/**
  * @brief Disable the PVD Interrupt Line.
  * @retval None
  */
#define __HAL_PWR_PVD_DISABLE_IT()  CLEAR_BIT(SYSCFG->PWRC_IER, SYSCFG_PWRC_IER_PVD_IE)


/**
  * @brief Check whether or not the PVD interrupt flag is set.
  * @retval PVD Line Status.
  */
#define __HAL_PWR_PVD_GET_FLAG()  (READ_BIT(SYSCFG->PWRC_ISCR, SYSCFG_PWRC_ISCR_PVD_ISC) == (SYSCFG_PWRC_ISCR_PVD_ISC)) ? 1UL : 0U

/**
  * @brief Clear the PVD interrupt flag.
  * @retval None
  */
#define __HAL_PWR_PVD_CLEAR_FLAG()  SET_BIT(SYSCFG->PWRC_ISCR, SYSCFG_PWRC_ISCR_PVD_ISC)

/**
  * @}
  */
  

/* Private macros --------------------------------------------------------*/
/** @defgroup PWR_Private_Macros  PWR Private Macros
  * @{
  */

#define IS_PWR_PVD_LEVEL(LEVEL) (((LEVEL) == PWR_PVDLEVEL_0) || ((LEVEL) == PWR_PVDLEVEL_1)|| \
                                 ((LEVEL) == PWR_PVDLEVEL_2) || ((LEVEL) == PWR_PVDLEVEL_3)|| \
                                 ((LEVEL) == PWR_PVDLEVEL_4) || ((LEVEL) == PWR_PVDLEVEL_5)|| \
                                 ((LEVEL) == PWR_PVDLEVEL_6) || ((LEVEL) == PWR_PVDLEVEL_7))

                                 
#define IS_PWR_PVD_MODE(MODE)  (((MODE) == PWR_PVD_MODE_NORMAL)              ||\
                                ((MODE) == PWR_PVD_MODE_IT))


#define IS_PWR_MODE(MODE)  (((MODE) == PWR_MODE_DEEPSTOP)              ||\
                            ((MODE) == PWR_MODE_SHUTDOWN))

/**
  * @}
  */

/* Include PWR HAL Extended module */
#include "bluenrg_lp_hal_pwr_ex.h"

/* Exported functions --------------------------------------------------------*/
/** @defgroup PWR_Exported_Functions  PWR Exported Functions
  * @{
  */

/** @defgroup PWR_Exported_Functions_Group1  Initialization and de-initialization functions 
  * @{
  */

/* Initialization and de-initialization functions *******************************/
void              HAL_PWR_DeInit(void);

/**
  * @}
  */

/** @defgroup PWR_Exported_Functions_Group2  Peripheral Control functions
  * @{
  */
/* Peripheral Control functions  ************************************************/
HAL_StatusTypeDef HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void              HAL_PWR_EnablePVD(void);
void              HAL_PWR_DisablePVD(void);
void              HAL_PWR_PVDCallback(void);
   
/* WakeUp pins configuration functions ****************************************/
void              HAL_PWR_EnableWakeUpSource(uint32_t WakeUpSource, uint32_t WakeUpPinPolarity);
void              HAL_PWR_DisableWakeUpSource(uint32_t WakeUpSource);

/* Low Power modes configuration functions ************************************/
void              HAL_PWR_EnterDEEPSTOPMode(void);
void              HAL_PWR_EnterSHUTDOWNMode(void);

/**
  * @}
  */

/**
  * @}
  */

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


#endif /* BLUENRG_LP_HAL_PWR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
