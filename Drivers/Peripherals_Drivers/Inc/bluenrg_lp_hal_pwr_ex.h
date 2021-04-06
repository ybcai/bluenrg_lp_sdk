/**
  ******************************************************************************
  * @file    bluenrg_lp_hal_pwr_ex.h
  * @author  RF Application Team
  * @brief   Header file of PWR HAL Extended module.
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
#ifndef BLUENRG_LP_HAL_PWR_EX_H
#define BLUENRG_LP_HAL_PWR_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal_def.h"

/** @addtogroup BLUENRG_LP_HAL_Driver
  * @{
  */

/** @defgroup PWREx PWREx
  * @brief PWR Extended HAL module driver
  * @{
  */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/** @defgroup PWREx_Exported_Constants  PWR Extended Exported Constants
  * @{
  */

/** @defgroup PWREx_WakeUp_Sources  PWR wake-up sources
  * @{
  */
#define PWR_WAKEUP_PB0            PWR_CR3_EWU0       /*!< Wakeup pin PB0 */
#define PWR_WAKEUP_PB1            PWR_CR3_EWU1       /*!< Wakeup pin PB1 */
#define PWR_WAKEUP_PB2            PWR_CR3_EWU2       /*!< Wakeup pin PB2 */
#define PWR_WAKEUP_PB3            PWR_CR3_EWU3       /*!< Wakeup pin PB3 */
#define PWR_WAKEUP_PB4            PWR_CR3_EWU4       /*!< Wakeup pin PB4 */
#define PWR_WAKEUP_PB5            PWR_CR3_EWU5       /*!< Wakeup pin PB5 */
#define PWR_WAKEUP_PB6            PWR_CR3_EWU6       /*!< Wakeup pin PB6 */
#define PWR_WAKEUP_PB7            PWR_CR3_EWU7       /*!< Wakeup pin PB7 */
#define PWR_WAKEUP_PB8            PWR_CR3_EWU8       /*!< Wakeup pin PB8 */
#define PWR_WAKEUP_PA9            PWR_CR3_EWU9       /*!< Wakeup pin PA9 */
#define PWR_WAKEUP_PA10           PWR_CR3_EWU10      /*!< Wakeup pin PA10 */
#define PWR_WAKEUP_PA11           PWR_CR3_EWU11      /*!< Wakeup pin PA11 */
#define PWR_WAKEUP_BLE            PWR_CR3_EWBLE      /*!< Wakeup on BLE event */
#define PWR_WAKEUP_BLEHCPU        PWR_CR3_EWBLEHCPU  /*!< Wakeup on BLE Host CPU event */
#define PWR_WAKEUP_RTC            PWR_CR3_EIWL       /*!< Wakeup on Internal event (RTC) */
/**
  * @}
  */

/** @defgroup PWREx_WakeUp_Polarity PWR wake-up Polarity configuration
  * @{
  */
#define PWR_WAKEUP_POLARITY_HIGH 0x00000000U
#define PWR_WKAEUP_POLARITY_LOW  0x00000001U
/**
  * @}
  */

/** @defgroup PWREx_PIN_Polarity PWREx Pin Polarity configuration
  * @{
  */
#define PWR_PIN_POLARITY_HIGH 0x00000000U
#define PWR_PIN_POLARITY_LOW  0x00000001U
/**
  * @}
  */

/** @defgroup PWREx_GPIO_Bit_Number GPIO bit number for I/O setting in DeepStop/Shutdown mode
  * @{
  */
#define PWR_GPIO_BIT_0   PWR_PUCRA_PA0    /*!< GPIO port I/O pin 0  */
#define PWR_GPIO_BIT_1   PWR_PUCRA_PA1    /*!< GPIO port I/O pin 1  */
#define PWR_GPIO_BIT_2   PWR_PUCRA_PA2    /*!< GPIO port I/O pin 2  */
#define PWR_GPIO_BIT_3   PWR_PUCRA_PA3    /*!< GPIO port I/O pin 3  */
#define PWR_GPIO_BIT_4   PWR_PUCRA_PA4    /*!< GPIO port I/O pin 4  */
#define PWR_GPIO_BIT_5   PWR_PUCRA_PA5    /*!< GPIO port I/O pin 5  */
#define PWR_GPIO_BIT_6   PWR_PUCRA_PA6    /*!< GPIO port I/O pin 6  */
#define PWR_GPIO_BIT_7   PWR_PUCRA_PA7    /*!< GPIO port I/O pin 7  */
#define PWR_GPIO_BIT_8   PWR_PUCRA_PA8    /*!< GPIO port I/O pin 8  */
#define PWR_GPIO_BIT_9   PWR_PUCRA_PA9    /*!< GPIO port I/O pin 9  */
#define PWR_GPIO_BIT_10  PWR_PUCRA_PA10   /*!< GPIO port I/O pin 10 */
#define PWR_GPIO_BIT_11  PWR_PUCRA_PA11   /*!< GPIO port I/O pin 11 */
#define PWR_GPIO_BIT_12  PWR_PUCRA_PA12   /*!< GPIO port I/O pin 12 */
#define PWR_GPIO_BIT_13  PWR_PUCRA_PA13   /*!< GPIO port I/O pin 14 */
#define PWR_GPIO_BIT_14  PWR_PUCRA_PA14   /*!< GPIO port I/O pin 14 */
#define PWR_GPIO_BIT_15  PWR_PUCRA_PA15   /*!< GPIO port I/O pin 15 */
/**
  * @}
  */
  
/** @defgroup PWREx_GPIO GPIO port
  * @{
  */
#define PWR_GPIO_A   0x00000000U      /*!< GPIO port A */
#define PWR_GPIO_B   0x00000001U      /*!< GPIO port B */
/**
  * @}
  */

/** @defgroup PWREx_RAMRetention_BanK RAM bank retention in DEEPSTOP mode
  * @{
  */
#define PWR_RAMRET_BANK1   LL_PWR_RAMRET_1     /*!< RAM1 bank retention in DEEPSTOP mode */
#define PWR_RAMRET_BANK2   LL_PWR_RAMRET_2     /*!< RAM2 bank retention in DEEPSTOP mode */
#define PWR_RAMRET_BANK3   LL_PWR_RAMRET_3     /*!< RAM3 bank retention in DEEPSTOP mode */
/**
  * @}
  */

/** @defgroup PWREx_SMPS_OPERATING_MODES SMPS step down converter operating modes
  * @{
  */
#define PWR_SMPS_BYPASS                 (PWR_SR2_SMPSBYPR) /*!< SMPS step down in bypass mode  */
#define PWR_SMPS_STEP_DOWN              (PWR_SR2_SMPSENR)  /*!< SMPS step down in step down mode */
/**
  * @}
  */

/** @defgroup PWREx_SMPS_OUTPUT_VOLTAGE_LEVEL SMPS step down converter output voltage scaling voltage level
  * @{
  */
/* Note: SMPS voltage is trimmed during device production to control
         the actual voltage level variation from device to device. */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V20  LL_PWR_SMPS_OUTLVL_1V2     /*!< SMPS step down converter supply output voltage 1.20V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V25  LL_PWR_SMPS_OUTLVL_1V25    /*!< SMPS step down converter supply output voltage 1.25V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V30  LL_PWR_SMPS_OUTLVL_1V3     /*!< SMPS step down converter supply output voltage 1.30V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V35  LL_PWR_SMPS_OUTLVL_1V35    /*!< SMPS step down converter supply output voltage 1.35V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V40  LL_PWR_SMPS_OUTLVL_1V4     /*!< SMPS step down converter supply output voltage 1.40V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V45  LL_PWR_SMPS_OUTLVL_1V45    /*!< SMPS step down converter supply output voltage 1.45V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V50  LL_PWR_SMPS_OUTLVL_1V5     /*!< SMPS step down converter supply output voltage 1.50V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V55  LL_PWR_SMPS_OUTLVL_1V55    /*!< SMPS step down converter supply output voltage 1.55V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V60  LL_PWR_SMPS_OUTLVL_1V6     /*!< SMPS step down converter supply output voltage 1.60V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V65  LL_PWR_SMPS_OUTLVL_1V65    /*!< SMPS step down converter supply output voltage 1.65V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V70  LL_PWR_SMPS_OUTLVL_1V7     /*!< SMPS step down converter supply output voltage 1.70V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V75  LL_PWR_SMPS_OUTLVL_1V75    /*!< SMPS step down converter supply output voltage 1.75V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V80  LL_PWR_SMPS_OUTLVL_1V8     /*!< SMPS step down converter supply output voltage 1.80V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V85  LL_PWR_SMPS_OUTLVL_1V85    /*!< SMPS step down converter supply output voltage 1.85V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V90  LL_PWR_SMPS_OUTLVL_1V9     /*!< SMPS step down converter supply output voltage 1.90V */
#define PWR_SMPS_OUTPUT_VOLTAGE_1V95  LL_PWR_SMPS_OUTLVL_1V95    /*!< SMPS step down converter supply output voltage 1.95V */
/**
  * @}
  */

/** @defgroup PWREx_Flag  PWR Status Flags
  *        Elements values convention: 0000 0000 00XX YYYYb
  *           - YYYY: Flag position in the XX register (4 bits)
  *           - XX  : Status register (2 bits)
  *                 - 01: SR1 register
  *                 - 10: SR2 register
  *                 - 11: C2_SCR register
  *        The only exception is PWR_FLAG_WUF, encompassing all
  *        wake-up flags and set to PWR_SR1_WUF.    
  * @{
  */
/*--------------------------------SR1-------------------------------*/
#define PWR_FLAG_WPB0                       (0x0010U)   /*!< Wakeup event on wakeup pin 1 */
#define PWR_FLAG_WPB1                       (0x0011U)   /*!< Wakeup event on wakeup pin 2 */
#define PWR_FLAG_WPB2                       (0x0012U)   /*!< Wakeup event on wakeup pin 3 */
#define PWR_FLAG_WPB3                       (0x0013U)   /*!< Wakeup event on wakeup pin 4 */
#define PWR_FLAG_WPB4                       (0x0014U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPB5                       (0x0015U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPB6                       (0x0016U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPB7                       (0x0017U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPA8                       (0x0018U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPA9                       (0x0019U)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPA10                      (0x001AU)   /*!< Wakeup event on wakeup pin 5 */
#define PWR_FLAG_WPA11                      (0x001BU)   /*!< Wakeup event on wakeup pin 5 */

#define PWR_FLAG_WBLE                       (0x001CU)   /*!< BLE wakeup Flag              */
#define PWR_FLAG_WBLEHCPU                   (0x001DU)   /*!< BLE Host CPU wakeup Flag     */
#define PWR_FLAG_RTC                        (0x001FU)   /*!< Internal wakeup Flag (RTC)   */

/*--------------------------------SR2-------------------------------*/
#define PWR_FLAG_SMPSBYP                    (0x0020U)   /*!< SMPS PRECHARGE mode status Flag       */
#define PWR_FLAG_SMPSENR                    (0x0021U)   /*!< SMPS RUN mode status Flag             */
#define PWR_FLAG_SMPSRDY                    (0x0022U)   /*!< SMPS ready status Flag                */

#define PWR_FLAG_REGLPS                     (0x0028U)   /*!< Low Power regulator ready status Flag */
#define PWR_FLAG_REGMS                      (0x0029U)   /*!< Main regulator ready status Flag      */

#define PWR_FLAG_PVDO                       (0x002BU)   /*!< Power voltage Detector Output Flag    */

#define PWR_FLAG_IOBOOTVAL0                 (0x002CU)   /*!< PA8 input value latched at POR Flag   */
#define PWR_FLAG_IOBOOTVAL1                 (0x002DU)   /*!< PA9 input value latched at POR Flag   */
#define PWR_FLAG_IOBOOTVAL2                 (0x002EU)   /*!< PA10 input value latched at POR Flag  */
#define PWR_FLAG_IOBOOTVAL3                 (0x002FU)   /*!< PA11 input value latched at POR Flag  */
   
/*------------------------------EXTSRR---------------------------*/
#define PWR_FLAG_DEEPSTOPF                  (0x0039U)   /*!< System DEEPTSTOP Flag                 */
#define PWR_FLAG_RFPHASEF                   (0x003AU)   /*!< Critical radio system phase Flag      */

/**
  * @}
  */

/**
  * @}
  */
/* Private define ------------------------------------------------------------*/ 
/* Exported macros -----------------------------------------------------------*/
/* Private macros --------------------------------------------------------*/
/** @addtogroup  PWREx_Private_Macros   PWR Extended Private Macros
  * @{
  */

#define IS_PWR_WAKEUP_SOURCE(SOURCE) (((SOURCE) == PWR_WAKEUP_PB0)     ||  \
                                      ((SOURCE) == PWR_WAKEUP_PB1)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB2)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB3)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB4)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB5)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB6)     || \
                                      ((SOURCE) == PWR_WAKEUP_PB7)     || \
                                      ((SOURCE) == PWR_WAKEUP_PA8)     || \
                                      ((SOURCE) == PWR_WAKEUP_PA9)     || \
                                      ((SOURCE) == PWR_WAKEUP_PA10)    || \
                                      ((SOURCE) == PWR_WAKEUP_PA11)    || \
                                      ((SOURCE) == PWR_WAKEUP_BLE)     || \
                                      ((SOURCE) == PWR_WAKEUP_BLEHCPU) || \
                                      ((SOURCE) == PWR_WAKEUP_RTC))

#define IS_PWR_WAKEUP_PIN_POLARITY(POLARITY)  (((POLARITY) == PWR_PIN_POLARITY_HIGH) || \
                                               ((POLARITY) == PWR_PIN_POLARITY_LOW))


#define IS_PWR_GPIO_BIT_NUMBER(BIT_NUMBER) (((BIT_NUMBER) & GPIO_PIN_MASK) != (uint32_t)0x00)
                             
#define IS_PWR_GPIO(GPIO) (((GPIO) == PWR_GPIO_A) ||\
                           ((GPIO) == PWR_GPIO_B))

#define IS_PWR_SMPS_MODE(SMPS_MODE) (((SMPS_MODE) == PWR_SMPS_BYPASS)    ||\
                                     ((SMPS_MODE) == PWR_SMPS_STEP_DOWN))

#define IS_PWR_SMPS_OUTPUT_VOLTAGE(SMPS_OUTPUT_VOLTAGE) (((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V20) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V25) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V30) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V35) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V40) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V45) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V50) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V55) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V60) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V65) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V70) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V75) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V80) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V85) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V90) ||\
                                                         ((SMPS_OUTPUT_VOLTAGE) == PWR_SMPS_OUTPUT_VOLTAGE_1V95))


#define IS_PWR_RAMRET_BANK(BANK) (((BANK) == PWR_RAMRET_BANK1) || \
                                  ((BANK) == PWR_RAMRET_BANK2) || \
                                  ((BANK) == PWR_RAMRET_BANK3))
/**
  * @}
  */
  

/** @addtogroup PWREx_Exported_Functions PWR Extended Exported Functions
  * @{
  */
  
/** @addtogroup PWREx_Exported_Functions_Group1 Extended Peripheral Control functions 
  * @{
  */


/* Peripheral Control functions  **********************************************/
void              HAL_PWREx_EnableBORinSDN(void);
void              HAL_PWREx_DisableBORinSDN(void);

HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
void              HAL_PWREx_EnablePullUpPullDownConfig(void);
void              HAL_PWREx_DisablePullUpPullDownConfig(void);

void              HAL_PWREx_EnableSRAMRetention(uint32_t bank);
void              HAL_PWREx_DisableSRAMRetention(uint32_t bank);

HAL_StatusTypeDef HAL_PWREx_ConfigSMPS(uint32_t outputVoltage);
void              HAL_PWREx_SMPS_SetMode(uint32_t OperatingMode);
uint32_t          HAL_PWREx_SMPS_GetMode(void);

/* WakeUp pins configuration functions ****************************************/
uint32_t          HAL_PWREx_GetWakeupFlag(uint32_t WakeUpFlag);
void              HAL_PWREx_ClearWakeupFlag(uint32_t WakeUpFlag);

/* Low Power modes configuration functions ************************************/
void              HAL_PWREx_EnterDEEPSTOPMode(void);
void              HAL_PWREx_EnterSHUTDOWNMode(void);

void              HAL_PWREx_PVD_IRQHandler(void);

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


#endif /* BLUENRG_LP_HAL_PWR_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
