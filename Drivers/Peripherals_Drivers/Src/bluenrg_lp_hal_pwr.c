/**
  ******************************************************************************
  * @file    bluenrg_lp_hal_pwr.c
  * @author  RF Application Team
  * @brief   PWR HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Power Controller (PWR) peripheral:
  *           + Initialization/de-initialization functions
  *           + Peripheral Control functions
  *
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

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_hal.h"

/** @addtogroup BLUENRG_LP_HAL_Driver
  * @{
  */

/** @addtogroup PWR
  * @{
  */

#ifdef HAL_PWR_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup PWR_Private_Defines
  * @{
  */

/** @defgroup PWR_Register_Reset_Values  PWR Register Reset Values
  * @{
  */
/* Definitions of PWR registers reset value */
#define PWR_CR1_RESET_VALUE     (0x00000010U)
#define PWR_CR2_RESET_VALUE     (0x00000100U)
#define PWR_CR3_RESET_VALUE     (0x00000000U)
#define PWR_CR4_RESET_VALUE     (0x00000000U)
#define PWR_CR5_RESET_VALUE     (0x00000014U)
#define PWR_PUCRA_RESET_VALUE   (0x0000FFF7U)
#define PWR_PDCRA_RESET_VALUE   (0x00000008U)
#define PWR_PUCRB_RESET_VALUE   (0x0000FFFFU)
#define PWR_PDCRB_RESET_VALUE   (0x00000000U)
#define PWR_IOxCFG_RESET_VALUE  (0x00000000U)
#define PWR_ENGTRIM_RESET_VALUE (0x00000000U)
/**
  * @}
  */

 /**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @addtogroup PWR_Exported_Functions  PWR Exported Functions
  * @{
  */

/** @addtogroup PWR_Exported_Functions_Group1  Initialization and de-initialization functions
  * @brief  Initialization and de-initialization functions
  *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]

@endverbatim
  * @{
  */

/**
  * @brief  Deinitialize the HAL PWR peripheral registers to their default reset values.
  * @retval None
  */
void HAL_PWR_DeInit(void)
{
  /* Apply reset values to all PWR registers */
  /* Note: Update of each register required since PWR global reset is not     */
  /*       available at RCC level on this BlueNRG_LP serie.                        */
  LL_PWR_WriteReg(CR1, PWR_CR1_RESET_VALUE);
  LL_PWR_WriteReg(CR2, PWR_CR2_RESET_VALUE);
  LL_PWR_WriteReg(CR3, PWR_CR3_RESET_VALUE);
  LL_PWR_WriteReg(CR4, PWR_CR4_RESET_VALUE);
  LL_PWR_WriteReg(CR5, PWR_CR5_RESET_VALUE);
  LL_PWR_WriteReg(PUCRA, PWR_PUCRA_RESET_VALUE);
  LL_PWR_WriteReg(PDCRA, PWR_PDCRA_RESET_VALUE);
  LL_PWR_WriteReg(PUCRB, PWR_PUCRB_RESET_VALUE);
  LL_PWR_WriteReg(PDCRB, PWR_PDCRB_RESET_VALUE);
  LL_PWR_WriteReg(IOxCFG, PWR_IOxCFG_RESET_VALUE);
  LL_PWR_WriteReg(ENGTRIM, PWR_ENGTRIM_RESET_VALUE);
  
  /* Clear all flags */
  LL_PWR_WriteReg(SR1,
                    LL_PWE_EWS_EW0 
                  | LL_PWE_EWS_EW1
                  | LL_PWE_EWS_EW2
                  | LL_PWE_EWS_EW3
                  | LL_PWE_EWS_EW4
                  | LL_PWE_EWS_EW5
                  | LL_PWE_EWS_EW6
                  | LL_PWE_EWS_EW7
                  | LL_PWE_EWS_EW8
                  | LL_PWE_EWS_EW9
                  | LL_PWE_EWS_EW10
                  | LL_PWE_EWS_EW11
                  | LL_PWR_EWS_BLE
                  | LL_PWR_EWS_BLEHOST
                 );
  
  LL_PWR_WriteReg(EXTSRR,
                    PWR_EXTSRR_DEEPSTOPF
                  | PWR_EXTSRR_RFPHASEF
                 );
}

/**
  * @}
  */



/** @addtogroup PWR_Exported_Functions_Group2  Peripheral Control functions
  *  @brief Low Power modes configuration functions
  *
@verbatim

 ===============================================================================
                 ##### Peripheral Control functions #####
 ===============================================================================

    [..]
     *** PVD configuration ***
    =========================
    [..]
      (+) The PVD is used to monitor the VDD power supply by comparing it to a
          threshold selected by the PVD Level (PVDLS[2:0] bits in PWR_CR2 register).
      (+) PVDO flag is available to indicate if VDD/VDDA is higher or lower
          than the PVD threshold. This event can generate an interrupt if enabled. 
          This is done through  __HAL_PVD_ENABLE_IT() macro.
      (+) The PVD is stopped in Shutdown mode.

    *** WakeUp pin configuration ***
    ================================
    [..]
      (+) WakeUp pins are used to wakeup the system from deepstop mode. 
          The polarity of these pins can be set to configure event detection on high 
          level (rising edge) or low level (falling edge).

    *** Low Power modes configuration ***
    =====================================
    [..]
      The devices feature 2 low-power modes:

      (+) DeepStop mode: 
                         - All clocks are stopped except LSI and LSE depending on the software configuration
                         - Core is stopped, main regulator off, low power regulator on
                         - The RAM0 bank is kept in retentio, the other RAM banks are in retention or not, 
                           depending on software choice in PWRC_CR2 register
                         - The RTC and the IWDOG are still active and able to generate respectively a wakeup
                           event and a reset
                         - The MR_BLE block is able to generate events to wakeup the system
                         - Twelve I/Os are able to wakeup the system
                         - Four I/Os among those twelve I/Os are able to be in output driving either a static low or
                           high level, the slow clock information or the RTC_OUT

      (+) Shutdown mode: 
                         - All clocks are stopped
                         - Main and low power regulators are off
                         - The only wakeup source is a low pulse on the RSTN pad. 
                         - BOR can be enabled during shutdown mode
                         - A SHUTDOWN exit is similar to a POR startup of the board


   *** DeepStop mode configuration ***
   ===============================
    [..]
      (+) Entry:
          (++) The DeepStop mode is entered thru HAL_PWR_EnterDEEPSTOPMode() API.
       
      (+) Exit:
          (++) WKUP pins, RTC, IWDG, BLE IP wakeup time is reached, Host CPU wakeup time is reached

      [..] When exiting DeepStop mode, the MCU is in Run mode. 

   *** Shutdown mode ***
   ====================
    [..] In Standby mode all clocks off, SRAM and register contents are lost.

      (+) Entry:
          (++) The Standby mode is entered thru HAL_PWR_EnterSTANDBYMode() API. 
               SRAM and register contents are lost.
      (+) Exit:
          (++) External reset in NRST pin.
   
      [..] After waking up from Shutdonw mode, program execution restarts in the same way as after a Reset.
          


   *** Auto-wakeup (AWU) from low-power mode ***
   =============================================
    [..]
      The MCU can be woken up from low-power mode by an RTC Alarm event, an RTC
      Wakeup event, without depending on an external interrupt (Auto-wakeup mode).

      (+) RTC auto-wakeup (AWU) from the DeepStop mode
  

        (++) To wake up from the Stop mode with an RTC alarm event, it is necessary to
             configure the RTC to generate the RTC alarm using the HAL_RTC_SetAlarm_IT() function.

        (++) To wake up from the Stop mode with an RTC WakeUp event, it is necessary to
              configure the RTC to generate the RTC WakeUp event using the HAL_RTCEx_SetWakeUpTimer_IT() function.

@endverbatim
  * @{
  */

/**
  * @brief  Configure the voltage threshold detected by the Power Voltage Detector (PVD).
  * @param  sConfigPVD pointer to a PWR_PVDTypeDef structure that contains the PVD 
  *         configuration information.
  * @note   Refer to the electrical characteristics of your device datasheet for
  *         more details about the voltage thresholds corresponding to each
  *         detection level.
  * @retval None
  */
HAL_StatusTypeDef HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD)
{
  /* Check the parameters */
  assert_param(IS_PWR_PVD_LEVEL(sConfigPVD->PVDLevel));
  assert_param(IS_PWR_PVD_MODE(sConfigPVD->Mode));

  /* Set PVDLS bits according to PVDLevel value */
  MODIFY_REG(PWR->CR2, PWR_CR2_PVDLS, sConfigPVD->PVDLevel);
  
  /* Clear any previous config. Keep it clear if IT mode is selected */
  __HAL_PWR_PVD_DISABLE_IT();
    
  /* Configure interrupt mode */
  if((sConfigPVD->Mode & PWR_PVD_MODE_IT) == PWR_PVD_MODE_IT)
  {
    __HAL_PWR_PVD_ENABLE_IT();
  }
  
  return HAL_OK;
}

/**
  * @brief Enables the Power Voltage Detector(PVD).
  * @retval None
  */
void HAL_PWR_EnablePVD(void)
{
  /* Enable the power voltage detector */
  SET_BIT(PWR->CR2, PWR_CR2_PVDE);
}

/**
  * @brief Disables the Power Voltage Detector(PVD).
  * @retval None
  */
void HAL_PWR_DisablePVD(void)
{
  /* Disable the power voltage detector */
  CLEAR_BIT(PWR->CR2, PWR_CR2_PVDE);
}


/**
  * @brief Enable the WakeUp Source functionality.
  * @param WakeUpSource Specifies which Wake-Up source to enable.
  *         This parameter can be one of the following values:
  *           @arg @ref PWR_WAKEUP_PB0
  *           @arg @ref PWR_WAKEUP_PB1
  *           @arg @ref PWR_WAKEUP_PB2
  *           @arg @ref PWR_WAKEUP_PB3
  *           @arg @ref PWR_WAKEUP_PB4
  *           @arg @ref PWR_WAKEUP_PB5
  *           @arg @ref PWR_WAKEUP_PB6
  *           @arg @ref PWR_WAKEUP_PB7
  *           @arg @ref PWR_WAKEUP_PA8
  *           @arg @ref PWR_WAKEUP_PA9
  *           @arg @ref PWR_WAKEUP_PA10
  *           @arg @ref PWR_WAKEUP_PA11
  *           @arg @ref PWR_WAKEUP_BLE
  *           @arg @ref PWR_WAKEUP_BLEHCPU
  *           @arg @ref PWR_WAKEUP_RTC
  * @param WakeUpPinPolarity Specifies the polarity of the Wake-Up pin.
  *         This parameter can be one of the following values:
  *           @arg @ref PWR_WAKEUP_POLARITY_HIGH
  *           @arg @ref PWR_WKAEUP_POLARITY_LOW
  * @retval None
  * @note The polarity is applicable only if the wakeup happens from an external pin
  */
void HAL_PWR_EnableWakeUpSource(uint32_t WakeUpSource, uint32_t WakeUpPinPolarity)
{
  assert_param(IS_PWR_WAKEUP_SOURCE(WakeUpSource));
  assert_param(IS_PWR_WAKEUP_POLARITY(WakeUpPinPolarity)); 
  
  /* Specifies the Wake-Up pin polarity for the event detection 
    (rising or falling edge) */
  if (WakeUpPinPolarity == PWR_WKAEUP_POLARITY_LOW)
  {
    SET_BIT(PWR->CR4, WakeUpSource);
  }
  else
  {
    CLEAR_BIT(PWR->CR4, WakeUpSource);
  }
  
  /* Enable wake-up source */
  SET_BIT(PWR->CR3, WakeUpSource);
}

/**
  * @brief  Disable the WakeUp Source functionality.
  * @param WakeUpSource Specifies the Power Wake-Up source to disable.
  *         This parameter can be one of the following values:
  *           @arg @ref PWR_WAKEUP_PB0
  *           @arg @ref PWR_WAKEUP_PB1
  *           @arg @ref PWR_WAKEUP_PB2
  *           @arg @ref PWR_WAKEUP_PB3
  *           @arg @ref PWR_WAKEUP_PB4
  *           @arg @ref PWR_WAKEUP_PB5
  *           @arg @ref PWR_WAKEUP_PB6
  *           @arg @ref PWR_WAKEUP_PB7
  *           @arg @ref PWR_WAKEUP_PA8
  *           @arg @ref PWR_WAKEUP_PA9
  *           @arg @ref PWR_WAKEUP_PA10
  *           @arg @ref PWR_WAKEUP_PA11
  *           @arg @ref PWR_WAKEUP_BLE
  *           @arg @ref PWR_WAKEUP_BLEHCPU
  *           @arg @ref PWR_WAKEUP_RTC
  * @retval None
  */
void HAL_PWR_DisableWakeUpSource(uint32_t WakeUpSource)
{
  assert_param(IS_PWR_WAKEUP_SOURCE(WakeUpSource));

  CLEAR_BIT(PWR->CR3, WakeUpSource);
}

/**
  * @brief Enter Stop mode
  * @note  In Stop mode, all I/O pins keep the same state as in Run mode.
  * @note  All clocks in the VCORE domain are stopped; the PLL, 
  *        the HSI and the HSE oscillators are disabled.
  *        SRAM0, SRAM1, SRAM2 and SRMA3 contents are preserved.
  *        The BOR is available.
  * @note  According to system power policy, system entering in Stop mode
  *        is depending on other CPU power mode.
  * @retval None
  */
void HAL_PWR_EnterDEEPSTOPMode(void)
{
  HAL_PWREx_EnterDEEPSTOPMode();
}


/**
  * @brief Enter Shutdown mode.
  * @note  In Shutdown mode, the PLL, the HSI and the HSE oscillators are switched 
  *        off. The voltage regulator is disabled.
  *        SRAM and register contents are lost.
  * @note  According to system power policy, system entering in Standby mode
  *        is depending on other CPU power mode.
  * @note  BOR can be enabled during shutdown mode
  * @note  The only wakeup source is a low pulse on the RSTN pad.
  * @retval None
  */
void HAL_PWR_EnterSHUTDOWNMode(void)
{  
  /* Set Shutdown mode */
  MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_MODE_SHUTDOWN);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

/* This option is used to ensure that store operations are completed */
#if defined ( __CC_ARM)
  __force_stores();
#endif

  /* Request Wait For Interrupt */
  __WFI();

  /* Following code is executed after wake up if system did not go to STANDBY
     mode according to system power policy */

  /* Reset SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
WEAK_FUNCTION(void HAL_PWR_PVDCallback(void))
{
  /* NOTE : This function should not be modified; when the callback is needed,
            the HAL_PWR_PVDCallback can be implemented in the user file
  */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_PWR_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
