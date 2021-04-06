/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : bluenrg_lp_hal_power_manager.c
* Author             : AMG - RF-Application Team
* Version            : V1.0.0
* Date               : 07-May-2019
* Description        : Power Save manager
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_ll_bus.h"
#include "bluenrg_lp_ll_rcc.h"
#include "bluenrg_lp_ll_system.h"
#include "bluenrg_lp_hal_power_manager.h"

/**** Private function prototype ***********************************************/
static uint8_t PowerSave_Setup(PowerSaveLevels ps_level, WakeupSourceConfig_TypeDef wsConfig);
static void SystemDeepSleepCmd(uint8_t NewState);
static void AHBUPCONV_ConfigRestore(void);
static void AHBUPCONV_WaitConfigrurationEnd(void);

/**** Macro definition ********************************************************/
#undef DEBUG_POWER_SAVE_LEVEL
#define DEBUG_POWER_SAVE_LEVEL 0

#define MIN(a,b) ((a) < (b) )? (a) : (b)

#define ATOMIC_SECTION_BEGIN() uint32_t uwPRIMASK_Bit = __get_PRIMASK(); \
                                __disable_irq(); \
/* Must be called in the same or in a lower scope of ATOMIC_SECTION_BEGIN */
#define ATOMIC_SECTION_END() __set_PRIMASK(uwPRIMASK_Bit)

#ifndef SAVE_CONTEXT_ON_CSTACK
#define SAVE_CONTEXT_ON_CSTACK 1
#endif

#if SAVE_CONTEXT_ON_CSTACK
#define CTXT
#else
#define CTXT   static
#endif

/* Enable Context Save and Restore for AHB0 Peripherals */
#ifndef POWER_SAVE_N_RESTORE_DMA
#define POWER_SAVE_N_RESTORE_DMA 1
#endif
#ifndef POWER_SAVE_N_RESTORE_RNG
#define POWER_SAVE_N_RESTORE_RNG 1
#endif
#ifndef POWER_SAVE_N_RESTORE_PKA
#define POWER_SAVE_N_RESTORE_PKA 1
#endif
#ifndef POWER_SAVE_N_RESTORE_CRC
#define POWER_SAVE_N_RESTORE_CRC 1
#endif
#ifndef POWER_SAVE_N_RESTORE_GPIOA
#define POWER_SAVE_N_RESTORE_GPIOA 1
#endif
#ifndef POWER_SAVE_N_RESTORE_GPIOB
#define POWER_SAVE_N_RESTORE_GPIOB 1
#endif

/* Enable Context Save and Restore for APB0 Peripherals */
#ifndef POWER_SAVE_N_RESTORE_TIM1
#define POWER_SAVE_N_RESTORE_TIM1 1
#endif
#ifndef POWER_SAVE_N_RESTORE_SYSCFG
#define POWER_SAVE_N_RESTORE_SYSCFG 1
#endif

/* Enable Context Save and Restore for APB1 Peripherals */
#ifndef POWER_SAVE_N_RESTORE_SPI1
#define POWER_SAVE_N_RESTORE_SPI1 1
#endif
#ifndef POWER_SAVE_N_RESTORE_SPI2
#define POWER_SAVE_N_RESTORE_SPI2 1
#endif
#ifndef POWER_SAVE_N_RESTORE_SPI3
#define POWER_SAVE_N_RESTORE_SPI3 1
#endif
#ifndef POWER_SAVE_N_RESTORE_ADC
#define POWER_SAVE_N_RESTORE_ADC 1
#endif
#ifndef POWER_SAVE_N_RESTORE_LPUART
#define POWER_SAVE_N_RESTORE_LPUART 1
#endif
#ifndef POWER_SAVE_N_RESTORE_USART
#define POWER_SAVE_N_RESTORE_USART 1
#endif
#ifndef POWER_SAVE_N_RESTORE_I2C1
#define POWER_SAVE_N_RESTORE_I2C1 1
#endif
#ifndef POWER_SAVE_N_RESTORE_I2C2
#define POWER_SAVE_N_RESTORE_I2C2 1
#endif

/* System Tick Priority register */
#define SHPR3_REG 0xE000ED20
                          
/* AHBUPCONV defines */
#define SYSCLK_DOUBLE_BLECLK         0x29
#define SYSCLK_FOURFOLD_BLECLK       0x2A
#define AHB_STALLED   0x08

/**** Weak function definition ************************************************/

WEAK_FUNCTION(PowerSaveLevels App_PowerSaveLevel_Check(PowerSaveLevels level))
{
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}
WEAK_FUNCTION(PowerSaveLevels BLE_STACK_SleepCheck(void))
{
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}
WEAK_FUNCTION(PowerSaveLevels HAL_VTIMER_PowerSaveLevelCheck(PowerSaveLevels level))
{
  return POWER_SAVE_LEVEL_STOP_NOTIMER;
}

/**** Global Variable ***********************************************************/
uint32_t cStackPreamble[CSTACK_PREAMBLE_NUMBER];
volatile uint32_t* ptr ;

#if DEBUG_POWER_SAVE_LEVEL
uint32_t PowerSaveLevel_selected[4]={0,};
#endif

static volatile uint32_t wakeupSources_VR;

/**** Private function definition **********************************************/

static uint8_t IO_IRQ_Enabled(uint32_t wkSource, uint8_t *IRQA_enabled, uint8_t *IRQB_enabled)
{
  uint32_t IO_A, IO_B;
  uint8_t ret;
  
  ret = 0;
  *IRQA_enabled = 0;
  *IRQB_enabled = 0;
  
  /* Select GPIOA wakeup sources */
  IO_A = ((wkSource&0xF0FF0000)>>16)|(wkSource & 0xF00);
  
  /* Select GPIOB wakeup sources */
  IO_B = ((wkSource&0x0F000000)>>16)|(wkSource & 0xFF);
  
  /* Check if the GPIOB interurpt is enabled */
  if (IO_B && (SYSCFG->IO_IER & (IO_B << 16))) {
    *IRQB_enabled = 1;
    ret = 1;
  }
  
  /* Check if the GPIOA interurpt is enabled */
  if (IO_A && (SYSCFG->IO_IER & IO_A)) {
    *IRQA_enabled = 1;
    ret = 1;
  }
  
  return ret;
}

static void AHBUPCONV_ConfigRestore(void)
{
  uint32_t BLECoreClock;
  
  if (LL_RCC_GetRFClockSource() == LL_RCC_RF_RC64MPLL_DIV2)
  {
    BLECoreClock = 32000000;
  } 
  else
  {
    BLECoreClock = 16000000;
  }
  
  /* Configure AHBUPCONV */
  if (SystemCoreClock != BLECoreClock) 
  {
    /* Configuration with System Clock four time faster than the MR_BLE clock */
    if ((SystemCoreClock == 64000000) && (BLECoreClock == 16000000)) 
    {
      AHBUPCONV->COMMAND = SYSCLK_FOURFOLD_BLECLK;
    }
    /* Configuration with System Clock two time faster than the MR_BLE clock */
    else if (((SystemCoreClock == 64000000) && (BLECoreClock == 32000000)) ||
             ((SystemCoreClock == 32000000) && (BLECoreClock == 16000000)))
    {
      AHBUPCONV->COMMAND = SYSCLK_DOUBLE_BLECLK;
    }
  }
}

static void AHBUPCONV_WaitConfigrurationEnd(void)
{
  while(AHBUPCONV->STATUS & AHB_STALLED); 
}

static void SystemDeepSleepCmd(uint8_t NewState)
{
  if (NewState != 0) {
    /* Enable Sleep/Stop mode entry on system level */
    SCB->SCR |= 1<<SCB_SCR_SLEEPDEEP_Pos;
  }
  else {
    /* Disable Sleep/Stop mode entry on system level */
    SCB->SCR &= ~(1<<SCB_SCR_SLEEPDEEP_Pos);
  }
}  

static uint8_t PowerSave_Setup(PowerSaveLevels ps_level, WakeupSourceConfig_TypeDef wsConfig)
{
  uint8_t i, radio_enabled, ret_val=SUCCESS, max_timeout, timeout, direct_hse_enabled;
  uint32_t dev_cut_version;
  
  /* Variables used to store system peripheral registers in order to restore the state after
   exit from DeepStop mode */

  /* AHB0 Peripherals Config RAM virutal register */
#ifdef POWER_SAVE_N_RESTORE_DMA
  CTXT DMAMUX_Channel_TypeDef DMAMUX_vr[8];
  CTXT DMA_Channel_TypeDef DMA_vr[8]={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_RNG
  CTXT RNG_TypeDef RNG_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_PKA
  CTXT PKA_TypeDef PKA_vr;
#endif
#ifdef POWER_SAVE_N_RESTORE_CRC
  CTXT CRC_TypeDef CRC_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOA
  CTXT GPIO_TypeDef GPIOA_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOB
  CTXT GPIO_TypeDef GPIOB_vr={0};
#endif

  /*** APB0 Peripherals Config RAM virtual register ***/
#ifdef POWER_SAVE_N_RESTORE_TIM1
  CTXT TIM_TypeDef TIM1_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_SYSCFG
  CTXT SYSCFG_TypeDef SYSCFG_vr={0};
#endif

  /*** APB1 Peripherals Config RAM virtual register ***/
#ifdef POWER_SAVE_N_RESTORE_SPI1
  CTXT SPI_TypeDef SPI1_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI2
  CTXT SPI_TypeDef SPI2_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI3
  CTXT SPI_TypeDef SPI3_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_ADC
  CTXT ADC_TypeDef ADC_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_LPUART
  CTXT USART_TypeDef LPUART_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_USART
  CTXT USART_TypeDef USART_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C1
  CTXT I2C_TypeDef I2C1_vr={0};
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C2
  CTXT I2C_TypeDef I2C2_vr={0};
#endif

  /* Vector Table offset Config RAM virtual register */
  CTXT uint32_t SCB_VTOR_vr;

  /* NVIC Config RAM virtual register */
  CTXT uint32_t NVIC_ISER_vr, NVIC_IPR_vr[8];

  /* Systick Config RAM virtual register */
  CTXT uint32_t SYSTICK_IPR_vr, SYST_CSR_vr, SYST_RVR_vr;;

  /* FLASH Config RAM virtual register */
  CTXT uint32_t FLASH_CONFIG_vr;

  /* RCC Config virtual register */
  CTXT uint32_t RCC_AHBRSTR_vr, RCC_APB1RSTR_vr, RCC_AHBENR_vr, RCC_APB1ENR_vr, RCC_CR_vr;
    
  /* Save the peripherals configuration */

  /* AHB0 Peripherals Config RAM virutal register */
#ifdef POWER_SAVE_N_RESTORE_DMA
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_DMA)) {
    memcpy(&DMAMUX_vr[0], DMAMUX1_Channel0, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[1], DMAMUX1_Channel1, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[2], DMAMUX1_Channel2, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[3], DMAMUX1_Channel3, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[4], DMAMUX1_Channel4, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[5], DMAMUX1_Channel5, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[6], DMAMUX1_Channel6, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMAMUX_vr[7], DMAMUX1_Channel7, sizeof(DMAMUX_Channel_TypeDef));
    memcpy(&DMA_vr[0], DMA1_Channel1, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[1], DMA1_Channel2, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[2], DMA1_Channel3, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[3], DMA1_Channel4, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[4], DMA1_Channel5, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[5], DMA1_Channel6, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[6], DMA1_Channel7, sizeof(DMA_Channel_TypeDef));
    memcpy(&DMA_vr[7], DMA1_Channel8, sizeof(DMA_Channel_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_RNG
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_RNG)) {
    memcpy(&RNG_vr, RNG, sizeof(RNG_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_PKA   
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_PKA)) {
    memcpy(&PKA_vr, PKA, sizeof(PKA_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_CRC
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_CRC)) {
    memcpy(&CRC_vr, CRC, sizeof(CRC_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOA
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_GPIOA)) {
    memcpy(&GPIOA_vr, GPIOA, sizeof(GPIO_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOB 
  if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_GPIOB)) {
    memcpy(&GPIOB_vr, GPIOB, sizeof(GPIO_TypeDef));
  }
#endif

  /* APB0 Peripherals Config RAM virtual register */
#ifdef POWER_SAVE_N_RESTORE_SYSCFG
  if (LL_APB0_IsEnabledClock(LL_APB0_PERIPH_SYSCFG)) {
    memcpy(&SYSCFG_vr, SYSCFG, sizeof(SYSCFG_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_TIM1
  if (LL_APB0_IsEnabledClock(LL_APB0_PERIPH_TIM1)) {
    memcpy(&TIM1_vr, TIM1, sizeof(TIM_TypeDef));
  }
#endif
    
  /* APB1 Peripherals Config RAM virtual register */
#ifdef POWER_SAVE_N_RESTORE_SPI1
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI1)) {
    memcpy(&SPI1_vr, SPI1, sizeof(SPI_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI2
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI2)) {
    memcpy(&SPI2_vr, SPI2, sizeof(SPI_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI3
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI3)) {
    memcpy(&SPI3_vr, SPI3, sizeof(SPI_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_ADC
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_ADCDIG)) {
    memcpy(&ADC_vr, ADC, sizeof(ADC_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_LPUART
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_LPUART)) {
    memcpy(&LPUART_vr, LPUART1, sizeof(USART_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_USART
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_USART)) {
    memcpy(&USART_vr, USART1, sizeof(USART_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C1
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_I2C1)) {
    memcpy(&I2C1_vr, I2C1, sizeof(I2C_TypeDef));
  }
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C2
  if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_I2C2)) {
    memcpy(&I2C2_vr, I2C2, sizeof(I2C_TypeDef));
  }
#endif

  /* Vector Table offset Config RAM virtual register */
  SCB_VTOR_vr = SCB->VTOR;

  /* Flash Config RAM virtual register */
  FLASH_CONFIG_vr = FLASH->CONFIG;
    
  /* NVIC Config RAM virtual register */
  NVIC_ISER_vr = NVIC->ISER[0];
  for (i=0; i<8; i++) {
    NVIC_IPR_vr[i] = NVIC->IP[i];
  }

  /* Systick Config RAM virtual register */
  SYSTICK_IPR_vr = *(volatile uint32_t *)SHPR3_REG;
  SYST_CSR_vr = SysTick->CTRL;
  SYST_RVR_vr = SysTick->LOAD;

  /* RCC Config RAM virtual register */
  RCC_AHBRSTR_vr  = RCC->AHBRSTR;
  RCC_APB1RSTR_vr = RCC->APB1RSTR;
  RCC_AHBENR_vr   = RCC->AHBENR;
  RCC_APB1ENR_vr  = RCC->APB1ENR;
    
  /* Enable Power Save In the MR_BLE if the radio peripheral is disabled */
  radio_enabled = LL_APB2_IsEnabledClock(LL_APB2_PERIPH_MRBLE);
  if (!radio_enabled) {
    LL_APB2_EnableClock(LL_APB2_PERIPH_MRBLE);
  }
  if ((WAKEUP->BLUE_SLEEP_REQUEST_MODE & WAKEUP_BLUE_SLEEP_REQUEST_MODE_SLEEP_EN) == 0) {
    WAKEUP->BLUE_SLEEP_REQUEST_MODE |= WAKEUP_BLUE_SLEEP_REQUEST_MODE_FORCE_SLEEPING;
  } else {
    WAKEUP->BLUE_SLEEP_REQUEST_MODE &= ~WAKEUP_BLUE_SLEEP_REQUEST_MODE_FORCE_SLEEPING;
  }
  if (!radio_enabled) {
    LL_APB2_DisableClock(LL_APB2_PERIPH_MRBLE);
  }
  
  /* Disable the Low Speed oscillator if the request is STOP_NOTIMER */
  if (ps_level == POWER_SAVE_LEVEL_STOP_NOTIMER) {
    RCC_CR_vr = RCC->CR;
    if (LL_RCC_LSE_IsEnabled()) {
      LL_RCC_LSE_Disable();
    } else {
      LL_RCC_LSI_Disable();
    }
  }
  
  /* Save the CSTACK number of words that will be restored at wakeup reset */
  i = 0;
  ptr = __vector_table[0].__ptr ;
  ptr -= CSTACK_PREAMBLE_NUMBER;
  do {
    cStackPreamble[i] = *ptr;
    i++;
    ptr++;
  } while (i < CSTACK_PREAMBLE_NUMBER); 

  /* Enable the BlueNRG-LP deep stop configuration */
  LL_PWR_LowPowerMode(LL_PWR_MODE_DEEPSTOP);
  
  /* If a wakeup source is already active, no need to enable the power save */
  if (LL_PWR_GetWakeupSource() & (wsConfig.IO_Mask_High_polarity|wsConfig.IO_Mask_Low_polarity)) {
    return ret_val;
  }
  
  /* Enable Cortex deep sleep */
  SystemDeepSleepCmd(ENABLE);

  /* If Direct HSE is enabled to enter in DEEPSTOP we need to disable the feature */
  if (LL_RCC_DIRECT_HSE_IsEnabled()) {
    LL_RCC_DIRECT_HSE_Disable();
    direct_hse_enabled = TRUE;
  } else {
    direct_hse_enabled = FALSE;
  }

  /* The __disable_irq() used at the beginning of the HAL_PWR_MNGR_Request() function
     masks all the interrupts. The interrupts will be enabled at the end of the 
     context restore. Now induce a context save. */
  void CS_contextSave(void);
  CS_contextSave();
    
  /* Disable deep sleep, because if no reset occours for an interrrupt pending,
     the register value remain set and if a simple CPU_HALT command is called from the
     application the BlueNRG-LP enters in deep sleep without make a context save.
     So, exiting from the deep sleep the context is restored with wrong random value. */
  SystemDeepSleepCmd(DISABLE);

  /* Enable the Low Speed Clock */
  if (ps_level == POWER_SAVE_LEVEL_STOP_NOTIMER) {
    if ((RCC_CR_vr & LL_RCC_LSCO_LSI) == LL_RCC_LSCO_LSI)
      LL_RCC_LSI_Enable();
    if ((RCC_CR_vr & LL_RCC_LSCO_LSE) == LL_RCC_LSCO_LSE)
      LL_RCC_LSE_Enable();    
  }

  if (RAM_VR.WakeupFromSleepFlag) {
    /* Restore the CSTACK number of words that will be saved before the sleep */
    i = 0;
    ptr = __vector_table[0].__ptr ;
    ptr -= CSTACK_PREAMBLE_NUMBER;
    do {
      *ptr = cStackPreamble[i];
      i++;
      ptr++;
    } while (i < CSTACK_PREAMBLE_NUMBER); 
    

    /* Restore the peripherals configuration */

    /* Vector Table offset Config */
    SCB->VTOR = SCB_VTOR_vr;
    
    /* FLASH Peripheral Config */
    FLASH->CONFIG = FLASH_CONFIG_vr;
    
    /* NVIC Peripheral Config */
    NVIC->ISER[0] = NVIC_ISER_vr;
    for (i=0; i<8; i++) {
    	NVIC->IP[i] = NVIC_IPR_vr[i];
    }

    /* RCC Peripheral Config */
    RCC->AHBRSTR  = RCC_AHBRSTR_vr;
    RCC->APB1RSTR = RCC_APB1RSTR_vr;
    RCC->AHBENR   = RCC_AHBENR_vr;
    RCC->APB1ENR  = RCC_APB1ENR_vr;
        
    /* AHB0 Peripherals Config */
#ifdef POWER_SAVE_N_RESTORE_DMA
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_DMA)) {
      memcpy(DMAMUX1_Channel0, &DMAMUX_vr[0], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel1, &DMAMUX_vr[1], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel2, &DMAMUX_vr[2], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel3, &DMAMUX_vr[3], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel4, &DMAMUX_vr[4], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel5, &DMAMUX_vr[5], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel6, &DMAMUX_vr[6], sizeof(DMAMUX_Channel_TypeDef));
      memcpy(DMAMUX1_Channel7, &DMAMUX_vr[7], sizeof(DMAMUX_Channel_TypeDef));
      DMA_vr[0].CNDTR = 0;
      DMA_vr[1].CNDTR = 0;
      DMA_vr[2].CNDTR = 0;
      DMA_vr[3].CNDTR = 0;
      DMA_vr[4].CNDTR = 0;
      DMA_vr[5].CNDTR = 0;
      DMA_vr[6].CNDTR = 0;
      DMA_vr[7].CNDTR = 0;
      memcpy(DMA1_Channel1, &DMA_vr[0], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel2, &DMA_vr[1], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel3, &DMA_vr[2], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel4, &DMA_vr[3], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel5, &DMA_vr[4], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel6, &DMA_vr[5], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel7, &DMA_vr[6], sizeof(DMA_Channel_TypeDef));
      memcpy(DMA1_Channel8, &DMA_vr[7], sizeof(DMA_Channel_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_RNG
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_RNG)) {
      memcpy(RNG, &RNG_vr, sizeof(RNG_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_PKA   
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_PKA)) {
      memcpy(PKA, &PKA_vr, sizeof(PKA_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_CRC
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_CRC)) {
      memcpy(CRC, &CRC_vr, sizeof(CRC_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOA
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_GPIOA)) {
      GPIOA->AFR[0] = GPIOA_vr.AFR[0]; /* To avoid glitch in the line when an AF is set */
      GPIOA->AFR[1] = GPIOA_vr.AFR[1];
      memcpy(GPIOA, &GPIOA_vr, sizeof(GPIO_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_GPIOB 
    if (LL_AHB_IsEnabledClock(LL_AHB_PERIPH_GPIOB)) {
      GPIOB->AFR[0] = GPIOB_vr.AFR[0]; /* To avoid glitch in the line when an AF is set */
      GPIOB->AFR[1] = GPIOB_vr.AFR[1];
      memcpy(GPIOB, &GPIOB_vr, sizeof(GPIO_TypeDef));
    }
#endif

  /* APB0 Peripherals Config */
#ifdef POWER_SAVE_N_RESTORE_SYSCFG
    if (LL_APB0_IsEnabledClock(LL_APB0_PERIPH_SYSCFG)) {
      memcpy(SYSCFG, &SYSCFG_vr, sizeof(SYSCFG_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_TIM1
    if (LL_APB0_IsEnabledClock(LL_APB0_PERIPH_TIM1)) {
      uint32_t app;
      app = TIM1_vr.CR1;
      TIM1_vr.CR1 &= ~TIM_CR1_CEN;
      memcpy(TIM1, &TIM1_vr, sizeof(TIM_TypeDef));
      TIM1->CR1 = app;
    }
#endif
    
    /* APB1 Peripherals Config */
#ifdef POWER_SAVE_N_RESTORE_SPI1
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI1)) {
      uint32_t app;
      app = SPI1_vr.CR1;
      SPI1_vr.CR1 &= ~SPI_CR1_SPE;
      memcpy(SPI1, &SPI1_vr, 12); /* Skip DR */
      memcpy((uint8_t*)(&(SPI1->CRCPR)), (uint8_t*)(&SPI1_vr.CRCPR), 20);
      SPI1->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI2
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI2)) {
      uint32_t app;
      app = SPI2_vr.CR1;
      SPI2_vr.CR1 &= ~SPI_CR1_SPE;
      memcpy(SPI2, &SPI2_vr, 12); /* Skip DR */
      memcpy((uint8_t*)(&(SPI2->CRCPR)), (uint8_t*)(&SPI2_vr.CRCPR), 20);
      SPI2->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_SPI3
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_SPI3)) {
      uint32_t app;
      app = SPI3_vr.CR1;
      SPI3_vr.CR1 &= ~SPI_CR1_SPE;
      memcpy(SPI3, &SPI3_vr, 12); /* Skip DR */
      memcpy((uint8_t*)(&(SPI3->CRCPR)), (uint8_t*)(&SPI3_vr.CRCPR), 20);
      SPI3->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_ADC
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_ADCDIG)) {
      memcpy(ADC, &ADC_vr, sizeof(ADC_TypeDef));
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_LPUART
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_LPUART)) {
      uint32_t app;
      app = LPUART_vr.CR1;
      LPUART_vr.CR1 &= ~USART_CR1_UE;
      memcpy(LPUART1, &LPUART_vr, 36); /* Skip RDR and TDR */
      LPUART1->PRESC = LPUART_vr.PRESC;
      LPUART1->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_USART
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_USART)) {
      uint32_t app;
      app = USART_vr.CR1;
      USART_vr.CR1 &= ~USART_CR1_UE;
      memcpy(USART1, &USART_vr, 36); /* Skip RDR and TDR */
      USART1->PRESC = USART_vr.PRESC;
      USART1->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C1
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_I2C1)) {
      uint32_t app;
      app = I2C1_vr.CR1;
      I2C1_vr.CR1 &= ~I2C_CR1_PE;
      memcpy(I2C1, &I2C1_vr, 32); /* Skip PECR, RDR and TDR */
      I2C1->CR1 = app;
    }
#endif
#ifdef POWER_SAVE_N_RESTORE_I2C2
    if (LL_APB1_IsEnabledClock(LL_APB1_PERIPH_I2C2)) {
      uint32_t app;
      app = I2C2_vr.CR1;
      I2C2_vr.CR1 &= ~I2C_CR1_PE;      
      memcpy(I2C2, &I2C2_vr, 32); /* Skip PECR, RDR and TDR */
      I2C2->CR1 = app;
    }
#endif

    if (!LL_APB2_IsEnabledClock(LL_APB2_PERIPH_MRBLE)) {
      /* Disable Power Save In the MR_BLE if the radio peripheral is disabled */
      LL_APB2_EnableClock(LL_APB2_PERIPH_MRBLE);
      WAKEUP->BLUE_SLEEP_REQUEST_MODE &= ~WAKEUP_BLUE_SLEEP_REQUEST_MODE_FORCE_SLEEPING;
      LL_APB2_DisableClock(LL_APB2_PERIPH_MRBLE);
    }
    
    
    /* Wait until the HSE is ready */
    SystemTimer_TimeoutConfig(SystemCoreClock, 350, TRUE);
    while(LL_RCC_HSE_IsReady() == 0U)      
    {
      if (SystemTimer_TimeoutExpired()) {
        ret_val = SYSTEM_CONFIG_HSE_READY_ERROR;
        break;
      }
    }
    SystemTimer_TimeoutConfig(0, 0, FALSE);
    
    if (direct_hse_enabled == FALSE) {      
      /* Wait until the RC64M PLL is ready */
      SystemTimer_TimeoutConfig(SystemCoreClock, 200, TRUE);
      while(LL_RCC_RC64MPLL_IsReady() == 0U)
      {
        if (SystemTimer_TimeoutExpired()) {
          ret_val = SYSTEM_CONFIG_PLL_READY_ERROR;
          break;
        }
      }
      SystemTimer_TimeoutConfig(0, 0, FALSE);
    } else { /* Restore DIRECT_HSE  configuration */
      LL_RCC_DIRECT_HSE_Enable();
    }
    
    /* AHBUPCONV configuration restore no need that the RC64MPLL is ready */ 
    dev_cut_version = (LL_SYSCFG_GetDeviceVersion()<<4)|LL_SYSCFG_GetDeviceRevision();  
    if (LL_APB2_IsEnabledClock(LL_APB2_PERIPH_MRBLE) && (dev_cut_version == LL_BLUENRG_LP_CUT_10)) {
      /* Wait until the AHBUPCONV configuration is ended */
      AHBUPCONV_ConfigRestore();
      AHBUPCONV_WaitConfigrurationEnd();
      /* MR_BLE Trim Bias setup @ DEEPSTOP reset */
      MrBleBiasTrimConfig(FALSE);
    }

    /* Wait until the Low Speed clock is ready */
    if (ps_level == POWER_SAVE_LEVEL_STOP_NOTIMER) {
      SystemTimer_TimeoutConfig(SystemCoreClock, 350, TRUE);
      if (SystemCoreClock == 64000000)
        max_timeout = 4;
      else
        max_timeout = 1;
      timeout = 0;
      if (LL_RCC_LSE_IsEnabled())
      {
        while (LL_RCC_LSE_IsReady() == 0U)
        {
          if (SystemTimer_TimeoutExpired()) {
            SystemTimer_TimeoutConfig(0, 0, FALSE);
            if (timeout++ >= max_timeout) {
              ret_val = SYSTEM_CONFIG_LSE_READY_ERROR;
              break;
            } else {
              SystemTimer_TimeoutConfig(SystemCoreClock, 350, TRUE);
            }
          } 
        }
      }
      else 
      {
        while (LL_RCC_LSI_IsReady() == 0U)
        {
          if (SystemTimer_TimeoutExpired()) {
            ret_val = SYSTEM_CONFIG_LSI_READY_ERROR;
            break;
          }
        }        
      }
      /* Disable the System Timer */
      SystemTimer_TimeoutConfig(0, 0, FALSE);
    }
        
    /* Systick Peripheral Config */
    *(volatile uint32_t *)SHPR3_REG = SYSTICK_IPR_vr;
    SysTick->LOAD = SYST_RVR_vr;
    SysTick->VAL = 0;
    SysTick->CTRL = SYST_CSR_vr;

  }
  
  return ret_val;
}


/**** Public function definition **********************************************/

uint8_t HAL_PWR_MNGR_Request(PowerSaveLevels level, WakeupSourceConfig_TypeDef wsConfig, PowerSaveLevels *negotiatedLevel)
{
  PowerSaveLevels app_powerSave_level, vtimer_powerSave_level, final_level;
  uint8_t ret_val=SUCCESS;
  uint8_t IRQA_enabled, IRQB_enabled;
  
  /* Mask all the interrupt */
  ATOMIC_SECTION_BEGIN();
  
  /* Flag to signal if a wakeup from standby or sleep occurred */
  RAM_VR.WakeupFromSleepFlag = 0; 

  /* BLE Stack allows to enable the power save */
  if (BLE_STACK_SleepCheck() != POWER_SAVE_LEVEL_RUNNING) {
  
    app_powerSave_level = App_PowerSaveLevel_Check(level);
    vtimer_powerSave_level = HAL_VTIMER_PowerSaveLevelCheck(level);
    final_level = (PowerSaveLevels)MIN(app_powerSave_level, level);
    final_level = (PowerSaveLevels)MIN(vtimer_powerSave_level, final_level);
    
#if DEBUG_POWER_SAVE_LEVEL
    PowerSaveLevel_selected[final_level]++;
#endif

    /* Clear previous wakeup sources */
    LL_PWR_ClearWakeupSource(LL_PWE_EWS_ALL);
    
    if (final_level == POWER_SAVE_LEVEL_RUNNING) {
      *negotiatedLevel = POWER_SAVE_LEVEL_RUNNING;
      ATOMIC_SECTION_END();
      return ret_val;
    }
  
    if (final_level == POWER_SAVE_LEVEL_CPU_HALT) {
      /* Wait for interrupt is called: the core execution is halted until an interrupt occurs. */
      __WFI();
      *negotiatedLevel = POWER_SAVE_LEVEL_CPU_HALT;
      ATOMIC_SECTION_END();
      return ret_val;
    }

    /* Enable wakeup on RTC */
    if (wsConfig.RTC_enable)
      LL_PWR_EnableWakeupSource(LL_PWR_EWS_INT);

    /* Enable wakeup on BLE Host CPU event or BLE event */
    if (final_level == POWER_SAVE_LEVEL_STOP_WITH_TIMER)
      LL_PWR_EnableWakeupSource(LL_PWR_EWS_BLEHOST | LL_PWR_EWS_BLE);

    /* Enable wakeup sources and polarity on IO events */
    LL_PWR_EnableWakeupSource(wsConfig.IO_Mask_Low_polarity | wsConfig.IO_Mask_High_polarity);
    LL_PWR_SetWakeupIOPolarity(wsConfig.IO_Mask_High_polarity, LL_PWR_WUP_RISIEDG);
    LL_PWR_SetWakeupIOPolarity(wsConfig.IO_Mask_Low_polarity, LL_PWR_WUP_FALLEDG);     
    
    /* Save all the peripherals register configuration and enable the power save level */
    ret_val = PowerSave_Setup(final_level, wsConfig);
    
    /* Wakeup Sources Virtual Register */
    wakeupSources_VR = LL_PWR_GetWakeupSource();
    
    /* Disable all the wakeup sources */
    LL_PWR_DisableWakeupSource(LL_PWE_EWS_ALL);    
  }
  
  ATOMIC_SECTION_END();

  /* If for some reason the power save is skipped (i.e. wakeup source already active.....) */
  if (RAM_VR.WakeupFromSleepFlag == 0) {
    final_level = POWER_SAVE_LEVEL_RUNNING;
  } else {
    /* User callback to execute IO IRQ */
    if (wakeupSources_VR & 0xFFFF0FFF){
      if (IO_IRQ_Enabled(wakeupSources_VR, &IRQA_enabled, &IRQB_enabled)) {
        if (IRQA_enabled && (wakeupSources_VR & 0xF0FF0F00)) {
          NVIC_DisableIRQ(GPIOA_IRQn);
        }
        if (IRQB_enabled && (wakeupSources_VR & 0x0F0000FF)) {
          NVIC_DisableIRQ(GPIOB_IRQn);
        }
        HAL_PWR_MNGR_WakeupIOCallback(wakeupSources_VR);
        if (IRQA_enabled && (wakeupSources_VR & 0xF0FF0F00)) {
          NVIC_EnableIRQ(GPIOA_IRQn);
        }
        if (IRQB_enabled && (wakeupSources_VR & 0x0F0000FF)) {
          NVIC_EnableIRQ(GPIOB_IRQn);
        }
      }
    }
  }
  
  *negotiatedLevel = final_level;
  
  return ret_val;
}

uint8_t HAL_PWR_MNGR_ShutdownRequest(uint8_t BOR_enabled)
{
  uint8_t radio_enabled;
  
  /* Enable Power Save In the MR_BLE if the radio peripheral is disabled */
  radio_enabled = LL_APB2_IsEnabledClock(LL_APB2_PERIPH_MRBLE);
  if (!radio_enabled) {
    LL_APB2_EnableClock(LL_APB2_PERIPH_MRBLE);
  }
  WAKEUP->BLUE_SLEEP_REQUEST_MODE = WAKEUP_BLUE_SLEEP_REQUEST_MODE_FORCE_SLEEPING;
  if (!radio_enabled) {
    LL_APB2_DisableClock(LL_APB2_PERIPH_MRBLE);
  }
  
  /* BOR configuration during Shutdown mode */
  if (BOR_enabled) {
    LL_PWR_EnableBORinSDN();
  } else {
    LL_PWR_EnableBORinSDN();
  }
    
  /* Enable the BlueNRG-LP Shutdown configuration */
  LL_PWR_LowPowerMode(LL_PWR_MODE_SHUTDOWN);
  
  /* Disable DIRECT HSE configuration to allow shutdown request */
  if (LL_RCC_DIRECT_HSE_IsEnabled()) {
    LL_RCC_DIRECT_HSE_Disable();
  }
  
  /* Enable Cortex deep sleep */
  SystemDeepSleepCmd(ENABLE);
  
  /* Device in shutdown mode */
  __WFI();

  return ERROR;
}

uint32_t HAL_PWR_MNGR_WakeupSource(void)
{
  return wakeupSources_VR; //LL_PWR_GetWakeupSource();
}

void HAL_PWR_MNGR_ClearWakeupSource(uint32_t source)
{
  wakeupSources_VR &= ~source;
}

WEAK_FUNCTION(void HAL_PWR_MNGR_WakeupIOCallback(uint32_t source))
{
  return;
}
