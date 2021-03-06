
/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : TIM_OutputCompare_Init_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the TIM functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  TIM_OutputCompare_Init/TIM_OutputCompare_Init_main.c
 * @brief Configuration of the TIM peripheral to generate an output 
 * waveform in different output compare modes. This example is based on the 
 * BLUENRG_LP TIM LL API. The peripheral initialization uses 
 * LL unitary service functions for optimization purposes (performance and size).
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\TIM\\TIM_OutputCompare_Init\\MDK-ARM\\{STEVAL-IDB011V1}\\TIM_OutputCompare_Init.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP DK x.x.x\\Projects\\Peripheral_Examples\\Examples_LL\\TIM\\TIM_OutputCompare_Init\\EWARM\\{STEVAL-IDB011V1}\\TIM_OutputCompare_Init.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c TIM_OutputCompare_Init - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1



* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  |   STEVAL-IDB011V1  |
-----------------------------------
|     A1     |      Not Used      |
|     A11    |      Not Used      |
|     A12    |      Not Used      |
|     A13    |      Not Used      |
|     A14    |      Not Used      |
|     A15    |      Not Used      |
|     A4     |      TIM1 CH1      |
|     A5     |      Not Used      |
|     A6     |      Not Used      |
|     A7     |      Not Used      |
|     A8     |      USART TX      |
|     A9     |      USART RX      |
|     B0     |      Not Used      |
|     B14    |      Not Used      |
|     B2     |      Not Used      |
|     B3     |      Not Used      |
|     B4     |      Not Used      |
|     B5     |      Not Used      |
|     B7     |      Not Used      |
|     B8     |      Not Used      |
|     B9     |      Not Used      |
|     GND    |      Not Used      |
|     RST    |      Not Used      |
|    VBAT    |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |      STEVAL-IDB011V1     |
-----------------------------------------
|     DL1    |         Not Used         |
|     DL2    |         Not Used         |
|     DL3    |         Not Used         |
|     DL4    |         Not Used         |
|     U5     |         Not Used         |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |        STEVAL-IDB011V1       |
-------------------------------------------------
|      PUSH1     |  Change output compare mode  |
|      PUSH2     |           Not Used           |
|      RESET     |       Reset BlueNRG-LP       |

@endtable

* \section Usage Usage

Configuration of the TIM peripheral to generate an output waveform in different output compare modes. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

TIM1 input clock (TIM1CLK) is 64MHz, since prescaler is equal to 1.
    TIM1CLK = 64 Mhz

To set the TIM1 counter clock frequency to 10 KHz, the pre-scaler (PSC) is calculated as follows:
PSC = (TIM1CLK / TIM1 counter clock) - 1
PSC = (64 MHz /10 KHz) - 1

SystemCoreClock is set to 64 MHz for BLUENRG_LP Devices.

Auto-reload (ARR) is calculated to get a time base period of 100 ms, meaning a time base frequency of 10 Hz.
ARR = (TIM1 counter clock / time base frequency) - 1
ARR = (TIM1 counter clock / 10) - 1

The capture/compare register (CCR1) of the output channel is set to half the auto-reload value. 
Therefore the timer output compare delay is 50 ms.
Generally speaking this delay is calculated as follows:
CC1_delay = TIM1 counter clock / CCR1

The timer output channel must be connected to PB11 on board BlueNRG_LP-EVB.
Thus TIM1_CH1 status (on/off) mirrors the timer output level (active v.s. inactive).

User push-button (PUSH1) can be used to change the output compare mode:
  - When the output channel is configured in output compare toggle:  TIM1_CH1 
    TOGGLES when the counter (CNT) matches the capture/compare register (CCR1).
  - When the output channel is configured in output compare active:  TIM1_CH1 
    switched ON when the counter (CNT) matches the capture/compare register (CCR1).
  - When the output channel is configured in output compare inactive:  TIM1_CH1 
    switched OFF when the counter (CNT) matches the capture/compare register (CCR1).
    
Initially the output channel is configured in output compare toggle mode.


BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms:
- TIM1_CH1 : PA.4
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin (GPIO PA.09) to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin (GPIO PA.08) to PC COM port TX signal
- Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 
**/
   
/* Includes ------------------------------------------------------------------*/
#include "TIM_OutputCompare_Init_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Number of output compare modes */
#define TIM_OC_MODES_NB 3

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/* Output compare modes */
static uint32_t aOCMode[TIM_OC_MODES_NB] = {
  LL_TIM_OCMODE_TOGGLE,
  LL_TIM_OCMODE_ACTIVE,
  LL_TIM_OCMODE_INACTIVE
};

/* Output compare mode index */
static uint8_t iOCMode = 0;

/* Compare match count */
static uint32_t uwCompareMatchCount = 0;

/* TIM1 Clock */
static uint32_t tim_prescaler = 0;
static uint32_t tim_period = 0;
static uint32_t tim_pulse_value = 0;

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
static void UserButton_Init(void);
static void MX_TIM1_Init(void);
__STATIC_INLINE void     Configure_OCMode(uint32_t OCMode);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, BLE_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* BSP COM Init */
  /* needed to use printf over the USART */
  BSP_COM_Init(NULL);
 
/* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz */
  tim_prescaler = __LL_TIM_CALC_PSC(TIM_PERIPHCLK, 10000);

  /* Set the auto-reload value to have a counter frequency of 10 Hz */
  /* TIM1CLK = TIM_PERIPHCLK / (APB prescaler & multiplier)               */
  tim_period = __LL_TIM_CALC_ARR(TIM_PERIPHCLK, tim_prescaler, 10);

  /* Set output compare active/inactive delay to half of the auto-reload value */
  tim_pulse_value = tim_period / 2;

  /* Initialize button in EXTI mode */
  UserButton_Init();
  MX_TIM1_Init();

  /**************************/
  /* TIM1 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1*/
  LL_TIM_EnableIT_CC1(TIM1);
  
  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1 */
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

  /* Enable the outputs (set the MOE bit in TIMx_BDTR register). */
  LL_TIM_EnableAllOutputs(TIM1);  
  
  /* Enable counter */
  LL_TIM_EnableCounter(TIM1);

  printf("User push-button (PUSH1) can be used to change the output compare mode.\n\r");
         
  /* Infinite loop */
  while (1)
  {
    if(ubButtonPress == 1)
    {             
      /* Set new OC mode */
      iOCMode = (iOCMode + 1) % TIM_OC_MODES_NB;
        
      /* Switch to next OC mode */
      Configure_OCMode(aOCMode[iOCMode]);
  
      ubButtonPress = 0;
    }
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  LL_APB0_EnableClock(LL_APB0_PERIPH_TIM1);

  /**TIM1 GPIO Configuration  
  PA4 / AF4   ------> TIM1_CH1 
  */
  GPIO_InitStruct.Pin = TIM1_CH1_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = TIM1_CH1_AF;
  LL_GPIO_Init(TIM1_CH1_GPIO_PORT, &GPIO_InitStruct);
  
  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(TIM1_IRQn);

  TIM_InitStruct.Prescaler = tim_prescaler;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = tim_period;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_TOGGLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = tim_pulse_value;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

}

/**
  * @brief  Configures User push-button (PUSH1) in GPIO or EXTI Line Mode.
  * @param  None 
  * @retval None
  */
void UserButton_Init(void)
{
  /* Enable the BUTTON Clock */
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);

  /* Enable a rising trigger External line 10 Interrupt */
  USER_BUTTON_EXTI_LINE_ENABLE();
  USER_BUTTON_EXTI_RISING_TRIG_ENABLE();
  
  /* Clear the event occurred on the interrupt line 10 port A. */
  if (LL_EXTI_IsInterruptPending(USER_BUTTON_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearInterrupt(USER_BUTTON_EXTI_LINE);
  }

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);

  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
  * @brief  Changes the output compare mode.
  * @param  None
  * @retval None
  */
__STATIC_INLINE void Configure_OCMode(uint32_t OCMode)
{
  /* Disable the counter */
  LL_TIM_DisableCounter(TIM1);
  
  /* Reset the counter */
  LL_TIM_SetCounter(TIM1, 0);
  
  /* Reset the compare match count */
  uwCompareMatchCount = 0;
  
  /* Set the output level (active v.s. inactive) according to the new OC mode */
  switch (OCMode)
  {
    case LL_TIM_OCMODE_TOGGLE:
       /* Set the output channel to its toggles on compare match*/
      printf("Set the output channel to its toggles on compare match.\n\r");
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
      break; 
      
    case LL_TIM_OCMODE_ACTIVE:
      /* Set the output channel to its inactive level (LOW)*/
      printf("Set the output channel to its inactive level (LOW).\n\r");
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_INACTIVE);
      break;
      
    case LL_TIM_OCMODE_INACTIVE:
      /* Set the output channel to its active level (HIGH)*/
      printf("Set the output channel to its active level (HIGH).\n\r");
      LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FORCED_ACTIVE);
      break;
        
    default:
      break;
  }
  
  /* Update the output channel mode */
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, OCMode);
  
  /* Re-enable the counter */
  LL_TIM_EnableCounter(TIM1);
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  User button interrupt processing
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
  * @brief  Timer capture/compare interrupt processing
  * @note   The capture/compare interrupt is generated whatever the compare
  *         mode is (as long as the timer counter is enabled).
  * @param  None
  * @retval None
  */
void TimerCaptureCompare_Callback(void)
{
   /* Upon compare match, the counter value  should be equal to the */
   /* capture/compare register value */
  if(LL_TIM_GetCounter(TIM1) == LL_TIM_OC_GetCompareCH1(TIM1))
  {
    /* Increment the compare match count */
    uwCompareMatchCount++;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/