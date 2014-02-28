/*
 * wdg.c
 *
 *  Created on: 10.04.2013
 *      Author: avgorbi

The IWDG timeout is set to 250 ms (the timeout may varies due to LSI frequency
dispersion).

First, the TIM5 timer is configured to measure the LSI frequency as the
LSI is internally connected to TIM5 CH4, in order to adjust the IWDG clock.

The LSI measurement using the TIM5 is described below:
 - Configure the TIM5 to remap internally the TIM5 Channel 4 Input Capture to
   the LSI clock output.
 - Enable the TIM5 Input Capture interrupt: after one cycle of LSI clock, the
   period value is stored in a variable and compared to the HCLK clock to get
   its real value.

Then, the IWDG reload counter is configured to obtain 250ms according to the
measured LSI frequency.

The IWDG reload counter must be refreshed in time less then 250ms in the main
program infinite loop to prevent a IWDG reset using this function:

	IWDG_ReloadCounter();

 */

#include "main.h"

//#define LSI_TIM_MEASURE		// Use TIM5 to measure LSI frequency

__IO uint16_t IC1ReadValue1 = 0, IC1ReadValue2 = 0;
__IO uint16_t CaptureNumber = 0;
__IO uint32_t Capture = 0;
__IO uint32_t LsiFreq = 40000;

void wdgHandler(void)
{
    if(CaptureNumber == 0)
    {
      /* Get the Input Capture value */
      IC1ReadValue1 = TIM_GetCapture4(TIM5);
    }
    else if(CaptureNumber == 1)
    {
      /* Get the Input Capture value */
      IC1ReadValue2 = TIM_GetCapture4(TIM5);

      /* Capture computation */
      if (IC1ReadValue2 > IC1ReadValue1)
      {
        Capture = (IC1ReadValue2 - IC1ReadValue1);
      }
      else
      {
        Capture = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2);
      }
      /* Frequency computation */
      LsiFreq = (uint32_t) SystemCoreClock / Capture;
      LsiFreq *= 8;
    }

    CaptureNumber++;
}

#ifdef LSI_TIM_MEASURE
/**
  * @brief  Configures TIM5 to measure the LSI oscillator frequency.
  * @param  None
  * @retval None
  */
void TIM5_ConfigForLSI(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;

  /* Enable TIM5 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  /* Enable the TIM5 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel 				   = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 	   = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd 			   = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure TIM5 prescaler */
  TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);

  /* Connect internally the TM5_CH4 Input Capture to the LSI clock output */
  GPIO_PinRemapConfig(GPIO_Remap_TIM5CH4_LSI, ENABLE);

  /* TIM5 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM5 CH4
     The Rising edge is used as active edge,
     The TIM5 CCR4 is used to compute the frequency value
  ------------------------------------------------------------ */
  TIM_ICInitStructure.TIM_Channel 	  = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
  TIM_ICInitStructure.TIM_ICFilter 	 = 0;
  TIM_ICInit(TIM5, &TIM_ICInitStructure);

  /* TIM5 Counter Enable */
  TIM_Cmd(TIM5, ENABLE);

  /* Reset the flags */
  TIM5->SR = 0;

  /* Enable the CC4 Interrupt Request */
  TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);
}
#endif

void wdgInit(void)
{
#ifdef LSI_TIM_MEASURE
	  /* Enable the LSI OSC */
	  RCC_LSICmd(ENABLE);

	  /* Wait till LSI is ready */
	  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	  {}

	  /* TIM Configuration -------------------------------------------------------*/
	  TIM5_ConfigForLSI();

	  /* Wait until the TIM5 get 2 LSI edges */
	  while(CaptureNumber != 2)
	  {
	  }

	  /* Disable TIM5 CC4 Interrupt Request */
	  TIM_ITConfig(TIM5, TIM_IT_CC4, DISABLE);
#endif

	  DBGMCU->CR |=  DBGMCU_CR_DBG_IWDG_STOP; // Debug Independent Watchdog stopped when Core is halted

	  /* Enable write access to IWDG_PR and IWDG_RLR registers */
	  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	  /* IWDG counter clock: LSI/32 */
	  IWDG_SetPrescaler(IWDG_Prescaler_32);

	  /*
	     Set counter reload value to obtain 250ms IWDG TimeOut.
	     Counter Reload Value = 250ms/IWDG counter clock period
	                          = 250ms / (LSI/32)
	                          = 0.25s / (LsiFreq/32)
	                          = LsiFreq/(32 * 4)
	                          = LsiFreq/128
	   */
	  IWDG_SetReload(LsiFreq/128);

	  /* Reload IWDG counter */
	  IWDG_ReloadCounter();

	  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	  IWDG_Enable();
}
