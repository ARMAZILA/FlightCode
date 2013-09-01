#include "main.h"
/*
	Board ADC signals defines in file board-110-10-04-02.h
 */

static volatile uint16_t adcValues[ADC_NUM_CHANNELS];

void adcInit(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // ADC assumes all the GPIO was already placed in 'AIN' mode
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr 	 = (uint32_t)&adcValues;
    DMA_InitStructure.DMA_DIR 				 = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize 		 = ADC_NUM_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc 	 = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc 		 = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize 	 = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode 				 = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority 			 = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M 				 = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_InitStructure.ADC_Mode 				 = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode 		 = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv 	 = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign 		 = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel 		 = ADC_NUM_CHANNELS;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_TempSensorVrefintCmd(ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_9,  			1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 			2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 			3, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 	4, ADC_SampleTime_55Cycles5);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t adcGetChannel(uint8_t channel)
{
    return adcValues[channel];
}
