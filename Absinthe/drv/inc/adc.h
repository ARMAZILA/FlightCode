#pragma once

enum {
	ADC_VIDEO_BATTERY = 0,
	ADC_VOLTAGE_SENSOR,
	ADC_CURRENT_SENSOR,
	ADC_TEMP_SENSOR,
    ADC_NUM_CHANNELS
};

void adcInit(void);
uint16_t adcGetChannel(uint8_t channel);
