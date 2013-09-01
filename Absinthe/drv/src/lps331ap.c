/*
 * lps331ap.c
 *
 *  Created on: 10.03.2013
 *      Author: avgorbi
 *
 *      BARO_INT1 - PB14
 *      BARO_INT2 - PB15
 *
 */

#include "main.h"

#if USE_SENSOR_IRQ
uint32_t lps331ap_irq_count = 0;

/**
 * @brief  This function handles BARO_INT1 External lines interrupt request.
 * @param  None
 * @retval None
 */
void BARO_INT1_IRQHandler(void)
{
	lps331ap_irq_count++;

	/* Clear the BARO_INT1 EXTI line pending bit */
	EXTI_ClearITPendingBit(BARO_INT1_EXTI_LINE);
}

static void lps331apIrqConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOB clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* Configure BARO_INT1 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = BARO_INT1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(BARO_INT1_GPIO_PORT, &GPIO_InitStructure);

	/* Enable AFIO clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI14 Line to BARO_INT1 pin */
	GPIO_EXTILineConfig(BARO_INT1_EXTI_PORT_SRC, BARO_INT1_EXTI_PIN_SRC);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = BARO_INT1_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set BARO_INT1 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = BARO_INT1_IRQCHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}
#else
#define lps331apIrqConfig()	;
#endif

void lps331apConfig(void)
{
	// Power down the device (clean start)
	i2cWrite(LPS331AP_ADDRESS, LPS331AP_CTRL_REG1, 0x00); // @0x20=0x00

	// Set the pressure sensor to higher-precision
	// Please note that 0x7A is forbidden in auto-mode (25Hz,25Hz): use 0x79 instead
	i2cWrite(LPS331AP_ADDRESS, LPS331AP_RES, 0x6A); // @0x10=0x6A

	// Turn on the pressure sensor PD = 1
	// Set output data rate 25 Hz ODR2, ODR1, ODR0 = 1, 1, 1
	// Interrupt circuit enable DIFF_EN = 1
	// Enable block data update BDU = 1
	i2cWrite(LPS331AP_ADDRESS, LPS331AP_CTRL_REG1, 0xFC);

#if USE_SENSOR_IRQ
	// Configure interrupts pins
	// Interrupt active high INT_H_L = 0
	// Push-pull selection on interrupt pads PP_OD = 0
	// Interrupts on data ready
	i2cWrite(LPS331AP_ADDRESS, LPS331AP_CTRL_REG3, 0x24);
#endif
}

bool lps331apDetect(void)
{
	uint8_t deviceid;

	i2cRead(LPS331AP_ADDRESS, LPS331AP_WHO_AM_I, 1, &deviceid);
	if (deviceid != LPS331AP_ID)
		return false;

	lps331apIrqConfig();

	lps331apConfig();

	return true;
}

void lps331apRead(float* Pressure_mb, float* Temperature_DegC)
{
	uint8_t pu8[3];

	// Read the Temperature measurement (2 bytes to read)
	i2cRead(LPS331AP_ADDRESS, LPS331AP_TEMP_OUT, 2, pu8); // @0x2B~2C

	int16_t Temp_Reg_s16 = ((u16) pu8[1] << 8) | pu8[0]; // make a SIGNED 16 bit variable
	*Temperature_DegC = 42.5 + Temp_Reg_s16 / 480; // scale and offset

	// Read the Temperature-compensated Pressure measurement
	i2cRead(LPS331AP_ADDRESS, LPS331AP_PRESS_OUT, 3, pu8); // reading auto-incremented @0x28/29/2A

	uint32_t pressure_u32 = ((u32) pu8[2] << 16) | ((u32) pu8[1] << 8) | pu8[0];

	// make a unsigned 32 bit variable
	*Pressure_mb = pressure_u32 / 4096.0; // scale

	// Check the temperature and pressure values make sense
	// Reading fixed 760 mb, means the sensing element is damaged.
}

void lps331ReadPressure(int16_t * Pressure_mb)
{
	uint8_t pu8[3];

	// Read the Temperature-compensated Pressure measurement
	i2cRead(LPS331AP_ADDRESS, LPS331AP_PRESS_OUT, 3, pu8); // reading auto-incremented @0x28/29/2A

	uint32_t pressure_u32 = ((u32) pu8[2] << 16) | ((u32) pu8[1] << 8) | pu8[0];

	// make a unsigned 32 bit variable
	*Pressure_mb = pressure_u32 / 4096.0; // scale

	// Check the temperature and pressure values make sense
	// Reading fixed 760 mb, means the sensing element is damaged.
}

void lps331ReadTemp(float* Temperature_DegC)
{
	uint8_t pu8[3];

	// Read the Temperature measurement (2 bytes to read)
	i2cRead(LPS331AP_ADDRESS, LPS331AP_TEMP_OUT, 2, pu8); // @0x2B~2C

	int16_t Temp_Reg_s16 = ((u16) pu8[1] << 8) | pu8[0]; // make a SIGNED 16 bit variable
	*Temperature_DegC = 42.5 + Temp_Reg_s16 / (120 * 4); // scale and offset
}

// From_Pressure_mb_To_Altitude_US_Std_Atmosphere_1976_cm
void From_Pressure_mb_To_Altitude_cm(float* Pressure_mb, float* Altitude_cm)
{
	// =(1-(A18/1013.25)^0.190284)*145366.45
	*Altitude_cm = (1 - pow(*Pressure_mb / 1013.25, 0.190284)) * 145366.45 / 3.280839895 * 100;
}

void From_ft_To_m(double* ft, double* m)
{
	// =D18/3.280839895
	*m = *ft / 3.280839895;
}
