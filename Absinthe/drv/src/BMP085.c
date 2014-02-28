/*
 * BMP085.c
 *
 *  Created on: 24.02.2014
 *      Author: Алексей Молчанов
 */
#include "main.h"

enum bmp085_settings
{
	BMP085_SETTINGS_DEVICE_ADDRESS = 0x77, // адрес
	BMP085_SETTINGS_OSS = 0, //
	BMP085_SETTINGS
};

union bmp085_calibration_coefficients_type
{
	uint16_t raw[11];
	struct
	{
		int16_t ac1;
		int16_t ac2;
		int16_t ac3;
		uint16_t ac4;
		uint16_t ac5;
		uint16_t ac6;
		int16_t b1;
		int16_t b2;
		int16_t mb;
		int16_t mc;
		int16_t md;
		int32_t b5;
	};
};
struct bmp085_type
{
	// калибровочные коэффициенты
	union bmp085_calibration_coefficients_type calibration_coefficients;

	// нескомпенсированная температура
	signed long uncompensated_temperature;

	// нескомпенсированное  давление
	int32_t uncompensated_pressure;
};

struct bmp085_type bmp085;

bool bmp085_Detect(void)
{
	uint8_t Hi,Lo,index;

	if (!i2cRead(BMP085_SETTINGS_DEVICE_ADDRESS, 0xAA, sizeof(bmp085.calibration_coefficients.raw), (uint8_t *)&bmp085.calibration_coefficients.raw[0]))
		return false;

	bmp085_start_temperature();

	for (index = 0; index < (sizeof(bmp085.calibration_coefficients.raw)/sizeof(bmp085.calibration_coefficients.raw[0])); index++)
	{
		Lo = bmp085.calibration_coefficients.raw[index]>>8;
		Hi = 0xFF & bmp085.calibration_coefficients.raw[index];
		bmp085.calibration_coefficients.raw[index] = Lo + (Hi << 8);
	}
	return true;
}

void bmp085_init(void)
{
	uint8_t Hi,Lo,index;

	if (!i2cRead(BMP085_SETTINGS_DEVICE_ADDRESS, 0xAA, sizeof(bmp085.calibration_coefficients.raw), (uint8_t *)&bmp085.calibration_coefficients.raw[0]))
		return;

		bmp085_start_temperature();
		for (index = 0; index < (sizeof(bmp085.calibration_coefficients.raw)/sizeof(bmp085.calibration_coefficients.raw[0])); index++)
		{
			Lo = bmp085.calibration_coefficients.raw[index]>>8;
			Hi = 0xFF & bmp085.calibration_coefficients.raw[index];
			bmp085.calibration_coefficients.raw[index] = Lo + (Hi << 8);
		}
    bmp085_start_temperature();
}

void BMP085Read(float* Pressure_mb)//
{
	volatile int32_t x1;
	    volatile int32_t x2;
	    volatile int32_t x3;
	    volatile int32_t b6;
	    volatile int32_t b3;
	    volatile uint32_t b4;
	    volatile uint32_t b7;
	    volatile int32_t p;

	  //const  uint16_t StartCode = 0x34;

	uint8_t pu8[3];
	// Read the An-compensated Pressure measurement
	if(i2cRead(BMP085_SETTINGS_DEVICE_ADDRESS, 0xF6, 3, pu8))
	{
		bmp085.uncompensated_pressure = ((int32_t) pu8[0]) << 16;
	    bmp085.uncompensated_pressure |= ((int32_t) pu8[1]) << 8;
	    bmp085.uncompensated_pressure |= (int32_t) pu8[2];
	    bmp085.uncompensated_pressure >>= (8 - BMP085_SETTINGS_OSS);
	}

	    // корректировка
	    b6 = bmp085.calibration_coefficients.b5 - 4000;

	    x1 = b6 * b6;
	    x1 >>= 12;
	    x1 *= bmp085.calibration_coefficients.b2;
	    x1 >>= 11;

	    x2 = b6 * bmp085.calibration_coefficients.ac2;
	    x2 >>= 11;

	    x3 = x1 + x2;

	    b3 = (bmp085.calibration_coefficients.ac1 * 4) + x3;
	    b3 <<= BMP085_SETTINGS_OSS;
	    b3 += 2;
	    b3 /= 4;

	    x1 = bmp085.calibration_coefficients.ac3 * b6;
	    x1 >>= 13;

	    x2 = b6 * b6;
	    x2 >>= 12;
	    x2 *= bmp085.calibration_coefficients.b1;
	    x2 >>= 16;

	    x3 = x1 + x2;
	    x3 += 2;
	    x3 >>= 2;

	    b4 = (uint32_t) (x3 + 32768) * bmp085.calibration_coefficients.ac4;
	    b4 >>= 15;

	    b7 = (uint32_t) (bmp085.uncompensated_pressure - b3) * (50000 >> BMP085_SETTINGS_OSS);

	    if (b7 < 0x80000000)
	    {
	        p = b7 * 2 / b4;
	    }
	    else
	    {
	        p = b7 * b4 / 2;
	    }

	    x1 = (p >> 8);
	    x1 *= x1 * 3038;
	    x1 >>= 16;

	    x2 = -7357 * p;
	    x2 >>= 16;

	    *Pressure_mb =  1.0f/75.0063f*(float)((p + (x1 + x2 + 3791) / 16) * 3) / 4;//

}

void bmp085_read_temperature(float* Temperature_DegC)
{
	uint8_t pu8[2];
    int32_t x1;
    int32_t x2;

    // чтение результата преобразования
    if(i2cRead(BMP085_SETTINGS_DEVICE_ADDRESS, 0xF6, 2, pu8))
    {
		bmp085.uncompensated_temperature = ((int32_t) pu8[0]) << 8;
		bmp085.uncompensated_temperature |= (uint32_t) pu8[1];
    }

    // корректировка
    x1 = bmp085.uncompensated_temperature - bmp085.calibration_coefficients.ac6;
    x1 *= bmp085.calibration_coefficients.ac5;
    x1 /= 1<<15;

    x2 = bmp085.calibration_coefficients.mc;
    x2 *= 1<<11;
    x2 /= (x1 + bmp085.calibration_coefficients.md);



    bmp085.calibration_coefficients.b5 = x1 + x2;

    *Temperature_DegC = 0.1f*(float)((bmp085.calibration_coefficients.b5 + 8) >> 4);
}

bool bmp085_start_temperature(void)
{
	const  uint16_t StartCode = 0x2E;
	// запуск преобразования
	return i2cWriteBuffer(BMP085_SETTINGS_DEVICE_ADDRESS, 0xF4, 2, (uint8_t *)&StartCode);
}

bool bmp085_start_pressure(void)
{
	uint16_t StartCode = 0x34;
	StartCode += BMP085_SETTINGS_OSS << 6;
	// запуск преобразования
	return i2cWriteBuffer(BMP085_SETTINGS_DEVICE_ADDRESS, 0xF4, 2, (uint8_t *)&StartCode);
}
