/*
 * BMP085.h
 *
 *  Created on: 25.02.2014
 *      Author: Алексей Молчанов
 */

#ifndef BMP085_H_
#define BMP085_H_

bool bmp085_Detect(void);
void bmp085_init(void);
void BMP085Read(float* Pressure_mb);
void bmp085_read_temperature(float* Temperature_DegC);
bool bmp085_start_temperature(void);
bool bmp085_start_pressure(void);

#endif /* BMP085_H_ */
