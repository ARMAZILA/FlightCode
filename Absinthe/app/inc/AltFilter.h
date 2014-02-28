/*
 * AltFilter.h
 *
 *  Created on: 21.02.2014
 *      Author: Алексей Молчанов
 */

#ifndef ALTFILTER_H_
#define ALTFILTER_H_

void  AltFilterInit(float dt, float accnoice, float Alt);
float AltFilterStep(float dt, float accnoice, float altnoice, float Alt, float Acc);
float AltFilterGetAlt(void);
float AltFilterGetVario(void);

#endif /* ALTFILTER_H_ */
