/*
 * rtc.h
 *
 *  Created on: 25.03.2013
 *      Author: avgorbi
 */

#ifndef RTC_H_
#define RTC_H_

typedef struct {
	uint8_t hour;		// time of day
	uint8_t minute;
	uint8_t second;
	uint8_t day;		// date
	uint8_t month;
	uint8_t year;
} rtc_struct;

void rtcTickdHandler(void);


#endif /* RTC_H_ */
