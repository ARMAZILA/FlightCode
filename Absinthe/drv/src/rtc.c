
#include "main.h"

// Global variables

rtc_struct datetime;	// time registers

/******************************************************************************
*
*	Calculate days number in current month
*
*******************************************************************************/
uint8_t daysinmount(void)
{
	return datetime.month == 2 ?
		28 + ( 1 >> ( datetime.year & 3 )) :
		30 + (((uint16_t) 0x15AA >> datetime.month ) & 1 );
}

/******************************************************************************
*
*	RTC service routine. Executes each seconds for increment time register
*
*******************************************************************************/
void rtcTickdHandler(void)
{
	datetime.second++;							// increment seconds
	if(datetime.second > 59) {				// check seconds overflow
		datetime.second = 0;
		datetime.minute++;						// increment minutes
		if(datetime.minute > 59) {			// check minutes overflow
			datetime.minute = 0;
			datetime.hour++;					// increment hours
			if(datetime.hour > 23) {			// check hours overflow
				datetime.hour = 0;
				datetime.day++;					// increment days
				// check days overflow
				if(datetime.day == daysinmount()) {
					datetime.day = 1;
					datetime.month++;			// increment months
					if(datetime.month == 13) {		// check months overflow
						datetime.month = 1;
						datetime.year++;			// increment years
					}
				}
			}
		}
	}
}
