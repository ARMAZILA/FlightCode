/*
 * sbus.c
 *
 *  Created on: 05.06.2013
 *      Author: avgorbi
 *
 * 		Read Futaba S.Bus receiver serial stream
 *
 */

/* Includes */
#include "sbus.h"

static volatile uint16_t sbusValue[SBUS_NUM_INPUTS]; 	// interval [1000;2000]

/*
 // Channel order for SBUS RX Configs
 static uint8_t rcChannel[SBUS_NUM_INPUTS] =
 	{PITCH, YAW, THROTTLE, ROLL, AUX1, AUX2, AUX3, AUX4, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};
*/

/**
 * Get the value of an input channel
 * \param[in] channel Number of the channel desired (zero based)
 * \output RCVR_INVALID channel not available
 * \output RCVR_TIMEOUT failsafe condition or missing receiver
 * \output >0 channel value
 */
int16_t sbusReadRawRC(uint8_t channel)
{
	/* return error if channel is not available */
	if (channel >= SBUS_NUM_INPUTS)
	{
		return RCVR_INVALID;
	}

	/* Perhaps you may change the term "/ 2 + 976" -> center will be 1486 */
	return sbusValue[channel] / 1.5 + 816.0;
}

/**
 * Compute rcValue[] from sbus[].
 * For efficiency it unrolls first 8 channels without loops and does the
 * same for other 8 channels. Also 2 discrete channels will be set.
 */
void sbusCallback(uint16_t data)
{
	static uint8_t sbus[SBUS_FRAME_LENGTH] = {0};
	static uint8_t sbusIndex = 0;

	if (sbusIndex == 0 && data != SBUS_SOF_BYTE)
		return;

	sbus[sbusIndex++] = data;

	if(sbusIndex == SBUS_FRAME_LENGTH)
	{
		sbusIndex = 0;

		/* unroll channels 1-8 */
		sbusValue[0] = ((sbus[1] | sbus[2] << 8) & 0x07FF);
		sbusValue[1] = ((sbus[2] >> 3 | sbus[3] << 5) & 0x07FF);
		sbusValue[2] = ((sbus[3] >> 6 | sbus[4] << 2 | sbus[5] << 10) & 0x07FF);
		sbusValue[3] = ((sbus[5] >> 1 | sbus[6] << 7) & 0x07FF);
		sbusValue[4] = ((sbus[6] >> 4 | sbus[7] << 4) & 0x07FF);
		sbusValue[5] = ((sbus[7] >> 7 | sbus[8] << 1 | sbus[9] << 9) & 0x07FF);
		sbusValue[6] = ((sbus[9] >> 2 | sbus[10] << 6) & 0x07FF);
		sbusValue[7] = ((sbus[10] >> 5 | sbus[11] << 3) & 0x07FF);

		/* unroll channels 9-16 */
		sbusValue[8] = ((sbus[12] | sbus[13] << 8) & 0x07FF);
		sbusValue[9] = ((sbus[13] >> 3 | sbus[14] << 5) & 0x07FF);
		sbusValue[10] = ((sbus[14] >> 6 | sbus[15] << 2 | sbus[16] << 10) & 0x07FF);
		sbusValue[11] = ((sbus[16] >> 1 | sbus[17] << 7) & 0x07FF);
		sbusValue[12] = ((sbus[17] >> 4 | sbus[18] << 4) & 0x07FF);
		sbusValue[13] = ((sbus[18] >> 7 | sbus[19] << 1 | sbus[20] << 9) & 0x07FF);
		sbusValue[14] = ((sbus[20] >> 2 | sbus[21] << 6) & 0x07FF);
		sbusValue[15] = ((sbus[21] >> 5 | sbus[22] << 3) & 0x07FF);

		/* unroll discrete channels 17 and 18 */
		sbusValue[16] = (sbus[23] & SBUS_FLAG_DC1) ? SBUS_VALUE_MAX : SBUS_VALUE_MIN;
		sbusValue[17] = (sbus[23] & SBUS_FLAG_DC2) ? SBUS_VALUE_MAX : SBUS_VALUE_MIN;

		// Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
		if (cfg.failsafe)
		{
			if (!(sbus[23] & SBUS_FLAG_FS))
			{
				if(failsafeCnt > 20)
					failsafeCnt -= 20;
				else
					failsafeCnt = 0;
			} // clear FailSafe counter
		}
	}
}

/* Initialise S.Bus receiver interface */
void sbusInit(void)
{
	uint8_t i;

	for (i = 0; i < SBUS_NUM_INPUTS; i++)
		sbusValue[i] = 1486;

	/* Init UART5 for S.Bus protocol */
	uartInit(SERIAL_UART5, 100000, sbusCallback);

	return;
}
