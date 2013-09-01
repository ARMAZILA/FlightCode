/*
 * sbus.h
 *
 *  Created on: 08.05.2013
 *      Author: avgorbi
 */

#ifndef SBUS_H_
#define SBUS_H_

#include "main.h"

/*
 * S.Bus serial port settings:
 *  100000bps inverted serial stream, 8 bits, even parity, 2 stop bits
 *  frame period is 7ms (HS) or 14ms (FS)
 *
 * Frame structure:
 *  1 byte  - 0x0f (start of frame byte)
 * 22 bytes - channel data (11 bit/channel, 16 channels, LSB first)
 *  1 byte  - bit flags:
 *                   0x01 - discrete channel 1,
 *                   0x02 - discrete channel 2,
 *                   0x04 - lost frame flag,
 *                   0x08 - failsafe flag,
 *                   0xf0 - reserved
 *  1 byte  - 0x00 (end of frame byte)
 */
#define SBUS_FRAME_LENGTH		(1 + 22 + 1 + 1)
#define SBUS_SOF_BYTE			0x0f
#define SBUS_EOF_BYTE			0x00
#define SBUS_FLAG_DC1			0x01
#define SBUS_FLAG_DC2			0x02
#define SBUS_FLAG_FL			0x04
#define SBUS_FLAG_FS			0x08

/*
 * S.Bus protocol provides 16 proportional and 2 discrete channels.
 * Do not change unless driver code is updated accordingly.
 */
#define SBUS_NUM_INPUTS 18

#if (SBUS_NUM_INPUTS != (16 + 2))
#error "S.Bus protocol provides 16 proportional and 2 discrete channels"
#endif

/* Discrete channels represented as bits, provide values for them */
#define	SBUS_VALUE_MIN			352
#define	SBUS_VALUE_MAX			1696

/*! Define error codes for PIOS_RCVR_Get */
enum RCVR_errors {
	/*! Indicates that a failsafe condition or missing receiver detected for that channel */
	RCVR_TIMEOUT = 0,
	/*! Channel is invalid for this driver (usually out of range supported) */
	RCVR_INVALID = -1,
	/*! Indicates that the driver for this channel has not been initialized */
	RCVR_NODRIVER = -2
};

/*
 * S.Bus configuration programmable invertor
 */

void sbusInit(void);
int16_t sbusReadRawRC(uint8_t channel);
void sbusUnrollChannels();

#endif /* SBUS_H_ */
