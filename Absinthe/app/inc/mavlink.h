/** @file
 *	@brief MAVLink comm protocol built from ardupilotmega.xml
 *	@see http://pixhawk.ethz.ch/software/mavlink
 */
#ifndef MAVLINK_H
#define MAVLINK_H

#ifndef MAVLINK_STX
#define MAVLINK_STX 254
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

/* MAVLink adapter header */
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#define MAVLINK_UART	SERIAL_UART1

#include "mavlink_types.h"

/* Struct that stores the communication settings of this system. */
mavlink_system_t mavlink_system;

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use
 *  	MAVLINK_COMM_0 = VCP2
 *  	MAVLINK_COMM_1 = UART1
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (chan == MAVLINK_COMM_0)
	{
		// Send to VCP2
    	vcpSendByte(1, ch);
	}
	else if (chan == MAVLINK_COMM_1)
	{
		uartWrite(MAVLINK_UART, ch);
	}
}

#endif // MAVLINK_H
