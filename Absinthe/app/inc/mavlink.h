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

#include "main.h"
#include "mavlink_types.h"

#define MAVLINK_UART	SERIAL_UART1

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use
 *  	MAVLINK_COMM_0 = UART1
 *  	MAVLINK_COMM_1 = VCP1
 *  	MAVLINK_COMM_2 = VCP2
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	switch (chan)
	{
	case MAVLINK_COMM_0:
		uartWrite(MAVLINK_UART, ch);	// UART1
    	break;

	case MAVLINK_COMM_1:
    	vcpSendByte(0, ch); // VCP1
		break;

	case MAVLINK_COMM_2:
    	vcpSendByte(1, ch); // VCP2
		break;
	}
}

/* Struct that stores the communication settings of this system. */
mavlink_system_t mavlink_system;

#include "common/common.h"

#define MAVLINK_WPM_MAX_WP_COUNT	10

/* Waypoint flash store structure */
typedef struct wp_flash_store_t {
	uint8_t	size;
	mavlink_mission_item_t wp_list[MAVLINK_WPM_MAX_WP_COUNT];
	uint8_t crc;
	uint16_t magic;
} wp_flash_store_t;

#endif // MAVLINK_H

