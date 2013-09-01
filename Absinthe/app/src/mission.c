/*
 * mission.c
 *
 * Mission planer module
 *
 *  Created on: 21.08.2013
 *
 */

#include "main.h"
#include "mavlink.h"
#include "common/common.h"

void waypoint_changed(mavlink_mission_item_t * wp)
{
	float lat, lon, alt, yaw;

	/* Update controller setpoints */
	if (wp->frame == (int)MAV_FRAME_GLOBAL) {
		/* global, absolute waypoint */
		lat = wp->x * 1e7f;
		lon = wp->y * 1e7f;
		alt = wp->z;
		yaw = wp->param4;

		// go to new position...
	}
}

void mp_message_handler(mavlink_channel_t chan, mavlink_message_t* msg)
{
	char buf[50];

	// Handle message
	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t cl;
		mavlink_msg_command_long_decode(msg, &cl);

		sprintf(buf, "Mission: Msg COMMAND_LONG. Command (%u)", cl.command);
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);

		break;
	} // case MAVLINK_MSG_ID_COMMAND_LONG

	} // switch (msg->msgid)
}
