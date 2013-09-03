/*
 * mission.c
 *
 * Mission planer module
 *
 *  Created on: 21.08.2013
 *
 */

#include "mavlink.h"

extern wp_flash_store_t wpf;
extern int16_t current_active_wp_id;
extern bool pos_reached;

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

/*
 * Calculate distance in global frame.
 *
 * The distance calculation is based on the WGS84 geoid (GPS)
 */
float mavlink_wpm_distance_to_point_global_wgs84(uint16_t seq, float lat, float lon, float alt)
{
	//TODO: implement for z once altidude contoller is implemented

	if (seq < wpf.size) {
		mavlink_mission_item_t *cur = &(wpf.wp_list[seq]);

		float current_x_rad = DEG2RAD(cur->x);
		float current_y_rad = DEG2RAD(cur->y);
		float x_rad = DEG2RAD(lat);
		float y_rad = DEG2RAD(lon);

		float d_lat = x_rad - current_x_rad;
		float d_lon = y_rad - current_y_rad;

		float sin_d_lat_2 = sin(d_lat / 2.0);
		float sin_d_lon_2 = sin(d_lon / 2.0);

		float a = sin_d_lat_2 * sin_d_lat_2 + sin_d_lon_2 * sin_d_lon_2 * cos(current_x_rad) * cos(x_rad);
		float c = 2 * atan2(sqrt(a), sqrt(1 - a));

		const float radius_earth = 6371000.0;

		return radius_earth * c;
	} else {
		return -1.0f;
	}
}


void check_waypoints_reached(void)
{
	float 	orbit = wpf.wp_list[current_active_wp_id].param2;
	uint8_t frame = wpf.wp_list[current_active_wp_id].frame;
	float dist = -1.0f;

	if (frame == MAV_FRAME_GLOBAL) {
//		dist = mavlink_wpm_distance_to_point_global_wgs84(current_active_wp_id, (float)global_pos->lat * 1e-7f, (float)global_pos->lon * 1e-7f, global_pos->alt);
	} else if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
//		dist = mavlink_wpm_distance_to_point_global_wgs84(current_active_wp_id, global_pos->lat, global_pos->lon, global_pos->relative_alt);
	} else if (frame == MAV_FRAME_LOCAL_ENU || frame == MAV_FRAME_LOCAL_NED) {
//		dist = mavlink_wpm_distance_to_point_local(current_active_wp_id, local_pos->x, local_pos->y, local_pos->z);
	} else if (frame == MAV_FRAME_MISSION) {
		/* Check if conditions of mission item are satisfied */
		// XXX TODO
	}

	if (dist >= 0.f && dist <= orbit /*&& wpm->yaw_reached*/) { //TODO implement yaw
		// Setpoint reached
		pos_reached = true;
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
