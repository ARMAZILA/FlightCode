/*
 * waypoints.c
 *
 *  Created on: 19.08.2013
 *
 */

#include "mavlink.h"

#define FLASH_WAYPOINT_ADDR        (0x08000000 + (uint32_t)0x0007F800)       // use sector 255 for storage
#define	WPF_MAGIC					0xA55A

enum MAVLINK_WPM_STATES
{
	MAVLINK_WPM_STATE_IDLE = 0,
	MAVLINK_WPM_STATE_SENDLIST,
	MAVLINK_WPM_STATE_SENDLIST_SENDWPS,
	MAVLINK_WPM_STATE_GETLIST,
	MAVLINK_WPM_STATE_GETLIST_GETWPS,
	MAVLINK_WPM_STATE_GETLIST_GOTALL,
	MAVLINK_WPM_STATE_ENUM_END
};

enum MAVLINK_WPM_CODES
{
	MAVLINK_WPM_CODE_OK = 0,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_ACTION_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_FRAME_NOT_SUPPORTED,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_OUT_OF_BOUNDS,
	MAVLINK_WPM_CODE_ERR_WAYPOINT_MAX_NUMBER_EXCEEDED,
	MAVLINK_WPM_CODE_ENUM_END
};

enum MAVLINK_WPM_STATES current_state;

wp_flash_store_t wpf;
int16_t current_wp_id; ///< Waypoint in current transmission
uint16_t current_count;
uint8_t current_partner_sysid;
uint8_t current_partner_compid;
uint64_t timestamp_lastaction;
int16_t current_active_wp_id;
bool pos_reached;

void wp_message_timeout(void)
{
	if (micros() - timestamp_lastaction > 5000000 && current_state != MAVLINK_WPM_STATE_IDLE)
	{
		// printf("Last operation (state=%u) timed out, changing state to MAVLINK_WPM_STATE_IDLE\n", wpm->current_state);

		current_state = MAVLINK_WPM_STATE_IDLE;
		current_count = 0;
		current_partner_sysid = 0;
		current_partner_compid = 0;
		current_wp_id = -1;
		current_active_wp_id = -1;
	}
}

/* Write waypoint list to the flash */
void wp_flash_write()
{
    FLASH_Status status;
    uint8_t tries = 0;
    uint32_t i;

    wpf.magic = WPF_MAGIC;

retry:
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	if (FLASH_ErasePage(FLASH_WAYPOINT_ADDR) == FLASH_COMPLETE) {
		for (i = 0; i < sizeof(wp_flash_store_t); i += 4) {
			status = FLASH_ProgramWord(FLASH_WAYPOINT_ADDR + i, *(uint32_t *) ((char *)&wpf + i));
			if (status != FLASH_COMPLETE) {
				FLASH_Lock();
				tries++;
				if (tries < 3)
					goto retry;
				else
					break;
			}
		}
	}
	FLASH_Lock();

	// Flash write failed - just die now
	if (tries == 3) {
		mixerWwriteAllMotors(cfg.mincommand);
		failureMode(4);
	}
}

/* Read waypoint list from flash */
void wp_flash_read()
{
    const wp_flash_store_t *temp = (const wp_flash_store_t *)FLASH_WAYPOINT_ADDR;

    // check magic number
    if (temp->magic != WPF_MAGIC)
    {
    	wpf.size = 0;
        return;
    }

    // TODO: add crc check
    memcpy(&wpf, (char *)FLASH_WAYPOINT_ADDR, sizeof(wp_flash_store_t));
}

void wp_message_handler(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	char buf[50];
	uint64_t now = micros();

	switch (msg->msgid)
	{

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
	{
		mavlink_mission_request_list_t wprl;
		mavlink_msg_mission_request_list_decode(msg, &wprl);

		if ((wprl.target_system != mavlink_system.sysid /* || wprl.target_component != mavlink_wpm_comp_id*/))
			break;

		if ((current_state != MAVLINK_WPM_STATE_IDLE && current_state != MAVLINK_WPM_STATE_SENDLIST))
			break;

		if (wpf.size > 0)
			current_state = MAVLINK_WPM_STATE_SENDLIST;

		timestamp_lastaction = now;
		current_wp_id = 0;
		current_partner_sysid = msg->sysid;
		current_partner_compid = msg->compid;
		current_count = wpf.size;

		mavlink_msg_mission_count_send(chan, msg->sysid, msg->compid, current_count);

		break;
	} // case MAVLINK_MSG_ID_MISSION_REQUEST_LIST

	case MAVLINK_MSG_ID_MISSION_REQUEST:
	{
		mavlink_mission_request_t wpr;
		mavlink_msg_mission_request_decode(msg, &wpr);

		if (msg->sysid != current_partner_sysid || msg->compid != current_partner_compid)
			break;

		if (wpr.target_system != mavlink_system.sysid /* || wpr.target_component != mavlink_wpm_comp_id*/)
			break;

		/* ensure that we are in the correct state and that the first request has id 0 and
		 * the following requests have either the last id (re-send last waypoint) or last_id+1 (next waypoint)
		 */
		if (!((current_state == MAVLINK_WPM_STATE_SENDLIST && wpr.seq == 0) ||
				(current_state == MAVLINK_WPM_STATE_SENDLIST_SENDWPS &&
						(wpr.seq == current_wp_id || wpr.seq == current_wp_id + 1) && wpr.seq < wpf.size)))
			break;

		timestamp_lastaction = now;
		current_state = MAVLINK_WPM_STATE_SENDLIST_SENDWPS;
		current_wp_id = wpr.seq;

		mavlink_mission_item_t *wp = &(wpf.wp_list[current_wp_id]);

		mavlink_msg_mission_item_send(chan, msg->sysid, msg->compid,
				current_wp_id, wp->frame, wp->command, wp->current, wp->autocontinue,
				wp->param1, wp->param2, wp->param3, wp->param4, wp->x, wp->y, wp->z);
		break;
	} // case MAVLINK_MSG_ID_MISSION_REQUEST

	case MAVLINK_MSG_ID_MISSION_ACK:
	{
		mavlink_mission_ack_t wpa;
		mavlink_msg_mission_ack_decode(msg, &wpa);

		if ((msg->sysid != current_partner_sysid || msg->compid != current_partner_compid))
			break;

		if ((wpa.target_system != mavlink_system.sysid /* || wpa.target_component != mavlink_wpm_comp_id*/))
			break;

		if (current_state != MAVLINK_WPM_STATE_SENDLIST && current_state != MAVLINK_WPM_STATE_SENDLIST_SENDWPS)
			break;

		if (current_wp_id == wpf.size - 1)
		{
			//printf("Received ACK after having sent last waypoint, going to state MAVLINK_WPM_STATE_IDLE\n");
			timestamp_lastaction = now;
			current_state = MAVLINK_WPM_STATE_IDLE;
			current_wp_id = 0;
		}
		break;
	} // case MAVLINK_MSG_ID_MISSION_ACK

	case MAVLINK_MSG_ID_MISSION_COUNT:
	{
		mavlink_mission_count_t wpc;
		mavlink_msg_mission_count_decode(msg, &wpc);

//		sprintf(buf, "MISSION_COUNT cnt:%u sysid:%u compid:%u", wpc.count, wpc.target_system, wpc.target_component);
//		mavlink_msg_statustext_send(chan, MAV_SEVERITY_DEBUG, buf);

		if (wpc.target_system != mavlink_system.sysid /* || wpc.target_component != mavlink_wpm_comp_id*/)
			break;

		if (current_state == MAVLINK_WPM_STATE_IDLE || (current_state == MAVLINK_WPM_STATE_GETLIST && current_wp_id == 0))
		{
			if (wpc.count > 0)
			{
				// printf("clearing receive buffer and readying for receiving waypoints\n");
				timestamp_lastaction = now;
				current_state = MAVLINK_WPM_STATE_GETLIST;
				current_wp_id = 0;
				current_partner_sysid = msg->sysid;
				current_partner_compid = msg->compid;
				current_count = wpc.count;

				/* Send request for waypoint id=0 */
				mavlink_msg_mission_request_send(chan,
						current_partner_sysid,
						current_partner_compid,
						0);
			}
		}
		break;
	} // case MAVLINK_MSG_ID_MISSION_COUNT

	case MAVLINK_MSG_ID_MISSION_ITEM:
	{
		mavlink_mission_item_t wp;
		mavlink_msg_mission_item_decode(msg, &wp);

//		sprintf(buf, "MISSION_ITEM seq:%u sysid:%u compid:%u", wp.seq, wp.target_system, wp.target_component);
//		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);

		if (wp.target_system != mavlink_system.sysid /* || wp.target_component != mavlink_system.compid */)
			break;

		timestamp_lastaction = now;

		/* ensure that we are in the correct state */
		if (current_state != MAVLINK_WPM_STATE_GETLIST && current_state != MAVLINK_WPM_STATE_GETLIST_GETWPS)
		{
			mavlink_msg_mission_ack_send(chan,
					current_partner_sysid,
					current_partner_compid,
					MAV_MISSION_INVALID_SEQUENCE);

			break;
		}

		/* ensure that the waypoints have the correct ids */
		if (wp.seq == current_wp_id && wp.seq < current_count)
		{
			current_state = MAVLINK_WPM_STATE_GETLIST_GETWPS;
			memcpy(&(wpf.wp_list[wp.seq]), &wp, sizeof(mavlink_mission_item_t));
			current_wp_id = wp.seq + 1;
		}

		if (wp.seq >= MAVLINK_WPM_MAX_WP_COUNT)
		{
			/* mission item exceeds storage space */
			wpf.size = wp.seq;

			mavlink_msg_mission_ack_send(chan,
					current_partner_sysid,
					current_partner_compid,
					MAV_MISSION_NO_SPACE);

			current_state = MAVLINK_WPM_STATE_IDLE;

			wp_flash_write();

			sprintf(buf, "Waypoint: Exceeds storage size (%u)", wpf.size);
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);
			break;
		}

		/* we have receive the last waypoint */
		if (current_wp_id == current_count)
		{
			wpf.size = current_count;

			mavlink_msg_mission_ack_send(chan,
					current_partner_sysid,
					current_partner_compid,
					MAV_MISSION_ACCEPTED);

			current_state = MAVLINK_WPM_STATE_IDLE;

			wp_flash_write();

			sprintf(buf, "Waypoint: New WP list loaded (%u)", wpf.size);
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);

			break;
		}

		/* request the next waipoint */
		mavlink_msg_mission_request_send(chan, current_partner_sysid, current_partner_compid, current_wp_id);

		break;
	} // case MAVLINK_MSG_ID_MISSION_ITEM

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
	{
		mavlink_mission_clear_all_t wpca;
		mavlink_msg_mission_clear_all_decode(msg, &wpca);

		if (wpca.target_system != mavlink_system.sysid /* || wpca.target_component != mavlink_wpm_comp_id */)
			break;

		if (current_state != MAVLINK_WPM_STATE_IDLE)
			break;

		// Delete all waypoints
		timestamp_lastaction = now;
		wpf.size = 0;
		mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid,	MAV_MISSION_ACCEPTED);

		wp_flash_write();

		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "Waypoint: All WP cleared");

		break;
	} // case MAVLINK_MSG_ID_MISSION_CLEAR_ALL

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
	{
		mavlink_mission_current_t wpc;
		mavlink_msg_mission_current_decode(msg, &wpc);

		current_active_wp_id = wpc.seq;

		uint8_t i;

		for (i = 0; i < wpf.size; i++)
		{
			if (i == current_active_wp_id)
				wpf.wp_list[i].current = true;
			else
				wpf.wp_list[i].current = false;
		}
		mavlink_msg_mission_current_send(chan, current_active_wp_id);

		sprintf(buf, "Waypoint: Set new current WP:%u", current_active_wp_id);
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);

		break;
	} // case MAVLINK_MSG_ID_MISSION_SET_CURRENT
#if 0
	default:
		sprintf(buf, "Waypoint: Unknown message type received (%u)", msg->msgid);
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);
		break;
#endif
	} // switch (msg->msgid)
}

void wpInit(void)
{
	current_state = MAVLINK_WPM_STATE_IDLE;
	current_partner_sysid = 0;
	current_partner_compid = 0;
	timestamp_lastaction = 0;
	wp_flash_read();
}
