/*
 * mavlink.c
 *
 *  Created on: 14.07.2013
 *      Author: Александр Горбачев
 */

#include "mavlink.h"

uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; 	///< Booting up
uint32_t custom_mode = 0; 							///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; 			///< System ready for flight

static uint16_t m_parameter_i = 0;
static int packet_drops = 0;

extern volatile uint32_t sysTickUptime;

serialReadByte_t	mlReadByte = NULL;
serialHasData_t		mlHasData = NULL;

// waypoints.c
void wpInit(void);
void wp_message_timeout(void);
void wp_message_handler(mavlink_channel_t chan, const mavlink_message_t *msg);

// mission.c
void mp_message_handler(mavlink_channel_t chan, mavlink_message_t* msg);

#include "fifo_buffer.h"

#define	ML_RX_BUFFER_SIZE	32
static t_fifo_buffer		ml_Rx_Buffer_Hnd;
static uint8_t 				ml_Rx_Buffer[ML_RX_BUFFER_SIZE];

/* UART1 helper functions */
static void mlCallback_uart1(uint16_t data)	{ fifoBuf_putByte(&ml_Rx_Buffer_Hnd, data); }
static uint16_t mlHasData_uart1(void)		{ return (fifoBuf_getUsed(&ml_Rx_Buffer_Hnd) == 0) ? false : true; }
static uint8_t mlReadByte_uart1(void)		{ return fifoBuf_getByte(&ml_Rx_Buffer_Hnd); }

static uint8_t mlReadByte_vcp2(void)		{ return vcpGetByte(1); }
static uint16_t mlHasData_vcp2(void)		{ return vcpHasData(1); }

static void ml_send_25Hz(mavlink_channel_t chan)
{
	mavlink_msg_attitude_send(chan,
		sysTickUptime,					// Timestamp (milliseconds since system boot)
		angle_rad[ROLL ],				// Roll angle (rad, -pi..+pi)
		angle_rad[PITCH],				// Pitch angle (rad, -pi..+pi)
		heading_rad,	 				// Yaw angle (rad, -pi..+pi)
		gyro_sensor.data_rad[ROLL ], 	// Roll angular speed (rad/s)
		gyro_sensor.data_rad[PITCH],	// Pitch angular speed (rad/s)
		gyro_sensor.data_rad[YAW  ]		// Yaw angular speed (rad/s)
	);

	mavlink_msg_servo_output_raw_send(chan,
			sysTickUptime,		// Timestamp (milliseconds since system boot)
			1,					// Servo output port
			motor[0], motor[1], motor[2], motor[3], motor[4], motor[5], motor[6], motor[7]);
/*
	mavlink_msg_servo_output_raw_send(chan,
			sysTickUptime,						// Timestamp (milliseconds since system boot)
			2, servo[0], servo[1], servo[2], servo[3], servo[4], servo[5], servo[6], servo[7]);
*/

}

static void ml_send_10Hz(mavlink_channel_t chan)
{
	/*
	mavlink_msg_named_value_float_send(chan,
			sysTickUptime,		// Timestamp (milliseconds since system boot)
			"DEBUG_ALT",
			EstAlt
			);
	*/

	mavlink_msg_rc_channels_raw_send(chan,
			sysTickUptime,		// Timestamp (milliseconds since system boot)
			0,					// Servo output port
			rcData[0],			// RC channel 1 value, in microseconds
			rcData[1],
			rcData[2],
			rcData[3],
			rcData[4],
			rcData[5],
			rcData[6],
			rcData[7],
			rssi				// RSSI
	);
/*
	mavlink_msg_rc_channels_scaled_send(chan,
			sysTickUptime,				// Timestamp (milliseconds since system boot)
			0,							// Servo output port
			(1500 - rcData[0]) * 20,	// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000
			(1500 - rcData[1]) * 20,
			(1500 - rcData[2]) * 20,
			(1500 - rcData[3]) * 20,
			(1500 - rcData[4]) * 20,
			(1500 - rcData[5]) * 20,
			(1500 - rcData[6]) * 20,
			(1500 - rcData[7]) * 20,
			0							// RSSI
	);
*/
	int16_t vfr_heading = heading;

    // Align to the value 0...360
    if (vfr_heading < 0)
    	vfr_heading = vfr_heading + 360;

	mavlink_msg_vfr_hud_send(chan,
			gps.speed / 10,						// Current airspeed in m/s
			gps.speed / 10,						// Current ground speed in m/s
			vfr_heading,						// Current heading in degrees, in compass units (0..360, 0=north)
			(rcCommand[THROTTLE] - 1000) / 10,	// Current throttle setting in integer percent, 0 to 100
			EstAlt / 100,						// Current altitude (MSL), in meters
			vario / 100							// Current climb rate in meters/second
	);
}

static void ml_send_1Hz(mavlink_channel_t chan)
{
	// Pack the message
	mavlink_msg_heartbeat_send(chan, mavlink_system.type,
		MAV_AUTOPILOT_ARMAZILA,
		system_mode,
		custom_mode,
		system_state
	);

	mavlink_msg_sys_status_send(chan,
/* Value of 1: present. Indices:
	0: 3D gyro,
	1: 3D acc,
	2: 3D mag,
	3: absolute pressure,
	4: differential pressure,
	5: GPS,
	6: optical flow,
	7: computer vision position,
	8: laser based position,
	9: external ground-truth (Vicon or Leica). Controllers:
	10: 3D angular rate control
	11: attitude stabilization,
	12: yaw position,
	13: z/altitude control,
	14: x/y position control,
	15: motor outputs / control. */

		// Controllers and sensors are present.
		1 << 0 |					// 3D gyro
   		1 << 1 | 					// 3D acc
   		1 << 2 |					// 3D mag
   		1 << 4 |					// differential pressure
   		(cfg.gps_baudrate ? 1 : 0) << 5 |	// GPS
   		1 << 15,					// motor outputs / control

   		// Controllers and sensors are enabled.
		1 << 0 |					// 3D gyro
   		1 << 1 | 					// 3D acc
   		1 << 2 |					// 3D mag
   		1 << 4 |					// differential pressure
   		(cfg.gps_baudrate ? 1 : 0) << 5 |	// GPS
   		1 << 15,					// motor outputs / control

   		// Controllers and sensors are operational or have an error.
		1 << 0 |					// 3D gyro
   		1 << 1 | 					// 3D acc
   		1 << 2 |					// 3D mag
   		1 << 4 |					// differential pressure
   		sensors(SENSOR_GPS) << 5 |	// GPS
   		1 << 15,					// motor outputs / control
   		counters.cycleTime / 10,	// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000)
		power_sensor.flightBatteryVoltage * 100,	// Battery voltage, in millivolts (1 = 1 millivolt)
		power_sensor.flightBatteryCurrent / 10,	// Battery current, in 10*milliamperes (1 = 10 milliampere)
		(cfg.flightBatteryCapacity - power_sensor.flightBatteryConsumed) / cfg.flightBatteryCapacity * 100, // Remaining battery energy: (0%: 0, 100%: 100)
		0,							// Communication drops in percent, (0%: 0, 100%: 10'000)
		packet_drops,				// Communication errors
		0,							// Autopilot-specific errors
		0,							// Autopilot-specific errors
		0,							// Autopilot-specific errors
		0							// Autopilot-specific errors
	);

//	GPS_RAW_INT (#24)
	mavlink_msg_gps_raw_int_send(chan,
		(uint64_t) micros(),
		3,						// fix_type
		gps.coord[LAT], // 55.667248 * 1e7,		// Latitude in 1E7 degrees
		gps.coord[LON], // 37.601129 * 1e7,		// Longitude in 1E7 degrees
		EstAlt * 10,			// Altitude in 1E3 meters (millimeters) above MSL
		gps.hdop,				// HDOP in cm (m*100). If unknown, set to: 65535
		65535,					// VDOP in cm (m*100). If unknown, set to: 65535
		gps.speed,				// GPS ground speed (m/s * 100). If unknown, set to: 65535
		gps.ground_course * 10,	// Course over ground in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
		gps.numSat				// satellites_visible
	);
}

static float paramGet(const param_value_t *var)
{
    union
    {
    	uint8_t		u8[4];
    	int8_t		i8[4];
    	uint16_t	u16[2];
    	int16_t		i16[2];
    	uint32_t	u32;
    	int32_t		i32;
    	float 		f;
    } value_u;

    value_u.u32 = 0;

    switch (var->type) {
    case VAR_UINT8:
        value_u.u32 = *(uint8_t *)var->ptr;
        break;

    case VAR_INT8:
        value_u.i32 = *(int8_t *)var->ptr;
        break;

    case VAR_UINT16:
        value_u.u32 = *(uint16_t *)var->ptr;
        break;

    case VAR_INT16:
        value_u.i32 = *(int16_t *)var->ptr;
        break;

    case VAR_UINT32:
        value_u.u32 = *(uint32_t *)var->ptr;
        break;

    case VAR_FLOAT:
        value_u.f = *(float *)var->ptr;
        break;
    }

	return value_u.f;
}

/**
* @brief Converter and save config parameter
*
* This function takes data from float value, convert it to appropriate type
* and store it in config parameter table.
*/
static void paramSetFromFloatValue(const param_value_t *var, float value)
{
    union
    {
    	uint8_t		u8[4];
    	int8_t		i8[4];
    	uint16_t	u16[2];
    	int16_t		i16[2];
    	uint32_t	u32;
    	float 		f;
    } value_u;

    value_u.f = value;

    switch (var->type)
    {
    case VAR_UINT8:
        *(uint8_t *) var->ptr = value_u.u8[0];
        break;

    case VAR_INT8:
        *(int8_t *) var->ptr = value_u.i8[0];
        break;

    case VAR_UINT16:
        *(uint16_t *) var->ptr = value_u.u16[0];
        break;

    case VAR_INT16:
        *(int16_t *) var->ptr = value_u.i16[0];
        break;

    case VAR_UINT32:
        *(uint32_t *) var->ptr = value_u.u32;
        break;

    case VAR_FLOAT:
		// Only write and emit changes if there is actually a difference
		// AND only write if new value is NOT "not-a-number"
		// AND is NOT infinity
		if (*(float *)var->ptr != value && !isnan(value) && !isinf(value))
		{
			*(float *) var->ptr = value;
		}
        break;
    }
}

static void ml_param_set(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(msg, &set);

	// Check if this message is for this system
	if ((uint8_t) set.target_system == (uint8_t) mavlink_system.sysid &&
		(uint8_t) set.target_component == (uint8_t) mavlink_system.compid)
	{
		char* key = (char*) set.param_id;

		for (uint16_t i = 0; i < valueTableCount; i++)
		{
			bool match = true;
			for (uint8_t j = 0; j < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN; j++)
			{
				// Compare
				if (((char) (valueTable[i].name[j])) != (char) (key[j]))
				{
					match = false;
				}

				// End matching if null termination is reached
				if (((char) valueTable[i].name[j]) == '\0')
				{
					break;
				}
			}

			// Check if matched
			if (match)
			{
				paramSetFromFloatValue(&valueTable[i], set.param_value);

				// Report back new value
				mavlink_msg_param_value_send(chan,
					valueTable[i].name,
					paramGet(&valueTable[i]),
					set.param_type,
					valueTableCount, i);
			}
		}
	}
}

static void ml_command_long(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	mavlink_command_long_t cl;
	mavlink_msg_command_long_decode(msg, &cl);
	char buf[50];

	switch (cl.command)
	{
	case MAV_CMD_PREFLIGHT_STORAGE:
		if (cl.param1 == 1)
		{
			writeFlashConfig(1);
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "mavlink: Write config to flash");
		}
		break;

	case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
		if (cl.param2 > 0)
		{
			if (flag(FLAG_ARMED))
			{
				mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "mavlink: Can't reboot when armed");
				break;
			}
			mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, "mavlink: Rebooting onboard computer");
			vTaskDelay(1000 / portTICK_RATE_MS);	// Delay 1 second
			systemReset(false);						// Don't go to bootloader
		}
		break;

	default:
		sprintf(buf, "mavlink: COMMAND_LONG (%d) %d %d", cl.command, (int32_t) cl.param1, (int32_t) cl.param2);
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);
		break;
	}
}

static void ml_set_mode(mavlink_channel_t chan, const mavlink_message_t *msg)
{
	mavlink_set_mode_t sm;
	mavlink_msg_set_mode_decode(msg, &sm);
	char buf[50];

	sprintf(buf, "MAVLink: Set mode (%u)", sm.base_mode);
	mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);
}

/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
static void ml_message_handler(mavlink_channel_t chan, mavlink_message_t* msg)
{
	char buf[50];

	// Handle message
	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		// Start sending parameters
		m_parameter_i = 0;
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		// E.g. read GCS heartbeat and go into
		// comm lost mode if timer times out
		break;

	case MAVLINK_MSG_ID_PARAM_SET:
		ml_param_set(chan, msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_LONG:
		ml_command_long(chan, msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		ml_set_mode(chan, msg);
		break;

	default:
		sprintf(buf, "MAVLink: Unknown message type received (%u)", msg->msgid);
		mavlink_msg_statustext_send(chan, MAV_SEVERITY_INFO, buf);
		break;
	}
}

/**
* @brief Send low-priority messages at a maximum rate of xx Hertz
*
* This function sends messages at a lower rate to not exceed the wireless
* bandwidth. It sends one message each time it is called until the buffer is empty.
* Call this function with xx Hertz to increase/decrease the bandwidth.
*/
static void ml_queued_send(mavlink_channel_t chan)
{
	//send parameters one by one
	if (m_parameter_i < valueTableCount)
	{
		uint par_type = MAVLINK_TYPE_FLOAT;

		switch (valueTable[m_parameter_i].type)
		{
		case VAR_FLOAT:
			par_type = MAVLINK_TYPE_FLOAT;
			break;

		case VAR_UINT8:
		case VAR_UINT16:
		case VAR_UINT32:
			par_type = MAV_PARAM_TYPE_UINT32;
			break;

		case VAR_INT8:
		case VAR_INT16:
			par_type = MAV_PARAM_TYPE_INT32;
			break;
		}

		mavlink_msg_param_value_send(chan,
			valueTable[m_parameter_i].name,
			paramGet(&valueTable[m_parameter_i]),
			par_type,
			valueTableCount,
            m_parameter_i);

		m_parameter_i++;
	}
}

/*
 * 	MAVLink protocol task
 * 	We use VCP2 or UART1
 */
portTASK_FUNCTION_PROTO(mavlinkTask, pvParameters)
{
	portTickType xLastWakeTime;
	mavlink_channel_t chan;
	uint8_t CycleCount1Hz = 0;
	uint8_t CycleCount10Hz = 0;
	uint8_t CycleCount25Hz = 0;
	uint16_t CycleCount5Sec = 0;

	mavlink_system.sysid  = cfg.mavlink_sysid; 		///< ID 20 for this UAV
	mavlink_system.compid = cfg.mavlink_compid; 	///< The component sending the message
	// TODO: Decode MW mixer type
	mavlink_system.type   = MAV_TYPE_QUADROTOR; 	///< This system is an quadrocopter.

	if (cfg.uart1_mode == UART1_MODE_MAVLINK)
	{
		fifoBuf_init(&ml_Rx_Buffer_Hnd, &ml_Rx_Buffer, ML_RX_BUFFER_SIZE);

		uartInit(MAVLINK_UART, cfg.uart1_baudrate, mlCallback_uart1);

		mlReadByte = mlReadByte_uart1;
		mlHasData  = mlHasData_uart1;
		chan = MAVLINK_COMM_0;	// Use UART1
	}
	else
	{
		mlReadByte = mlReadByte_vcp2;
		mlHasData  = mlHasData_vcp2;
		chan = MAVLINK_COMM_2;	// Use VCP2
	}

	wpInit();

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        mavlink_message_t msg;
        mavlink_status_t status;

    	while (mlHasData())	// Check if VCP2 has data?
    	{
    		uint8_t c = mlReadByte();	// Get VCP2 data byte

    		if (mavlink_parse_char(chan, c, &msg, &status))
    		{
    			ml_message_handler(chan, &msg);  	// Process parameter protocol
    			mp_message_handler(chan, &msg);  	// Process mission planner command
    			wp_message_handler(chan, &msg);		// Process waypoint protocol
    		}

    		// Update global packet drops counter
        	packet_drops += status.packet_rx_drop_count;
    	}

    	if (++CycleCount25Hz == 4)
		{
			CycleCount25Hz = 0;
			ml_send_25Hz(chan);					// Send paket at 25 Hz
		}

    	if (++CycleCount10Hz == 10)
		{
			CycleCount10Hz = 0;
	    	ml_queued_send(chan);				// Send parameters at 10 Hz, if previously requested
			ml_send_10Hz(chan);					// Send paket at 10 Hz
		}

    	if (++CycleCount1Hz == 100)
		{
    		if (flag(FLAG_ARMED))
    		{
    			system_state = MAV_STATE_ACTIVE;
    			system_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    		}
    		else
    		{
    			system_state = MAV_STATE_STANDBY;
    			system_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
    		}

			CycleCount1Hz = 0;
			ml_send_1Hz(chan);					// Send heartbeat and status paket at 1 Hz
		}

    	if (++CycleCount5Sec == 50)
		{
			CycleCount5Sec = 0;
			wp_message_timeout();				// Process waypoint protocol timeouts
		}

	    // Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 10); 	// Task cycle rate 100 Hz
	}
}

