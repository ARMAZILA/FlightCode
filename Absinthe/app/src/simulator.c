/*
 * simulator.c
 *
 *  Created on: 06.09.2013
 */

#include "main.h"

extern xSemaphoreHandle gpsSemaphore;
extern xTimerHandle gpsLostTimer;

/* Simulator output / armazila input data protocol struct */
typedef struct {
    float		lat;		// Latitude in degrees
    float		lon;		// Longitude in degrees
    float		alt;		// Altitude in meters
    float		head;		// Course over ground in degrees
    float		airspeed;	// Air speed 0.1 m/s
    float		groundspeed;// Ground speed 0.1 m/s
    float		downspeed;	// m/s
    float		acc_x;		// m/ss
    float		acc_y;		// m/ss
    float		acc_z;		// m/ss
    float		gyro_x;		// degrees per second
    float		gyro_y;		// degrees per second
    float		gyro_z;		// degrees per second
    float		roll;		// Roll in degrees
    float		pitch;		// Pitch in degrees
} sim_pack_in_t;

/* Simulator input / armazila output data protocol structure */
/* All controls goes -1...1. Zerro = neutral */
typedef struct {
    float		aileron;
    float		elevator;
    float		rudder;
    float		throttle;
} sim_pack_out_t;

// State mashine codes
typedef enum {
    IDLE,
    MAGIC_4C,
    MAGIC_56,
    MAGIC_41,
    MAGIC_4D,
    PAYLOAD,
} c_state_t;

static bool sim_parse_char(uint8_t c, char * inBuf)
{
	static uint8_t	bufferIndex = 0;
	static uint8_t state = IDLE;

	switch (state)
	{
	case IDLE:
		state = (c == 0x4C) ? MAGIC_4C : IDLE;
		break;

	case MAGIC_4C:
		state = (c == 0x56) ? MAGIC_56 : IDLE;
		break;

	case MAGIC_56:
		state = (c == 0x41) ? MAGIC_41 : IDLE;
		break;

	case MAGIC_41:
		if (c == 0x4D)
		{
			state = MAGIC_4D;
			bufferIndex = 0;
		}
		else
			state = IDLE;
		break;

	case MAGIC_4D:
	{
		inBuf[bufferIndex++] = c;
		if (bufferIndex == sizeof(sim_pack_in_t))
		{
			state = IDLE;
			bufferIndex = 0;
			return true;
		}
		break;
	}

	} // switch

	return false;
}

static float swap_float(float f) {
    char *c = (char *) &f;
    return * (float *) (char[]) {c[3], c[2], c[1], c[0] };
}

#if 0
static void simSend32(uint32_t data)
{
	uint8_t port = 1;

	vcpSendByte(port, (data >> 24) & 0xFF);
	vcpSendByte(port, (data >> 16) & 0xFF);
	vcpSendByte(port, (data >>  8) & 0xFF);
	vcpSendByte(port, (data      ) & 0xFF);
}

static void simSendBin(void)
{
	float data;

	data = 1.0;
	simSend32((uint32_t) data);

	data = 0.9;
	simSend32((uint32_t) data);

	data = 0.5;
	simSend32((uint32_t) data);

	data = 0.2;
	simSend32((uint32_t) data);	// Throttle
}
#endif

static void simPrint(char *str)
{
    while (*str)
    	vcpSendByte(1, *(str++));
}

static void simSendTxt(void)
{
	char buf[12];

	ftoa(1.0, buf);			// Throttle0
	simPrint(buf);
	vcpSendByte(1, '\t');

	ftoa(0.95, buf);		// Throttle1
	simPrint(buf);
	vcpSendByte(1, '\t');

	ftoa(0.9, buf);			// Throttle2
	simPrint(buf);
	vcpSendByte(1, '\t');

	ftoa(0.85, buf);		// Throttle3
	simPrint(buf);
	simPrint("\r\n");
}

/*
 * 	Simulator protocol task
 * 	We use VCP2 or UART1 if configured
 */
portTASK_FUNCTION_PROTO(simTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint8_t inBuf[sizeof(sim_pack_in_t)];
	sim_pack_in_t * packet = (sim_pack_in_t *) &inBuf[0];
	uint8_t gpsCycleCount = 0;
	uint8_t sendCycleCount = 0;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    while (1)
    {

    	while (vcpHasData(1))	// Check if VCP2 has data?
    	{
    		uint8_t ch;

    		ch = vcpGetByte(1);	// Get VCP2 data byte

    		if (sim_parse_char(ch, (char *) packet))
    		{
    			// Aply value from simulator
    			EstAlt 				= swap_float(packet->alt);

    			heading 			= swap_float(packet->head);
    			heading_rad			= DEG2RAD(swap_float(packet->head));

    		    accSmooth[ROLL ]	= swap_float(packet->acc_x) / 9.80665f * acc_1G;
    		    accSmooth[PITCH] 	= swap_float(packet->acc_y) / 9.80665f * acc_1G;
    		    accSmooth[YAW  ]   	= -swap_float(packet->acc_z) / 9.80665f * acc_1G;

    			gyroData[ROLL ]		= swap_float(packet->gyro_x);
    			gyroData[PITCH]		= swap_float(packet->gyro_y);
    			gyroData[YAW  ]		= swap_float(packet->gyro_z);

    			angle[ROLL ]		= swap_float(packet->roll) * 10;
    			angle[PITCH]		= -swap_float(packet->pitch) * 10;
    			angle_rad[ROLL ]	= DEG2RAD(swap_float(packet->roll));
    			angle_rad[PITCH]	= -DEG2RAD(swap_float(packet->pitch));

    			if (gpsCycleCount == 20)
    			{
    				// Simulate 5Hz GPS update
    				gpsCycleCount = 0;

					GPS_coord[LAT] 		= swap_float(packet->lat) * 1e7;
					GPS_coord[LON] 		= swap_float(packet->lon) * 1e7;
	    			GPS_altitude		= swap_float(packet->alt) / 10.0f;
	    			GPS_speed			= swap_float(packet->groundspeed);
	    			GPS_ground_course 	= swap_float(packet->head) * 10;

	    			gps_frames_rx ++;

	    		    sensorsSet(SENSOR_GPS);

	    		    xTimerReset(gpsLostTimer, 10);

	    		    GPS_update = (GPS_update == 1 ? 0 : 1);

	    		    flagSet(FLAG_GPS_FIX);

	    		    GPS_numSat = 12;
	    		    GPS_hdop = 1;

	    		    xSemaphoreGive(gpsSemaphore);
    			}
    		}
    	}

    	if (gpsCycleCount < 20)
    		gpsCycleCount++;

    	if (++sendCycleCount == 5)
		{
			sendCycleCount = 0;
			simSendTxt();
		}

	    // Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 10); 	// Task cycle rate 100 Hz
	}
}

