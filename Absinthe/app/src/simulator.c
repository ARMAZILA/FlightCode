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

// State mashine codes
typedef enum {
    IDLE,
    MAGIC_4C,
    MAGIC_56,
    MAGIC_41,
    MAGIC_4D,
    PAYLOAD,
} c_state_t;

#include "fifo_buffer.h"

#define	ML_RX_BUFFER_SIZE	32
static t_fifo_buffer		sim_Rx_Buffer_Hnd;
static uint8_t 				sim_Rx_Buffer[ML_RX_BUFFER_SIZE];
static uint32_t				simVcpPort;

serialSendByte_t 	simSendByte = NULL;
serialReadByte_t	simReadByte = NULL;
serialHasData_t		simHasData = NULL;

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

/* UART1 helper functions */
static void simSendByte_uart1(uint8_t data) 	{ uartWrite(SERIAL_UART1, data); }
static void simCallback_uart1(uint16_t data) 	{ fifoBuf_putByte(&sim_Rx_Buffer_Hnd, data); }
static uint16_t simHasData_uart1(void)			{ return (fifoBuf_getUsed(&sim_Rx_Buffer_Hnd) == 0) ? false : true; }
static uint8_t simReadByte_uart1(void)			{ return fifoBuf_getByte(&sim_Rx_Buffer_Hnd); }

/* VCP helper functions */
static void simSendByte_vcp(uint8_t data)		{ vcpSendByte(simVcpPort, data); }
static uint8_t simReadByte_vcp(void)			{ return vcpGetByte(simVcpPort); }
static uint16_t simHasData_vcp(void)			{ return vcpHasData(simVcpPort); }

/* Print string to symulator port */
static void simPrint(char *str)
{
    while (*str)
    	simSendByte(*(str++));
}

static void simSendTxt(void)
{
	char buf[12];

	switch(cfg.mixerConfiguration)
	{
	case MULTITYPE_QUADP:
		// Use armazila.quad.xml protocol congiguration file
		ftoa((motor[0] - 1000) / 1000.0f, buf);	// Throttle0 (all values 0...1)
		simPrint(buf);
		simSendByte('\t');

		ftoa((motor[1] - 1000) / 1000.0f, buf);	// Throttle1
		simPrint(buf);
		simSendByte('\t');

		ftoa((motor[2] - 1000) / 1000.0f, buf);	// Throttle2
		simPrint(buf);
		simSendByte('\t');

		ftoa((motor[3] - 1000) / 1000.0f, buf);	// Throttle3
		simPrint(buf);
		simPrint("\r\n");

		break;

	case MULTITYPE_FLYING_WING:
		// Use armazila.wing.xml protocol congiguration file
		ftoa((servo[0] - 1500) / 500.0f, buf);		// Left aileron (-1...1)
		simPrint(buf);
		simSendByte('\t');

		ftoa((servo[1] - 1500) / 500.0f, buf);		// Right aileron (-1...1)
		simPrint(buf);
		simSendByte('\t');

		ftoa((motor[0] - 1000) / 1000.0f, buf);		// Throttle (0...1)
		simPrint(buf);
		simPrint("\r\n");

		break;
	}
}

/*
 * 	Flightgear flight simulator generic protocol task
 * 	Use armazila.quad.xml protocol congiguration file for mixer type QUADP
 *	Use armazila.wing.xml protocol congiguration file for mixer type FLYING_WING
 * 	We use VCP2 or UART1 if configured
 *
 * 	an error is detected in flightgear 2.10. Description here:
 * 	http://code.google.com/p/flightgear-bugs/issues/detail?id=1191
 */
portTASK_FUNCTION_PROTO(simTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint8_t inBuf[sizeof(sim_pack_in_t)];
	sim_pack_in_t * packet = (sim_pack_in_t *) &inBuf[0];
	uint8_t gpsCycleCount = 0;
	uint8_t sendCycleCount = 0;

	simVcpPort = (cfg.uart1_mode == UART1_MODE_MSP ? 0 : 1);

	if (cfg.uart1_mode == UART1_MODE_HIL)
	{
		fifoBuf_init(&sim_Rx_Buffer_Hnd, &sim_Rx_Buffer, ML_RX_BUFFER_SIZE);

		uartInit(SERIAL_UART1, cfg.uart1_baudrate, simCallback_uart1);

		simSendByte = simSendByte_uart1;
		simReadByte = simReadByte_uart1;
		simHasData  = simHasData_uart1;
	}
	else
	{
		simSendByte = simSendByte_vcp;
		simReadByte = simReadByte_vcp;
		simHasData  = simHasData_vcp;
	}

	flagSet(FLAG_ACC_CALIBRATED);
	flagSet(FLAG_GYRO_CALIBRATED);
	flagSet(FLAG_MAG_CALIBRATED);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
    	while (simHasData())	// Check if SIM port has data?
    	{
    		uint8_t ch;

    		ch = simReadByte();	// Get SIM port data byte

    		if (sim_parse_char(ch, (char *) packet))
    		{
    			// Aply value from simulator
    			// 0 - HIL off
    		    // 1 - sensor off
    		    // 2 - sensor & IMU off

    			if (cfg.hil_mode == 2)
    			{
        			angle[ROLL ]		= swap_float(packet->roll) * 10;
        			angle[PITCH]		= -swap_float(packet->pitch) * 10;
        			angle_rad[ROLL ]	= DEG2RAD(swap_float(packet->roll));
        			angle_rad[PITCH]	= -DEG2RAD(swap_float(packet->pitch));
        			heading 			= swap_float(packet->head);
        			heading_rad			= DEG2RAD(swap_float(packet->head));
        			EstAlt 				= swap_float(packet->alt);
    			}

    		    accSmooth[ROLL ]	= swap_float(packet->acc_x) / 9.80665f * acc_1G;
    		    accSmooth[PITCH] 	= swap_float(packet->acc_y) / 9.80665f * acc_1G;
    		    accSmooth[YAW  ]   	= -swap_float(packet->acc_z) / 9.80665f * acc_1G;

    			gyroData[ROLL ]		= swap_float(packet->gyro_x);
    			gyroData[PITCH]		= swap_float(packet->gyro_y);
    			gyroData[YAW  ]		= swap_float(packet->gyro_z);

    			if (gpsCycleCount == 20)
    			{
    				// Simulate 5Hz GPS update
    				gpsCycleCount = 0;

					gps.coord[LAT] 		= swap_float(packet->lat) * 1e7;
					gps.coord[LON] 		= swap_float(packet->lon) * 1e7;
	    			gps.altitude		= swap_float(packet->alt) / 10.0f;
	    			gps.speed			= swap_float(packet->groundspeed);
	    			gps.ground_course 	= swap_float(packet->head) * 10;
	    		    gps.update 			= (gps.update == 1 ? 0 : 1);
	    		    gps.numSat 			= 12;
	    		    gps.hdop 			= 1;
	    			gps.frames_rx ++;

	    		    sensorsSet(SENSOR_GPS);
	    		    flagSet(FLAG_GPS_FIX);

	    		    xTimerReset(gpsLostTimer, 10);
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

