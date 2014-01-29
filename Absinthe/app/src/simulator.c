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
    float		head;		// PLANE HEADING DEGREES TRUE in Radians
    float		airspeed;	// AIRSPEED INDICATED in meter per second
    float		speed;		// GROUND VELOCITY in meter per second
    float		acc_x;		// meter per second squared
    float		acc_y;		// GForce
    float		acc_z;		// meter per second squared
    float		gyro_x;		// radian per second
    float		gyro_y;		// radian per second
    float		gyro_z;		// radian per second
    float		roll;		// Roll in Radians
    float		pitch;		// Pitch in Radians
    float		eng_rpm;	// Engine 1 rpm
} sim_pack_in_t;

#include "fifo_buffer.h"

#define	ML_RX_BUFFER_SIZE	32
static t_fifo_buffer		sim_Rx_Buffer_Hnd;
static uint8_t 				sim_Rx_Buffer[ML_RX_BUFFER_SIZE];
static uint32_t				simVcpPort;

serialSendByte_t 	simSendByte = NULL;
serialReadByte_t	simReadByte = NULL;
serialHasData_t		simHasData = NULL;

/* UART1 helper functions */
static void simSendByte_uart1(uint8_t data) 	{ uartWrite(SERIAL_UART1, data); }
static void simCallback_uart1(uint16_t data) 	{ fifoBuf_putByte(&sim_Rx_Buffer_Hnd, data); }
static uint16_t simHasData_uart1(void)			{ return (fifoBuf_getUsed(&sim_Rx_Buffer_Hnd) == 0) ? false : true; }
static uint8_t simReadByte_uart1(void)			{ return fifoBuf_getByte(&sim_Rx_Buffer_Hnd); }

/* VCP helper functions */
static void simSendByte_vcp(uint8_t data)		{ vcpSendByte(simVcpPort, data); }
static uint8_t simReadByte_vcp(void)			{ return vcpGetByte(simVcpPort); }
static uint16_t simHasData_vcp(void)			{ return vcpHasData(simVcpPort); }

static bool simParseChar(uint8_t c, char * inBuf)
{
	static uint8_t	bufferIndex = 0;
	static uint32_t sequence = {0xFFFFFFFF};

	sequence = sequence << 8;
	sequence &= 0xFFFFFF00;
	sequence |= c;

	if(sequence == 0x0000C0FF)
		bufferIndex = 0;
	else
		if(bufferIndex < sizeof(sim_pack_in_t))
		{
			inBuf[bufferIndex++] = c;
			if (bufferIndex == sizeof(sim_pack_in_t))
					{
						return true;
					}
		}

	return false;
}

void simSend32(uint32_t a)
{
	  simSendByte((a    ) & 0xFF);
	  simSendByte((a>> 8) & 0xFF);
	  simSendByte((a>>16) & 0xFF);
	  simSendByte((a>>24) & 0xFF);
}

/* 32-bit Parameter Type */
typedef union {
  uint32_t  UInt;
  uint8_t	Byte[4];
  float		Float;
} Param32;

// Send the packet to the simulator Prepar3D
// Format: Heading, Roll, Pitch, Yaw, Throttle, debug1, debug2
static void simSendPacket(void)
{
	static Param32 p;

	switch(cfg.mixerConfiguration)
	{
	case MULTITYPE_QUADP:

		break;

	case MULTITYPE_FLYING_WING:

		break;

	case MULTITYPE_HELI_90_DEG:
		// Send float NaN
		simSend32(0xFFC00000);

		p.Float = constrain((servo[0] - cfg.servotrim[0]) / 5, -100, 100);
		simSend32(p.UInt);

		p.Float = constrain((servo[1] - cfg.servotrim[1]) / 5, -100, 100);;
		simSend32(p.UInt);

		p.Float = constrain((servo[2] - cfg.servotrim[2]) / 5, -100, 100);
		simSend32(p.UInt);

		p.Float = constrain((motor[0] - cfg.minthrottle) / 10, 0, 100);
		simSend32(p.UInt);

		p.Float = 0;
		simSend32(p.UInt);

		p.Float = 0;
		simSend32(p.UInt);
		break;
	}
}

/*
 * 	Prepar3D flight simulator protocol task
 * 	We use VCP2 or UART1 if configured
 */
portTASK_FUNCTION_PROTO(simTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint8_t inBuf[sizeof(sim_pack_in_t)];
	sim_pack_in_t * packet = (sim_pack_in_t *) &inBuf[0];
	uint8_t gpsCycleCount = 0;
	uint8_t sendCycleCount = 0;

	if (cfg.port_map == PORT_MAP_UART1xSIM_VCP1xMSP_VCP2xMAVLINK)
	{
		fifoBuf_init(&sim_Rx_Buffer_Hnd, &sim_Rx_Buffer, ML_RX_BUFFER_SIZE);

		uartInit(SERIAL_UART1, cfg.uart1_baudrate, simCallback_uart1);

		simSendByte = simSendByte_uart1;
		simReadByte = simReadByte_uart1;
		simHasData  = simHasData_uart1;
	}
	else
	{
		simVcpPort = cfg.port_map;

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

    		if (simParseChar(ch, (char *) packet))
    		{

				// Aply value from simulator
				// 0 - HIL off
				// 1 - sensor off
				// 2 - sensor & IMU off

				if (cfg.sim_mode == 2)
				{
					imu.rpy[ROLL ]		= RAD2DEG(-packet->roll) * 10;
					imu.rpy[PITCH]		= RAD2DEG(-packet->pitch) * 10;
					imu.rpy_rad[ROLL ]	= -packet->roll;
					imu.rpy_rad[PITCH]	= -packet->pitch;
					imu.rpy[YAW] 		= RAD2DEG(packet->head);
					imu.rpy_rad[YAW]	= packet->head;
					EstAlt 				= packet->alt * 100;
				}

				accSmooth[ROLL ]	= packet->acc_z / 9.80665f * acc_1G;
				accSmooth[PITCH] 	= packet->acc_x / 9.80665f * acc_1G;
				accSmooth[YAW  ]   	= -packet->acc_y / 9.80665f * acc_1G;

				gyroData[ROLL ]		= packet->gyro_z;
				gyroData[PITCH]		= packet->gyro_x;
				gyroData[YAW  ]		= packet->gyro_y;

				if (gpsCycleCount == 20)
				{
					// Simulate 5Hz GPS update
					gpsCycleCount = 0;

					gps.coord[LAT] 		= packet->lat * 1e7;
					gps.coord[LON] 		= packet->lon * 1e7;
					gps.altitude		= packet->alt * 100;
					gps.speed			= packet->speed * 10;
					gps.ground_course 	= RAD2DEG(packet->head) * 10;
					gps.update 			= (gps.update == 1 ? 0 : 1);
					gps.numSat 			= 12;
					gps.hdop 			= 1;
					gps.frames_rx ++;	// Just for control

					sensorsSet(SENSOR_GPS);
					flagSet(FLAG_GPS_FIX);

					xTimerReset(gpsLostTimer, 10);
					xSemaphoreGive(gpsSemaphore);	// Signal to navigate
				}

			}
    	}

    	if (gpsCycleCount < 20)
    		gpsCycleCount++;

    	if (++sendCycleCount == 5)
		{
			sendCycleCount = 0;
			simSendPacket();
		}

	    // Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 10); 	// Task cycle rate 100 Hz
	}
}

