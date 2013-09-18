/* 
 * FrSky Telemetry implementation by silpstream @ rcgroups
 */

#include "main.h"

#define PROTOCOL_HEADER       0x5E
#define PROTOCOL_TAIL         0x5E

// Data Ids  (bp = before decimal point; af = after decimal point)
// Official data IDs
#define ID_GPS_ALTIDUTE_BP    0x01
#define ID_GPS_ALTIDUTE_AP    0x09
#define ID_TEMPRATURE1        0x02
#define ID_RPM                0x03
#define ID_FUEL_LEVEL         0x04
#define ID_TEMPRATURE2        0x05
#define ID_VOLT               0x06
#define ID_ALTITUDE_BP        0x10
#define ID_ALTITUDE_AP        0x21
#define ID_GPS_SPEED_BP       0x11
#define ID_GPS_SPEED_AP       0x19
#define ID_LONGITUDE_BP       0x12
#define ID_LONGITUDE_AP       0x1A
#define ID_E_W                0x22
#define ID_LATITUDE_BP        0x13
#define ID_LATITUDE_AP        0x1B
#define ID_N_S                0x23
#define ID_COURSE_BP          0x14
#define ID_COURSE_AP          0x1C
#define ID_DATE_MONTH         0x15
#define ID_YEAR               0x16
#define ID_HOUR_MINUTE        0x17
#define ID_SECOND             0x18
#define ID_ACC_X              0x24
#define ID_ACC_Y              0x25
#define ID_ACC_Z              0x26
#define ID_VOLTAGE_AMP_BP     0x3A
#define ID_VOLTAGE_AMP_AP     0x3B
#define ID_CURRENT            0x28
// User defined data IDs
#define ID_GYRO_X             0x40
#define ID_GYRO_Y             0x41
#define ID_GYRO_Z             0x42

#define FRSKY_UART			SERIAL_UART5

static void sendDataHead(uint8_t id)
{
    uartWrite(FRSKY_UART, PROTOCOL_HEADER);
    uartWrite(FRSKY_UART, id);
}

static void sendTelemetryTail(void)
{
    uartWrite(FRSKY_UART, PROTOCOL_TAIL);
}

static void serializeFrsky(uint8_t data)
{
    // take care of byte stuffing
    if (data == 0x5e) {
        uartWrite(FRSKY_UART, 0x5d);
        uartWrite(FRSKY_UART, 0x3e);
    } else if (data == 0x5d) {
        uartWrite(FRSKY_UART, 0x5d);
        uartWrite(FRSKY_UART, 0x3d);
    } else
        uartWrite(FRSKY_UART, data);
}

static void serialize16(int16_t a)
{
    uint8_t t;
    t = a;
    serializeFrsky(t);
    t = a >> 8 & 0xff;
    serializeFrsky(t);
}

static void sendAccel(void)
{
    int i;

    for (i = 0; i < 3; i++) {
        sendDataHead(ID_ACC_X + i);
        serialize16(((float)accSmooth[i] / acc_1G) * 1000);
    }
}

static void sendBaro(void)
{
    sendDataHead(ID_ALTITUDE_BP);
    serialize16(EstAlt / 100);
    sendDataHead(ID_ALTITUDE_AP);
    serialize16(EstAlt % 100);
}

static void sendTemperature1(void)
{
    sendDataHead(ID_TEMPRATURE1);
    serialize16(baro_temp / 10);
}

static void sendTime(void)
{
    uint32_t seconds = millis() / 1000;
    uint8_t minutes = (seconds / 60) % 60;

    // if we fly for more than an hour, something's wrong anyway
    sendDataHead(ID_HOUR_MINUTE);
    serialize16(minutes << 8);
    sendDataHead(ID_SECOND);
    serialize16(seconds % 60);
}

static void sendGPS(void)
{
    sendDataHead(ID_LATITUDE_BP);
    serialize16(abs(gps.coord[LAT]) / 100000);
    sendDataHead(ID_LATITUDE_AP);
    serialize16((abs(gps.coord[LAT]) / 10) % 10000);

    sendDataHead(ID_N_S);
    serialize16(gps.coord[LAT] < 0 ? 'S' : 'N');

    sendDataHead(ID_LONGITUDE_BP);
    serialize16(abs(gps.coord[LON]) / 100000);
    sendDataHead(ID_LONGITUDE_AP);
    serialize16((abs(gps.coord[LON]) / 10) % 10000);
    sendDataHead(ID_E_W);
    serialize16(gps.coord[LON] < 0 ? 'W' : 'E');
}

static void sendVoltage(void)
{
    uint16_t voltage;

    voltage = (power.flightBatteryVoltage * 110) / 21;

    sendDataHead(ID_VOLTAGE_AMP_BP);
    serialize16(voltage / 100);
    sendDataHead(ID_VOLTAGE_AMP_AP);
    serialize16(((voltage % 100) + 5) / 10);
}

static void sendHeading(void)
{
    sendDataHead(ID_COURSE_BP);
    serialize16(heading);
    sendDataHead(ID_COURSE_AP);
    serialize16(0);
}

/*
   1   |  2   |   3    |   4    |    5      |   6         | 7 .. 10 |   11
  Head | PRIM | Analog | Analog | (Up) Link | (Down) Link | 4 Bytes | End Byte
       |      | value  | value  | Quality   | Quality     |         |
  0x7E | 0xFE | Port1  | Port2  | Min » 40  | 2 * RSSI    |    00   |   0x7E
       |      |        |        | Max 110   |

	Byte stuffing method:
	When byte 0x7D is received, discard this byte, and the next byte is XORed with 0x20;
 */

// Receive buffer state machine state defs
enum FRSKY_TWP_STATES
{
	FRSKY_TWP_IDLE = 0,
	FRSKY_TWP_DATASTART,
	FRSKY_TWP_DATAINFRAME,
	FRSKY_TWP_DATAXOR,
	FRSKY_TWP_STATE_ENUM_END
};

#define START_STOP      		0x7e
#define BYTESTUFF       		0x7d
#define STUFF_MASK      		0x20
#define FRSKY_RX_PACKET_SIZE	18

uint8_t frskyRxBuffer[FRSKY_RX_PACKET_SIZE];   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)

void frskyProcessPacket(uint8_t * packet)
{
	switch (packet[0])
	{
	case 0xFE:	// Remote voltage and link quality
//		debug[0] = packet[1];
//		debug[1] = packet[2];
//		debug[2] = packet[3];
//		debug[3] = packet[4] / 2;
		rssi = packet[3];
		break;
	}
}

void frskyCallback(uint16_t data)
{
	static uint8_t numPktBytes = 0;
	static uint8_t dataState = FRSKY_TWP_IDLE;

	switch (dataState)
	{
	case FRSKY_TWP_DATASTART:
//		if (data == START_STOP)
//			break; // Remain in userDataStart if possible 0x7e,0x7e doublet found.

		if (numPktBytes < FRSKY_RX_PACKET_SIZE)
			frskyRxBuffer[numPktBytes++] = data;
		dataState = FRSKY_TWP_DATAINFRAME;
		break;

	case FRSKY_TWP_DATAINFRAME:
		if (data == BYTESTUFF)
		{
			dataState = FRSKY_TWP_DATAXOR; // XOR next byte
			break;
		}
		if (data == START_STOP) // end of frame detected
		{
			frskyProcessPacket(frskyRxBuffer); // FrskyRxBufferReady = 1;
			dataState = FRSKY_TWP_IDLE;
			break;
		}
		if (numPktBytes < FRSKY_RX_PACKET_SIZE)
			frskyRxBuffer[numPktBytes++] = data;
		break;

	case FRSKY_TWP_DATAXOR:
		if (numPktBytes < FRSKY_RX_PACKET_SIZE)
			frskyRxBuffer[numPktBytes++] = data ^ STUFF_MASK;
		dataState = FRSKY_TWP_DATAINFRAME;
		break;

	case FRSKY_TWP_IDLE:
		if (data == START_STOP)
		{
			numPktBytes = 0;
			dataState = FRSKY_TWP_DATASTART;
		}
		break;

	} // switch
}

// Task sends FrSky telemetry
portTASK_FUNCTION_PROTO(frskyTask, pvParameters)
{
	portTickType xLastWakeTime;
	static uint8_t cycleNum = 0;

	// Init UART5 for FrSky telemetry
	uartInit(FRSKY_UART, 9600, frskyCallback);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 125);	// Task cycle time 125 ms

        cycleNum++;

        // Sent every 125ms
        sendAccel();
        sendTelemetryTail();

        if ((cycleNum % 4) == 0) {      // Sent every 500ms
            sendBaro();
            sendHeading();
            sendTelemetryTail();
        }

        if ((cycleNum % 8) == 0) {      // Sent every 1s
            sendTemperature1();
            sendVoltage();
            sendGPS();
            sendTelemetryTail();
        }

        if (cycleNum == 40) {     //Frame 3: Sent every 5s
            cycleNum = 0;
            sendTime();
            sendTelemetryTail();
        }
    }
}
