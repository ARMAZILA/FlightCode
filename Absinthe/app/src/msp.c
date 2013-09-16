
#include "main.h"

// Multiwii Serial Protocol 0 
#define MSP_VERSION              0
#define PLATFORM_32BIT           0x80000000

#define MSP_IDENT                100    //out message         multitype + version
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_SERVO                103    //out message         8 servos
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan
#define MSP_RAW_GPS              106    //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107    //out message         distance home, direction home
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         1 altitude
#define MSP_BAT                  110    //out message         vbat, powermetersum
#define MSP_RC_TUNING            111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112    //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113    //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114    //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES             116    //out message         the aux switch names
#define MSP_PIDNAMES             117    //out message         the PID names
#define MSP_WP                   118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_RAW_GPS          201    //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202    //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203    //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205    //in message          no param
#define MSP_MAG_CALIBRATION      206    //in message          no param
#define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208    //in message          no param
#define MSP_WP_SET               209    //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   	//in message          Select Setting Number (0-2)

#define MSP_EEPROM_WRITE         250    //in message          no param

#define MSP_DEBUGMSG             253    //out message         debug string buffer
#define MSP_DEBUG                254    //out message         debug1,debug2,debug3,debug4

#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values

#define INBUF_SIZE 64

static const char boxnames[] =
    "ANGLE;"
    "HORIZON;"
    "ATL HOLD;"
    "MAG;"
    "CAMSTAB;"
    "CAMTRIG;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "LEDMAX;"
    "LLIGHTS;"
    "HEADADJ;";

static const char pidnames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "ALT;"
    "Pos;"
    "PosR;"
    "NavR;"
    "LEVEL;"
    "MAG;"
    "VEL;";

// MSP protocol state mashine code
typedef enum {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
} c_state_t;

typedef struct com_port_t {
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];
    uint8_t cmdMSP;
    uint8_t offset;
    uint8_t dataSize;
    c_state_t c_state;
    serialSendByte_t tx;
    serialReadByte_t rx;
    serialHasData_t da;	// Data Available
} com_port_t;

uint8_t cliMode = 0;
static com_port_t com_drv[1];
#define MSP_UART		SERIAL_UART1 // VCP1 is the same value 0!!!

void serialize8(uint8_t port, uint8_t a)
{
	com_drv[port].tx(a);
    com_drv[port].checksum ^= a;
}

void serialize16(uint8_t port, int16_t a) {
  serialize8(port, (a   ) & 0xFF);
  serialize8(port, (a>>8) & 0xFF);
}

void serialize32(uint8_t port, uint32_t a) {
  serialize8(port, (a    ) & 0xFF);
  serialize8(port, (a>> 8) & 0xFF);
  serialize8(port, (a>>16) & 0xFF);
  serialize8(port, (a>>24) & 0xFF);
}

uint8_t read8(uint8_t port)
{
    return com_drv[port].inBuf[com_drv[port].indRX++] & 0xff;
}

uint16_t read16(uint8_t port)
{
    uint16_t t = read8(port);
    t += (uint16_t) read8(port) << 8;
    return t;
}

uint32_t read32(uint8_t port)
{
    uint32_t t = read16(port);
    t += (uint32_t) read16(port) << 16;
    return t;
}

void headSerialResponse(uint8_t port, uint8_t err, uint8_t s)
{
    serialize8(port, '$');
    serialize8(port, 'M');
    serialize8(port, err ? '!' : '>');
    com_drv[port].checksum = 0;               // start calculating a new checksum
    serialize8(port, s);
    serialize8(port, com_drv[port].cmdMSP);
}

void headSerialReply(uint8_t port, uint8_t s)
{
    headSerialResponse(port, 0, s);
}

void headSerialError(uint8_t port, uint8_t s)
{
    headSerialResponse(port, 1, s);
}

void tailSerialReply(uint8_t port)
{
    serialize8(port, com_drv[port].checksum);
}

void serializeNames(uint8_t port, const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(port, *c);
}

static void evaluateCommand(uint8_t port)
{
    uint32_t i;
    uint8_t wp_no;

    switch (com_drv[port].cmdMSP) {
    case MSP_SET_RAW_RC:
        for (i = 0; i < 8; i++)
            rcData[i] = read16(port);
        headSerialReply(port, 0);
        break;
    case MSP_SET_ACC_TRIM:
        cfg.angleTrim[PITCH] = read16(port);
        cfg.angleTrim[ROLL]  = read16(port);
        headSerialReply(port, 0);
        break;
    case MSP_SET_RAW_GPS:
        if (read8(port))
        	flagSet(FLAG_GPS_FIX);
        else
        	flagClear(FLAG_GPS_FIX);
        GPS_numSat = read8(port);
        GPS_coord[LAT] = read32(port);
        GPS_coord[LON] = read32(port);
        GPS_altitude = read16(port);
        GPS_speed = read16(port);
        GPS_update |= 2;        // New data signalisation to GPS functions
        headSerialReply(port, 0);
        break;
    case MSP_SET_PID:
        for (i = 0; i < PIDITEMS; i++) {
            cfg.P8[i] = read8(port);
            cfg.I8[i] = read8(port);
            cfg.D8[i] = read8(port);
        }
        headSerialReply(port, 0);
        break;
    case MSP_SET_BOX:
        for (i = 0; i < CHECKBOXITEMS; i++)
            cfg.activate[i] = read16(port);
        headSerialReply(port, 0);
        break;
    case MSP_SET_RC_TUNING:
        cfg.rcRate8 = read8(port);
        cfg.rcExpo8 = read8(port);
        cfg.rollPitchRate = read8(port);
        cfg.yawRate = read8(port);
        cfg.dynThrPID = read8(port);
        cfg.thrMid8 = read8(port);
        cfg.thrExpo8 = read8(port);
        headSerialReply(port, 0);
        break;
    case MSP_SET_MISC:
        headSerialReply(port, 0);
        break;
    case MSP_IDENT:
        headSerialReply(port, 7);
        serialize8(port, MW_VERSION);             // multiwii version
        serialize8(port, cfg.mixerConfiguration); // type of multicopter
        serialize8(port, MSP_VERSION);            // MultiWii Serial Protocol Version
        serialize32(port, PLATFORM_32BIT);        // "capability"
        break;
    case MSP_STATUS:
        headSerialReply(port, 11);
        serialize16(port, cycleTime);
        serialize16(port, i2cGetErrorCounter());
        serialize16(port,
        		sensors(SENSOR_ACC) |
        		sensors(SENSOR_BARO) << 1 |
        		sensors(SENSOR_MAG) << 2 |
        		sensors(SENSOR_GPS) << 3 |
        		sensors(SENSOR_SONAR) << 4);
        serialize32(port,
        		flag(FLAG_ANGLE_MODE) << BOXANGLE |
        		flag(FLAG_HORIZON_MODE) << BOXHORIZON |
        		flag(FLAG_ALTHOLD_MODE) << BOXALTHOLD |
        		flag(FLAG_MAG_MODE) << BOXMAG |
        		flag(FLAG_ARMED) << BOXARM |
                rcOptions[BOXCAMSTAB] << BOXCAMSTAB |
                rcOptions[BOXCAMTRIG] << BOXCAMTRIG |
                flag(FLAG_GPSHOME_MODE) << BOXGPSHOME |
                flag(FLAG_GPSHOLD_MODE) << BOXGPSHOLD |
                flag(FLAG_HEADFREE_MODE) << BOXHEADFREE |
                flag(FLAG_PASSTHRU_MODE) << BOXPASSTHRU |
                rcOptions[BOXBEEPERON] << BOXBEEPERON |
                rcOptions[BOXLEDMAX] << BOXLEDMAX |
                rcOptions[BOXLLIGHTS] << BOXLLIGHTS |
                rcOptions[BOXHEADADJ] << BOXHEADADJ);
        serialize8(port, 0);	// TODO: Add setting profiles
        break;
    case MSP_RAW_IMU:
        headSerialReply(port, 18);
        for (i = 0; i < 3; i++)
            serialize16(port, accSmooth[i]);
        for (i = 0; i < 3; i++)
            serialize16(port, gyroData[i]);
        for (i = 0; i < 3; i++)
            serialize16(port, mag_sensor_data[i]);
        break;
    case MSP_SERVO:
        headSerialReply(port, 16);
        for (i = 0; i < 8; i++)
            serialize16(port, servo[i]);
        break;
    case MSP_MOTOR:
        headSerialReply(port, 16);
        for (i = 0; i < 8; i++)
            serialize16(port, motor[i]);
        break;
    case MSP_RC:
        headSerialReply(port, 16);
        for (i = 0; i < 8; i++)
            serialize16(port, rcData[i]);
        break;
    case MSP_RAW_GPS:
        headSerialReply(port, 14);
        serialize8(port, flag(FLAG_GPS_FIX));
        serialize8(port, GPS_numSat);
        serialize32(port, GPS_coord[LAT]);
        serialize32(port, GPS_coord[LON]);
        serialize16(port, GPS_altitude);
        serialize16(port, GPS_speed);
        break;
    case MSP_COMP_GPS:
        headSerialReply(port, 5);
        serialize16(port, GPS_distanceToHome);
        serialize16(port, GPS_directionToHome);
        serialize8(port, GPS_update & 1);
        break;
    case MSP_ATTITUDE:
        headSerialReply(port, 8);
        for (i = 0; i < 2; i++)
            serialize16(port, angle[i]);
        serialize16(port, heading);
        serialize16(port, headFreeModeHold);
        break;
    case MSP_ALTITUDE:
        headSerialReply(port, 4);
        serialize32(port, EstAlt);
        break;
    case MSP_BAT:
        headSerialReply(port, 3);
        serialize8(port, vbat);
        serialize16(port, 0); // power meter trash
        break;
    case MSP_RC_TUNING:
        headSerialReply(port, 7);
        serialize8(port, cfg.rcRate8);
        serialize8(port, cfg.rcExpo8);
        serialize8(port, cfg.rollPitchRate);
        serialize8(port, cfg.yawRate);
        serialize8(port, cfg.dynThrPID);
        serialize8(port, cfg.thrMid8);
        serialize8(port, cfg.thrExpo8);
        break;
    case MSP_PID:
        headSerialReply(port, 3 * PIDITEMS);
        for (i = 0; i < PIDITEMS; i++) {
            serialize8(port, cfg.P8[i]);
            serialize8(port, cfg.I8[i]);
            serialize8(port, cfg.D8[i]);
        }
        break;
    case MSP_BOX:
        headSerialReply(port, 2 * CHECKBOXITEMS);
        for (i = 0; i < CHECKBOXITEMS; i++)
            serialize16(port, cfg.activate[i]);
        break;
    case MSP_BOXNAMES:
        headSerialReply(port, sizeof(boxnames) - 1);
        serializeNames(port, boxnames);
        break;
    case MSP_PIDNAMES:
        headSerialReply(port, sizeof(pidnames) - 1);
        serializeNames(port, pidnames);
        break;
    case MSP_MISC:
        headSerialReply(port, 2);
        serialize16(port, 0); // intPowerTrigger1
        break;
    case MSP_MOTOR_PINS:
        headSerialReply(port, 8);
        for (i = 0; i < 8; i++)
            serialize8(port, i + 1);
        break;
    case MSP_WP:
        wp_no = read8(port);    // get the wp number
        headSerialReply(port, 12);
        if (wp_no == 0) {
            serialize8(port, 0);                   // wp0
            serialize32(port, GPS_home[LAT]);
            serialize32(port, GPS_home[LON]);
            serialize16(port, 0);                  // altitude will come here
            serialize8(port, 0);                   // nav flag will come here
        } else if (wp_no == 16) {
            serialize8(port, 16);                  // wp16
            serialize32(port, GPS_hold[LAT]);
            serialize32(port, GPS_hold[LON]);
            serialize16(port, 0);                  // altitude will come here
            serialize8(port, 0);                   // nav flag will come here
        }
        break;
    case MSP_RESET_CONF:
        checkFirstTime(true);
        headSerialReply(port, 0);
        break;
    case MSP_ACC_CALIBRATION:
    	flagSet(FLAG_CALIBRATE_ACC);
        headSerialReply(port, 0);
        break;
    case MSP_MAG_CALIBRATION:
        flagSet(FLAG_CALIBRATE_MAG);
        headSerialReply(port, 0);
        break;
    case MSP_EEPROM_WRITE:
        writeFlashConfig(0);
        headSerialReply(port, 0);
        break;
    case MSP_ACC_TRIM:
        headSerialReply(port, 4);
        serialize16(port, cfg.angleTrim[PITCH]);
        serialize16(port, cfg.angleTrim[ROLL]);
        break;
    case MSP_DEBUG:
        headSerialReply(port, 8);
        for (i = 0; i < 4; i++)
            serialize16(port, debug[i]);      // 4 variables are here for general monitoring purpose
        break;
    default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
        headSerialError(port, 0);
        break;
    }
    tailSerialReply(port);
}

// evaluate all other incoming serial data
static void evaluateOtherData(uint8_t sr)
{
    switch (sr) {
        case '#':
            cliProcess();
            break;
        case 'R':
            systemReset(true);      // reboot to bootloader
            break;
    }
}

void serialCom(void)
{
    uint8_t c;
    uint8_t port = 0;

	// in cli mode, all uart stuff goes to here. enter cli mode by sending #.
	if (cliMode) {
		cliProcess();
		return;
	}

	while (com_drv[port].da())
	{
		c = com_drv[port].rx();

		if (com_drv[port].c_state == IDLE) {
			com_drv[port].c_state = (c == '$') ? HEADER_START : IDLE;
			if (com_drv[port].c_state == IDLE && port == 0)
				evaluateOtherData(c);				// evaluate all other incoming serial data
		} else if (com_drv[port].c_state == HEADER_START) {
			com_drv[port].c_state = (c == 'M') ? HEADER_M : IDLE;
		} else if (com_drv[port].c_state == HEADER_M) {
			com_drv[port].c_state = (c == '<') ? HEADER_ARROW : IDLE;
		} else if (com_drv[port].c_state == HEADER_ARROW) {
			if (c > INBUF_SIZE) {					// now we are expecting the payload size
				com_drv[port].c_state = IDLE;
				continue;
			}
			com_drv[port].dataSize = c;
			com_drv[port].offset = 0;
			com_drv[port].checksum = c;
			com_drv[port].indRX = 0;
			com_drv[port].c_state = HEADER_SIZE;	// the command is to follow
		} else if (com_drv[port].c_state == HEADER_SIZE) {
			com_drv[port].cmdMSP = c;
			com_drv[port].checksum ^= c;
			com_drv[port].c_state = HEADER_CMD;
		} else if (com_drv[port].c_state == HEADER_CMD && com_drv[port].offset < com_drv[port].dataSize) {
			com_drv[port].checksum ^= c;
			com_drv[port].inBuf[com_drv[port].offset++] = c;
		} else if (com_drv[port].c_state == HEADER_CMD && com_drv[port].offset >= com_drv[port].dataSize) {
			if (com_drv[port].checksum == c) {		// compare calculated and transferred checksum
				evaluateCommand(port);      		// we got a valid packet, evaluate it
			}
			com_drv[port].c_state = IDLE;
		}
	}
}

#include "fifo_buffer.h"

#define	MSP_RX_BUFFER_SIZE	64
static t_fifo_buffer		msp_Rx_Buffer_Hnd;
static uint8_t	 			msp_Rx_Buffer[MSP_RX_BUFFER_SIZE];

/* UART helper functions */
static void mspCallback(uint16_t data)	{ fifoBuf_putByte(&msp_Rx_Buffer_Hnd, data); }
static uint16_t mspHasData(void)		{ return (fifoBuf_getUsed(&msp_Rx_Buffer_Hnd) == 0) ? false : true; }
static uint8_t mspRead(void)			{ return fifoBuf_getByte(&msp_Rx_Buffer_Hnd); }
static void mspWrite(uint8_t data)		{ uartWrite(MSP_UART, data); }

portTASK_FUNCTION_PROTO(mspTask, pvParameters)
{
	portTickType xLastWakeTime;

	// Setup port depending cfg.uart1_mode: see UART1_MODE_t ENUM
	if (cfg.uart1_mode == UART1_MODE_MSP)
	{
		fifoBuf_init(&msp_Rx_Buffer_Hnd, &msp_Rx_Buffer, MSP_RX_BUFFER_SIZE);
		uartInit(MSP_UART, cfg.uart1_baudrate, mspCallback);

		com_drv[0].c_state = IDLE;
		com_drv[0].da = mspHasData;
		com_drv[0].rx = mspRead;
		com_drv[0].tx = mspWrite;
	}
	else
	{
		com_drv[0].c_state = IDLE;
		com_drv[0].da = vcp1HasData;
		com_drv[0].rx = vcp1GetByte;
		com_drv[0].tx = vcp1SendByte;
	}

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
    	serialCom();

	    // Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 10); // Task cycle time 10 ms
	}
}
