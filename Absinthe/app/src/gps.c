
#include "main.h"

// **********************
// GPS
// **********************
int32_t GPS_coord[2];
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint8_t GPS_numSat;
uint16_t GPS_distanceToHome; 	// distance to home point in meters
int16_t GPS_directionToHome; 	// direction to home or hol point in degrees 0..360
uint16_t GPS_altitude;			// altitude in 0.1m
uint16_t GPS_speed; 			// speed in 0.1m/s
uint8_t GPS_update = 0; 		// it's a binary toogle to distinct a GPS position update
int16_t GPS_angle[2] = { 0, 0 }; // it's the angles that must be applied for GPS correction
uint16_t GPS_ground_course = 0; // degrees * 10
uint16_t GPS_hdop;


/* GPS diagnostic counters */
uint32_t gps_bytes_rx = 0;
uint32_t gps_frames_rx = 0;

xSemaphoreHandle gpsSemaphore = NULL;
xTimerHandle gpsLostTimer;

#define GPS_UART			SERIAL_UART3

const uint32_t init_speed[5] = { 9600, 19200, 38400, 57600, 115200 };

static const char * const gpsInitStrings[] = {
    "$PUBX,41,1,0003,0001,19200,0*23\r\n",      // UBX0..3
    "$PUBX,41,1,0003,0001,38400,0*26\r\n",
    "$PUBX,41,1,0003,0001,57600,0*2D\r\n",
    "$PUBX,41,1,0003,0001,115200,0*1E\r\n",
    "$PMTK251,19200*22\r\n",                    // MTK4..7
    "$PMTK251,38400*27\r\n",
    "$PMTK251,57600*2C\r\n",
    "$PMTK251,115200*1F\r\n",
};

static const uint8_t ubloxInit[] = {
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,                                  // disable all default NMEA messages
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                                  // set POSLLH MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                                  // set STATUS MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                                  // set SOL MSG rate
     0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                                  // set VELNED MSG rate
     0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41,    // set WAAS to EGNOS
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A                 // set rate to 5Hz
};

static void gpsPrint(const char *str)
{
    while (*str) {
        uartWrite(GPS_UART, *str);
        if (cfg.gps_type == GPS_UBLOX)
            vTaskDelay(4 / portTICK_RATE_MS);	// Delay on 4 ms
        str++;
    }
    // wait to send all
    while (!uartTransmitEmpty(GPS_UART));
    vTaskDelay(30 / portTICK_RATE_MS);	// Delay on 30 ms
}

// This code is used for parsing NMEA data

/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *s)
{
    char *p = s, *d = s;
    uint8_t min, deg = 0;
    uint16_t frac = 0, mult = 10000;

    while (*p) {                // parse the string until its end
        if (d != s) {
            frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
            mult /= 10;
        }
        if (*p == '.')
            d = p;              // locate '.' char in the string
        p++;
    }
    if (p == s)
        return 0;
    while (s < d - 2) {
        deg *= 10;              // convert degrees : all chars before minutes ; for the first iteration, deg = 0
        deg += *(s++) - '0';
    }
    min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;       // convert minutes : 2 previous char before '.'
    return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
*/

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    int i;

    // scan for decimal point or end of field
    for (p = s; isdigit(*p); p++)
        ;
    q = s;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.') {
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit(*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
    }
    return tmp;
}

static uint8_t hex_c(uint8_t n)
{                               // convert '0'..'9','A'..'F' to 0..15
    n -= '0';
    if (n > 9)
        n -= 7;
    n &= 0x0F;
    return n;
}

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/
#define FRAME_GGA  1
#define FRAME_RMC  2

static bool GPS_NMEA_newFrame(char c)
{
	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, frame = 0;

	if (c == '$')
	{
		param = 0;
		offset = 0;
		parity = 0;
	}
	else if (c == ',' || c == '*')
	{
		string[offset] = 0;
		if (param == 0)
		{ //frame identification
			frame = 0;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G'
					&& string[3] == 'G' && string[4] == 'A')
				frame = FRAME_GGA;
			if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R'
					&& string[3] == 'M' && string[4] == 'C')
				frame = FRAME_RMC;
		}
		else if (frame == FRAME_GGA)
		{
			if (param == 2)
			{
				GPS_coord[LAT] = GPS_coord_to_degrees(string);
			}
			else if (param == 3 && string[0] == 'S')
				GPS_coord[LAT] = -GPS_coord[LAT];
			else if (param == 4)
			{
				GPS_coord[LON] = GPS_coord_to_degrees(string);
			}
			else if (param == 5 && string[0] == 'W')
				GPS_coord[LON] = -GPS_coord[LON];
			else if (param == 6)
			{
				if (string[0] > '0')
					flagSet(FLAG_GPS_FIX);
				else
					flagClear(FLAG_GPS_FIX);
			}
			else if (param == 7)
			{
				GPS_numSat = grab_fields(string, 0);
			}
			else if (param == 9)
			{
				GPS_altitude = grab_fields(string, 0); // altitude in meters added by Mis
			}
		}
		else if (frame == FRAME_RMC)
		{
			if (param == 7)
			{
				GPS_speed = (grab_fields(string, 1) * 5144L) / 1000L; // speed in cm/s added by Mis
			}
			else if (param == 8)
			{
				GPS_ground_course = grab_fields(string, 1); // ground course deg * 10
			}
		}
		param++;
		offset = 0;
		if (c == '*')
			checksum_param = 1;
		else
			parity ^= c;
	}
	else if (c == '\r' || c == '\n')
	{
		if (checksum_param)
		{ // parity checksum
			uint8_t checksum = hex_c(string[0]);
			checksum <<= 4;
			checksum += hex_c(string[1]);
			if (checksum == parity)
				frameOK = 1;
		}
		checksum_param = 0;
	}
	else
	{
		if (offset < 15)
			string[offset++] = c;
		if (!checksum_param)
			parity ^= c;
	}
	return frameOK && (frame == FRAME_GGA);
}


// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubs_protocol_bytes;

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;
static uint8_t _class;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

//static uint8_t _disable_counter;

// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    uint8_t bytes[64];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}

static bool UBLOX_parse_gps(void)
{
    switch (_msg_id) {
    case MSG_POSLLH:
        //i2c_dataset.time                = _buffer.posllh.time;
        GPS_coord[LON] = _buffer.posllh.longitude;
        GPS_coord[LAT] = _buffer.posllh.latitude;
        GPS_altitude = _buffer.posllh.altitude_msl / 10 / 100;  //alt in m
        if (next_fix)
        	flagSet(FLAG_GPS_FIX);
        else
        	flagClear(FLAG_GPS_FIX);
        _new_position = true;
        break;
    case MSG_STATUS:
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            flagClear(FLAG_GPS_FIX);
        break;
    case MSG_SOL:
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            flagClear(FLAG_GPS_FIX);
        GPS_numSat = _buffer.solution.satellites;
        GPS_hdop   = _buffer.solution.position_DOP;
        // debug[3] = GPS_hdop;
        break;
    case MSG_VELNED:
        // speed_3d = _buffer.velned.speed_3d;  // cm/s
        GPS_speed = _buffer.velned.speed_2d;    // cm/s
        GPS_ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}

static bool GPS_UBLOX_newFrame(uint8_t data)
{
    bool parsed = false;

    switch (_step) {
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        case 0:
            if (PREAMBLE1 == data)
                _step++;
            break;
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;   // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length = data; // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length += (uint16_t) (data << 8);
            if (_payload_length > 512) {
                _payload_length = 0;
                _step = 0;
            }
            _payload_counter = 0;   // prepare to receive payload
            break;
        case 6:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data)
                _step = 0;          // bad checksum
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)
                break;              // bad checksum
            if (UBLOX_parse_gps()) {
                parsed = true;
            }
            break;
    }                           //end switch
    return parsed;
}

static bool GPS_newFrame(char c)
{
    switch (cfg.gps_type) {
        case GPS_NMEA: // NMEA
        case GPS_MTK: // MTK outputs NMEA too
            return GPS_NMEA_newFrame(c);
        case GPS_UBLOX: // UBX
            return GPS_UBLOX_newFrame(c);
    }
    return false;
}

void GPS_NewData(uint16_t c)
{
	gps_bytes_rx ++;

	if (!GPS_newFrame(c)) return;

	gps_frames_rx ++;

    sensorsSet(SENSOR_GPS);
    xTimerReset(gpsLostTimer, 10);

    GPS_update = (GPS_update == 1 ? 0 : 1);

	if (flag(FLAG_GPS_FIX) && GPS_numSat > 4)
		xSemaphoreGive(gpsSemaphore);
}

/* GPS signal lost timer callback function */
void gpsLostTimerCallback(xTimerHandle pxTimer)
{
    sensorsClear(SENSOR_GPS);
    flagClear(FLAG_GPS_FIX);
}

void gpsInit(uint32_t baudrate)
{
    int i;
    int offset = 0;

	// Create semaphore for OSD frame drawing syncronization
	vSemaphoreCreateBinary(gpsSemaphore);

	// Create GPS sensor lost timer at 3 seconds
	gpsLostTimer = xTimerCreate((signed char *) "TimGPSlost", 3000, pdFALSE, (void *) NULL, gpsLostTimerCallback);

	// Start GPS lost timer
	xTimerStart(gpsLostTimer, 10);

    GPS_set_pids();

    uartInit(GPS_UART, baudrate, GPS_NewData);

    if (cfg.gps_type == GPS_UBLOX)
        offset = 0;
    else if (cfg.gps_type == GPS_MTK)
        offset = 4;

    if (cfg.gps_type != GPS_NMEA) {
        for (i = 0; i < 5; i++) {
            uartChangeBaud(GPS_UART, init_speed[i]);
            switch (baudrate) {
                case 19200:
                    gpsPrint(gpsInitStrings[offset]);
                    break;
                case 38400:
                    gpsPrint(gpsInitStrings[offset + 1]);
                    break;
                case 57600:
                    gpsPrint(gpsInitStrings[offset + 2]);
                    break;
                case 115200:
                    gpsPrint(gpsInitStrings[offset + 3]);
                    break;
            }
            vTaskDelay(10 / portTICK_RATE_MS);	// Delay on 10 ms
        }
    }

    uartChangeBaud(GPS_UART, baudrate);
    if (cfg.gps_type == GPS_UBLOX) {
        for (i = 0; i < sizeof(ubloxInit); i++) {
            uartWrite(GPS_UART, ubloxInit[i]); // send ubx init binary
            vTaskDelay(4 / portTICK_RATE_MS);	// Delay on 4 ms
        }
    } else if (cfg.gps_type == GPS_MTK) {
        gpsPrint("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");  // only GGA and RMC sentence
        gpsPrint("$PMTK220,200*2C\r\n");                                    // 5 Hz update rate
    }
}

