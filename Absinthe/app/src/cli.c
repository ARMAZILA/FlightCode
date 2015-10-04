
#include "main.h"

 /* define the Key codes */
#define KEY_NUL 0 /**< ^@ Null character */
#define KEY_SOH 1 /**< ^A Start of heading, = console interrupt */
#define KEY_STX 2 /**< ^B Start of text, maintenance mode on HP console */
#define KEY_ETX 3 /**< ^C End of text */
#define KEY_EOT 4 /**< ^D End of transmission, not the same as ETB */
#define KEY_ENQ 5 /**< ^E Enquiry, goes with ACK; old HP flow control */
#define KEY_ACK 6 /**< ^F Acknowledge, clears ENQ logon hand */
#define KEY_BEL 7 /**< ^G Bell, rings the bell... */
#define KEY_BS  8 /**< ^H Backspace, works on HP terminals/computers */
#define KEY_HT  9 /**< ^I Horizontal tab, move to next tab stop */
#define KEY_LF  10  /**< ^J Line Feed */
#define KEY_VT  11  /**< ^K Vertical tab */
#define KEY_FF  12  /**< ^L Form Feed, page eject */
#define KEY_CR  13  /**< ^M Carriage Return*/
#define KEY_SO  14  /**< ^N Shift Out, alternate character set */
#define KEY_SI  15  /**< ^O Shift In, resume defaultn character set */
#define KEY_DLE 16  /**< ^P Data link escape */
#define KEY_DC1 17  /**< ^Q XON, with XOFF to pause listings; "okay to send". */
#define KEY_DC2 18  /**< ^R Device control 2, block-mode flow control */
#define KEY_DC3 19  /**< ^S XOFF, with XON is TERM=18 flow control */
#define KEY_DC4 20  /**< ^T Device control 4 */
#define KEY_NAK 21  /**< ^U Negative acknowledge */
#define KEY_SYN 22  /**< ^V Synchronous idle */
#define KEY_ETB 23  /**< ^W End transmission block, not the same as EOT */
#define KEY_CAN 24  /**< ^X Cancel line, MPE echoes !!! */
#define KEY_EM  25  /**< ^Y End of medium, Control-Y interrupt */
#define KEY_SUB 26  /**< ^Z Substitute */
#define KEY_ESC 27  /**< ^[ Escape, next character is not echoed */
#define KEY_FS  28  /**< ^\ File separator */
#define KEY_GS  29  /**< ^] Group separator */
#define KEY_RS  30  /**< ^^ Record separator, block-mode terminator */
#define KEY_US  31  /**< ^_ Unit separator */

#define KEY_DEL 127 /**< Delete (not a real control character...) */

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliCMix(char *cmdline);
static void cliDefault(char *cmdline);
static void cliDFU(char *cmdline);
static void cliExit(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliOs(char *cmdline);
static void cliSave(char *cmdline);
static void cliSensor(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);

static void cliTest(char *cmdline);
static void cliTele(char *cmdline);
static void cliBeep(char *cmdline);

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

serialReadByte_t rx;
serialHasData_t da;	// Data Available

static float _atof(const char *p);
char *ftoa(float x, char *floatString);

// sync this with MultiType enum from mw.h
const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI", "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "QUADC", "CUSTOM", NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
	{ "beep",	  "Play sound", 						cliBeep },
    { "cmix",	  "design custom mixer", 				cliCMix },
    { "default",  "reset to defaults and reboot", 		cliDefault },
    { "dfu",      "reset to DFU mode", 					cliDFU },
    { "exit",	  "close CLI session",					cliExit },
    { "help",	  "what you see",						cliHelp },
    { "map", 	  "mapping of rc channel order", 		cliMap },
    { "mixer", 	  "mixer name or list", 				cliMixer },
    { "os",       "FreeRTOS statistics",                cliOs },
    { "save", 	  "save and reboot", 					cliSave },
    { "sensor",	  "show & calibrate sensor", 			cliSensor },
    { "set", 	  "name=value or blank or * for list", 	cliSet },
    { "status",   "show system status", 				cliStatus },
    { "tele", 	  "",									cliTele },
    { "test",     "", 									cliTest },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

static void cliSetVar(const param_value_t *var, const int32_t value);
static void cliPrintVar(const param_value_t *var, uint32_t full);

#ifndef HAVE_ITOA_FUNCTION

/*
** The following two functions together make up an itoa()
** implementation. Function i2a() is a 'private' function
** called by the public itoa() function.
**
** itoa() takes three arguments:
**        1) the integer to be converted,
**        2) a pointer to a character conversion buffer,
**        3) the radix for the conversion
**           which can range between 2 and 36 inclusive
**           range errors on the radix default it to base10
** Code from http://groups.google.com/group/comp.lang.c/msg/66552ef8b04fe1ab?pli=1
*/

static char *i2a(unsigned i, char *a, unsigned r)
{
    if (i / r > 0) 
        a = i2a(i / r, a, r);
    *a = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"[i % r];
    return a + 1;
}

char *itoa(int i, char *a, int r)
{
    if ((r < 2) || (r > 36))
        r = 10;
    if (i < 0) {
        *a = '-';
        *i2a(-(unsigned)i, a + 1, r) = 0;
    } else 
        *i2a(i, a, r) = 0;
    return a;
} 

#endif

////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
static float _atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////////////////////////////////
// FTOA
///////////////////////////////////////////////////////////////////////////////
char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

static void cliPrompt(void)
{
    printf("\r\n# ");
}

static int cliCompare(const void *a, const void *b)
{
    const clicmd_t *ca = a, *cb = b;
    return strncasecmp(ca->name, cb->name, strlen(cb->name));
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (len == 0) {
        printf("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++) {
            if (cfg.customMixer[i].throttle == 0.0f)
                break;
            mixsum[i] = 0.0f;
            num_motors++;
            printf("#%d:\t", i + 1);
            printf("%s\t", ftoa(cfg.customMixer[i].throttle, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].roll, buf));
            printf("%s\t", ftoa(cfg.customMixer[i].pitch, buf));
            printf("%s\r\n", ftoa(cfg.customMixer[i].yaw, buf));
        }
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += cfg.customMixer[i].roll;
            mixsum[1] += cfg.customMixer[i].pitch;
            mixsum[2] += cfg.customMixer[i].yaw;
        }
        printf("Sanity check:\t");
        for (i = 0; i < 3; i++)
            printf(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        printf("\r\n");
        return;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    printf("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i);
                    printf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].throttle = _atof(++ptr); 
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].roll = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].pitch = _atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                cfg.customMixer[i].yaw = _atof(++ptr);
                check++;
            }
            if (check != 4) {
                printf("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            printf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDefault(char *cmdline)
{
	printf("Resetting to defaults...\r\n");
    checkFirstTime(true);
    printf("Rebooting...");
    vTaskDelay(3000 / portTICK_RATE_MS);	// Delay on 3 seconds
    systemReset(false);
}

static void cliDFU(char *cmdline)
{
	printf("Restart system in DFU mode...\r\n");
	printf("Use DfuSe tools for wirmware upgrade.");
    vTaskDelay(100 / portTICK_RATE_MS);	// Delay on 100 ms
    systemReset(true);
}

static void cliExit(char *cmdline)
{
	printf("\r\nLeaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    // save and reboot... I think this makes the most sense
    //cliSave(cmdline);
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    printf("Available commands:\r\n\r\n");
    for (i = 0; i < CMD_COUNT; i++)
        printf("%s\t\t%s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint8_t i;

    printf("Current assignment\r\n");
    printf("Func\tChan\r\n");
    printf("------------\r\n");
    printf("Roll\t%d\r\n", cfg.rcmap[0]);
    printf("Pitch\t%d\r\n", cfg.rcmap[1]);
    printf("Yaw\t%d\r\n", cfg.rcmap[2]);
    printf("Thro\t%d\r\n", cfg.rcmap[3]);

    for (i = 4; i < 16; i++)
        printf("Aux%d\t%d\r\n", i - 3, cfg.rcmap[i]);
}

static void cliMixer(char *cmdline)
{
    uint8_t i;
    uint8_t len;
    
    len = strlen(cmdline);

    if (len == 0) {
        printf("Current mixer: %s\r\n", mixerNames[cfg.mixerConfiguration - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        printf("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            printf("%s ", mixerNames[i]);
        }
        printf("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            printf("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            cfg.mixerConfiguration = i + 1;
            printf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliSave(char *cmdline)
{
	printf("Saving...");
    writeFlashConfig(0);
    printf("\r\nRebooting...");
    printf("\r\nYou have to close the terminal for 3 seconds.");
    vTaskDelay(3000 / portTICK_RATE_MS);	// Delay on 3 seconds
    systemReset(false);
}

static void cliPrintVar(const param_value_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    switch (var->type) {
        case VAR_UINT8:
            value = *(uint8_t *)var->ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)var->ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)var->ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)var->ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)var->ptr;
            break;

        case VAR_FLOAT:
            printf("%s", ftoa(*(float *)var->ptr, buf));
            if (full) {
                printf(" %s", ftoa((float)var->min, buf));
                printf(" %s", ftoa((float)var->max, buf));
            }
            return; // return from case for float only
    }
    printf("%d", value);
    if (full)
        printf(" %d %d", var->min, var->max);
}

static void cliSetVar(const param_value_t *var, const int32_t value)
{
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = *(float *)&value;
            break;
    }
}

static void cliSensor(char *cmdline)
{
	char buf[12];
	uint8_t len;

    len = strlen(cmdline);

    if (len == 1 && cmdline[0] == 'g')
	{
		flagSet(FLAG_CALIBRATE_GYRO);
		printf("\r\nCalibrate gyros\r\n");
	}
    else if (len == 1 && cmdline[0] == 'a')
	{
		flagSet(FLAG_CALIBRATE_ACC);
		printf("\r\nCalibrate accel\r\n");
	}
    else if (len == 1 && cmdline[0] == 'b')
	{
		flagSet(FLAG_CALIBRATE_BARO);
		printf("\r\nCalibrate baro\r\n");
	}

	printf("Show sensor real data:\r\n");
	printf("Sensors cycles    = %u\r\n", counters.sensorCycleCount);
	printf("Gyro count        = %u\r\n", gyro_sensor.sample_count);
	printf("Gyro overruns     = %u\r\n", gyro_sensor.overrun_count);
	printf("Acc count         = %u\r\n", acc_sensor.sample_count);
	printf("Acc overruns      = %u\r\n\r\n", acc_sensor.overrun_count);

	printf("Gyro Temp         = %d.%d\r\n", gyro_sensor.temp / 10, abs(gyro_sensor.temp) % 10);

	ftoa(gyro_sensor.data_deg[0], buf);
	printf("Gyro Data         = %s", buf);
	ftoa(gyro_sensor.data_deg[1], buf);
	printf(" %s", buf);
	ftoa(gyro_sensor.data_deg[2], buf);
	printf(" %s\r\n", buf);

	ftoa(gyro_sensor.average[0], buf);
	printf("Gyro average      = %s", buf);
	ftoa(gyro_sensor.average[1], buf);
	printf(" %s", buf);
	ftoa(gyro_sensor.average[2], buf);
	printf(" %s\r\n", buf);

	ftoa(cfg.gyro_bias[0], buf);
	printf("Gyro zero         = %s", buf);
	ftoa(cfg.gyro_bias[1], buf);
	printf(" %s", buf);
	ftoa(cfg.gyro_bias[2], buf);
	printf(" %s\r\n", buf);

	ftoa(gyro_sensor.variance[0], buf);
	printf("Gyro variance     = %s", buf);
	ftoa(gyro_sensor.variance[1], buf);
	printf(" %s", buf);
	ftoa(gyro_sensor.variance[2], buf);
	printf(" %s\r\n\r\n", buf);

	printf("Acc Temp          = %d.%d\r\n", acc_sensor.temp / 10, abs(acc_sensor.temp) % 10);

	ftoa(acc_sensor.data_mss[0], buf);
	printf("Acc Data          = %s", buf);
	ftoa(acc_sensor.data_mss[1], buf);
	printf(" %s", buf);
	ftoa(acc_sensor.data_mss[2], buf);
	printf(" %s\r\n", buf);

	ftoa(acc_sensor.average[0], buf);
	printf("Acc average       = %s", buf);
	ftoa(acc_sensor.average[1], buf);
	printf(" %s", buf);
	ftoa(acc_sensor.average[2], buf);
	printf(" %s\r\n", buf);

	ftoa(cfg.acc_bias[0], buf);
	printf("Acc zero          = %s", buf);
	ftoa(cfg.acc_bias[1], buf);
	printf(" %s", buf);
	ftoa(cfg.acc_bias[2], buf);
	printf(" %s\r\n", buf);

	ftoa(acc_sensor.variance[0], buf);
	printf("Acc variance      = %s", buf);
	ftoa(acc_sensor.variance[1], buf);
	printf(" %s", buf);
	ftoa(acc_sensor.variance[2], buf);
	printf(" %s\r\n", buf);

	ftoa(cfg.acc_magnitude, buf);
	printf("1G magnitude      = %s\r\n\r\n", buf);

	printf("Mag data          = %d %d %d\r\n\r\n", mag_sensor_data[ROLL], mag_sensor_data[PITCH], mag_sensor_data[YAW]);

	printf("Baro Temp         = %d.%d\r\n", alt_sensor.baroTemp / 10, abs(alt_sensor.baroTemp) % 10);
	printf("Baro alt (cm)     = %d\r\n", alt_sensor.baroAlt);
	printf("Baro alt ground   = %d\r\n", alt_sensor.baroAltGround);
	printf("Sonar alt (cm)    = %d\r\n", alt_sensor.sonarAlt);
	printf("GPS altitude      = %d\r\n", gps.altitude);
	printf("Altitude          = %d\r\n", EstAlt);

	if (len == 0)
	{
		printf("\r\nUse:\r\nsensor g - calibrate gyros\r\n");
		printf("sensor a - calibrate accel\r\n");
	}
	// TODO: Sensor
}


static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const param_value_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        printf("Current settings: \r\n");
        for (i = 0; i < valueTableCount; i++) {
            val = &valueTable[i];
            printf("%14s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            printf("\r\n");
        }
    } else if ((eqptr = strstr(cmdline, "="))) {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = _atof(eqptr);
        for (i = 0; i < valueTableCount; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    printf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    printf("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        printf("ERR: Unknown variable name\r\n");
    }
}

static void cliStatus(char *cmdline)
{
    uint8_t i;
    uint32_t mask;
	char buf[12];

    printf("System Uptime:     %d seconds\r\n", millis() / 1000);

	ftoa(power_sensor.flightBatteryVoltage / 10.0f, buf);
    printf("Voltage:          %s V (%dS battery)\r\n", buf, power_sensor.flightBatteryCellCount);
	printf("Current:           %d mA\r\n", power_sensor.flightBatteryCurrent);
	printf("Consumed:          %u mAh\r\n", power_sensor.flightBatteryConsumed);
	ftoa(power_sensor.videoBatteryVoltage / 10.0f, buf);
	printf("Video Batt:       %s V\r\n", buf);
    printf("CPU clock:         %d MHz\r\n", (SystemCoreClock / 1000000));
    printf("Cycle time:        %d us\r\n", counters.cycleTime);
    printf("Sensor read time:  %d us\r\n", counters.sensorReadTime);
    printf("I2C Errors:        %d\r\n", i2cGetErrorCounter());
    printf("Watchdog events:   %d\r\n", cfg.wdg_counter);

	ftoa(cpuTemp() / 10.0f, buf);
	printf("CPU Temp:         %s deg C\r\n", buf);

	uint16_t sz, ds0, ds1, ds2;

	sz = *(__IO uint16_t*)(0x1FFFF7E0);
	printf("Flash memory size: %d kB\r\n", sz);

	ds0 = *(__IO uint32_t*)(0x1FFFF7E8);
	ds1 = *(__IO uint32_t*)(0x1FFFF7EC);
	ds2 = *(__IO uint32_t*)(0x1FFFF7F0);
	printf("MCU unique ID:     %08lX%08lX%08lX\r\n", ds0, ds1, ds2);

	printf("Config size:       %d Byte\r\n", sizeof(config_t));
	printf("Config version:    %u\r\n", cfg.version);

	printf("Detected sensors:  ");
    mask = sensorsMask();
    for (i = 0; ; i++)
    {
        if (sensorNames[i] == NULL) break;
        if (mask & (1 << i)) printf("%s ", sensorNames[i]);
    }
    printf("\r\n");
}

static void cliTele(char *cmdline)
{
#if 0
	uint8_t c, p = 0;

    while (!vcp1HasData())
    {
    	if (uartAvailable(SERIAL_UART1))
		{
			c = uartRead(SERIAL_UART1);
			if (p == 0x7E && c == 0x7E)
			{
				printf("\r\n");
			}
			printf("%02X ", c);
			p = c;
		}
    }
#endif
}

static void cliBeep(char *cmdline)
{
    buzzerPlay(cmdline);
}

static void cliTest(char *cmdline)
{
//	char buf[12];
//	float Temp;
    char *eqptr = NULL;
    int32_t value = 0;

	printf("Test command:\r\n");

    if ((eqptr = strstr(cmdline, "="))) {
		// has equal, set var
		eqptr++;
		value = atoi(eqptr);
		buzzerFreq(value);
	}
	else if (strstr(cmdline, "+"))
		buzzerOn();
	else if (strstr(cmdline, "-"))
		buzzerOff();
	else if (strstr(cmdline, "a"))
		signalBUZZ(BUZZMODE_1);
	else if (strstr(cmdline, "s"))
		signalBUZZ(BUZZMODE_2);
	else if (strstr(cmdline, "q"))
		signalLED(LED_NAV, LEDMODE_BLINK1);
	else
	{
		printf("\r\nOSD\r\n");
		printf("maxScanLine       = %u\r\n", osdData.maxScanLine);
		printf("osdCicleTimer     = %u\r\n", osdCicleTime);

		printf("\r\nRSSI              = %u\r\n", rssi);
	}
	// TODO: Diagnostics
}

void cliOs(char *cmdline)
{
	char buf[256];

	vTaskList((int8_t *) buf);
	printf("Task\t\tState\tPrio\tStack\tNum\r\n%s", buf);

	vTaskGetRunTimeStats((int8_t *) buf);
	printf("\r\nTask\t\tAbs time\t%% Time\r\n%s", buf);
}

void cliProcess(void)
{
    if (!cliMode)
    {
        cliMode = 1;
        printf("Armazila UAV Flight Controller v" SW_VERSION " ("  __DATE__ "/" __TIME__ ")\r\n");
        printf("Entering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }

    while (da())
    {
        uint8_t c = rx();
        if (c == KEY_HT || c == '?')
        {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++)
            {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0)) continue;
                if (!pstart) pstart = cmd;
                pend = cmd;
            }
            if (pstart)    /* Buffer matches one or more commands */
            {
            	for (; ; bufferIndex++)
            	{
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex]) break;
                    if (!pstart->name[bufferIndex])
                    {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend)
            {
                /* Print list of ambiguous matches */
                printf("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++)
                {
                    printf(cmd->name);
                    printf("\t");
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++) printf("%c", cliBuffer[i]);
        } else if (!bufferIndex && c == 4)
        {
            cliExit(cliBuffer);
            return;
        }
        else if (c == 12)
        {
            // clear screen
            printf("\033[2J\033[1;1H");
            cliPrompt();
        }
        else if (bufferIndex && (c == KEY_LF || c == KEY_CR))
        {
            // enter pressed
            clicmd_t *cmd = NULL;
            clicmd_t target;
            printf("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            target.name = cliBuffer;
            target.param = NULL;
            
            cmd = bsearch(&target, cmdTable, CMD_COUNT, sizeof cmdTable[0], cliCompare);
            if (cmd) cmd->func(cliBuffer + strlen(cmd->name) + 1);
            else printf("ERR: Unknown command, try 'help'");

            memset(cliBuffer, 0, sizeof(cliBuffer));
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode) return;
            cliPrompt();
        }
        else if (c == KEY_DEL || c == KEY_BS)
        {
            // backspace
            if (bufferIndex)
            {
                cliBuffer[--bufferIndex] = 0;
                printf("\010 \010");
            }
		}
		else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126)
		{
			if (!bufferIndex && c == 32) continue;
			cliBuffer[bufferIndex++] = c;
			printf("%c", c);
		}
    }
}

static void vcp1Write(void *p, char c)
{
	vcp1SendByte(c);
}

static void uart1Write(void *p, char data)
{
	mspWrite(data);
}

void cliInit(void)
{
	if (cfg.port_map == PORT_MAP_UART1xMSP_VCP1xSIM_VCP2xMAVLINK)
	{
		/* redirect cli to UART1 */
		da = mspHasData;
		rx = mspRead;

		/* redirect printf to UART1 */
		init_printf(NULL, uart1Write);
	}
	else
	{
		/* redirect cli to USB VCP1 */
		da = vcp1HasData;
		rx = vcp1GetByte;

		/* redirect printf to USB VCP1 */
		init_printf(NULL, vcp1Write);
	}
}

