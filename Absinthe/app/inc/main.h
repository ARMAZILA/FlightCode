#ifndef __MAIN_H
#define __MAIN_H

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"
#include "printf.h"

/* Kernel FREERTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Board hardware definitions */
#include "board-110-10-04-02.h"

#include "adc.h"
#include "hcsr04.h"
#include "i2c.h"
#include "l3gd20.h"
#include "ledring.h"
#include "lps331ap.h"
#include "lsm303dlhc.h"
#include "pwm.h"
#include "rtc.h"
#include "spi.h"
#include "system.h"         // timers, delays, etc
#include "uart.h"
#include "wdg.h"
#include "osd_core.h"
#include "sbus.h"

#define MW_VERSION  			211
#define SW_VERSION				"1.3"

#define LAT  					0
#define LON  					1

#define BARO_TAB_SIZE_MAX   	48

#define MAGIC_NUMBER		(0xCA6B)	//	Magic word to access bootloader from main programm

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

// Serial GPS only variables
// navigation mode
typedef enum NavigationMode
{
    NAV_MODE_NONE = 0,
    NAV_MODE_POSHOLD,
    NAV_MODE_WP
} NavigationMode;

// Syncronized with GUI. Only exception is mixer > 11, which is always returned as 11 during serialization.
typedef enum MultiType
{
    MULTITYPE_TRI 			= 1,
    MULTITYPE_QUADP 		= 2,
    MULTITYPE_QUADX 		= 3,
    MULTITYPE_BI 			= 4,
    MULTITYPE_GIMBAL 		= 5,
    MULTITYPE_Y6 			= 6,
    MULTITYPE_HEX6 			= 7,
    MULTITYPE_FLYING_WING 	= 8,
    MULTITYPE_Y4 			= 9,
    MULTITYPE_HEX6X 		= 10,
    MULTITYPE_OCTOX8 		= 11,       // Java GUI is same for the next 3 configs
    MULTITYPE_OCTOFLATP 	= 12,       // MultiWinGui shows this differently
    MULTITYPE_OCTOFLATX 	= 13,       // MultiWinGui shows this differently
    MULTITYPE_AIRPLANE 		= 14,       // airplane / singlecopter / dualcopter (not yet properly supported)
    MULTITYPE_HELI_120_CCPM = 15,
    MULTITYPE_HELI_90_DEG 	= 16,
    MULTITYPE_VTAIL4 		= 17,
    MULTITYPE_CUSTOM 		= 18,       // no current GUI displays this
    MULTITYPE_LAST 			= 19
} MultiType;

typedef enum GimbalFlags {
    GIMBAL_NORMAL 		= 1 << 0,
    GIMBAL_TILTONLY 	= 1 << 1,
    GIMBAL_DISABLEAUX34 = 1 << 2,
    GIMBAL_FORWARDAUX 	= 1 << 3,
    GIMBAL_MIXTILT 		= 1 << 4,
} GimbalFlags;

/* Available sensors */
typedef enum {
    SENSOR_ACC   = 1 << 0,
    SENSOR_BARO  = 1 << 1,
    SENSOR_MAG   = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS   = 1 << 4,
} AvailableSensors;

// RC protocol
typedef enum {
    RC_PWM = 0,
    RC_PPM,
    RC_SPEKTRUM,
    RC_SBUS,
} RCprotocols;

typedef enum {
    GPS_NMEA  = 0,
    GPS_UBLOX = 1,
    GPS_MTK   = 2,
} GPSHardware;

// Serial ports map
typedef enum {
    PORT_MAP_UART1xMSP_VCP1xSIM_VCP2xMAVLINK = 0,	// UART1 -> MSP, 	 VCP1 -> SIM, VCP2 -> MAVLINK
    PORT_MAP_UART1xMAVLINK_VCP1xMSP_VCP2xSYM = 1,	// UART1 -> MAVLINK, VCP1 -> MSP, VCP2 -> SYM
    PORT_MAP_UART1xSIM_VCP1xMSP_VCP2xMAVLINK = 2,	// UART1 -> SIM, 	 VCP1 -> MSP, VCP2 -> MAVLINK
} PORT_MAP_t;

typedef struct gpio_config_t
{
    GPIO_TypeDef 	 *gpio;
    uint16_t 		 pin;
    GPIOMode_TypeDef mode;
} gpio_config_t;

typedef enum LEDMODE_t {
    LEDMODE_OFF			= 0,	// Led switch off
    LEDMODE_ON 			= 1,	// Led switch on
    LEDMODE_TOGGLE		= 2,	// Led toggle
    LEDMODE_BLINK1 		= 3,	// Led blink, 100ms on, 800 ms off
    LEDMODE_BLINK2 		= 4,
} LEDMODE_t;

typedef enum LEDNUM_t {
    LED_STS			= 0,
    LED_ERR			= 1,
    LED_NAV 		= 2,
} LEDNUM_t;

typedef enum BUZZMODE_t {
    BUZZMODE_OFF	= 0,	// Switch off buzzer
    BUZZMODE_ON 	= 1,	// Switch on
    BUZZMODE_1 		= 2,	// Buzzer sequence 1
    BUZZMODE_2 		= 3,
} BUZZMODE_t;

typedef int16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// #define LEDRING

#undef SOFT_I2C                 // enable to test software i2c

/* RC alias */
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

/* PIDs */
enum {
    PIDROLL = 0,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PIDITEMS
};

/* Boxes */
enum {
    BOXANGLE = 0,
    BOXHORIZON,
    BOXALTHOLD,
    BOXMAG,
    BOXCAMSTAB,
    BOXCAMTRIG,
    BOXARM,
    BOXGPSHOME,
    BOXGPSHOLD,
    BOXPASSTHRU,
    BOXHEADFREE,
    BOXBEEPERON,
    BOXLEDMAX,
    BOXLLIGHTS,
    BOXHEADADJ,
    CHECKBOXITEMS
};

/* Param types. Should correspond to mavlink_message_type_t */
typedef enum {
    VAR_UINT8	= 1,
    VAR_INT8	= 2,
    VAR_UINT16	= 3,
    VAR_INT16	= 4,
    VAR_UINT32	= 5,
    VAR_FLOAT	= 9
} vartype_e;

typedef enum {
	ML_TELEMETRY_SERVO_OUTPUT_RAW	= 1 << 0,
	ML_TELEMETRY_SCALED_IMU			= 1 << 1,
	ML_TELEMETRY_RC_CHANNELS_RAW	= 1 << 2,
	ML_TELEMETRY_NAMED_VALUE		= 1 << 3,
	ML_TELEMETRY_HUD				= 1 << 4,
	ML_TELEMETRY_GPS				= 1 << 5,
} ml_telemetry_flag_t;

/* Param table */
typedef struct param_value_t {
    const char 		*name;
    const uint8_t 	type; // vartype_e
    void 			*ptr;
    const int32_t 	min;
    const int32_t 	max;
} param_value_t;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define RAD2DEG(rad)    ((rad) * (180.0f / M_PI))
#define DEG2RAD(deg)    ((deg) * (M_PI / 180.0f))

typedef struct motorMixer_t {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

/* Config structure */
typedef struct config_t {
    uint8_t version;
    uint16_t size;
    uint8_t magic_be;                       // magic number, should be 0xBE
    uint8_t mixerConfiguration;

    uint8_t P8[PIDITEMS];
    uint8_t I8[PIDITEMS];
    uint8_t D8[PIDITEMS];

    uint8_t rcRate8;
    uint8_t rcExpo8;
    uint8_t thrMid8;
    uint8_t thrExpo8;

    uint8_t rollPitchRate;
    uint8_t yawRate;
    uint8_t dynThrPID;

    // sensor-related stuff
    float	 	acc_bias[3];
    float	 	acc_scale[3]; 				// Accel FS=8g, Sensitivity = 4 mg/lsb
    float	 	acc_temp_comp[3];			// Accel temperature compensation factor
    float	 	acc_magnitude;
    uint8_t  	acc_lpf_factor;             // Set the Low Pass Filter factor for ACC. Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter
    uint8_t  	acc_lpf_for_velocity;       // ACC lowpass for AccZ height hold
    uint8_t  	accz_deadband;              // ??
    uint8_t 	acc_inflight_cal;			// Enable inflight accel calibration feature

    float	 	gyro_bias[3];
    float	 	gyro_scale[3]; 				// Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb
    float	 	gyro_temp_comp[3];			// Gyro temperature compensation factor
    uint16_t 	gyro_cmpf_factor;           // Set the Gyro Weight for Gyro/Acc complementary filter. Increasing this value would reduce and delay Acc influence on the output of the filter.
    uint8_t  	gyro_smooth[3];			    // How much to smoothen with per axis
    uint8_t  	gyro_smoothing;			    // Enable ssmoothing

    int16_t  	magZero[3];
    int16_t  	mag_declination;            // Get your magnetic decliniation from here : http://magnetic-declination.com/

    uint8_t  	baro_tab_size;              // size of baro filter array
    float 	 	baro_noise_lpf;             // additional LPF to reduce baro noise
    float 	 	baro_cf;                    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
    float	 	baro_cf_alt;
    float	 	baro_cf_vel;

    int16_t  	angleTrim[2];               // accelerometer trim
    uint8_t  	imu_algorithm;				// select imu calculation algoritm
    uint8_t		hil_mode;					// Hardware-in-the-loop simulation mode
    										// 0 - HIL off
    										// 1 - sensor off
    										// 2 - sensor & IMU off

    uint16_t 	activate[CHECKBOXITEMS];    // activate switches
    uint8_t  	batvscale;                  // adjust this to match battery voltage to reported value
    uint8_t  	batmaxcellvoltage;          // maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
    uint8_t  	batmincellvoltage;          // minimum voltage per cell, this triggers battery out alarms, in 0.1V units, default is 33 (3.3V)
    uint8_t  	batiscale;                  // adjust this to match battery current to reported value
    int16_t  	batioffset;                 // adjust this to match battery current offset to reported value
    uint8_t	 	batalarm;					// Enable/disable low battery warning sound
	uint16_t 	flightBatteryCapacity; 		// flight battery capacity in 1mAh

    // Radio/ESC-related configuration
    uint8_t 	rcprotocol;					// Radio protocol mode PWM/PPM/SPEKTRUM/SBUS
    uint8_t 	rcmap[16];                  // mapping of radio channels to internal RPYTA+ order
    uint8_t		failsafe;					// Enable failsafe feature
    uint8_t deadband;                       // introduce a deadband around the stick center for pitch and roll axis. Must be greater than zero.
    uint8_t yawdeadband;                    // introduce a deadband around the stick center for yaw axis. Must be greater than zero.
    uint8_t alt_hold_throttle_neutral;      // defines the neutral zone of throttle stick during altitude hold, default setting is +/-20
    uint8_t spektrum_hires;                 // spektrum high-resolution y/n (1024/2048bit)
    uint16_t midrc;                         // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                      // minimum rc end
    uint16_t maxcheck;                      // maximum rc end
    uint8_t retarded_arm;                   // allow disarsm/arm on throttle down + roll left/right

    // Failsafe related configuration
    uint8_t failsafe_delay;                 // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example (10)
    uint8_t failsafe_off_delay;             // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example (200)
    uint16_t failsafe_throttle;             // Throttle level used for landing - specify value between 1000..2000 (pwm pulse width for slightly below hover). center throttle = 1500.

    // motor/esc/servo related stuff
    uint8_t		motor_stop;					// Enable motor stop feature
    uint16_t minthrottle;                   // Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
    uint16_t maxthrottle;                   // This is the maximum value for the ESCs at full power this value can be increased up to 2000
    uint16_t mincommand;                    // This is the value for the ESCs when they are not armed. In some cases, this value must be lowered down to 900 for some specific ESCs
    uint16_t pwm_group1_rate;               // The update rate of group1 outputs (50-498Hz)
    uint16_t pwm_group2_rate;               // The update rate of group2 outputs (50-498Hz)
    uint16_t pwm_group3_rate;               // The update rate of group3 outputs (50-498Hz)
    uint16_t pwm_group4_rate;               // The update rate of group4 outputs (50-498Hz)
    int16_t servotrim[8];                   // Adjust Servo MID Offset & Swash angles
    int8_t servoreverse[8];                 // Invert servos by setting -1

    // mixer-related configuration
    int8_t yaw_direction;
    uint16_t tri_yaw_middle;                // tail servo center pos. - use this for initial trim
    uint16_t tri_yaw_min;                   // tail servo min
    uint16_t tri_yaw_max;                   // tail servo max

    // flying wing related configuration
    uint16_t wing_left_min;                 // min/mid/max servo travel
    uint16_t wing_left_mid;
    uint16_t wing_left_max;
    uint16_t wing_right_min;
    uint16_t wing_right_mid;
    uint16_t wing_right_max;
    int8_t pitch_direction_l;               // left servo - pitch orientation
    int8_t pitch_direction_r;               // right servo - pitch orientation (opposite sign to pitch_direction_l if servos are mounted mirrored)
    int8_t roll_direction_l;                // left servo - roll orientation
    int8_t roll_direction_r;                // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)

    // gimbal-related configuration
    uint8_t		gimbal;						// Gimbal feature enable
    int8_t 		gimbal_pitch_gain;          // gimbal pitch servo gain (tied to angle) can be negative to invert movement
    int8_t 		gimbal_roll_gain;           // gimbal roll servo gain (tied to angle) can be negative to invert movement
    uint8_t 	gimbal_flags;               // in servotilt mode, various things that affect stuff
    uint16_t 	gimbal_pitch_min;           // gimbal pitch servo min travel
    uint16_t 	gimbal_pitch_max;           // gimbal pitch servo max travel
    uint16_t 	gimbal_pitch_mid;           // gimbal pitch servo neutral value
    uint16_t 	gimbal_roll_min;            // gimbal roll servo min travel
    uint16_t 	gimbal_roll_max;            // gimbal roll servo max travel
    uint16_t 	gimbal_roll_mid;            // gimbal roll servo neutral value

    // gps-related stuff
    uint8_t 	gps_type;                   // Type of GPS hardware. 0: NMEA 1: UBX 2+ ??
    uint32_t 	gps_baudrate;               // GPS baudrate
    uint16_t 	gps_wp_radius;              // if we are within this distance to a waypoint then we consider it reached (distance is in cm)
    uint8_t 	gps_lpf;                    // Low pass filter cut frequency for derivative calculation (default 20Hz)
    uint8_t 	nav_slew_rate;              // Adds a rate control to nav output, will smoothen out nav angle spikes
    uint8_t 	nav_controls_heading;       // copter faces toward the navigation point, maghold must be enabled for it
    uint16_t 	nav_speed_min;              // cm/sec
    uint16_t 	nav_speed_max;              // cm/sec

    uint8_t 	osd_screen;					// OSD screen number
    uint16_t 	wdg_counter;				// Watch-dog reboot event counter

    uint8_t 	port_map;					// Serial ports map: see PORT_MAP_t ENUM
    uint32_t 	uart1_baudrate;				// UART1 baudrate

    uint8_t		mavlink_sysid;				// MAVLink system ID
    uint8_t		mavlink_compid;				// MAVLink component ID
    uint32_t	mavlink_telemetry_flag;		//

    motorMixer_t customMixer[MAX_MOTORS];   // custom mixtable
    uint8_t magic_ef;                       // magic number, should be 0xEF
    uint8_t chk;                            // XOR checksum
} config_t;

/* Sensors raw data  */
typedef struct counter_t
{
	uint32_t	sensorCycleCount;				// Sensor task loop counter. Should be increased with 100 Hz freq.
	uint16_t 	cycleTime; 						// this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
	uint16_t	sensorReadTime;					// Time elapsed to read sensors data
} counters_t;

typedef struct gyro_sensor_t {
	uint32_t	sample_count;					// Gyro sensor loop counter. Should be increased with 760 Hz freq (Gyro ODR).
	uint32_t	overrun_count;					// Gyro sensor data not abalible counter. Should be 0.
	int16_t		temp;							// Gyro sensor temperature
	float		data_deg[3];					// Gyro sensor data. Scaled to degrees / second
	float		data_rad[3];					// Gyro sensor data. Scaled to radian / second
	float		average[3];						// Average value (degrees / second)
	float		variance[3];					// Variance (degrees / second)
} gyro_sensor_t;

typedef struct acc_sensor_t {
	uint32_t	sample_count;					// Accel sensor loop counter. Should be increased with 760 Hz freq (Gyro ODR).
	uint32_t	overrun_count;					// Accel sensor data not abalible counter. Should be 0.
	int16_t		temp;							// Accel sensor temperature
	float		data_mss[3];					// Accel sensor data scaled to the m/s2
	float		data_mw[3];						// Accel sensor data scaled to MultiWii
	float		average[3];						// Average value (degrees / second)
	float		variance[3];					// Variance (degrees / second)
} acc_sensor_t;

typedef struct gps_sensor_t {
	int32_t 	coord[2];
	int32_t 	home[2];
	int32_t 	hold[2];
	uint8_t 	numSat;
	uint16_t 	distanceToHome; 				// distance to home point in meters
	int16_t 	directionToHome; 				// direction to home or hol point in degrees 0..360
	uint16_t 	altitude;						// altitude in cm
	uint16_t 	speed; 							// speed in 0.1m/s
	uint8_t 	update; 						// it's a binary toogle to distinct a GPS position update
	int16_t 	angle[2]; 						// it's the angles that must be applied for GPS correction
	uint16_t 	ground_course; 					// degrees * 10
	uint16_t 	hdop;
	uint32_t 	bytes_rx;
	uint32_t 	frames_rx;
} gps_sensor_t;

typedef struct power_sensor_t {
	uint8_t 	flightBatteryVoltage;  			// flight battery voltage in 0.1V steps
	int16_t 	flightBatteryCurrent; 			// flight battery current in 10mA steps. + discharge / - charge
	uint16_t 	flightBatteryConsumed; 			// flight battery consumed energy in 1mAh steps
	uint16_t 	flightBatteryWarningVoltage;
	uint8_t 	flightBatteryCellCount;
	uint8_t		videoBatteryVoltage;  			// video battery voltage in 0.1V steps
} power_sensor_t;

typedef struct alt_sensor_t {
	float		baroPressure_mb;				// Temperature-compensated Pressure measurement in mb
	int32_t 	baroAlt;  						// in cm. Altitude from barometr
	int32_t 	baroAltGround;					// in cm.
	int16_t 	baroTemp;						// baro sensor temperature in 0.1 degrees
	int16_t 	sonarAlt;						// in cm. < 0: bad data
} alt_sensor_t;

typedef struct imu {
	int16_t	rpy[3];			// Absolute angle inclination in multiple of 0.1 degree. ROLL: -1800...1800, PITCH -900..900, YAW -180...+180 degrees
	float	rpy_rad[3];		// The same in radiant

} imu_t;

/* Available Flags */
typedef enum {
	FLAG_OK_TO_ARM 			= 1 << 0,	//
	FLAG_ARMED 				= 1 << 1,	// Autopilot is armed. In copter mode motors starts
	FLAG_ACC_CALIBRATED 	= 1 << 2,	// Accelerometr is calibrated
	FLAG_MAG_CALIBRATED 	= 1 << 3,	// Magnitometr is calibrated
	FLAG_GYRO_CALIBRATED	= 1 << 4,	// Gyroscope is calibrated
    FLAG_CALIBRATE_ACC		= 1 << 5,	// Request to calibrate accelerometr
    FLAG_CALIBRATE_MAG		= 1 << 6,	// Requrst to calibrate magnitometr
    FLAG_CALIBRATE_GYRO		= 1 << 7,	// Request to calibrate gyro
    FLAG_CALIBRATE_BARO		= 1 << 8,	// Requrst to calibrate altimeter
	FLAG_BARO_CALIBRATED	= 1 << 9,	// Altimeter is calibrated
	FLAG_ANGLE_MODE			= 1 << 10,
	FLAG_HORIZON_MODE		= 1 << 11,
	FLAG_MAG_MODE			= 1 << 12,
	FLAG_ALTHOLD_MODE		= 1 << 13,
	FLAG_GPSHOME_MODE		= 1 << 14,
	FLAG_GPSHOLD_MODE		= 1 << 15,
	FLAG_HEADFREE_MODE		= 1 << 16,
	FLAG_PASSTHRU_MODE		= 1 << 17,	// Direct passthru from RX
	FLAG_GPS_FIX			= 1 << 18,
	FLAG_GPS_FIX_HOME		= 1 << 19,
    FLAG_SMALL_ANGLES_25	= 1 << 29,
	FLAG_WDG_OCCURRED		= 1 << 30,	// System restarts by watchdog
} AvailableFlags_t;

typedef void (* serialSendByte_t)(uint8_t data);
typedef uint8_t (* serialReadByte_t)(void);
typedef uint16_t (* serialHasData_t)(void);

extern int16_t	axisPID[3];
extern int16_t 	BaroPID;
extern int16_t	rcCommand[4];
extern uint8_t	rcOptions[CHECKBOXITEMS];
extern int16_t	failsafeCnt;
extern uint8_t	rssi;					// 0..255 -> 0%..100%
extern int16_t 	debug[4];

extern int16_t 	headFreeModeHold;
extern int16_t 	magHold;
extern int16_t 	motor[MAX_MOTORS];
extern int16_t 	servo[MAX_SERVOS];
extern int16_t 	rcData[8];
extern int16_t 	lookupPitchRollRC[6];   // lookup table for expo & RC rate PITCH+ROLL
extern int16_t 	lookupThrottleRC[11];   // lookup table for expo & mid THROTTLE

extern int16_t  nav[2];
extern int8_t   nav_mode;				// Navigation mode
extern int16_t  nav_rated[2];			// Adding a rate controller to the navigation to make it smoother

// from osd_core.c
extern osdData_t osdData;
extern int16_t osdCicleTime;
extern uint16_t osd_x;
extern uint16_t osd_y;
extern uint16_t osd_h;
extern int16_t _GPS_directionToHome;
extern uint16_t _GPS_distanceToHome;

// control
extern rcReadRawDataPtr rcReadRawFunc; // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)
int16_t rxReadRawRC(uint8_t chan);
portTASK_FUNCTION_PROTO(rcLoopTask, pvParameters);
void stabilize(float dT);

// IMU
extern float	gyroData[3];
extern float	accSmooth[3];
extern imu_t	imu;
extern int32_t  vario;
extern int32_t 	EstAlt;

void imuInit(void);
void computeIMU(void);
void setAltHold(int32_t newAltitude);
void getEstimatedAltitude(void);
void imuAHRSupdate();

// Sensors
extern counters_t 		counters;
extern acc_sensor_t		acc_sensor;
extern gyro_sensor_t	gyro_sensor;
extern int16_t 			mag_sensor_data[3];
extern power_sensor_t	power_sensor;
extern alt_sensor_t		alt_sensor;
extern uint16_t 		acc_1G;
void sensorInit(void);
int16_t cpuTemp(void);
portTASK_FUNCTION_PROTO(sensorTask, pvParameters);
portTASK_FUNCTION_PROTO(powerSensorTask, pvParameters);
portTASK_FUNCTION_PROTO(sonarTask, pvParameters);

// Mixer
void mixerInit(void);
void mixerLoadMix(int index);
void mixerWwriteAllMotors(int16_t mc);
void mixerTable(void);
void mixerWrite();

// Serial
void serialInit(uint32_t baudrate);
void serialCom(void);
portTASK_FUNCTION_PROTO(mspTask, pvParameters);

// Config
extern config_t cfg;
extern const param_value_t valueTable[];
extern uint8_t valueTableCount;

void parseRcChannels(const char *input);
void readFlashConfig(void);
void writeFlashConfig(uint8_t b);
void checkFirstTime(bool reset);
bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);
bool flag(AvailableFlags_t mask);
void flagSet(AvailableFlags_t mask);
void flagClear(AvailableFlags_t mask);

// spektrum
void spektrumInit(void);
bool spektrumFrameComplete(void);
int16_t spektrumReadRawRC(uint8_t chan);

// signal
void buzzerOn(void);
void buzzerOff(void);
void buzzerFreq(uint16_t freq);
//void buzzer(uint8_t warn_vbat);
void signalLED(LEDNUM_t led_num, LEDMODE_t led_mode);
void signalBUZZ(BUZZMODE_t buzz_mode);
void buzzerPlay(char * pattern);
portTASK_FUNCTION_PROTO(signalTask, pvParameters);

// cli
void cliProcess(void);
char *ftoa(float x, char *floatString);

// gps
extern gps_sensor_t gps;

void gpsInit(uint32_t baudrate);
void GPS_reset_home_position(void);
void GPS_reset_nav(void);
void GPS_set_next_wp(int32_t* lat, int32_t* lon);
int32_t wrap_18000(int32_t error);
void GPS_set_pids(void);
void gps_navigate(void);

portTASK_FUNCTION_PROTO(navigateTask, pvParameters);

// FrSky telemetry
void initTelemetry(bool State);
void sendTelemetry(void);
portTASK_FUNCTION_PROTO(frskyTask, pvParameters);

// osd_gen
portTASK_FUNCTION_PROTO(osdTask, pvParameters);

// hw_config
uint16_t vcp1HasData(void);
uint8_t vcp1GetByte(void);
void vcp1SendByte(uint8_t data);
bool vcp1TransmitEmpty(void);

uint8_t vcpGetByte(uint8_t chan);
uint16_t vcpHasData(uint8_t chan);
void vcpSendByte(uint8_t chan, uint8_t data);
bool vcpTransmitEmpty(uint8_t chan);

// mavlink
portTASK_FUNCTION_PROTO(mavlinkTask, pvParameters);

// Simulator
portTASK_FUNCTION_PROTO(simTask, pvParameters);

#endif
