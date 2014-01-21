
#include "main.h"
#include <string.h>

#define COMPILE_TIME_ASSERT(pred) switch(0) { case 0:case pred:; }
#define ASSERT_SIZE_EXCEEDED(type, size) COMPILE_TIME_ASSERT(sizeof(type) <= size)

const param_value_t valueTable[] = {
    { "MAVLINK_SYSID",	VAR_UINT8,  &cfg.mavlink_sysid, 	0, 255 },
    { "MAVLINK_COMPID",	VAR_UINT8,  &cfg.mavlink_compid, 	0, 255 },
    { "MAVLINK_TLM",	VAR_UINT32, &cfg.mavlink_telemetry_flag, 0, 0xFFFFFFFF },

    { "BOX_ANGLE",		VAR_UINT16, &cfg.activate[BOXANGLE], 0, 0xFFFF },
    { "BOX_HORIZON",	VAR_UINT16, &cfg.activate[BOXHORIZON], 0, 0xFFFF },
    { "BOX_ALTHOLD",	VAR_UINT16, &cfg.activate[BOXALTHOLD], 0, 0xFFFF },
    { "BOX_MAG",		VAR_UINT16, &cfg.activate[BOXMAG], 0, 0xFFFF },
    { "BOX_GPSHOME",	VAR_UINT16, &cfg.activate[BOXGPSHOME], 0, 0xFFFF },
    { "BOX_GPSHOLD",	VAR_UINT16, &cfg.activate[BOXGPSHOLD], 0, 0xFFFF },

    { "RCMAP_ROLL",		VAR_UINT8, 	&cfg.rcmap[0], 1, 18 },
    { "RCMAP_PITCH",	VAR_UINT8, 	&cfg.rcmap[1], 1, 18 },
    { "RCMAP_YAW",		VAR_UINT8, 	&cfg.rcmap[2], 1, 18 },
    { "RCMAP_THRO",		VAR_UINT8, 	&cfg.rcmap[3], 1, 18 },
    { "RCMAP_AUX1",		VAR_UINT8, 	&cfg.rcmap[4], 1, 18 },
    { "RCMAP_AUX2",		VAR_UINT8, 	&cfg.rcmap[5], 1, 18 },
    { "RCMAP_AUX3",		VAR_UINT8, 	&cfg.rcmap[6], 1, 18 },
    { "RCMAP_AUX4",		VAR_UINT8, 	&cfg.rcmap[7], 1, 18 },
    { "RCMAP_AUX5",		VAR_UINT8, 	&cfg.rcmap[8], 1, 18 },
    { "RCMAP_AUX6",		VAR_UINT8, 	&cfg.rcmap[9], 1, 18 },
    { "RCMAP_AUX7",		VAR_UINT8, 	&cfg.rcmap[10], 1, 18 },
    { "RCMAP_AUX8",		VAR_UINT8, 	&cfg.rcmap[11], 1, 18 },
    { "RCMAP_AUX9",		VAR_UINT8, 	&cfg.rcmap[12], 1, 18 },
    { "RCMAP_AUX10",	VAR_UINT8, 	&cfg.rcmap[13], 1, 18 },
    { "RCMAP_AUX11",	VAR_UINT8, 	&cfg.rcmap[14], 1, 18 },
    { "RCMAP_AUX12",	VAR_UINT8, 	&cfg.rcmap[15], 1, 18 },

    { "RC_PROTOCOL",	VAR_UINT8, 	&cfg.rcprotocol, 0, 3 },
    { "RC_RetArm",		VAR_UINT8, 	&cfg.retarded_arm, 0, 1 },
    { "RC_DB",			VAR_UINT8, 	&cfg.deadband, 0, 32 },
    { "RC_DB_Y",		VAR_UINT8, 	&cfg.yawdeadband, 0, 100 },
    { "AltHoldThroN",	VAR_UINT8, 	&cfg.alt_hold_throttle_neutral, 1, 250 },
    { "RC_MID",			VAR_UINT16, &cfg.midrc, 1200, 1700 },
    { "RC_MINTHRO",		VAR_UINT16, &cfg.minthrottle, 0, 2000 },
    { "RC_MAXTHRO",		VAR_UINT16, &cfg.maxthrottle, 0, 2000 },
    { "RC_MINCMD",		VAR_UINT16, &cfg.mincommand, 0, 2000 },
    { "RC_MINCHK",		VAR_UINT16, &cfg.mincheck, 0, 2000 },
    { "RC_MAXCHK",		VAR_UINT16, &cfg.maxcheck, 0, 2000 },
    { "RC_MOTORSTOP", 	VAR_UINT8,  &cfg.motor_stop, 0, 1 },
    { "RC_FAILSAFE", 	VAR_UINT8,  &cfg.failsafe, 0, 1 },
    { "RC_FS_DELAY",	VAR_UINT8, 	&cfg.failsafe_delay, 0, 200 },
    { "RC_FS_OFf_DEL",	VAR_UINT8, 	&cfg.failsafe_off_delay, 0, 200 },
    { "RC_FS_THROTT", 	VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "RC_SPEK_HIRES",	VAR_UINT8, 	&cfg.spektrum_hires, 0, 1 },
    { "RC_RP_RATE",		VAR_UINT8, 	&cfg.rollPitchRate, 0, 255 },
    { "RC_Y_RATE",		VAR_UINT8, 	&cfg.yawRate, 0, 255 },
    { "RC_RATE",		VAR_UINT8, 	&cfg.rcRate8, 0, 255 },

    { "PWM_RATEGROUP1",	VAR_UINT16, &cfg.pwm_group1_rate, 50, 498 },
    { "PWM_RATEGROUP2",	VAR_UINT16, &cfg.pwm_group2_rate, 50, 498 },
    { "PWM_RATEGROUP3",	VAR_UINT16, &cfg.pwm_group3_rate, 50, 498 },
    { "PWM_RATEGROUP4",	VAR_UINT16, &cfg.pwm_group4_rate, 50, 498 },

    { "BAT_VSCALE",		VAR_UINT8, 	&cfg.batvscale, 10, 200 },
    { "BAT_MaxCellV",	VAR_UINT8, 	&cfg.batmaxcellvoltage, 10, 50 },
    { "BAT_MinCellV",	VAR_UINT8, 	&cfg.batmincellvoltage, 10, 50 },
    { "BAT_ISCALE",		VAR_UINT8, 	&cfg.batiscale, 10, 250 },
    { "BAT_IOFFSET",	VAR_INT16, 	&cfg.batioffset, -10000, 10000 },
    { "BAT_CAPACITY",	VAR_UINT16,	&cfg.flightBatteryCapacity, 1, 10000 },
    { "BAT_ALARM",		VAR_UINT8,	&cfg.batalarm, 0, 1 },

    { "YAWDIR",			VAR_INT8, 	&cfg.yaw_direction, -1, 1 },
    { "TRI_YAW_MIN", 	VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "TRI_YAW_MID", 	VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "TRI_YAW_MAX", 	VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },

    { "WING_LEFT_MIN",	VAR_UINT16, &cfg.wing_left_min, 0, 2000 },
    { "WING_LEFT_MID",	VAR_UINT16, &cfg.wing_left_mid, 0, 2000 },
    { "WING_LEFT_MAX",	VAR_UINT16, &cfg.wing_left_max, 0, 2000 },
    { "WING_RIGHT_MIN", VAR_UINT16, &cfg.wing_right_min, 0, 2000 },
    { "WING_RIGHT_MID", VAR_UINT16, &cfg.wing_right_mid, 0, 2000 },
    { "WING_RIGHT_MAX", VAR_UINT16, &cfg.wing_right_max, 0, 2000 },
    { "WING_DIR_P_L",	VAR_INT8, 	&cfg.pitch_direction_l, -1, 1 },
    { "WING_DIR_P_R",	VAR_INT8, 	&cfg.pitch_direction_r, -1, 1 },
    { "WING_DIR_R_L",	VAR_INT8, 	&cfg.roll_direction_l, -1, 1 },
    { "WING_DIR_R_R",	VAR_INT8, 	&cfg.roll_direction_r, -1, 1 },

    { "GIMBAL_ON",		VAR_UINT8,	&cfg.gimbal, 0, 1},
    { "GIMBAL_FLAG",	VAR_UINT8,	&cfg.gimbal_flags, 0, 255},
    { "GIMBAL_GAIN_P",	VAR_INT8, 	&cfg.gimbal_pitch_gain, -100, 100 },
    { "GIMBAL_GAIN_R",	VAR_INT8, 	&cfg.gimbal_roll_gain, -100, 100 },
    { "GIMBAL_MIN_P",	VAR_UINT16, &cfg.gimbal_pitch_min, 100, 3000 },
    { "GIMBAL_MAX_P",	VAR_UINT16, &cfg.gimbal_pitch_max, 100, 3000 },
    { "GIMBAL_MID_P",	VAR_UINT16, &cfg.gimbal_pitch_mid, 100, 3000 },
    { "GIMBAL_MIN_R",	VAR_UINT16, &cfg.gimbal_roll_min, 100, 3000 },
    { "GIMBAL_MAX_R",	VAR_UINT16, &cfg.gimbal_roll_max, 100, 3000 },
    { "GIMBAL_MID_R",	VAR_UINT16, &cfg.gimbal_roll_mid, 100, 3000 },

    { "MAG_DECLIN",		VAR_INT16, 	&cfg.mag_declination, -18000, 18000 },

    { "ACC_SCALE_R",	VAR_FLOAT, 	&cfg.acc_scale[ROLL ], 	   -1, 1 },
    { "ACC_SCALE_P",	VAR_FLOAT, 	&cfg.acc_scale[PITCH], 	   -1, 1 },
    { "ACC_SCALE_Y",	VAR_FLOAT, 	&cfg.acc_scale[YAW  ], 	   -1, 1 },
    { "ACC_TCOMP_R",	VAR_FLOAT, 	&cfg.acc_temp_comp[ROLL ], -1, 1 },
    { "ACC_TCOMP_P",	VAR_FLOAT, 	&cfg.acc_temp_comp[PITCH], -1, 1 },
    { "ACC_TCOMP_Y",	VAR_FLOAT, 	&cfg.acc_temp_comp[YAW  ], -1, 1 },
    { "ACC_LPF_FAC",	VAR_UINT8, 	&cfg.acc_lpf_factor, 	   0, 250 },
    { "ACC_LPF_VEL",	VAR_UINT8, 	&cfg.acc_lpf_for_velocity, 1, 250 },
    { "ACC_TRIM_R",		VAR_INT16, 	&cfg.angleTrim[ROLL], 	   -300, 300 },
    { "ACC_TRIM_P",		VAR_INT16, 	&cfg.angleTrim[PITCH], 	   -300, 300 },

    { "GYRO_SCALE_R",	VAR_FLOAT, 	&cfg.gyro_scale[ROLL ], -1, 1 },
    { "GYRO_SCALE_P",	VAR_FLOAT, 	&cfg.gyro_scale[PITCH], -1, 1 },
    { "GYRO_SCALE_Y",	VAR_FLOAT, 	&cfg.gyro_scale[YAW  ], -1, 1 },
    { "GYRO_TCOPM_R",	VAR_FLOAT, 	&cfg.gyro_temp_comp[ROLL ], -1, 1 },
    { "GYRO_TCOMP_P",	VAR_FLOAT, 	&cfg.gyro_temp_comp[PITCH], -1, 1 },
    { "GYRO_TCOMP_Y",	VAR_FLOAT, 	&cfg.gyro_temp_comp[YAW  ], -1, 1 },
    { "GYRO_CMPF_FAC",	VAR_UINT16, &cfg.gyro_cmpf_factor, 100, 1000 },
    { "GYRO_SMOOTHING",	VAR_UINT8,  &cfg.gyro_smoothing, 0, 1 },

    { "BARO_TAB_SIZE",	VAR_UINT8, 	&cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "BARO_NOISE_LPF", VAR_FLOAT, 	&cfg.baro_noise_lpf, 0, 1 },
    { "BARO_CF",		VAR_FLOAT, 	&cfg.baro_cf, 0, 1 },
    { "BARO_CF_ALT",	VAR_FLOAT, 	&cfg.baro_cf_alt, 0, 1 },
    { "BARO_CF_VEL",	VAR_FLOAT, 	&cfg.baro_cf_vel, 0, 1 },

    { "GPS_TYPE",		VAR_UINT8, 	&cfg.gps_type, 0, 3 },
    { "GPS_BAUDRATE",	VAR_UINT32,	&cfg.gps_baudrate, 1200, 115200 },
    { "GPS_POS_P",		VAR_UINT8, 	&cfg.P8[PIDPOS], 0, 200 },
    { "GPS_POS_I",		VAR_UINT8, 	&cfg.I8[PIDPOS], 0, 200 },
    { "GPS_POS_D",		VAR_UINT8, 	&cfg.D8[PIDPOS], 0, 200 },
    { "GPS_POSR_P",		VAR_UINT8, 	&cfg.P8[PIDPOSR], 0, 200 },
    { "GPS_POSR_I",		VAR_UINT8, 	&cfg.I8[PIDPOSR], 0, 200 },
    { "GPS_POSR_D",		VAR_UINT8, 	&cfg.D8[PIDPOSR], 0, 200 },
    { "GPS_NAV_P",		VAR_UINT8, 	&cfg.P8[PIDNAVR], 0, 200 },
    { "GPS_NAV_I",		VAR_UINT8, 	&cfg.I8[PIDNAVR], 0, 200 },
    { "GPS_NAV_D",		VAR_UINT8, 	&cfg.D8[PIDNAVR], 0, 200 },
    { "GPS_WP_RADIUS",	VAR_UINT16, &cfg.gps_wp_radius, 0, 2000 },
    { "GPS_BYTES_RX",	VAR_UINT32, &gps.bytes_rx, 0, 0 },
    { "GPS_FRAMES_RX",	VAR_UINT32, &gps.frames_rx, 0, 0 },

    { "NAV_CTRL_HEAD",	VAR_UINT8, 	&cfg.nav_controls_heading, 0, 1 },
    { "NAV_SPEED_MIN",	VAR_UINT16, &cfg.nav_speed_min, 10, 2000 },
    { "NAV_SPEED_MAX",	VAR_UINT16, &cfg.nav_speed_max, 10, 2000 },
    { "NAV_SLEW_RATE",	VAR_UINT8, 	&cfg.nav_slew_rate, 0, 100 },

    { "PID_R_P", 		VAR_UINT8, 	&cfg.P8[ROLL], 0, 200 },
    { "PID_R_I", 		VAR_UINT8, 	&cfg.I8[ROLL], 0, 200 },
    { "PID_R_D", 		VAR_UINT8, 	&cfg.D8[ROLL], 0, 200 },
    { "PID_P_P", 		VAR_UINT8, 	&cfg.P8[PITCH], 0, 200 },
    { "PID_P_I", 		VAR_UINT8, 	&cfg.I8[PITCH], 0, 200 },
    { "PID_P_D", 		VAR_UINT8, 	&cfg.D8[PITCH], 0, 200 },
    { "PID_Y_P", 		VAR_UINT8, 	&cfg.P8[YAW], 0, 200 },
    { "PID_Y_I", 		VAR_UINT8, 	&cfg.I8[YAW], 0, 200 },
    { "PID_Y_D", 		VAR_UINT8, 	&cfg.D8[YAW], 0, 200 },
    { "PID_ALT_P", 		VAR_UINT8, 	&cfg.P8[PIDALT], 0, 200 },
    { "PID_ALT_I", 		VAR_UINT8, 	&cfg.I8[PIDALT], 0, 200 },
    { "PID_ALT_I", 		VAR_UINT8, 	&cfg.D8[PIDALT], 0, 200 },
    { "PID_LEVEL_P", 	VAR_UINT8, 	&cfg.P8[PIDLEVEL], 0, 200 },
    { "PID_LEVEL_I", 	VAR_UINT8, 	&cfg.I8[PIDLEVEL], 0, 200 },
    { "PID_LEVEL_D", 	VAR_UINT8, 	&cfg.D8[PIDLEVEL], 0, 200 },

    { "OSD_SCREEN",		VAR_UINT8, 	&cfg.osd_screen, 0, 15 },
    { "OSD_X",			VAR_UINT16, &osd_x, 0, 300 },
    { "OSD_Y",			VAR_UINT16, &osd_y, 0, 300 },
    { "OSD_H",			VAR_UINT16, &osd_h, 0, 300 },
    { "OSD_DIR",		VAR_INT16, 	&_GPS_directionToHome, -400, 400 },
    { "OSD_DIS",		VAR_UINT16, &_GPS_distanceToHome, 0, 10000 },

    { "IMUALGORITHM",	VAR_UINT8, 	&cfg.imu_algorithm, 1, 4 },
    { "HIL_MODE",		VAR_UINT8, 	&cfg.hil_mode, 0, 2 },

    { "PORT_MAP",		VAR_UINT8,  &cfg.port_map, 0, 2 },
    { "UART1_BAUDRATE",	VAR_UINT32, &cfg.uart1_baudrate, 1200, 115200 },
};

uint8_t valueTableCount = (sizeof(valueTable) / sizeof(valueTable[0]));

config_t cfg;
const uint8_t FLASH_CONFIG_VERSION = 46;
static uint32_t enabledSensors = 0;
static uint32_t flagRegistor = 0;

static void resetFlashConfig(void);

static uint8_t validFlashConfig(void)
{
    const config_t *temp = (const config_t *)FLASH_CONFIG_ADDR;
    const uint8_t *p;
    uint8_t chk = 0;

    // check version number
    if (FLASH_CONFIG_VERSION != temp->version)
        return 0;

    // check size and magic numbers
    if (temp->size != sizeof(config_t) || temp->magic_be != 0xBE || temp->magic_ef != 0xEF)
        return 0;

    // verify integrity of temporary copy
    for (p = (const uint8_t *)temp; p < ((const uint8_t *)temp + sizeof(config_t)); p++)
        chk ^= *p;

    // checksum failed
    if (chk != 0)
        return 0;

    // looks good, let's roll!
    return 1;
}

void readFlashConfig(void)
{
    uint8_t i;

    // Read flash
    memcpy(&cfg, (char *)FLASH_CONFIG_ADDR, sizeof(config_t));

    for (i = 0; i < 6; i++)
        lookupPitchRollRC[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 2500;

    for (i = 0; i < 11; i++) {
        int16_t tmp = 10 * i - cfg.thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - cfg.thrMid8;
        if (tmp < 0)
            y = cfg.thrMid8;
        lookupThrottleRC[i] = 10 * cfg.thrMid8 + tmp * (100 - cfg.thrExpo8 + (int32_t) cfg.thrExpo8 * (tmp * tmp) / (y * y)) / 10;      // [0;1000]
        lookupThrottleRC[i] = cfg.minthrottle + (int32_t) (cfg.maxthrottle - cfg.minthrottle) * lookupThrottleRC[i] / 1000;     // [0;1000] -> [MINTHROTTLE;MAXTHROTTLE]
    }

    cfg.tri_yaw_middle = constrain(cfg.tri_yaw_middle, cfg.tri_yaw_min, cfg.tri_yaw_max);       //REAR

    /* Add Watch-dog events */
    if (flag(FLAG_WDG_OCCURRED)) cfg.wdg_counter ++;
}

void writeFlashConfig(uint8_t b)
{
	/*
	 * Проверяем длину структуры cfg, не больше размера сектора 2К
	 * При привышении длины объекта эта функция на этапе компиляции выдаст ошибку
	 * "error: duplicate case value"
	 */
	ASSERT_SIZE_EXCEEDED(config_t, 2048);

    FLASH_Status status;
    uint8_t tries = 0;
    uint32_t i;
    uint8_t chk = 0;
    const uint8_t *p;

    cfg.version  = FLASH_CONFIG_VERSION;
    cfg.size     = sizeof(config_t);
    cfg.magic_be = 0xBE;
    cfg.magic_ef = 0xEF;
    cfg.chk      = 0;

    // Recalculate checksum before writing
    for (p = (const uint8_t *)&cfg; p < ((const uint8_t *)&cfg + sizeof(config_t)); p++)
        chk ^= *p;

    cfg.chk = chk;

    // write it
retry:
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	if (FLASH_ErasePage(FLASH_CONFIG_ADDR) == FLASH_COMPLETE) {
		for (i = 0; i < sizeof(config_t); i += 4) {
			status = FLASH_ProgramWord(FLASH_CONFIG_ADDR + i, *(uint32_t *) ((char *)&cfg + i));
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
	if (tries == 3 || !validFlashConfig()) {
		mixerWwriteAllMotors(cfg.mincommand);
		failureMode(4);
	}

    readFlashConfig();

    if (b)
    {
    	signalLED(LED_STS, LEDMODE_BLINK1);
    	signalBUZZ(BUZZMODE_1);
    }
}

void checkFirstTime(bool reset)
{
    // check the EEPROM integrity before resetting values
    if (!validFlashConfig() || reset)
        resetFlashConfig();
}

// Default settings
static void resetFlashConfig(void)
{
    uint8_t i;

    memset(&cfg, 0, sizeof(config_t));

    cfg.version = FLASH_CONFIG_VERSION;
    cfg.mixerConfiguration = MULTITYPE_QUADX;

    cfg.P8[PIDROLL] 			= 33;	// * 0.1
    cfg.I8[PIDROLL] 			= 36;	// * 0.001
    cfg.D8[PIDROLL] 			= 17;	// * 1
    cfg.P8[PIDPITCH] 			= 33;	// * 0.1
    cfg.I8[PIDPITCH] 			= 35;	// * 0.001
    cfg.D8[PIDPITCH] 			= 17;	// * 1
    cfg.P8[PIDYAW] 				= 180;	// * 0.1
    cfg.I8[PIDYAW] 				= 60;	// * 0.001
    // cfg.D8[YAW] 				= 0;
    cfg.P8[PIDALT] 				= 20;	// * 0.1
    cfg.I8[PIDALT] 				= 36;	// * 0.001
    cfg.D8[PIDALT] 				= 15;	// * 1
    cfg.P8[PIDPOS] 				= 11; 	// * 0.01
    // cfg.I8[PIDPOS] 			= 0;	// * 0.01
    // cfg.D8[PIDPOS] 			= 0;
    cfg.P8[PIDPOSR] 			= 20; 	// * 0.1
    cfg.I8[PIDPOSR] 			= 8; 	// * 0.01
    cfg.D8[PIDPOSR] 			= 45; 	// * 0.001
    cfg.P8[PIDNAVR] 			= 14; 	// * 0.1
    cfg.I8[PIDNAVR] 			= 20; 	// * 0.01
    cfg.D8[PIDNAVR] 			= 80; 	// * 0.001
    cfg.P8[PIDLEVEL] 			= 37;	// * 0.1
    cfg.I8[PIDLEVEL] 			= 15;	// * 0.001
    cfg.D8[PIDLEVEL] 			= 5;	// * 1
    cfg.P8[PIDMAG] 				= 120;	// * 0.1
    // cfg.P8[PIDVEL] = 0;
    // cfg.I8[PIDVEL] = 0;
    // cfg.D8[PIDVEL] = 0;
    cfg.rcRate8 				= 90;	// * 0.01
    cfg.rcExpo8 				= 65;	// * 0.01
    cfg.rollPitchRate 			= 0;
    cfg.yawRate 				= 0;
    // cfg.dynThrPID = 0;
    cfg.thrMid8 				= 50;	// * 0.01
    cfg.thrExpo8 				= 0;

    // for (i = 0; i < CHECKBOXITEMS; i++)
    //     cfg.activate[i] = 0;

    cfg.acc_bias[ROLL ]			= 0.0f;
    cfg.acc_bias[PITCH]			= 0.0f;
    cfg.acc_bias[YAW  ] 		= 0.0f;
    cfg.acc_scale[ROLL ] 		= 0.0392266f;	// Accel FS=8g, Sensitivity = 4 mg/lsb, acc_scale: 0,004 * 9,80665 = 0,0392266 m/s2/lsb
    cfg.acc_scale[PITCH] 		= 0.0392266f;
    cfg.acc_scale[YAW  ] 		= 0.0392266f;
    cfg.acc_temp_comp[ROLL ] 	= 0.0f;		// Accel temperature compensation factor
    cfg.acc_temp_comp[PITCH] 	= 0.0f;
    cfg.acc_temp_comp[YAW  ] 	= 0.0f;
    cfg.acc_magnitude			= 9.80665f;
    cfg.acc_lpf_factor 			= 4;
    cfg.acc_lpf_for_velocity 	= 10;
    cfg.accz_deadband 			= 50;
    cfg.gyro_bias[ROLL ]		= 0.0f;
    cfg.gyro_bias[PITCH]		= 0.0f;
    cfg.gyro_bias[YAW  ]		= 0.0f;
    cfg.gyro_scale[ROLL ] 		= 0.07f;	// Gyro FS=2000 dps, Sensitivity = 70 mdps/lsb (L3GD20)
    cfg.gyro_scale[PITCH] 		= 0.07f;
    cfg.gyro_scale[YAW  ] 		= 0.07f;
    cfg.gyro_temp_comp[ROLL ] 	= 0.0f;		// Gyro temperature compensation factor
    cfg.gyro_temp_comp[PITCH] 	= 0.0f;
    cfg.gyro_temp_comp[YAW  ] 	= 0.0f;
    cfg.gyro_cmpf_factor 		= 600; 		// default MWC
    cfg.baro_tab_size 			= 21;
    cfg.baro_noise_lpf 			= 0.6f;
    cfg.baro_cf 				= 0.985f;
    cfg.baro_cf_alt				= 0.900f;
    cfg.baro_cf_vel				= 0.995f;
    cfg.gyro_smooth[ROLL ]		= 20;		// default factors of 20, 20, 3 for R/P/Y
    cfg.gyro_smooth[PITCH]		= 20;
    cfg.gyro_smooth[YAW  ]		= 3;
    //cfg.gyro_smoothing			= 0;

    //cfg.angleTrim[0] 			= 0;
    //cfg.angleTrim[1] 			= 0;
    cfg.mag_declination 		= 1025;		// We are in Moscow. Format is [sign]dddmm (degreesminutes)
    cfg.imu_algorithm 			= 1;

    // Power sensor
    cfg.batvscale 				= 60;
    cfg.batmaxcellvoltage 		= 43;
    cfg.batmincellvoltage 		= 33;
    cfg.batiscale 				= 159;		// Adapted for Armazila APS-45/90 Power Sensor (90A)
    //cfg.ibatoffset 				= 0;
    //cfg.batalarm				= 0;		// Battery warning disable
    cfg.flightBatteryCapacity	= 5000;

    // Radio
    cfg.rcprotocol				= RC_PPM;
    cfg.failsafe				= 0;
    // cfg.deadband = 0;
    // cfg.yawdeadband = 0;
    cfg.alt_hold_throttle_neutral = 20;
    // cfg.spektrum_hires = 0;
    cfg.midrc 					= 1500;
    cfg.mincheck 				= 1100;
    cfg.maxcheck 				= 1900;
    // cfg.retarded_arm = 0;       // disable arm/disarm on roll left/right

    // Function to channel mapping
    for (i = 0; i < 16; i++)
    	cfg.rcmap[i] = i + 1;

    // Failsafe Variables
    cfg.failsafe_delay 			= 10;    	// 1sec
    cfg.failsafe_off_delay 		= 200;      // 20sec
    cfg.failsafe_throttle 		= 1200;     // decent default which should always be below hover throttle for people.

    // Motor/ESC/Servo
    cfg.minthrottle 			= 1150;
    cfg.maxthrottle 			= 1850;
    cfg.mincommand 				= 1000;
    cfg.pwm_group1_rate			= 400;
    cfg.pwm_group2_rate			= 400;
    cfg.pwm_group3_rate			= 50;
    cfg.pwm_group4_rate			= 50;

    // servos
    cfg.yaw_direction 			= 1;
    cfg.tri_yaw_middle 			= 1500;
    cfg.tri_yaw_min 			= 1020;
    cfg.tri_yaw_max 			= 2000;

    // flying wing
    cfg.wing_left_min 			= 1020;
    cfg.wing_left_mid 			= 1500;
    cfg.wing_left_max 			= 2000;
    cfg.wing_right_min 			= 1020;
    cfg.wing_right_mid 			= 1500;
    cfg.wing_right_max 			= 2000;
    cfg.pitch_direction_l 		= 1;
    cfg.pitch_direction_r 		= -1;
    cfg.roll_direction_l 		= 1;
    cfg.roll_direction_r 		= 1;

    // gimbal
    cfg.gimbal					= 0;
    cfg.gimbal_pitch_gain 		= 10;
    cfg.gimbal_roll_gain 		= 10;
    cfg.gimbal_flags 			= GIMBAL_NORMAL;
    cfg.gimbal_pitch_min 		= 1020;
    cfg.gimbal_pitch_max 		= 2000;
    cfg.gimbal_pitch_mid 		= 1500;
    cfg.gimbal_roll_min 		= 1020;
    cfg.gimbal_roll_max 		= 2000;
    cfg.gimbal_roll_mid 		= 1500;

    // gps/nav stuff
    cfg.gps_type 				= GPS_UBLOX;
    cfg.gps_baudrate 			= 38400;
    cfg.gps_wp_radius 			= 200;
    cfg.gps_lpf 				= 20;
    cfg.nav_slew_rate 			= 30;
    cfg.nav_controls_heading 	= 1;
    cfg.nav_speed_min 			= 100;
    cfg.nav_speed_max 			= 300;

    // OSD configuration
    cfg.osd_screen 				= 4;

    cfg.wdg_counter 			= 0;

    cfg.port_map				= 2;	// HIL
    cfg.uart1_baudrate			= 115200;

    cfg.hil_mode				= 0;

    cfg.mavlink_sysid			= 20;
    cfg.mavlink_compid			= 200;
    cfg.mavlink_telemetry_flag	= ML_TELEMETRY_HUD | ML_TELEMETRY_GPS | ML_TELEMETRY_RC_CHANNELS_RAW | ML_TELEMETRY_SERVO_OUTPUT_RAW;

    // custom mixer. clear by defaults.
    for (i = 0; i < MAX_MOTORS; i++)
        cfg.customMixer[i].throttle = 0.0f;

    writeFlashConfig(0);
}

/* Sensors getter/setter */
bool sensors(uint32_t mask)
{
    return enabledSensors & mask;
}

void sensorsSet(uint32_t mask)
{
    enabledSensors |= mask;
}

void sensorsClear(uint32_t mask)
{
    enabledSensors &= ~(mask);
}

uint32_t sensorsMask(void)
{
    return enabledSensors;
}

/* Flag getter/setter */
bool flag(AvailableFlags_t mask)
{
    return flagRegistor & mask;
}

void flagSet(AvailableFlags_t mask)
{
	flagRegistor |= mask;
}

void flagClear(AvailableFlags_t mask)
{
	flagRegistor &= ~(mask);
}
