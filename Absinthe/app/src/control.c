
#include "main.h"

static xTimerHandle timerArming;

int16_t 	debug[4];
uint32_t 	currentTime = 0;
int16_t 	headFreeModeHold;

int16_t 	failsafeCnt = 0;
int16_t 	failsafeEvents = 0;
int16_t 	rcData[8] = { 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502 }; // interval [1000;2000]
int16_t 	rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t 	lookupPitchRollRC[6]; // lookup table for expo & RC rate PITCH+ROLL
int16_t 	lookupThrottleRC[11]; // lookup table for expo & mid THROTTLE

rcReadRawDataPtr rcReadRawFunc = NULL; // receive data from default (pwm/ppm) or additional (spek/sbus/?? receiver drivers)

uint8_t 	dynP8[3], dynI8[3], dynD8[3];
uint8_t 	rcOptions[CHECKBOXITEMS];

int16_t 	axisPID[3];

int16_t 	errorGyroI[3] = { 0, 0, 0 };
int16_t 	errorAngleI[2] = { 0, 0 };
uint8_t		rssi = 0;						// 0..255 -> 0%..100%

int16_t nav[2];
int16_t nav_rated[2]; // Adding a rate controller to the navigation to make it smoother
int8_t nav_mode = NAV_MODE_NONE; // Navigation mode

// Automatic ACC Offset Calibration
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;

#define BREAKPOINT 1500

int16_t rxReadRawRC(uint8_t chan)
{
	uint16_t data;

	data = pwmRead(chan);
	if (data < 750 || data > 2250)
		data = cfg.midrc;

	return data;
}

/* Read RC chanels raw data, filtered it and apply deadband */
void computeRC(void)
{
	static int16_t rcData4Values[8][4];
	static int16_t rcDataMean[8];
	static uint8_t rc4ValuesIndex = 0;
	uint8_t chan, a;

	rc4ValuesIndex++;
	for (chan = 0; chan < 8; chan++)
	{
		rcData4Values[chan][rc4ValuesIndex % 4] = rcReadRawFunc(cfg.rcmap[chan]);
		rcDataMean[chan] = 0;
		for (a = 0; a < 4; a++)
			rcDataMean[chan] += rcData4Values[chan][a];

		rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
		if (rcDataMean[chan] < rcData[chan] - 3)
			rcData[chan] = rcDataMean[chan] + 2;
		if (rcDataMean[chan] > rcData[chan] + 3)
			rcData[chan] = rcDataMean[chan] - 2;
	}
}

static void armingCallback(xTimerHandle pxTimer)
{
	flagSet(FLAG_ARMED);
	headFreeModeHold = heading;
	signalLED(LED_STS, LEDMODE_ON);
}

static void arming(void)
{
	buzzerPlay("1EFGH EFGH 4X");
	xTimerStart(timerArming, 10);
}

static void disarm(bool rearm)
{
	// Turn off "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
	// This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
	// to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm

	flagClear(FLAG_ARMED);

	if (rearm)
		flagClear(FLAG_OK_TO_ARM);

	signalLED(LED_STS, LEDMODE_OFF);
	buzzerPlay("2M");
}

void loop(void)
{
	static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
	uint8_t i;
	static int16_t initialThrottleHold;
	uint16_t auxState = 0;

	currentTime = micros();

	// this will return false if spektrum is disabled. shrug.
	if (spektrumFrameComplete())
		computeRC();

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	// TODO clean this up. computeRC should handle this check
	if (cfg.rcprotocol != RC_SPEKTRUM)
		computeRC();

	// Failsafe routine
	if (cfg.failsafe)
	{
		if (failsafeCnt > (5 * cfg.failsafe_delay) && flag(FLAG_ARMED))
		{ // Stabilize, and set Throttle to specified level
			for (i = 0; i < 3; i++)
				rcData[i] = cfg.midrc; // after specified guard time after RC signal is lost (in 0.1sec)
			rcData[THROTTLE] = cfg.failsafe_throttle;
			if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay))
			{
				disarm(true);
			}
			failsafeEvents++;
		}
		if (failsafeCnt > (5 * cfg.failsafe_delay) && !flag(FLAG_ARMED))
		{
			disarm(true);
		}
		failsafeCnt++;
	}

	if (rcData[THROTTLE] < cfg.mincheck)
	{
		errorGyroI[ROLL ] = 0;
		errorGyroI[PITCH] = 0;
		errorGyroI[YAW  ] = 0;
		errorAngleI[ROLL ] = 0;
		errorAngleI[PITCH] = 0;
		rcDelayCommand++;

////////////// Gyro calibration stick command
		if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck && !flag(FLAG_ARMED))
		{
			if (rcDelayCommand == 20)
			{
				flagSet(FLAG_CALIBRATE_GYRO);
				if (sensors(SENSOR_GPS))
					GPS_reset_home_position();
			}
		}
		else if (cfg.acc_inflight_cal && (!flag(FLAG_ARMED) && rcData[YAW] < cfg.mincheck
						&& rcData[PITCH] > cfg.maxcheck && rcData[ROLL] > cfg.maxcheck))
		{
			if (rcDelayCommand == 20)
			{
				if (AccInflightCalibrationMeasurementDone)
				{ // trigger saving into eeprom after landing
					AccInflightCalibrationMeasurementDone = 0;
					AccInflightCalibrationSavetoEEProm = 1;
				}
				else
				{
					AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
					if (AccInflightCalibrationArmed)
					{
//							toggleBeep = 2;
					}
					else
					{
//							toggleBeep = 3;
					}
				}
			}
		}
		else if (cfg.activate[BOXARM] > 0)
		{
			if (rcOptions[BOXARM] && flag(FLAG_OK_TO_ARM))
			{
				// TODO: feature(FEATURE_FAILSAFE) && failsafeCnt == 0
				arming();
			}
			else if (flag(FLAG_ARMED))
			{
				disarm(false);
			}
			rcDelayCommand = 0;
		}
		else if ((rcData[YAW] < cfg.mincheck || (cfg.retarded_arm == 1 && rcData[ROLL] < cfg.mincheck)) && flag(FLAG_ARMED))
		{
			if (rcDelayCommand == 20)
			{
				disarm(false);
			}
		}
		else if ((rcData[YAW] > cfg.maxcheck || (rcData[ROLL] > cfg.maxcheck && cfg.retarded_arm == 1))
				&& rcData[PITCH] < cfg.maxcheck && !flag(FLAG_ARMED) && flag(FLAG_ACC_CALIBRATED | FLAG_GYRO_CALIBRATED))
		{
			if (rcDelayCommand == 20)
			{
				arming();
			}
		}
		else
			rcDelayCommand = 0;
	}
	else if (rcData[THROTTLE] > cfg.maxcheck && !flag(FLAG_ARMED))
	{
////////////// Acc calibration stick command
		if (rcData[YAW] < cfg.mincheck && rcData[PITCH] < cfg.mincheck)
		{ // throttle=max, yaw=left, pitch=min
			if (rcDelayCommand == 20)
				flagSet(FLAG_CALIBRATE_ACC);
			rcDelayCommand++;
		}
////////////// Mag calibration stick command
		else if (rcData[YAW] > cfg.maxcheck && rcData[PITCH] < cfg.mincheck)
		{ // throttle=max, yaw=right, pitch=min
			if (rcDelayCommand == 20)
				flagSet(FLAG_CALIBRATE_MAG); // MAG calibration request
			rcDelayCommand++;
		}
////////////// Acc trim Pitch+ stick command
		else if (rcData[PITCH] > cfg.maxcheck)
		{
			cfg.angleTrim[PITCH] += 2;
			writeFlashConfig(1);
#ifdef LEDRING
			if (feature(FEATURE_LED_RING))
				ledringBlink();
#endif
		}
////////////// Acc trim Pitch- stick command
		else if (rcData[PITCH] < cfg.mincheck)
		{
			cfg.angleTrim[PITCH] -= 2;
			writeFlashConfig(1);
#ifdef LEDRING
			if (feature(FEATURE_LED_RING))
				ledringBlink();
#endif
		}
////////////// Acc trim Roll+ stick command
		else if (rcData[ROLL] > cfg.maxcheck)
		{
			cfg.angleTrim[ROLL] += 2;
			writeFlashConfig(1);
#ifdef LEDRING
			if (feature(FEATURE_LED_RING))
				ledringBlink();
#endif
		}
////////////// Acc trim Roll- stick command
		else if (rcData[ROLL] < cfg.mincheck)
		{
			cfg.angleTrim[ROLL] -= 2;
			writeFlashConfig(1);
#ifdef LEDRING
			if (feature(FEATURE_LED_RING))
				ledringBlink();
#endif
		}
		else
		{
			rcDelayCommand = 0;
		}
	}

	if (cfg.acc_inflight_cal)
	{
		if (AccInflightCalibrationArmed && flag(FLAG_ARMED) && rcData[THROTTLE] > cfg.mincheck && !rcOptions[BOXARM])
		{ // Copter is airborne and you are turning it off via boxarm : start measurement
			InflightcalibratingA = 50;
			AccInflightCalibrationArmed = 0;
		}
		if (rcOptions[BOXPASSTHRU])
		{ // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
			if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
				InflightcalibratingA = 50;
		}
		else if (AccInflightCalibrationMeasurementDone && !flag(FLAG_ARMED))
		{
			AccInflightCalibrationMeasurementDone = 0;
			AccInflightCalibrationSavetoEEProm = 1;
		}
	}

	for (i = 0; i < 4; i++)
		auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700)
						<< (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
	for (i = 0; i < CHECKBOXITEMS; i++)
		rcOptions[i] = (auxState & cfg.activate[i]) > 0;

	// note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
	if ((rcOptions[BOXANGLE] || (failsafeCnt > 5 * cfg.failsafe_delay)) && (sensors(SENSOR_ACC)))
	{
		// bumpless transfer to Level mode
		if (!flag(FLAG_ANGLE_MODE))
		{
			errorAngleI[ROLL ] = 0;
			errorAngleI[PITCH] = 0;
			flagSet(FLAG_ANGLE_MODE);
		}
	}
	else
	{
		flagClear(FLAG_ANGLE_MODE); // failsave support
	}

	if (rcOptions[BOXHORIZON])
	{
		if (!flag(FLAG_HORIZON_MODE))
		{
			errorAngleI[ROLL ] = 0;
			errorAngleI[PITCH] = 0;
			flagSet(FLAG_HORIZON_MODE);
		}
	}
	else
	{
		flagClear(FLAG_HORIZON_MODE);
	}

	if ((rcOptions[BOXARM]) == 0)
		flagSet(FLAG_OK_TO_ARM);

	if (sensors(SENSOR_BARO))
	{
		if (rcOptions[BOXALTHOLD])
		{
			if (!flag(FLAG_ALTHOLD_MODE))
			{
				flagSet(FLAG_ALTHOLD_MODE);
				setAltHold(EstAlt);
				initialThrottleHold = rcCommand[THROTTLE];
			}
		}
		else
			flagClear(FLAG_ALTHOLD_MODE);
	}

	if (sensors(SENSOR_MAG))
	{
		if (rcOptions[BOXMAG])
		{
			if (!flag(FLAG_MAG_MODE))
			{
				flagSet(FLAG_MAG_MODE);
				magHold = heading;
			}
		}
		else
			flagClear(FLAG_MAG_MODE);
		if (rcOptions[BOXHEADFREE])
		{
			if (!flag(FLAG_HEADFREE_MODE))
			{
				flagSet(FLAG_HEADFREE_MODE);
			}
		}
		else
		{
			flagClear(FLAG_HEADFREE_MODE);
		}
		if (rcOptions[BOXHEADADJ])
		{
			headFreeModeHold = heading; // acquire new heading
		}
	}

	if (sensors(SENSOR_GPS))
	{
		if (flag(FLAG_GPS_FIX) && gps.numSat >= 5)
		{
			if (rcOptions[BOXGPSHOME])
			{
				if (!flag(FLAG_GPSHOME_MODE))
				{
					flagSet(FLAG_GPSHOME_MODE);
					GPS_set_next_wp(&gps.home[LAT], &gps.home[LON]);
					nav_mode = NAV_MODE_WP;
				}
			}
			else
			{
				flagClear(FLAG_GPSHOME_MODE);
			}
			if (rcOptions[BOXGPSHOLD])
			{
				if (!flag(FLAG_GPSHOLD_MODE))
				{
					flagSet(FLAG_GPSHOLD_MODE);
					gps.hold[LAT] = gps.coord[LAT];
					gps.hold[LON] = gps.coord[LON];
					GPS_set_next_wp(&gps.hold[LAT], &gps.hold[LON]);
					nav_mode = NAV_MODE_POSHOLD;
				}
			}
			else
			{
				flagClear(FLAG_GPSHOLD_MODE);
			}
		}
	}

	if (rcOptions[BOXPASSTHRU])
	{
		flagSet(FLAG_PASSTHRU_MODE);
	}
	else
	{
		flagClear(FLAG_PASSTHRU_MODE);
	}

	if (cfg.mixerConfiguration == MULTITYPE_FLYING_WING || cfg.mixerConfiguration == MULTITYPE_AIRPLANE)
	{
		flagClear(FLAG_HEADFREE_MODE);
	}

	// void annexCode(void)

	uint16_t tmp, tmp2;
	uint8_t axis, prop1, prop2;

	// PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
	if (rcData[THROTTLE] < BREAKPOINT)
	{
		prop2 = 100;
	}
	else
	{
		if (rcData[THROTTLE] < 2000)
		{
			prop2 = 100	- (uint16_t) cfg.dynThrPID * (rcData[THROTTLE] - BREAKPOINT) / (2000 - BREAKPOINT);
		}
		else
		{
			prop2 = 100 - cfg.dynThrPID;
		}
	}

	for (axis = 0; axis < 3; axis++)
	{
		tmp = min(abs(rcData[axis] - cfg.midrc), 500);
		if (axis != 2)
		{ // ROLL & PITCH
			if (cfg.deadband)
			{
				if (tmp > cfg.deadband)
				{
					tmp -= cfg.deadband;
				}
				else
				{
					tmp = 0;
				}
			}

			tmp2 = tmp / 100;
			rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
			prop1 = 100 - (uint16_t) cfg.rollPitchRate * tmp / 500;
			prop1 = (uint16_t) prop1 * prop2 / 100;
		}
		else
		{ // YAW
			if (cfg.yawdeadband)
			{
				if (tmp > cfg.yawdeadband)
				{
					tmp -= cfg.yawdeadband;
				}
				else
				{
					tmp = 0;
				}
			}
			rcCommand[axis] = tmp;
			prop1 = 100 - (uint16_t) cfg.yawRate * tmp / 500;
		}
		dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
		dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;
		if (rcData[axis] < cfg.midrc)
			rcCommand[axis] = -rcCommand[axis];
	}

	tmp = constrain(rcData[THROTTLE], cfg.mincheck, 2000);
	tmp = (uint32_t) (tmp - cfg.mincheck) * 1000 / (2000 - cfg.mincheck); // [MINCHECK;2000] -> [0;1000]
	tmp2 = tmp / 100;
	rcCommand[THROTTLE] = lookupThrottleRC[tmp2]
		+ (tmp - tmp2 * 100)
		* (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2])
		/ 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

	if (flag(FLAG_HEADFREE_MODE))
	{
		float radDiff = DEG2RAD(heading - headFreeModeHold);
		float cosDiff = cosf(radDiff);
		float sinDiff = sinf(radDiff);
		int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
		rcCommand[ROLL ] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
		rcCommand[PITCH] = rcCommand_PITCH;
	}

#ifdef LEDRING
	if (feature(FEATURE_LED_RING))
	{
		static uint32_t LEDTime;
		if ((int32_t) (currentTime - LEDTime) >= 0)
		{
			LEDTime = currentTime + 50000;
			ledringState();
		}
	}
#endif

	static uint32_t calibratedAccTime;

	if ((int32_t) (currentTime - calibratedAccTime) >= 0)
	{
		if (!flag(FLAG_SMALL_ANGLES_25))
		{
			flagClear(FLAG_ACC_CALIBRATED);		// the multi uses ACC and is not calibrated or is too much inclinated
			signalLED(LED_ERR, LEDMODE_TOGGLE);
			calibratedAccTime = currentTime + 500000;
		}
		else
		{
			flagSet(FLAG_ACC_CALIBRATED);
			signalLED(LED_ERR, LEDMODE_OFF);
		}
	}

	if (sensors(SENSOR_GPS))
	{
		static uint32_t GPSLEDTime;
		if ((int32_t) (currentTime - GPSLEDTime) >= 0 && (gps.numSat >= 5))
		{
			GPSLEDTime = currentTime + 150000;
			signalLED(LED_NAV, LEDMODE_TOGGLE);
		}
	}

	if (flag(FLAG_ANGLE_MODE) || flag(FLAG_HORIZON_MODE))
	{
		signalLED(LED_NAV, LEDMODE_ON);
	}
	else
	{
		signalLED(LED_NAV, LEDMODE_OFF);
	}

	// End annexCode()

	if (sensors(SENSOR_MAG))
	{
		if (abs(rcCommand[YAW]) < 70 && flag(FLAG_MAG_MODE))
		{
			int16_t dif = heading - magHold;
			if (dif <= -180)
				dif += 360;
			if (dif >= +180)
				dif -= 360;
			if (flag(FLAG_SMALL_ANGLES_25))
				rcCommand[YAW] -= dif * cfg.P8[PIDMAG] / 30; // 18 deg
		}
		else
			magHold = heading;
	}

	if (sensors(SENSOR_BARO))
	{
		if (flag(FLAG_ALTHOLD_MODE))
		{
			if (abs(rcCommand[THROTTLE] - initialThrottleHold) > cfg.alt_hold_throttle_neutral)
			{
				flagClear(FLAG_ALTHOLD_MODE); // so that a new althold reference is defined
			}
			rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
		}
	}

	if (sensors(SENSOR_GPS))
	{
		// Check that we really need to navigate ?
		if ((!flag(FLAG_GPSHOME_MODE) && !flag(FLAG_GPSHOLD_MODE)) || !flag(FLAG_GPS_FIX_HOME))
		{
			// If not. Reset nav loops and all nav related parameters
			GPS_reset_nav();
		}
		else
		{
			float sin_yaw_y = sinf(heading_rad);
			float cos_yaw_x = cosf(heading_rad);
			if (cfg.nav_slew_rate)
			{
				nav_rated[LON] += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -cfg.nav_slew_rate, cfg.nav_slew_rate); // TODO check this on uint8
				nav_rated[LAT] += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -cfg.nav_slew_rate, cfg.nav_slew_rate);
				gps.angle[ROLL] = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
				gps.angle[PITCH] = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
			}
			else
			{
				gps.angle[ROLL] = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
				gps.angle[PITCH] = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
			}
		}
	}
}

//
// Calculates all axis PIDs
// Input : dT - delta time in seconds
//
void stabilize(float dT)
{
	uint8_t axis;
	int16_t prop;
	float   delta;
	float   error, errorAngle;
	float   deltaSum;
	float   PTerm, ITerm, DTerm;
	float   PTermACC = 0;
	float	ITermACC = 0;
	float   PTermGYRO = 0, ITermGYRO = 0;

	static float delta1[3], delta2[3];
	static float lastGyro[3] = { 0, 0, 0 };
	static float lastDTerm[3] = { 0.0, 0.0, 0.0 };

	// **** PITCH & ROLL & YAW PID ****
	// Pterm in 0,1
	// ITerm in 0,001
	// DTerm in 1
	// all in uint8_t

#define F_CUT   20.0f
#define RC      0.5f / (M_PI * F_CUT)

	prop = max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])); // range [0;500]

	for (axis = 0; axis < 3; axis++)
	{
		if ((flag(FLAG_ANGLE_MODE) || flag(FLAG_HORIZON_MODE)) && axis < 2)	// MODE relying on ACC
		{
		  // 50 degrees max inclination
			errorAngle = constrain(2 * rcCommand[axis] + gps.angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis];
			PTermACC = errorAngle * cfg.P8[PIDLEVEL] / 100.0f;
			PTermACC = constrain(PTermACC, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);
			errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000); // WindUp
			ITermACC = errorAngleI[axis] * cfg.I8[PIDLEVEL] / 4096.0f;
		}

		if (!flag(FLAG_ANGLE_MODE) || flag(FLAG_HORIZON_MODE) || axis == YAW)	// MODE relying on GYRO or YAW axis
		{
			error = rcCommand[axis] * 80.0f / cfg.P8[axis];
			error -= gyroData[axis];
			PTermGYRO = rcCommand[axis];
			errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000); // WindUp
			if (abs(gyroData[axis]) > 640)
				errorGyroI[axis] = 0;
			ITermGYRO = errorGyroI[axis] * cfg.I8[axis] / 8000.0f;
		}
		if (flag(FLAG_HORIZON_MODE) && axis < 2)
		{
			PTerm = (PTermACC * (500 - prop) + PTermGYRO * prop) / 500.0f;
			ITerm = (ITermACC * (500 - prop) + ITermGYRO * prop) / 500.0f;
		}
		else
		{
			if (flag(FLAG_ANGLE_MODE) && axis < 2)
			{
				PTerm = PTermACC;
				ITerm = ITermACC;
			}
			else
			{
				PTerm = PTermGYRO;
				ITerm = ITermGYRO;
			}
		}

		PTerm -= gyroData[axis] * dynP8[axis] / 80.0f;
		delta = gyroData[axis] - lastGyro[axis];
		lastGyro[axis] = gyroData[axis];
		deltaSum = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;

        // calculate PT1 element on deltaSum
		deltaSum = lastDTerm[axis] + (dT / (RC + dT)) * (deltaSum - lastDTerm[axis]);
		lastDTerm[axis] = deltaSum;
		DTerm = deltaSum * dynD8[axis] / 32.0f;
		axisPID[axis] = PTerm + ITerm - DTerm;
	}
}

portTASK_FUNCTION_PROTO(rcLoopTask, pvParameters)
{
	portTickType xLastWakeTime;

	timerArming = xTimerCreate((signed char *) "TimArming", 2000, pdFALSE, (void *) 0, armingCallback);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		loop();

		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 20);	// Task cycle time 20 ms (50 Hz)
	}
}
