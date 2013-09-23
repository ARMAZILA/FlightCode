
#include "main.h"

#define GYRO_LPF_BIAS		(0.01)
#define GYRO_LPF_VARIANCE	(0.01)

#define ACC_LPF_BIAS		(0.01)
#define ACC_LPF_VARIANCE	(0.01)

counters_t		counters;				// Counters
gyro_sensor_t	gyro_sensor;			// Structure for real time gyro sensor data
acc_sensor_t	acc_sensor;				// Structure for real time accel sensor data
int16_t 		mag_sensor_data[3];		// Magnetometer sensor data
power_sensor_t	power_sensor;
alt_sensor_t	alt_sensor;
uint16_t 		acc_1G = 256;  			// this is the 1G measured acceleration.

//extern uint16_t InflightcalibratingA;
//extern int16_t AccInflightCalibrationArmed;
//extern uint16_t AccInflightCalibrationMeasurementDone;
//extern uint16_t AccInflightCalibrationSavetoEEProm;
//extern uint16_t AccInflightCalibrationActive;

void sensorsInit(void)
{
	if (!l3gd20Detect()) {
		// if this fails, we get a beep + blink pattern. we're doomed, no gyro or i2c error.
		failureMode(3);
	}

	gyro_sensor.sample_count = 0;
	gyro_sensor.overrun_count = 0;

	// Init accelerometer and magnitometer
	lsm303dlhcConfig();
	sensorsSet(SENSOR_ACC);
   	sensorsSet(SENSOR_MAG);

   	acc_sensor.sample_count = 0;
   	acc_sensor.overrun_count = 0;

    if (cfg.magZero[ROLL] != 0 || cfg.magZero[PITCH] != 0 || cfg.magZero[YAW] != 0)
    	flagSet(FLAG_MAG_CALIBRATED);

    // Detect pressure sensor
    if (lps331apDetect()) {
    	sensorsSet(SENSOR_BARO);
    }
}

void batteryInit(void)
{
    uint32_t i;
    uint32_t voltage = 0;

    // average up some voltage readings
    for (i = 0; i < 32; i++)
    {
        voltage += adcGetChannel(ADC_VOLTAGE_SENSOR);
        vTaskDelay(10 / portTICK_RATE_MS);	// Delay on 10 ms
    }

    voltage = (((voltage / 32) * 3.3f) / 4095) * cfg.batvscale;

    // autodetect cell count, going from 2S..6S
    for (i = 2; i < 6; i++)
    {
        if (voltage < i * cfg.batmaxcellvoltage)
            break;
    }
    power_sensor.flightBatteryCellCount = i;
    power_sensor.flightBatteryWarningVoltage = i * cfg.batmincellvoltage; // 3.3V per cell minimum, configurable in CLI
}

/*
 * Reading accel data using FIFO buffer.
 */
static void accSensorUpdate(void)
{
    uint8_t axis;
    uint8_t i;
	int16_t accel[3];
	uint8_t count;
	float acc_vari_sqrt[3];
	int32_t acc_accum[3] = {0, 0, 0};

	GetFifoSourceFSS(&count);

	if (count == 0)	// Data not ready
	{
		acc_sensor.overrun_count ++;
		return;
	}

	for (i = 0; i < count; i++)
	{
		lsm303dlhcReadAcc(accel);

		// Counts raw data gyroscope
		acc_sensor.sample_count ++;

		// Accumulate the raw data of the gyro
		acc_accum[0] += accel[0];
		acc_accum[1] += accel[1];
		acc_accum[2] += accel[2];
	}

    // Temperature sensor refresh rate 1 Hz. Obtain through 100 cycles
    static uint8_t Temp_counter = 0;

    if (Temp_counter++ == 100)
    {
        Temp_counter = 0;
    	GetTemperature(&acc_sensor.temp);
    }

	for (axis = 0; axis < 3; axis++)
    {
    	// Scale to the m/s2 with the offset and temperature correction.
		// acc_scale: 0,004 * 9,80665 = 0,0392266 (m/s2/lsb)
    	acc_sensor.data_mss[axis] = (acc_accum[axis] / count * cfg.acc_scale[axis]) +
    			(cfg.acc_temp_comp[axis] * acc_sensor.temp) - cfg.acc_bias[axis];

    	// Scale to MultiWii format
    	acc_sensor.data_mw[axis] = acc_sensor.data_mss[axis] / cfg.acc_magnitude * acc_1G;

    	// Calculate the moving average
    	acc_sensor.average[axis] = acc_sensor.average[axis] * (1 - ACC_LPF_BIAS) +
    			acc_sensor.data_mss[axis] * ACC_LPF_BIAS;

    	// Calculate the moving average standard deviation
    	acc_vari_sqrt[axis] = acc_vari_sqrt[axis] * (1 - ACC_LPF_VARIANCE) +
    			pow(acc_sensor.data_mss[axis] - acc_sensor.average[axis], 2) * ACC_LPF_VARIANCE;
    	acc_sensor.variance[axis] = sqrtf(acc_vari_sqrt[axis]);
#if USE_ACC_ZERO_THRESHOLD
    	if (fabs(acc_data_scaled[axis]) < (gyro_variance[i] * 3.0f)) acc_data_scaled[axis] = 0.0f;
#endif
    }

	if (flag(FLAG_CALIBRATE_ACC))
	{
		flagClear(FLAG_CALIBRATE_ACC);
		flagClear(FLAG_ACC_CALIBRATED);

        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
		cfg.acc_magnitude = sqrt(
			acc_sensor.average[ROLL ] * acc_sensor.average[ROLL ] +
			acc_sensor.average[PITCH] * acc_sensor.average[PITCH] +
			acc_sensor.average[YAW  ] * acc_sensor.average[YAW  ]);

        cfg.acc_bias[ROLL ] += acc_sensor.average[ROLL ];
        cfg.acc_bias[PITCH] += acc_sensor.average[PITCH];
        cfg.acc_bias[YAW  ] += acc_sensor.average[YAW  ] - cfg.acc_magnitude;

        cfg.angleTrim[ROLL ] = 0;
        cfg.angleTrim[PITCH] = 0;

        writeFlashConfig(1);      // write accZero in EEPROM
        flagSet(FLAG_ACC_CALIBRATED);
    }

#if 0
    if (cfg.acc_inflight_cal) {
        static int32_t b[3];
        static int16_t accZero_saved[3] = { 0, 0, 0 };
        static int16_t angleTrim_saved[2] = { 0, 0 };

        // Saving old zeropoints before measurement
        if (InflightcalibratingA == 50) {
        	flagClear(FLAG_ACC_CALIBRATED);
            accZero_saved[ROLL ] = cfg.accZero[ROLL];
            accZero_saved[PITCH] = cfg.accZero[PITCH];
            accZero_saved[YAW  ] = cfg.accZero[YAW];
            angleTrim_saved[ROLL ] = cfg.angleTrim[ROLL];
            angleTrim_saved[PITCH] = cfg.angleTrim[PITCH];
        }
        if (InflightcalibratingA > 0) {
            signalLED(LED_STS, LEDMODE_TOGGLE);	// Calibration phasis
            for (axis = 0; axis < 3; axis++) {
                // Reset a[axis] at start of calibration
                if (InflightcalibratingA == 50)
                    b[axis] = 0;
                // Sum up 50 readings
                b[axis] += accel[axis];
                // Clear global variables for next reading
                // accADC[axis] = 0;
                cfg.accZero[axis] = 0;
            }
            // all values are measured
            if (InflightcalibratingA == 1) {
                AccInflightCalibrationActive = 0;
                AccInflightCalibrationMeasurementDone = 1;
                toggleBeep = 2;      // buzzer for indicatiing the end of calibration
                // recover saved values to maintain current flight behavior until new values are transferred
                cfg.accZero[ROLL ] = accZero_saved[ROLL];
                cfg.accZero[PITCH] = accZero_saved[PITCH];
                cfg.accZero[YAW  ] = accZero_saved[YAW];
                cfg.angleTrim[ROLL ] = angleTrim_saved[ROLL];
                cfg.angleTrim[PITCH] = angleTrim_saved[PITCH];
                flagSet(FLAG_ACC_CALIBRATED);
            }
            InflightcalibratingA--;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (AccInflightCalibrationSavetoEEProm == 1) {      // the copter is landed, disarmed and the combo has been done again
            AccInflightCalibrationSavetoEEProm = 0;
            cfg.accZero[ROLL ] = b[ROLL ] / 50;
            cfg.accZero[PITCH] = b[PITCH] / 50;
            cfg.accZero[YAW  ] = b[YAW  ] / 50 - acc_1G;    // for nunchuk 200=1G
            cfg.angleTrim[ROLL]  = 0;
            cfg.angleTrim[PITCH] = 0;
            writeFlashConfig(1);          // write accZero in EEPROM
        }
    }
#endif
}

/*
 * Reading gyro data using FIFO buffer.
 *
 * Use FIFO in stream mode
 *
 * Assuming that the data output rate gyro set higher frequency of the main loop algorithm. For
 * example, the gyroscope operates at 760 Hz, the main loop - 100 Hz. This buffer accumulates  ~7
 * measurements, the function reads data from the buffer and averages them.
 *
 * The results of the function is stored in the following variables:
 * gyro_sensor.data_deg - Scaled to deg/sec real angles
 * gyro_sensor.data_rad - Scaled to rad/sec real angles
 * gyro_sensor.average	- Average value
 * gyro_sensor.variance	- Standard deviation
 * gyro_sensor.temp     - Temperature sensor data. Refresh rate 1 Hz
 *
 * If defined USE_GYRO_ZERO_THRESHOLD small values of angular velocities are equal to zero.
 *
 */
static void gyroSensorUpdate(void)
{
	uint8_t count = 0;

	// Get FIFO stored data level
	count = l3gd20GetFIFOLevel();

	if (count == 0)	// No data avalible
	{
		gyro_sensor.overrun_count ++;
		return;
	}

	uint8_t i;
	int16_t gyro[3];
	int32_t gyro_accum[3] = {0, 0, 0};

	for (i = 0; i < count; i++)
	{
		l3gd20Read(gyro);

		// Counts raw gyroscope data
		gyro_sensor.sample_count ++;

		// Accumulate the raw data of the gyro
		gyro_accum[0] += gyro[0];
		gyro_accum[1] += gyro[1];
		gyro_accum[2] += gyro[2];
	}

	// Temperature sensor refresh rate 1 Hz. Obtain through 100 cycles
	static uint8_t temp_counter = 0;

	if (temp_counter++ == 100)
	{
		temp_counter = 0;
		l3gd20GetTemp(&gyro_sensor.temp);
	}

    uint8_t axis;
	static float gyro_vari_sqrt[3] 	= {0.0f, 0.0f, 0.0f};

    for (axis = 0; axis < 3; axis++)
	{
		// Scale to the degrees per second with the offset and temperature correction.
		gyro_sensor.data_deg[axis] = (gyro_accum[axis] / count * cfg.gyro_scale[axis]) +
				(cfg.gyro_temp_comp[axis] * gyro_sensor.temp) - cfg.gyro_bias[axis];

		// Scale to the radian per second
		gyro_sensor.data_rad[axis] = DEG2RAD(gyro_sensor.data_deg[axis]);

		// Calculate the moving average
		gyro_sensor.average[axis] = gyro_sensor.average[axis] * (1 - GYRO_LPF_BIAS) +
				gyro_sensor.data_deg[axis] * GYRO_LPF_BIAS;

		// Calculate the moving average standard deviation
		gyro_vari_sqrt[axis] = gyro_vari_sqrt[axis] * (1 - GYRO_LPF_VARIANCE) +
				pow(gyro_sensor.data_deg[axis] - gyro_sensor.average[axis], 2) * GYRO_LPF_VARIANCE;
		gyro_sensor.variance[axis] = sqrtf(gyro_vari_sqrt[axis]);
	}

	if (flag(FLAG_CALIBRATE_GYRO))
	{
        signalLED(LED_STS, LEDMODE_TOGGLE);		// Calibration phase

#define GIRO_VARI_MAX	1.5f

        if (fabs(gyro_sensor.variance[ROLL ]) < GIRO_VARI_MAX &&
        	fabs(gyro_sensor.variance[PITCH]) < GIRO_VARI_MAX &&
        	fabs(gyro_sensor.variance[YAW  ]) < GIRO_VARI_MAX)
        {
        	// If gyro_average by all axises is small value we don't calibrate the gyro

        	// Apply new gyro bias
			cfg.gyro_bias[ROLL ] += gyro_sensor.average[ROLL ];
			cfg.gyro_bias[PITCH] += gyro_sensor.average[PITCH];
			cfg.gyro_bias[YAW  ] += gyro_sensor.average[YAW  ];

			writeFlashConfig(1);

			flagClear(FLAG_CALIBRATE_GYRO);	// Clear gyro calibrate request flag
	        flagSet(FLAG_GYRO_CALIBRATED);	// Set gyro calibrated flag
        }
	}
}

static void magSensorUpdate(void)
{
	int16_t mag[3];
    static uint16_t calibratingM = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;

    // Read 3-axis mag values
    lsm303dlhcReadMag(mag);

#define MAG_CALIBRATING_CYCLE 	750

    if (flag(FLAG_CALIBRATE_MAG))
    {
    	flagClear(FLAG_MAG_CALIBRATED);
    	calibratingM = MAG_CALIBRATING_CYCLE;	// 30 seconds
        for (axis = 0; axis < 3; axis++)
        {
            cfg.magZero[axis] = 0;
            magZeroTempMin[axis] = mag[axis];
            magZeroTempMax[axis] = mag[axis];
        }
        flagClear(FLAG_CALIBRATE_MAG);
    }

	if (calibratingM > 0)
	{
        signalLED(LED_STS, LEDMODE_TOGGLE);	// Calibration phasis
		for (axis = 0; axis < 3; axis++)
		{
			if (mag[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = mag[axis];
			if (mag[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = mag[axis];
			if (calibratingM == 1)
			{
				cfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
				if (axis == 2)
				{
					writeFlashConfig(1);
					flagSet(FLAG_MAG_CALIBRATED);
				}
			}
		}
		calibratingM--;
	}

    if (flag(FLAG_MAG_CALIBRATED)) 	// we apply offset only once mag calibration is done
    {
        mag[ROLL ] -= cfg.magZero[ROLL ];
        mag[PITCH] -= cfg.magZero[PITCH];
        mag[YAW  ] -= cfg.magZero[YAW  ];
    }

    mag_sensor_data[ROLL ] = mag[ROLL ];
    mag_sensor_data[PITCH] = mag[PITCH];
    mag_sensor_data[YAW  ] = mag[YAW  ];
}

static void baroSensorUpdate(void)
{
	float pressure, temperature, altitude;

	lps331apRead(&pressure, &temperature);
    From_Pressure_mb_To_Altitude_cm(&pressure, &altitude);

	alt_sensor.baroAlt = altitude;
	alt_sensor.baroTemp = temperature * 10;
}

/*
 * This task only work with i2c sensors, so we don't need any semaphore
 * to manadge i2c bus resourse access.
 *
 * Sensor data rate        : update rate
 * Baro  -  25 Hz -  40 ms | 50 ms
 * Accel - 400 Hz - 2.5 ms | 10 ms
 * Mag   -  30 Hz - 4.5 ms | 40 ms
 * Gyro  - 760 Hz - 1.3 ms | 10 ms
 */

portTASK_FUNCTION_PROTO(sensorTask, pvParameters)
{
	portTickType xLastWakeTime;
	uint8_t		baroSensorCycleCount = 0;
	uint8_t		magSensorCycleCount = 0;
	uint16_t	cycleTime;
	uint16_t	sensorReadTime;

	counters.sensorCycleCount	= 0;

	/* Sensors alrady initialized in main.c */

	imuInit();

//	flagSet(FLAG_CALIBRATE_ACC);
	flagSet(FLAG_CALIBRATE_GYRO);
	flagSet(FLAG_SMALL_ANGLES_25);

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
		// Measure loop rate just afer reading the sensors
		uint32_t sTime = micros();

		if (cfg.hil_mode == 0) {
			// Read gyro sensor data each cycle 10 ms
			gyroSensorUpdate();

			// Read accel sensor data each cycle
			accSensorUpdate();

			// Baromert update rate 50 ms
			if (++baroSensorCycleCount == 5)
			{
				baroSensorCycleCount = 0;
				baroSensorUpdate();
				getEstimatedAltitude();
			}

			// Magnitometer update rate 40 ms
			if (++magSensorCycleCount == 4)
			{
				magSensorCycleCount = 0;
				magSensorUpdate();
			}
		}

		// Time to read sensors
		sensorReadTime = (int32_t) (micros() - sTime);
		counters.sensorReadTime = counters.sensorReadTime * 0.99 + sensorReadTime * 0.01;
		debug[3] = counters.sensorReadTime;

		// TODO: Switch off IMU by cfg.hil_mode
		switch (cfg.imu_algorithm)
		{
		case 1:
			computeIMU();
			break;

		case 2:
			imuAHRSupdate();
			break;
	    }

		static uint32_t pTime;
		uint32_t cTime = micros();
		float dTime = (int32_t)(cTime - pTime) * 0.000001f;
		pTime = cTime;

		stabilize(dTime);

		mixerTable();

		mixerWrite();

		cycleTime = (int32_t) (micros() - sTime);
		counters.cycleTime = counters.cycleTime * 0.99 + cycleTime * 0.01;
		counters.sensorCycleCount++;

#if 0
		// Check if all the sensors are calibrated
		if (flag(FLAG_ACC_CALIBRATED) && flag(FLAG_GYRO_CALIBRATED) && flag(FLAG_MAG_CALIBRATED))
		{
			signalLED(LED_STS, LEDMODE_OFF);
		}
#endif
    	// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 10);	// Task cycle time 10 ms
    }
}

/*
 * Flight battery power sensor task
 */
portTASK_FUNCTION_PROTO(powerSensorTask, pvParameters)
{
	portTickType xLastWakeTime;
	static uint16_t fbatTmp = 0;
	static uint16_t vbatTmp = 0;
	static uint16_t ibatTmp = 0;
	static float ebatTmp = 0;
	static uint8_t buzzerCycleCount = 0;

	batteryInit();

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount ();

    while (1)
    {
		vbatTmp = vbatTmp * 0.9 + adcGetChannel(ADC_VIDEO_BATTERY) * 0.1;
		power_sensor.videoBatteryVoltage = ((vbatTmp * 3.3f) / 4095) * 60;

		fbatTmp = fbatTmp * 0.9 + adcGetChannel(ADC_VOLTAGE_SENSOR) * 0.1;

	    // calculate battery voltage based on ADC reading
	    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 4095 = 12bit adc, 60 = 6:1 voltage divider
		power_sensor.flightBatteryVoltage = ((fbatTmp * 3.3f) / 4095) * cfg.batvscale;

		if ((cfg.batalarm) && (power_sensor.flightBatteryVoltage < power_sensor.flightBatteryWarningVoltage))
		{
			if (buzzerCycleCount == 0)
			{
				buzzerPlay("1AX");			// VBAT warning
				buzzerCycleCount = 20;		// Repeat warning sound in 2 seconds
			}
			else
			{
				buzzerCycleCount --;
			}
		}

		// Consumed energy calculate
		ibatTmp = ibatTmp * 0.9 + adcGetChannel(ADC_CURRENT_SENSOR) * 0.1;

	    // calculate battery current based on ADC reading
	    // result is Ibatt in 1 mA steps. 3.3V = ADC Vref, 4095 = 12bit adc, 2:1 voltage divider
		power_sensor.flightBatteryCurrent = ((ibatTmp * 3.3f) / 4095) * cfg.batiscale - cfg.batioffset;	// Battery current in mA

		ebatTmp += power_sensor.flightBatteryCurrent / 36000.0f;
		power_sensor.flightBatteryConsumed = ebatTmp;						// Battery consumed energy in mAh

		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 100);		// Task cycle time 100 ms
    }
}

portTASK_FUNCTION_PROTO(sonarTask, pvParameters)
{
	portTickType xLastWakeTime;
	int16_t alt;

    hcsr04_init();
    sensorsSet(SENSOR_SONAR);

#if 0
	uint8_t i;

	// Prevent the first incorrect sonar measurement
	for (i = 0; i < 10; i++)
	{
		hcsr04_get_distance(25, &alt);
		vTaskDelay(60);						// Delay 60 ms
	}
#endif

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while (1)
    {
    	// The adjusted temperature accelerometer taken
		hcsr04_get_distance(acc_sensor.temp - 7, &alt);

		// Check sonar data & maximum attitude angles
		if (alt < 0 || alt > 250 || abs(angle[ROLL]) > 200 || abs(angle[PITCH]) > 200)
		{
			alt = -1;	// Bad value
		}

		alt_sensor.sonarAlt = alt;

//		debug[0] = alt;
//		debug[1] = (int16_t) (alt * cosf(DEG2RAD(angle[ROLL] / 10.f)) * cosf(DEG2RAD(angle[PITCH] / 10.f)));
// 		Bad idea

		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 60);	// Task cycle time 60 ms
    }
}
