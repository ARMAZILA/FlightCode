
#include "main.h"
#include "MadgwickAHRS.h"

float accLPFVel[3];
int16_t BaroPID = 0;
static int32_t altitudeHold;
static int16_t errorAltitudeI = 0;
static int16_t acc_25deg = 0;
static float magneticDeclination = 0.0f; 	// in radiant. Filled at startup from config
static float accVelScale;
static uint8_t Smoothing[3] = { 0, 0, 0 };
float 	invG;
int32_t accSum[3];
uint32_t accTimeSum = 0;        		// keep track for integration of acc
int32_t	accSumCount = 0;

// **************
// Result IMU data
// **************
float   gyroData[3]  = { 0, 0, 0 };		// The angles of the gyroscope
float   accSmooth[3] = { 0, 0, 0 };		// The values of the accelerometr
int16_t angle[2] 	 = { 0, 0 };    	// Absolute angle inclination in multiple of 0.1 degree. ROLL: -1800...1800, PITCH -900..900.
float	angle_rad[2] = { 0.0f, 0.0f };
int16_t heading		 = 0;				// Magnetic cousre -180...+180 degrees
float	heading_rad	 = 0;
int32_t EstAlt;             			// Altitude in cm
int32_t vario = 0;                      // variometer in cm/s

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstM;
t_fp_vector EstG;

static void getEstimatedAttitude(void);

float InvSqrt(float x)
{
    union {
        int32_t i;
        float f;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int32_t isq(int32_t x)
{
    return x * x;
}

float fsq(float x)
{
    return x * x;
}

void imuInit(void)
{
    int16_t deg, min;

    acc_25deg = acc_1G * 0.423f;
    accVelScale = 9.80665f / acc_1G / 10000.0f;

    // Load from config and calculate magnetic declination
    deg = cfg.mag_declination / 100;
    min = cfg.mag_declination % 100;
    magneticDeclination = DEG2RAD((deg + ((float)min / 60.0f))); // Declination is in 1.00 rad units

    // initialize gyro smoothing
    Smoothing[ROLL ] = cfg.gyro_smooth[ROLL ];
    Smoothing[PITCH] = cfg.gyro_smooth[PITCH];
    Smoothing[YAW  ] = cfg.gyro_smooth[YAW  ];
}

void computeIMU(void)
{
    uint8_t axis;
    static float gyroYawSmooth = 0;
    static float gyroSmooth[3] = { 0, 0, 0 };

    getEstimatedAttitude();

    for (axis = 0; axis < 3; axis++) {
    	gyroData[axis] = gyro_sensor.data_deg[axis];
    }

    if (cfg.gyro_smoothing)
    {
        for (axis = 0; axis < 3; axis++)
        {
            gyroData[axis] = (((gyroSmooth[axis] * (Smoothing[axis] - 1)) + gyroData[axis] + 1 ) / Smoothing[axis]);
            gyroSmooth[axis] = gyroData[axis];
        }
    }
    else if (cfg.mixerConfiguration == MULTITYPE_TRI)
    {
        gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW]) / 3;
        gyroYawSmooth = gyroData[YAW];
    }
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Modified: 19/04/2011  by ziss_dm
// Version: V1.1
//
// code size deduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
// **************************************************

//******  advanced users settings *******************

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / ((float)cfg.gyro_cmpf_factor + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

#if INACCURATE
    v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
    v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
    v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
#else
    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(-delta[PITCH]);
    sinx = sinf(-delta[PITCH]);
    cosy = cosf(delta[ROLL]);
    siny = sinf(delta[ROLL]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    coszcosy = cosz * cosy;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
#endif
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

#define RADX10 (M_PI / 1800.0f)                  // 0.001745329252f

// rotate acc into Earth frame and calculate acceleration in it
void acc_calc(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    float rpy[3];
    t_fp_vector accel_ned;

    // the accel values have to be rotated into the earth frame
    rpy[0] = -angle_rad[ROLL];
    rpy[1] = -angle_rad[PITCH];
    rpy[2] = -heading_rad;

    accel_ned.V.X = accSmooth[0];
    accel_ned.V.Y = accSmooth[1];
    accel_ned.V.Z = accSmooth[2];

    rotateV(&accel_ned.V, rpy);

    if (1 /* cfg.acc_unarmedcal */ == 1) {
        if (!flag(FLAG_ARMED)) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    // apply Deadband to reduce integration drift and vibration influence
    accel_ned.V.Z = applyDeadband(accel_ned.V.Z, cfg.accz_deadband);
    accel_ned.V.X = applyDeadband(accel_ned.V.X, cfg.accz_deadband);
    accel_ned.V.Y = applyDeadband(accel_ned.V.Y, cfg.accz_deadband);

    // sum up Values for later integration to get velocity and distance
    accTimeSum += deltaT;
    accSumCount++;

    accSum[0] += accel_ned.V.X;
    accSum[1] += accel_ned.V.Y;
    accSum[2] += accel_ned.V.Z;
}

void accSum_reset(void)
{
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accSumCount = 0;
    accTimeSum = 0;
}

static void getEstimatedAttitude(void)
{
    uint8_t			axis;
	const float 	gyro_scale_us = M_PI / 180.0f / 1000000.0f;
    float			accMag = 0.0f;
    static float 	accLPF[3];
    float 			deltaGyroAngle[3];
    static uint32_t previousT;
    uint32_t 		currentT = micros();
    float 			dT;

    // Calculate dT
    dT = (currentT - previousT) * gyro_scale_us;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyro_sensor.data_deg[axis] * dT;
        if (cfg.acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / cfg.acc_lpf_factor)) + acc_sensor.data_mw[axis] * (1.0f / cfg.acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = acc_sensor.data_mw[axis];
        }
        accLPFVel[axis] = accLPFVel[axis] * (1.0f - (1.0f / cfg.acc_lpf_for_velocity)) + acc_sensor.data_mw[axis] * (1.0f / cfg.acc_lpf_for_velocity);
        accMag += fsq(accSmooth[axis]);
    }
    accMag = accMag * 100.0f / isq(acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);

    if (sensors(SENSOR_MAG))
        rotateV(&EstM.V, deltaGyroAngle);

    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
        flagSet(FLAG_SMALL_ANGLES_25);
    else
        flagClear(FLAG_SMALL_ANGLES_25);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < accMag && accMag < 133) {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)cfg.gyro_cmpf_factor + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    // Attitude of the estimated vector.
    float sqGZ = fsq(EstG.V.Z);
    float sqGX = fsq(EstG.V.X);
    float sqGY = fsq(EstG.V.Y);
    float sqGX_sqGZ = sqGX + sqGZ;
    float invmagXZ = InvSqrt(sqGX_sqGZ);
    invG = InvSqrt(sqGX_sqGZ + sqGY);

    taskENTER_CRITICAL();
    angle_rad[ROLL ] = atan2f(EstG.V.X, EstG.V.Z);
    angle_rad[PITCH] = atan2f(EstG.V.Y, invmagXZ * sqGX_sqGZ);
    angle[ROLL ] = RAD2DEG(angle_rad[ROLL ]) * 10.0f;
    angle[PITCH] = RAD2DEG(angle_rad[PITCH]) * 10.0f;
    taskEXIT_CRITICAL();

    if (sensors(SENSOR_MAG)) {
		for (axis = 0; axis < 3; axis++)
			EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + mag_sensor_data[axis]) * INV_GYR_CMPFM_FACTOR;

		// Heading calculation
        float rRAD = angle_rad[ROLL];
        float pRAD = -angle_rad[PITCH];
        float cr = cosf(rRAD);
        float sr = sinf(rRAD);
        float cp = cosf(pRAD);
        float sp = sinf(pRAD);
        float Xh = EstM.V.Y * cp + EstM.V.X * sr * sp + EstM.V.Z * cr * sp;
        float Yh = EstM.V.X * cr - EstM.V.Z * sr;

        taskENTER_CRITICAL();
        heading_rad = atan2f(-Yh, Xh) + magneticDeclination; 	// Add Magnetic Declination to the course
        heading = RAD2DEG(heading_rad);

        // Align to the value -180...180
        if (heading > 180)
            heading = heading - 360;
        else if (heading < -180)
            heading = heading + 360;
        taskEXIT_CRITICAL();
    }

    acc_calc(dT); // rotate acc vector into earth frame
}

// ****** find roll, pitch, yaw from quaternion ********
void Quaternion2RPY(const float q[4], float rpy[3])
{
	float R13, R11, R12, R23, R33;
	float q0s = q[0] * q[0];
	float q1s = q[1] * q[1];
	float q2s = q[2] * q[2];
	float q3s = q[3] * q[3];

	R13 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	R11 = q0s + q1s - q2s - q3s;
	R12 = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	R23 = 2.0f * (q[2] * q[3] + q[0] * q[1]);
	R33 = q0s - q1s - q2s + q3s;

	rpy[1] = asinf(-R13);	// pitch always between -pi/2 to pi/2
	rpy[2] = atan2f(R12, R11);
	rpy[0] = atan2f(R23, R33);

	//TODO: consider the cases where |R13| ~= 1, |pitch| ~= pi/2
}

void imuAHRSupdate()
{
    float q[4];
    float rpy[3];

    MadgwickAHRSsetSampleFreq(100);

    // Copy raw gyro & acc values to result variables
    taskENTER_CRITICAL();
    gyroData[ROLL ] = gyro_sensor.data_deg[ROLL ];
    gyroData[PITCH] = gyro_sensor.data_deg[PITCH];
    gyroData[YAW  ] = gyro_sensor.data_deg[YAW  ];

    accSmooth[ROLL]  = acc_sensor.data_mw[ROLL];
    accSmooth[PITCH] = acc_sensor.data_mw[PITCH];
    accSmooth[YAW]   = acc_sensor.data_mw[YAW];
    taskEXIT_CRITICAL();

    // Gyroscope units are radians/second, magnetometer units are Gaus,
    // accelerometer units are irrelevant as the vector is normalised.
    MadgwickAHRSupdate(
   		gyro_sensor.data_rad[ROLL],       gyro_sensor.data_rad[PITCH],     gyro_sensor.data_rad[YAW],
    	-acc_sensor.data_mss[PITCH],      acc_sensor.data_mss[ROLL],       acc_sensor.data_mss[YAW],
    	mag_sensor_data[PITCH] / 1000.0f, mag_sensor_data[ROLL] / 1000.0f, mag_sensor_data[YAW] / 1000.0f
    );

    MadgwickAHRSgetQuaternion(q);
    Quaternion2RPY(q, rpy);

    taskENTER_CRITICAL();
    angle_rad[ROLL ] = rpy[ROLL];
    angle_rad[PITCH] = rpy[PITCH];
    heading_rad      = rpy[YAW] + magneticDeclination;	    // Add Magnetic Declination to the course

    angle[ROLL ] = RAD2DEG(angle_rad[ROLL ]) * 10;
    angle[PITCH] = RAD2DEG(angle_rad[PITCH]) * 10;
    heading		 = RAD2DEG(heading_rad);

    // Align to the value -180...180
    if (heading > 180)
        heading = heading - 360;
    else if (heading < -180)
        heading = heading + 360;
    taskEXIT_CRITICAL();
}

#if 0
int16_t applyDeadband16(int16_t value, int16_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

float applyDeadbandFloat(float value, int16_t deadband)
{
    if (abs(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

/*
 *		Use Accel for altitide filtering
 *
 * 		http://habrahabr.ru/post/137595/
 */
int32_t filteredAltitude(void)
{
#define ACC_BARO_CMPF 300.0f
#define ACC_BARO_P 1.0f		// (cfg.P8[PIDALT] * 0.01f)		// 5
#define ACC_BARO_I 0.0001f	// (cfg.I8[PIDALT] * 0.00001f)	// 1
#define ACC_BARO_D 0.1f 	// (cfg.D8[PIDALT] * 0.001f)	// 30
#define VEL_SCALE ((1.0f - 1.0f / ACC_BARO_CMPF) / 1000000.0f)
#define ACC_SCALE  (9.80665f / acc_1G / 10000.0f)
#define DT 50000

    float errP;
    float accZ;
    static float errI = 0.0f;
    static float vel = 0.0f;
    static float alt = 0.0f;
    static uint8_t init_flag = 1;

    // First time
    if (init_flag)
    {
    	init_flag = 0;
    	alt = baroAlt;
    	return (int32_t) alt;
    }

    //debug[0] = BaroAlt;

    errP = (alt - baroAlt) / ACC_BARO_CMPF; 		// P term of error
    errI += errP * ACC_BARO_I; 						// I term of error
    //debug[1] = errI * 100;

    // invG calculated in getEstimatedAttitude()
    accZ = (accData[ROLL] * EstG.V.X + accData[PITCH] * EstG.V.Y + accData[YAW] * EstG.V.Z) * invG - acc_1G;
    //debug[2] = accZ * 100;

    accZ -= errI;

    // Integrator - velocity, cm/sec
    vel += (accZ - errP * ACC_BARO_P - vel * ACC_BARO_D) * DT * ACC_SCALE;
    //debug[3] = vel * 100;

    // Integrator - altitude, cm
    alt += vel * DT * VEL_SCALE - errP;

	return (int32_t) alt;
}

/*
 * Calculate complex altitude based on this sensors:
 *   sonarAlt
 *   GPS_altitude
 *   BaroAlt
 *
 */
int32_t complexAltitude(void)
{
    static float alt = 0.0f;
    static uint8_t init_flag = 10;
    int32_t BaroAltFiltered;

    BaroAltFiltered = filteredAltitude();
//    BaroAltFiltered = BaroAlt;

    // Init phase
    if (init_flag > 0)
    {
    	init_flag--;
    	BaroAltGround = BaroAltFiltered;
    }

    if (sensors(SENSOR_SONAR) && sonarAlt > 0)
    {
    	BaroAltGround = BaroAltFiltered - sonarAlt;
    	alt = sonarAlt;
    	return (int32_t) sonarAlt;
    }

    if (sensors(SENSOR_GPS) && flag(FLAG_GPS_FIX))
    {

    }

	alt = alt * 0.8 + (BaroAltFiltered - BaroAltGround) * 0.2;
    return (int32_t) alt;
}

void getEstimatedAltitude(void)
{
    int16_t error;
    int16_t accZ;
    static float vel = 0.0f;
    static int32_t lastAlt;
    float altVel;
    const uint32_t dTime = 50000;

    EstAlt = complexAltitude();

    // P
    error = constrain(altitudeHold - EstAlt, -300, 300);
    error = applyDeadband16(error, 10); // remove small P parametr to reduce noise near zero position
    BaroPID = constrain((cfg.P8[PIDALT] * error / 100), -150, +150);

    // I
    errorAltitudeI += error * cfg.I8[PIDALT] / 50;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    BaroPID += (errorAltitudeI / 500); // I in range +/-60

    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    // invG calculated in getEstimatedAttitude()
    accZ = (accLPFVel[ROLL] * EstG.V.X + accLPFVel[PITCH] * EstG.V.Y + accLPFVel[YAW] * EstG.V.Z) * invG - acc_1G; 
    accZ = applyDeadband16(accZ, acc_1G / cfg.accz_deadband);
    //debug[0] = accZ;

    // Integrator - velocity, cm/sec
    vel += accZ * accVelScale * dTime;

    altVel = (EstAlt - lastAlt) / (dTime / 1000000.0f);
    altVel = constrain(altVel, -300, 300); // constrain baro velocity +/- 300cm/s
    altVel = applyDeadbandFloat(altVel, 10); // to reduce noise near zero
    lastAlt = EstAlt;
    //debug[1] = altVel;

    // apply Complimentary Filter to keep near zero caluculated velocity based on altitude sensors velocity
    // cfg.baro_cf = 0.985f;
    vel = vel * cfg.baro_cf + altVel * (1.0f - cfg.baro_cf);
    // vel = constrain(vel, -300, 300); // constrain velocity +/- 300cm/s
    //debug[2] = vel;
    // debug[3] = applyDeadbandFloat(vel, 5);

    // D
    BaroPID -= constrain(cfg.D8[PIDALT] * applyDeadbandFloat(vel, 5) / 20, -150, 150);
    //debug[3] = BaroPID;
}
#endif


void setAltHold(int32_t newAltidude)
{
	altitudeHold = newAltidude;
	errorAltitudeI = 0;
	BaroPID = 0;
}

void getEstimatedAltitude(void)
{
    uint32_t dTime = 50000;
    int32_t error;
    int32_t baroVel;
    int32_t vel_tmp;
    static int32_t BaroAlt_fil;
    float vel_calc;
    static float vel = 0.0f;
    static float accAlt = 0.0f;
    static int32_t lastBaroAlt;

    BaroAlt_fil = BaroAlt_fil * cfg.baro_noise_lpf + alt_sensor.baroAlt * (1.0f - cfg.baro_noise_lpf); // additional LPF to reduce baro noise

    // Integrator - velocity, cm/sec
    vel_calc = (float) accSum[2] * accVelScale * (float) accTimeSum / (float) accSumCount;
    vel += vel_calc;

    // Integrator - Altitude in cm
    accAlt += vel * ((float) accTimeSum * 0.0000005f);                                  // integrate velocity to get distance (x= a/2 * t^2)
    accAlt = accAlt * cfg.baro_cf_alt + (float) BaroAlt_fil *(1.0f - cfg.baro_cf_alt);      // complementary filter for Altitude estimation (baro & acc)
    EstAlt = accAlt;

#if 1
    debug[0] = accSum[2] / accSumCount; // acceleration
    debug[1] = vel;                     // velocity
    debug[2] = accAlt;                  // height
#endif

    accSum_reset();

    //P
    error = constrain(altitudeHold - EstAlt, -300, 300);
    error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
    BaroPID = constrain((cfg.P8[PIDALT] * error / 128), -150, +150);

    //I
    errorAltitudeI += cfg.I8[PIDALT] * error / 64;
    errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
    BaroPID += errorAltitudeI / 512;     // I in range +/-60


    baroVel = (BaroAlt_fil - lastBaroAlt) * 1000000.0f / dTime;
    lastBaroAlt = BaroAlt_fil;

    baroVel = constrain(baroVel, -300, 300);    // constrain baro velocity +/- 300cm/s
    baroVel = applyDeadband(baroVel, 10);       // to reduce noise near zero

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * cfg.baro_cf_vel + baroVel * (1 - cfg.baro_cf_vel);

    // D
    vel_tmp = vel;
    vel_tmp = applyDeadband(vel_tmp, 5);
    vario = vel_tmp;
    BaroPID -= constrain(cfg.D8[PIDALT] * vel_tmp / 16, -150, 150);

    return;
}
