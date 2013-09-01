/*
 * stabilize.c
 *
 *  Created on: 19.08.2013
 *
 */

#include "main.h"

//
// Calculates all axis PIDs, than call mixer
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
	float   PTermACC, ITermACC = 0;
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
			errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -500, +500) - angle[axis] + cfg.angleTrim[axis];
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

	mixTable();
	writeServos();
	writeMotors();
}
