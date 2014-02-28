/*
 * AltFilter.c
 *
 *  Created on: 19.02.2014
 *      Author: Алексей Молчанов
 */

#include "math.h"
float  xhat1,xhat2,xhat3;
float P11, P12, P13, P22, P23, P33;// P21, P31, P32,
float Alt_prev;
static unsigned char filterstart = 0;

void  AltFilterInit(float dt, float accnoice, float Alt)
{
		float dt2 = dt*dt;
	    float dt3 = dt2*dt;
	    float dt4 = dt2*dt2;

		P11 = accnoice*dt4/4;
	    P12 = P13 = accnoice*dt3/3;// = P21 = P31
	    P22 = P23 = P33 = accnoice*dt2;// = P32

	    // Consistency check
	    if(isnan (Alt))
	    	xhat1 = 0;
	    else
	    	xhat1 = Alt;
	    xhat2 = xhat3 = 0;
	    Alt_prev = Alt;

	    filterstart = 1;
}

float AltFilterStep(float dt, float accnoice, float altnoice, float Alt, float Acc)
{
	if (!filterstart)
	{
		AltFilterInit(dt, accnoice, Alt);
		filterstart = 1;
		return Alt;
	}

	float A11, A12, A13, A21, A22, A23, A31, A32, A33;
	float Q11, Q12, Q13, Q22, Q23, Q33;// Q21, Q31, Q32,
	float T11, T12, T13, T22, T23, T33;// T21, T31, T32,
	float K1,K2,K3;
    float T1 = xhat1;
    float T2 = xhat2;
    float T3 = xhat3;
    float R = altnoice*altnoice;
    float Inn;


    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt2*dt2;
    float G1 = dt2/2;
    float G2 = dt;
    float G3 = 0;

    // transition matrix
    A11 = A22 = A33 = 1;
    A12 = A23 = dt;
    A13 = dt2/2;
    A21 = A31 = A32 = 0;


    Q11 = accnoice*dt4/4;
    Q12 = Q13 = accnoice*dt3/3;// Q21 = Q31 =
    Q22 = Q23 = Q33 = accnoice*dt2;// = Q32


    xhat1 = A11*T1 + A12*T2 + A13*T3 + G1*(Acc);
    xhat2 = A21*T1 + A22*T2 + A23*T3 + G2*(Acc);
    xhat3 = A31*T1 + A32*T2 + A33*T3 + G3*(Acc);

  //  if( fabs(Alt - Alt_prev) > 0.001) //Alt updated
    {
    	Alt_prev = Alt;
		// Innovation
		Inn = Alt - xhat1;
		if (isnan (Inn))
			Inn = 0;
		// Covariance of Innovation
		//s = c * P * c' + R;
		float s = P11 + R;
		// Gain matrix
		//K = a * P * c' * inv(s);

		K1 =  A11*P11 + A12*P12 + A13*P13;
		K1 = K1/s;
		K2 =  A21*P11 + A22*P12 + A23*P13;
		K2 = K2/s;
		K3 =  A31*P11 + A32*P12 + A33*P13;
		K3 = K3/s;


		// State estimate
		//xhat = xhat + K * Inn;
		xhat1 = xhat1 + K1 * Inn;
		xhat2 = xhat2 + K2 * Inn;
		xhat3 = xhat3 + K3 * Inn;
		// Covariance of prediction error
		//P = a * P * a' + Q - a * P * c' * inv(s) * c * P * a';

		T11 = P11 + A12*A12*P22 + A13*A13*P33 + A12*P12 + A13*P13 + A12*P12 + A13*P13 + A12*A13*P23 + A12*A13*P23;
		   T12 = P12 + A12*P22 + A23*P13 + A13*P23 + A12*A23*P23 + A13*A23*P33;
		   T13 = P13 + A12*P23 + A13*P33;
		//T21 = P21 + A12*P22 + A13*P23 + A23*P31 + A12*A23*P32 + A13*A23*P33;
		   T22 = P22 + A23*A23*P33 + A23*P23 + A23*P23;
		   T23 = P23 + A23*P33;
		//T31 = P31 + A12*P32 + A13*P33;
		   //T32 = P32 + A23*P33;
		   T33 = P33;

		   T11 =   T11 + Q11 - 1.f/s*(P11*P11 + A12*P11*P12 + A13*P11*P13 + A12*P11*P12 + A13*P11*P13 + A12*A12*P12*P12 + A13*A13*P13*P13 + A12*A13*P13*P12 + A12*A13*P12*P13);
			 T12 = T12 + Q12 - 1.f/s*(P11*P12 + A12*P12*P12 + A23*P11*P13 + A13*P12*P13 + A12*A23*P13*P12 + A13*A23*P13*P13);
			 T13 = T13 + Q13 - 1.f/s*(P11*P13 + A12*P13*P12 + A13*P13*P13);
		   //T21 =   T21 + Q21 - 1/s*(P11*P21 + A12*P12*P21 + A13*P13*P21 + A23*P11*P31 + A12*A23*P12*P31 + A13*A23*P13*P31);
			 T22 = T22 + Q22 - 1.f/s*(P12*P12 + A23*P13*P12 + A23*P12*P13 + A23*A23*P13*P13);
			 T23 = T23 + Q23 - 1.f/s*(P13*P12 + A23*P13*P13);
		   //T31 =   T31 + Q31 - 1/s*(P11*P31 + A12*P12*P31 + A13*P13*P31);
			 //T32 = T32 + Q32 - 1/s*(P12*P31 + A23*P13*P31);
			 T33 = T33 + Q33 - 1.f/s*(P13*P13);

		P11 = T11;
		P12 = T12;
		P13 = T13;
		//P21 = T12;
		P22 = T22;
		P23 = T23;
		//P31 = T13;
	   // P32 = T23;
		P33 = T33;
    }

    // Consistency check
    if(isnan (xhat1)||isnan (xhat2)||isnan (xhat3))
    {
    	if (isnan(Alt)) // Return to alive
    		xhat1 = 0;
    	else
    		xhat1 = Alt;
    	xhat2 = 0;
    	xhat3 = 0;
    }
    return xhat1; // return Altitude
}

float AltFilterGetAlt(void)
{
	return xhat1;
}

float AltFilterGetVario(void)
{
	return xhat2;
}
