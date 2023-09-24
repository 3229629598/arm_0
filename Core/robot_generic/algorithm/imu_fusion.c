//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "imu_fusion.h"
#include "util.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define BETA		0.1f		// 2 * proportional gain
#define TWOKP	(2.0f * 3.0f)	// 2 * proportional gain
#define TWOKI	(2.0f * 0.03f)	// 2 * integral gain

#define M_PI 3.14159265358979323846f
#define HALF_MAX_ANGLE 4096

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//---------------------------------------------------------------------------------------------------
// Variable definitions of madgwick
volatile float beta = BETA;								// 2 * proportional gain (Kp)

//---------------------------------------------------------------------------------------------------
// Variable definitions of mahony

volatile float twoKp = TWOKP;											// 2 * proportional gain (Kp)
volatile float twoKi = TWOKI;											// 2 * integral gain (Ki)
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------


//====================================================================================================
// Functions

void reset_imu_fusion(void){
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

void get_imu_eular(eular_t* eular_out){
	eular_out->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)*HALF_MAX_ANGLE/M_PI;	// pitch
	eular_out->roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)*HALF_MAX_ANGLE/M_PI;	// roll
	eular_out->yaw   = atan2(2* (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)*HALF_MAX_ANGLE/M_PI;	//yaw
}



//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void madgwick_imu(imu_data_fp_t* _imu, float dt){
	float recipNorm;
	float s0, s1, s2, s3;
	float ax, ay, az;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * _imu->gx - q2 * _imu->gy - q3 * _imu->gz);
	qDot2 = 0.5f * (q0 * _imu->gx + q2 * _imu->gz - q3 * _imu->gy);
	qDot3 = 0.5f * (q0 * _imu->gy - q1 * _imu->gz + q3 * _imu->gx);
	qDot4 = 0.5f * (q0 * _imu->gz + q1 * _imu->gy - q2 * _imu->gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((_imu->ax == 0.0f) && (_imu->ay == 0.0f) && (_imu->az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = fast_inv_sqrt(_imu->ax * _imu->ax + _imu->ay * _imu->ay + _imu->az * _imu->az);
		ax = recipNorm * _imu->ax;
		ay = recipNorm * _imu->ay;
		az = recipNorm * _imu->az;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = fast_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void mahony_imu(imu_data_fp_t* _imu, float dt){
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float ax, ay, az, gx, gy, gz;

	ax = _imu->ax; ay = _imu->ay; az = _imu->az;
	gx = _imu->gx; gy = _imu->gy; gz = _imu->gz;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


//====================================================================================================
// END OF CODE
//====================================================================================================
