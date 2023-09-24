//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef _IMU_ALGO_H
#define _IMU_ALGO_H

typedef struct{	
	float gx; float gy; float gz;
	float ax; float ay; float az;
}imu_data_fp_t;

typedef struct{	
	signed short gx, gy, gz;
	signed short ax, ay, az;
	float g_fullscale;
	float a_fullscale;
}imu_data_raw_t;

typedef struct{
	float q0; float q1; float q2; float q3;
}quaternion_fp_t;

//0-8192 angle, motor CCW for postive
typedef struct{
	float pitch;
	float roll;
	float yaw;
}eular_t;

//---------------------------------------------------------------------------------------------------
// Function declarations

void reset_imu_fusion(void);
void get_imu_eular(eular_t* eular_out);

void madgwick_imu(imu_data_fp_t* _imu, float dt);
void mahony_imu(imu_data_fp_t* _imu, float dt);


#endif
//=====================================================================================================
// End of file
//=====================================================================================================
