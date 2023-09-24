#ifndef _LINEAR_MOTION_H
#define _LINEAR_MOTION_H

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "algorithm/pid.h"
#include "base_drv/motor_ctrl.h"

#define RAW_ANGLE_RANGE 8192

//-----------------------------------------------------------------
//linear motion motors configure
#define LINEAR_MOTION_MOTORS_NUM 2u
#define LINEAR_MOTORS_HCAN hcan1

#define LIFT_MOTOR_LEFT_ID 0
#define LIFT_MOTOR_RIGHT_ID 0


#define MOVEING_RPM 40
#define MIN_ERR 20


typedef struct{
	motor_response_msg_t* motor;
	uint16_t last_raw_ang;
	int32_t current_pos;
	int32_t MAX_RANGE;
} linear_motion;

typedef enum{
	RESETING, RESETED, STUCKED, READY
} motion_state;

#endif

