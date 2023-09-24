#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "base_drv/rc.h"

//-----------------------------------------------------------------
//gimbal parameter configure
#define GIMBAL_MOTORS_HCAN hcan2

#define GIMBAL_PITCH_MAX 4642
#define GIMBAL_PITCH_MIN 3231
//#define GIMBAL_YAW_MAX 8000
//#define GIMBAL_YAW_MIN 100

#define GIMBAL_PITCH_CENTER 4096
#define GIMBAL_YAW_CENTER 4096

#define SOFT_START_DEADBAND 80
#define SOFT_START_SPEED 1.5
#define SOFT_START_MAX_CNT 4000

#define GIMBAL_MOVE_SPEED 6.5f

typedef enum{
	GIMBAL_UNRESET = 0,
	GIMBAL_RESETING,
	GIMBAL_RESETED,
	GIMBAL_MOTOR_CTRL,
	GIMBAL_IMU_CTRL
}gimbal_state_t;

uint32_t is_gimbal_reset(void);

void gimbal_init(uint8_t reset_all_flag);
void gimbal_control_loop(rc_ctrl_t* rc_recv);

#endif

