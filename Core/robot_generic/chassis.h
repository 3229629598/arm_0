#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "base_drv/rc.h"
#include "base_drv/motor_ctrl.h"

#define S_CURVE_VX_ACC 1.2f
#define S_CURVE_VY_ACC 0.9f

#define MAX_VX_SPEED 2.5f
#define MAX_VY_SPEED 2.0f
#define CONSTANT_SPINNING_SPEED 30.0f

#define SHUT_DEADBAND 500

void chassis_init(void);
void chassis_control_loop(rc_ctrl_t* rc_recv);

void reset_chassis_angle(void);
int16_t get_gimbal_angle(motor_response_msg_t chassis_motors[4] ,motor_response_msg_t* gimbal_motor);

float get_current_chassis_wz(void);

#endif

