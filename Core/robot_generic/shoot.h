#ifndef _SHOOT_H
#define _SHOOT_H

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE
#include "base_drv/rc.h"

#define SHOOT_MOTORS_HCAN hcan2

#define M2006_REDUCTION_RATIO 36
#define C_FRIC_WHEEL (0.069f*PI)
#define C_PUSHER_WHEEL (0.08f*PI)

#define DEFAULT_SHOOT_SPEED 15.0f
#define MAX_SHOOT_SPEED 35.0f

#define STUCK_MAX_TIME_CNT 250
#define STUCK_SPEED 5
#define STUCK_REVERSE_RPM -10
#define STUCK_REVERSE_TIME_CNT 100

#define PUSHER_WHEEL_SPEED 2000

#define SHOOT_SPEED_DEC 0.87f

void shoot_pid_init(void);
void shoot_control_loop(rc_ctrl_t* rc_recv);

#endif

