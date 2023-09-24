#ifndef _MOTOR_TESTS_H
#define _MOTOR_TESTS_H

#include "../base_drv/drv_conf.h"
#include HAL_INCLUDE

#define RPM_THRESOLD 10

void identify_motors(void);
void motor_spin(int16_t rpm_ref);
void motor_set_angle(uint16_t ang_ref);
void motor_vibrate(void);

#endif

