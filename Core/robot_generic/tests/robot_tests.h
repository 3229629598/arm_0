#ifndef _CHASSIS_TEST_H
#define _CHASSIS_TEST_H

#include "../algorithm/pid.h"
#include "../algorithm/util.h"
#include "../base_drv/chassis_base.h"

void init_robot_test(void);
void update_chassis_test_data(void);

void shoot_check(void);
void gimbal_range_check(void);

 HAL_StatusTypeDef chassis_advance_ctrl(pid_struct_t pPID[3],
 					  chassis_wheels_state_t* current_state, 
 					  chassis_state_t* ref_state, 
 					  chassis_power_lim_t* pow_lim);

#endif

