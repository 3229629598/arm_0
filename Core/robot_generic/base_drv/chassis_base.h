
#ifndef _CHASSIS_BASE_H
#define _CHASSIS_BASE_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if MOTOR_CAN_ENABLE == 1

#include "../algorithm/pid.h"
#include "motor_ctrl.h"

/**
  * @struct chassis_state_t
  * @brief speed of chassis motion, v_x,v_y in m/s,w_z in rpm
  */
/*整车底盘状态结构体：X轴速度，Y轴速度，Z轴速度*/
typedef struct{
	float v_x; ///< velocity of x axis(by default it is the forward direction) in m/s
	float v_y; ///< velocity of y axis(by default it is the right direction) in m/s
	float w_z; ///< angular velocity of z axis(by default it is CW direction) in rad/s
} chassis_state_t;

/**
  * @struct chassis_wheels_dir_t
  * @brief direction of 4 wheels
  */
typedef struct{
	int16_t DIR_RF; ///< motor of the right front
	int16_t DIR_LF; ///< motor of the left front
	int16_t DIR_LB; ///< motor of the left back
	int16_t DIR_RB; ///< motor of the right back
}chassis_wheels_dir_t;

/**
  * @struct chassis_wheels_state_t
  * @brief speed of 4 wheels in rpm or 4 wheels outputs
  */
/*电机四轮转速结构体*/
typedef struct{
	float M_RF; ///< motor of the right front
	float M_LF; ///< motor of the left front
	float M_LB; ///< motor of the left back
	float M_RB; ///< motor of the right back
}chassis_wheels_state_t;

/**
  * @struct chassis_power_lim_t
  * @brief chassis power limit parameter
  */
/*功率限制参数结构体*/
typedef struct{
	float max_total_out;	///< total maximum output of motor,set to >= 4*C620_OUTPUT_MAX to disable
	float vxy_ratio;		///< power reduction ratio of vx and vy output
	float vxy_output_max;	///< maximum output of vx or vy
	float wz_ratio;			///< power reduction ratio of wz output(unused in chassis_pid_ctrl)
	float wz_output_max;	///< maximum output of wz(unused in chassis_pid_ctrl)
}chassis_power_lim_t;


/*
	mecanum wheels
	L=left,R=right,F=front,B=back
	motor id & position
	1   0
	2   3  
*/
chassis_state_t mecanum_forward(chassis_wheels_state_t* s_wheel);
chassis_wheels_state_t mecanum_reverse(chassis_state_t* s_chassis);

void update_wheels_state(motor_response_msg_t* motor_data, chassis_wheels_state_t* current_wheels);

HAL_StatusTypeDef chassis_pid_ctrl(pid_struct_t pPID[4],
					  chassis_wheels_state_t* current_state, 
					  chassis_state_t* ref_state,
						chassis_power_lim_t* pow_lim);

HAL_StatusTypeDef chassis_pid_revctrl(pid_struct_t pPID[3],
					  chassis_wheels_state_t* current_state, 
					  chassis_state_t* ref_state, 
					  chassis_power_lim_t* pow_lim);

HAL_StatusTypeDef chassis_output_ctrl(chassis_wheels_state_t* s_wheel, float max_out);

#endif

#endif
