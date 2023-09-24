#include "chassis_base.h"

#if MOTOR_CAN_ENABLE == 1

#include "../algorithm/filter.h"
#include "motor_ctrl.h"

#include <math.h>
#include <string.h>

extern CAN_HandleTypeDef CHASSIS_MOTORS_HCAN;


//inline float fabsf(float t){return (t>0)?t:(-t);}
//inline int16_t abs(int16_t t){return (t>0)?t:(-t);}

/**
  * @brief mecanum forward calcuation
  * @param[in]  s_wheel     pointer to chassis_wheels_state_t (4 wheel rpm)
  * @return chassis_state_t including v_x,v_y,w_z
  */
/*麦克纳姆轮运动学正向解算，即从四轮轮速换算到整车速度*/
chassis_state_t mecanum_forward(chassis_wheels_state_t* s_wheel){
	chassis_state_t s_chassis;
	s_chassis.v_x = C_WHEEL*(s_wheel->M_LF - s_wheel->M_RF + s_wheel->M_LB - s_wheel->M_RB)/4 /60;
	s_chassis.v_y = C_WHEEL*(KXY_BACK*s_wheel->M_RF + KXY_BACK*s_wheel->M_LF
					-KXY_FRONT*s_wheel->M_LB - KXY_FRONT*s_wheel->M_RB)/(2*(KXY_BACK+KXY_FRONT))/60;
	s_chassis.w_z = C_WHEEL*(s_wheel->M_RF + s_wheel->M_LF + s_wheel->M_LB + s_wheel->M_RB)/(2*(KXY_BACK+KXY_FRONT))/60;
	return s_chassis;
}

/**
  * @brief mecanum reverse calcuation
  * @param[in]  s_chassis    pointer to chassis_state_t (v_x,v_y,w_z)
  * @return chassis_wheels_state_t (4 wheels speed)
  */
/*麦克纳姆轮运动学逆向解算，即从整车速度换算到四轮轮速*/
chassis_wheels_state_t mecanum_reverse(chassis_state_t* s_chassis){
	chassis_wheels_state_t s_wheel;
	s_wheel.M_RF = (60/C_WHEEL)*((-s_chassis->v_x + s_chassis->v_y) + KXY_FRONT * s_chassis->w_z);
	s_wheel.M_LF = (60/C_WHEEL)*(( s_chassis->v_x + s_chassis->v_y) + KXY_FRONT * s_chassis->w_z);
	s_wheel.M_LB = (60/C_WHEEL)*(( s_chassis->v_x - s_chassis->v_y) + KXY_BACK *  s_chassis->w_z);
	s_wheel.M_RB = (60/C_WHEEL)*((-s_chassis->v_x - s_chassis->v_y) + KXY_BACK *  s_chassis->w_z);

	return s_wheel;
}

#if USE_SWERVE_CHASSIS == 1
typedef struct{ float x; float y; }vector_t;
//舵轮的实验性代码
void swerve_reverse(chassis_state_t* s_chassis, chassis_wheels_state_t* s_wheel, chassis_wheels_dir_t* s_dir){
	vector_t vec_rf,vec_lf,vec_lb,vec_rb;
	vector_t vec_wz;
	vec_wz.x = K_SWERVE*cosf(SWERVE_SPIN_ANG*PI/180) * s_chassis->w_z;
	vec_wz.y = K_SWERVE*sinf(SWERVE_SPIN_ANG*PI/180) * s_chassis->w_z;


	vec_rf.x = s_chassis->v_x - vec_wz.x;
	vec_rf.y = s_chassis->v_y - vec_wz.y;

	vec_lf.x = s_chassis->v_x + vec_wz.x;
	vec_lf.y = s_chassis->v_y - vec_wz.y;

	vec_lb.x = s_chassis->v_x + vec_wz.x;
	vec_lb.y = s_chassis->v_y + vec_wz.y;

	vec_rb.x = s_chassis->v_x - vec_wz.x;
	vec_rb.y = s_chassis->v_y + vec_wz.y;


	s_dir->DIR_RF = atan2f(vec_rf.y,vec_rf.x)*ANGLE_RANGE/PI;
	s_dir->DIR_RF = (s_dir->DIR_RF + ANGLE_RANGE) % ANGLE_RANGE;

	s_dir->DIR_LF = atan2f(vec_lf.y,vec_lf.x)*ANGLE_RANGE/PI;
	s_dir->DIR_LF = (s_dir->DIR_LF + ANGLE_RANGE) % ANGLE_RANGE;

	s_dir->DIR_LB = atan2f(vec_lb.y,vec_lb.x)*ANGLE_RANGE/PI;
	s_dir->DIR_LB = (s_dir->DIR_LB + ANGLE_RANGE) % ANGLE_RANGE;

	s_dir->DIR_RB = atan2f(vec_rb.y,vec_rb.x)*ANGLE_RANGE/PI;
	s_dir->DIR_RB = (s_dir->DIR_RB + ANGLE_RANGE) % ANGLE_RANGE;


	s_wheel->M_RF = sqrtf(vec_rf.x*vec_rf.x + vec_rf.y*vec_rf.y)*60/C_WHEEL;
	s_wheel->M_LF = sqrtf(vec_rf.x*vec_lf.x + vec_rf.y*vec_lf.y)*60/C_WHEEL;
	s_wheel->M_LB = sqrtf(vec_rf.x*vec_lb.x + vec_rf.y*vec_lb.y)*60/C_WHEEL;
	s_wheel->M_RB = sqrtf(vec_rf.x*vec_rb.x + vec_rf.y*vec_rb.y)*60/C_WHEEL;
}

void swerve_forward(chassis_wheels_state_t* s_wheel, chassis_wheels_dir_t* s_dir, chassis_state_t* s_chassis){
	vector_t vec_rf,vec_lf,vec_lb,vec_rb;
	vector_t unit_vec_wz;
	unit_vec_wz.x = cosf(SWERVE_SPIN_ANG*PI/180);
	unit_vec_wz.y = sinf(SWERVE_SPIN_ANG*PI/180);

	vec_rf.x = s_wheel->M_RF*cosf(s_dir->DIR_RF*PI/(ANGLE_RANGE/2))*C_WHEEL/60;
	vec_rf.y = s_wheel->M_RF*sinf(s_dir->DIR_RF*PI/(ANGLE_RANGE/2))*C_WHEEL/60;

	vec_lf.x = s_wheel->M_LF*cosf(s_dir->DIR_LF*PI/(ANGLE_RANGE/2))*C_WHEEL/60;
	vec_lf.y = s_wheel->M_LF*sinf(s_dir->DIR_LF*PI/(ANGLE_RANGE/2))*C_WHEEL/60;

	vec_lb.x = s_wheel->M_LB*cosf(s_dir->DIR_LB*PI/(ANGLE_RANGE/2))*C_WHEEL/60;
	vec_lb.y = s_wheel->M_LB*sinf(s_dir->DIR_LB*PI/(ANGLE_RANGE/2))*C_WHEEL/60;

	vec_rb.x = s_wheel->M_RB*cosf(s_dir->DIR_RB*PI/(ANGLE_RANGE/2))*C_WHEEL/60;
	vec_rb.y = s_wheel->M_RB*sinf(s_dir->DIR_RB*PI/(ANGLE_RANGE/2))*C_WHEEL/60;

	//TODO: vector to chassis state
	s_chassis->v_x = (vec_rf.x + vec_lf.x + vec_lb.x + vec_rb.x)/4;
	s_chassis->v_y = (vec_rf.y + vec_lf.y + vec_lb.y + vec_rb.y)/4;
	s_chassis->w_z = (vec_rf.x * unit_vec_wz.x + vec_rf.y * unit_vec_wz.y)/K_SWERVE*60.0f/(2*PI);

}

#endif

/**
  * @brief update the global varible using motor_data with IIR filter
  * @param[in]  motor_data     pointer to the motor_response_msg_t of 4 wheels,
  			               id and position should match
  * @param[out]  current_wheels pointer to where the caller intended to store the result
  							in the type chassis_wheels_state_t
  * @return void
  */
/*电机数据滤波*/
void update_wheels_state(motor_response_msg_t* motor_data, chassis_wheels_state_t* current_wheels){
	current_wheels->M_RF = iir_filter_3((float)motor_data[MOTOR_RF_ID].speed_rpm/REDUCTION_RATIO,MOTOR_RF_ID);
	current_wheels->M_LF = iir_filter_3((float)motor_data[MOTOR_LF_ID].speed_rpm/REDUCTION_RATIO,MOTOR_LF_ID);
	current_wheels->M_LB = iir_filter_3((float)motor_data[MOTOR_LB_ID].speed_rpm/REDUCTION_RATIO,MOTOR_LB_ID);
	current_wheels->M_RB = iir_filter_3((float)motor_data[MOTOR_RB_ID].speed_rpm/REDUCTION_RATIO,MOTOR_RB_ID);
}


/**
  * @brief set 4 wheels output simutaniously
  * @param  s_wheel: pointer to chassis_wheels_state_t (4 wheel output)
  * @param  max_out: total maximum output, set to >= 4*C620_OUTPUT_MAX to disable
  * @return HAL_OK if transmission was success otherwise HAL_ERROR
  * @attention make sure to limit the output value in (-30000,30000)
  */
/*带功率限制的底盘四轮直接输出函数，直接限制输出峰值*/
HAL_StatusTypeDef chassis_output_ctrl(chassis_wheels_state_t* s_wheel, float max_out){
	motor_transmit_msg_t tx_msg;
	float total_output, limit_ratio;
	
	memset(tx_msg.D,0,sizeof(motor_transmit_msg_t));
	
	LIMIT_MIN_MAX(s_wheel->M_LF,-C620_OUTPUT_MAX, C620_OUTPUT_MAX); 
	LIMIT_MIN_MAX(s_wheel->M_RF,-C620_OUTPUT_MAX, C620_OUTPUT_MAX);
	LIMIT_MIN_MAX(s_wheel->M_LB,-C620_OUTPUT_MAX, C620_OUTPUT_MAX);
	LIMIT_MIN_MAX(s_wheel->M_RB,-C620_OUTPUT_MAX, C620_OUTPUT_MAX);
	total_output = fabsf(s_wheel->M_LF) + fabsf(s_wheel->M_RF) + fabsf(s_wheel->M_LB) + fabsf(s_wheel->M_RB);
	if(total_output > max_out){
		limit_ratio = max_out / total_output;
		s_wheel->M_LF *= limit_ratio; s_wheel->M_RF *= limit_ratio;
		s_wheel->M_LB *= limit_ratio; s_wheel->M_RB *= limit_ratio;
	}
	tx_msg.D[MOTOR_LF_ID] = (int16_t)s_wheel->M_LF; tx_msg.D[MOTOR_RF_ID] = (int16_t)s_wheel->M_RF;
	tx_msg.D[MOTOR_LB_ID] = (int16_t)s_wheel->M_LB; tx_msg.D[MOTOR_RB_ID] = (int16_t)s_wheel->M_RB;
	return set_motor_output(&(CHASSIS_MOTORS_HCAN),&tx_msg,C620_ID_BASE);
}

/**
  * @brief chassis motion control process
  * @param[in]  pPID           pointer to the pid_struct_t of 4 wheels speed
  * @param[in]  current_state  pointer to chassis_wheels_state_t (4 wheel speed)
  * @param[in]  ref_state      expected v_x,v_y,w_z in chassis_state_t
  * @param[in]  max_out        total maximum output, set to >= 4*C620_OUTPUT_MAX to disable
  * @return HAL_OK if transmission was success otherwise HAL_ERROR
  * @attention make sure to limit the output value in (-30000,30000)
  */
/*带功率限制的四轮PID控制底盘*/
HAL_StatusTypeDef chassis_pid_ctrl(pid_struct_t pPID[4],
					  chassis_wheels_state_t* current_state, 
					  chassis_state_t* ref_state,
						chassis_power_lim_t* pow_lim){
	motor_transmit_msg_t tx_msg;
	float total_output, limit_ratio,tmp[4];
	
	memset(tx_msg.D,0,sizeof(motor_transmit_msg_t));

	chassis_wheels_state_t wheels_ref = mecanum_reverse(ref_state);

	tmp[MOTOR_RF_ID] = pid_calc(&pPID[MOTOR_RF_ID], wheels_ref.M_RF,current_state->M_RF);
	tmp[MOTOR_LF_ID] = pid_calc(&pPID[MOTOR_LF_ID], wheels_ref.M_LF,current_state->M_LF);
	tmp[MOTOR_LB_ID] = pid_calc(&pPID[MOTOR_LB_ID], wheels_ref.M_LB,current_state->M_LB);
	tmp[MOTOR_RB_ID] = pid_calc(&pPID[MOTOR_RB_ID], wheels_ref.M_RB,current_state->M_RB);

	total_output = fabsf(tmp[MOTOR_RF_ID]) + fabsf(tmp[MOTOR_LF_ID]) 
				+ fabsf(tmp[MOTOR_LB_ID]) + fabsf(tmp[MOTOR_RB_ID]);
	if(total_output > pow_lim->max_total_out){
		limit_ratio = pow_lim->max_total_out / total_output;
	}else{
		limit_ratio = 1.0f;
	}
	if(pow_lim->vxy_ratio < 1) pow_lim->vxy_ratio = 1;
	
	tx_msg.D[MOTOR_RF_ID] = (int16_t)((tmp[MOTOR_RF_ID] * limit_ratio) / pow_lim->vxy_ratio);
	tx_msg.D[MOTOR_LF_ID] = (int16_t)((tmp[MOTOR_LF_ID] * limit_ratio) / pow_lim->vxy_ratio);
	tx_msg.D[MOTOR_LB_ID] = (int16_t)((tmp[MOTOR_LB_ID] * limit_ratio) / pow_lim->vxy_ratio);
	tx_msg.D[MOTOR_RB_ID] = (int16_t)((tmp[MOTOR_RB_ID] * limit_ratio) / pow_lim->vxy_ratio);

	return set_motor_output(&(CHASSIS_MOTORS_HCAN),&tx_msg,C620_ID_BASE);
	
}

/**
  * @brief  chassis motion control process in another algorithm
  * @param[in]  pPID           pointer to the pid_struct_t of 3 chassis state speed
  * @param[in]  current_state  pointer to chassis_wheels_state_t (4 wheel speed)
  * @param[in]  ref_state      expected v_x,v_y,w_z in chassis_state_t
  * @param[in]  pow_lim        power limit parameters
  * @return HAL_OK if transmission was success otherwise HAL_ERROR
  * @attention make sure to limit the output value in (-30000,30000)
  */
/*带功率限制的车速PID控制底盘*/
HAL_StatusTypeDef chassis_pid_revctrl(pid_struct_t pPID[3],
					  chassis_wheels_state_t* current_state, 
					  chassis_state_t* ref_state, 
					  chassis_power_lim_t* pow_lim){

	chassis_wheels_state_t wheels_output;
	chassis_state_t motion_output, motion_current;

	motion_current = mecanum_forward(current_state);
	
	pPID[0].out_max = pow_lim->vxy_output_max;
	pPID[1].out_max = pow_lim->vxy_output_max;
	pPID[2].out_max = pow_lim->wz_output_max;
						  
	motion_output.v_x = (int16_t)pid_calc_deadband(&pPID[0], ref_state->v_x,motion_current.v_x);
	motion_output.v_y = (int16_t)pid_calc_deadband(&pPID[1], ref_state->v_y,motion_current.v_y);
	motion_output.w_z = (int16_t)pid_calc(&pPID[2], ref_state->w_z,motion_current.w_z);

	if(pow_lim->vxy_ratio < 1) pow_lim->vxy_ratio = 1;
	if(pow_lim->wz_ratio < 1) pow_lim->wz_ratio = 1;

	motion_output.v_x /= pow_lim->vxy_ratio;
	motion_output.v_y /= pow_lim->vxy_ratio;
	motion_output.w_z /= pow_lim->wz_ratio;

	wheels_output = mecanum_reverse(&motion_output);
	return chassis_output_ctrl(&wheels_output, pow_lim->max_total_out);

}

#endif
