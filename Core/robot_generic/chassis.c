#include "chassis.h"

#include "base_drv/drv_conf.h"
#include "base_drv/chassis_base.h"
#include "base_drv/referee.h"

#include "tests/robot_tests.h"

#include "pid_parameters.h"
#include "power_ctrl.h"
#include "gimbal.h"

#include "../Src/mpu_imu/bsp_imu.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

chassis_wheels_state_t current_wheels;
chassis_power_lim_t chassis_power_lim;


static pid_struct_t wheels_pid[4];
static pid_struct_t chassis_pid[3];
static pid_struct_t chassis_follow_pid;

RAM_PERSIST float vx_cnt = 0;
RAM_PERSIST float vy_cnt = 0;

chassis_state_t expected_state;
//chassis_state_t current_state;
chassis_state_t transform_state;

extern CAN_HandleTypeDef CHASSIS_MOTORS_HCAN;

void chassis_spinning_move(uint16_t gimbal_yaw_ang, chassis_state_t* s_chassis_ref);

typedef enum{
	CHASSIS_FOLLOW = 0,
	CHASSIS_ISOLATED,
	SPINNING_PENDING_SHUT,
	SPINNING
}chassis_spin_t;

chassis_spin_t chassis_spin_state = CHASSIS_FOLLOW;

void chassis_init(void){
	for(int i=0; i<4; i++){
		//pid_init(&wheels_pid[i],5,0,0,C620_OUTPUT_MAX/10,C620_OUTPUT_MAX,0);
		memcpy(&wheels_pid[i],&wheels_pid_init_val,sizeof(pid_struct_t));
	}
	memcpy(&chassis_pid[0],&chassis_vx_pid_init_val,sizeof(pid_struct_t));
	memcpy(&chassis_pid[1],&chassis_vy_pid_init_val,sizeof(pid_struct_t));
	memcpy(&chassis_pid[2],&chassis_wz_pid_init_val,sizeof(pid_struct_t));
	
	memcpy(&chassis_follow_pid,&chassis_follow_pid_init_val,sizeof(pid_struct_t));

	power_ctrl_init(&chassis_power_lim);
}

void chassis_control_loop(rc_ctrl_t* rc_recv){
	//rc_ctrl_t rc_recv = get_rc_data();
	
	motor_response_msg_t* all_motor_data = get_motor_data_ptr();
	
	if(get_motor_rx_flags(CHASSIS_MOTORS_MASKS) == CHASSIS_MOTORS_MASKS){
		clear_motor_rx_flags(CHASSIS_MOTORS_MASKS);
		
		update_wheels_state(all_motor_data,&current_wheels);
		
		
		if(rc_recv->rc.switch_right == RC_SW_DOWN || is_rc_offline()){
			pid_reset(&chassis_pid[0]);
			pid_reset(&chassis_pid[1]);
			pid_reset(&chassis_pid[2]);
			
			pid_reset(&chassis_follow_pid);
			
			chassis_spin_state = CHASSIS_FOLLOW;
			
			shut_motors_output(&(CHASSIS_MOTORS_HCAN),C620_ID_BASE);
			
		}else if(rc_recv->rc.switch_right == RC_SW_MID || rc_recv->rc.switch_right == RC_SW_UP){ 
			if(rc_recv->rc.switch_right == RC_SW_MID){
				//expected_state.v_x = (float)rc_recv->rc.ch1 /300.0f;
				//expected_state.v_y = (float)rc_recv->rc.ch0 /300.0f;
				vx_cnt += CLAMP(rc_recv->rc.ch1 * T_ACC_CNT/660.0f -vx_cnt, S_CURVE_VX_ACC);
				vy_cnt += CLAMP(rc_recv->rc.ch0 * T_ACC_CNT/660.0f -vy_cnt, S_CURVE_VY_ACC);
				expected_state.v_x = s_curve(MAX_VX_SPEED,vx_cnt);
				expected_state.v_y = s_curve(MAX_VY_SPEED,vy_cnt);
				
			}else if(rc_recv->rc.switch_right == RC_SW_UP){
				if(is_key_pressed(KEY_W)){
					vx_cnt += CLAMP(T_ACC_CNT -vx_cnt, S_CURVE_VX_ACC);
				}else if(is_key_pressed(KEY_S)){
					vx_cnt += CLAMP(-T_ACC_CNT -vx_cnt, S_CURVE_VX_ACC);
				}else{
					vx_cnt += CLAMP(-vx_cnt, S_CURVE_VX_ACC);
				}
				
				if(is_key_pressed(KEY_D)){
					vy_cnt += CLAMP(T_ACC_CNT -vy_cnt, S_CURVE_VY_ACC);
				}else if(is_key_pressed(KEY_A)){
					vy_cnt += CLAMP(-T_ACC_CNT -vy_cnt, S_CURVE_VY_ACC);
				}else{
					vy_cnt += CLAMP(-vy_cnt, S_CURVE_VY_ACC);
				}
				
				expected_state.v_x = s_curve(MAX_VX_SPEED,vx_cnt);
				expected_state.v_y = s_curve(MAX_VY_SPEED,vy_cnt);
				
			}
			
			uint16_t yaw_ang = all_motor_data[CAN_GM6020_YAW_ID].raw_angle;
			power_ctrl(&chassis_power_lim, game_robot_status.chassis_power_limit, 0);
			
			if(rc_recv->rc.switch_left == RC_SW_MID){
				//expected_state.w_z = CONSTANT_SPINNING_SPEED;
				chassis_spin_state = SPINNING;
			}else{
				if(rc_recv->rc.last_switch_left == RC_SW_MID){
						chassis_spin_state = SPINNING_PENDING_SHUT;
					}
				if(abs(get_delta_ang(GIMBAL_YAW_CENTER,yaw_ang)) < SHUT_DEADBAND 
					&& chassis_spin_state == SPINNING_PENDING_SHUT){
					chassis_spin_state = CHASSIS_FOLLOW;
				}
			}
			
			if(is_gimbal_reset()){
				if(chassis_spin_state == CHASSIS_FOLLOW){
					//chassis follow pid
					expected_state.w_z = pid_calc_deadband(&chassis_follow_pid, GIMBAL_YAW_CENTER ,yaw_ang);
				}else if(chassis_spin_state == CHASSIS_ISOLATED){
					expected_state.w_z = 0;
				}else if(chassis_spin_state == SPINNING || chassis_spin_state == SPINNING_PENDING_SHUT){
					expected_state.w_z = CONSTANT_SPINNING_SPEED;
				}
			}else{
				expected_state.w_z = 0.0f;
				expected_state.v_x = 0.0f;
				expected_state.v_y = 0.0f;
			}
			chassis_spinning_move(yaw_ang,&expected_state);
			//chassis_spinning_move(0,&expected_state);
		}
	}
}

void chassis_spinning_move(uint16_t gimbal_yaw_ang, chassis_state_t* s_chassis_ref){
//	chassis_state_t transform_state;
	float angle_delta = (float)(gimbal_yaw_ang - GIMBAL_YAW_CENTER);
	if(fabsf(angle_delta) < chassis_follow_pid_init_val.k_deadband) angle_delta = 0;
	
	float sin_yaw = sinf(angle_delta * (2.0f * PI / ANGLE_RANGE));
    float cos_yaw = cosf(angle_delta * (2.0f * PI / ANGLE_RANGE));

	transform_state.v_x = sin_yaw * s_chassis_ref->v_y + cos_yaw * s_chassis_ref->v_x;
    transform_state.v_y = cos_yaw * s_chassis_ref->v_y - sin_yaw * s_chassis_ref->v_x;
    transform_state.w_z = s_chassis_ref ->w_z;

    //chassis_pid_revctrl(chassis_pid, &current_wheels, &transform_state, &chassis_power_lim);
	//chassis_advance_ctrl(chassis_pid, &current_wheels, &transform_state, &chassis_power_lim);
	chassis_pid_ctrl(wheels_pid,&current_wheels,&transform_state,&chassis_power_lim);

}

/**
  * @brief get w_z of current chassis 
  * @return chassis spinning speed w_z in rpm
  */
float get_current_chassis_wz(void){
	return (current_wheels.M_RF + current_wheels.M_LF 
		+ current_wheels.M_LB + current_wheels.M_RB)*(D_WHEEL/2.0f)/(2.0f*(KXY_BACK+KXY_FRONT));
}



int16_t get_chassis_delta_ang(int16_t current,int16_t last, int16_t cur_rpm){
	int16_t result = current - last;
	if((result > 0) && (cur_rpm < 0)) result-= ANGLE_RANGE;
	else if((result < 0) && (cur_rpm > 0)) result += ANGLE_RANGE;
	return result;
}

static int16_t last_ang[4],last_gimbal_ang;

void reset_chassis_angle(void){
	last_ang[0] =last_ang[1]= last_ang[2] = last_ang[3] = ANGLE_RANGE;
}

int16_t get_gimbal_angle(motor_response_msg_t chassis_motors[4] ,motor_response_msg_t* gimbal_motor){
	static float z_angle = 0;
	int16_t wheel_delta_ang[4];
	int16_t gimbal_delta_ang;
	int32_t wheel_delta_ang_sum = 0;
	
	if(last_ang[0] == ANGLE_RANGE || last_ang[1]== ANGLE_RANGE
	|| last_ang[2] == ANGLE_RANGE || last_ang[3] == ANGLE_RANGE || last_gimbal_ang == ANGLE_RANGE){
		for(int i =0;i<4;i++) last_ang[i] = (int16_t)chassis_motors[i].raw_angle;
		last_gimbal_ang = gimbal_motor->raw_angle;
		z_angle = gimbal_motor->raw_angle;
	}else{
		for(int i=0;i<4;i++){
			wheel_delta_ang[i] = get_chassis_delta_ang((int16_t)chassis_motors[i].raw_angle,last_ang[i],chassis_motors[i].speed_rpm);
			wheel_delta_ang_sum += wheel_delta_ang[i];
			last_ang[i] = (int16_t)chassis_motors[i].raw_angle;
		}
		gimbal_delta_ang = get_chassis_delta_ang(gimbal_motor->raw_angle,last_gimbal_ang,gimbal_motor->speed_rpm);
		last_gimbal_ang = (int16_t)gimbal_motor->raw_angle;
		
		z_angle += (gimbal_delta_ang - ((D_WHEEL/2)*(float)wheel_delta_ang_sum)/REDUCTION_RATIO/(2*(KXY_BACK+KXY_FRONT))*K_CHASSIS_CAL);
		if(z_angle > ((float)ANGLE_RANGE)) z_angle -= ANGLE_RANGE;
		else if(z_angle < 0) z_angle += ANGLE_RANGE;
	}
	return (int16_t)z_angle;
}

