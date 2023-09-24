#include "shoot.h"

#include "base_drv/motor_ctrl.h"
#include "base_drv/drv_conf.h"
#include "algorithm/pid.h"
#include "algorithm/filter.h"

#include "pid_parameters.h"

#include <stdlib.h>
#include <string.h>

static pid_struct_t shoot_pid[2],bullet_push_pid;
RAM_PERSIST static uint16_t pusher_stuck_cnt;
RAM_PERSIST uint8_t fric_state = 0;

extern CAN_HandleTypeDef SHOOT_MOTORS_HCAN;

HAL_StatusTypeDef set_multi_shoot_output(float shoot_ref_speed, int16_t bullet_pusher_ref_speed);

void shoot_control_loop(rc_ctrl_t* rc_recv){
	if(get_motor_rx_flags(SHOOT_MOTORS_MASKS) == SHOOT_MOTORS_MASKS){
		clear_motor_rx_flags(SHOOT_MOTORS_MASKS);
		
		if(rc_recv->rc.switch_right == RC_SW_DOWN || is_rc_offline()){
			
			fric_state = 0;
			shut_motors_output(&(SHOOT_MOTORS_HCAN),C620_ID_EXTEND);
			
		}else if(rc_recv->rc.switch_right == RC_SW_MID){

			if(rc_recv->rc.switch_left == RC_SW_DOWN){
				
				shut_motors_output(&(SHOOT_MOTORS_HCAN),C620_ID_EXTEND);
				
			}else if(rc_recv->rc.switch_left == RC_SW_MID){
				
				set_multi_shoot_output(15.0f,0);
				
			}else if(rc_recv->rc.switch_left == RC_SW_UP){
				
				set_multi_shoot_output(15.0f,PUSHER_WHEEL_SPEED);
			}
		}else if(rc_recv->rc.switch_right == RC_SW_UP){
			
			if(is_key_pressed(KEY_C) && (!is_key_last_pressed(KEY_C))){
				
				fric_state = 1;
				
			}else if(is_key_pressed(KEY_V) && (!is_key_last_pressed(KEY_V))){
				
				fric_state = 0;
			}
			
			if(fric_state){
				if(rc_recv->mouse.press_left == MOUSE_PRESS){
					
					set_multi_shoot_output(15.0f,PUSHER_WHEEL_SPEED);
				}else{
					set_multi_shoot_output(15.0f,0);
				}
			}else{
				shut_motors_output(&(SHOOT_MOTORS_HCAN),C620_ID_EXTEND);
			}
		}else{
			shut_motors_output(&(SHOOT_MOTORS_HCAN),C620_ID_EXTEND);
		}
	}
}


void shoot_pid_init(void){
	memcpy(&shoot_pid[0],&shoot_fric_pid_init_val,sizeof(pid_struct_t));
	memcpy(&shoot_pid[1],&shoot_fric_pid_init_val,sizeof(pid_struct_t));

	memcpy(&bullet_push_pid,&bullet_push_pid_init_val,sizeof(pid_struct_t));
	pusher_stuck_cnt = 0;
}


HAL_StatusTypeDef set_multi_shoot_output(float shoot_ref_speed, int16_t bullet_pusher_ref_speed){
	motor_response_msg_t* all_motor_data;
	motor_transmit_msg_t tx_msg;
	int16_t current_pusher_rpm;
	float fric_ref_speed;
	float fric_left_filtered,fric_right_filtered;

	all_motor_data = get_motor_data_ptr();
	memset(tx_msg.D,0,sizeof(motor_transmit_msg_t));
	
	if(shoot_ref_speed > 0 && shoot_ref_speed < MAX_SHOOT_SPEED){
		fric_ref_speed = (float)shoot_ref_speed / C_FRIC_WHEEL * 60;
		fric_left_filtered = iir_filter_2(all_motor_data[CAN_SHOOT_LEFT_ID].speed_rpm,SHOOT_LEFT_FILTER2_RPM_CH);
		fric_right_filtered = iir_filter_2(all_motor_data[CAN_SHOOT_RIGHT_ID].speed_rpm,SHOOT_RIGHT_FILTER2_RPM_CH);
		
		tx_msg.D[(CAN_SHOOT_LEFT_ID % 0x04)] = (int16_t)pid_calc(&shoot_pid[0]
			, -fric_ref_speed, fric_left_filtered);
		tx_msg.D[(CAN_SHOOT_RIGHT_ID % 0x04)] = (int16_t)pid_calc(&shoot_pid[1]
			, fric_ref_speed, fric_right_filtered);
	}else{
		tx_msg.D[(CAN_SHOOT_LEFT_ID % 0x04)] = 0;
		tx_msg.D[(CAN_SHOOT_RIGHT_ID % 0x04)] = 0;
	}

	current_pusher_rpm = all_motor_data[CAN_BULLET_PUSHER_ID].speed_rpm;

	//anti-stuck process
	if(abs(current_pusher_rpm) < STUCK_SPEED 
		&& bullet_pusher_ref_speed > STUCK_SPEED){
			if(pusher_stuck_cnt < STUCK_MAX_TIME_CNT){
				pusher_stuck_cnt++;
			}else if(pusher_stuck_cnt == STUCK_MAX_TIME_CNT){
				pusher_stuck_cnt += STUCK_REVERSE_TIME_CNT;
			}
	}else if(pusher_stuck_cnt <= STUCK_MAX_TIME_CNT){
		pusher_stuck_cnt = 0;
	}

	if(pusher_stuck_cnt > STUCK_MAX_TIME_CNT){
		//direction( + or -?)
		bullet_pusher_ref_speed = STUCK_REVERSE_RPM;
		pusher_stuck_cnt--;
	}
	
	tx_msg.D[(CAN_BULLET_PUSHER_ID % 0x04)] = (int16_t)pid_calc(&bullet_push_pid
		, bullet_pusher_ref_speed, current_pusher_rpm);


	return set_motor_output(&(SHOOT_MOTORS_HCAN),&tx_msg,C620_ID_EXTEND);
	
}

float get_bullet_inital_speed(int16_t bullet_pusher_ref_speed){
	return C_PUSHER_WHEEL*(bullet_pusher_ref_speed/M2006_REDUCTION_RATIO)/60;
}

