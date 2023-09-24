#include "gimbal.h"

#include "base_drv/drv_conf.h"
#include "base_drv/motor_ctrl.h"
#include "base_drv/ext_imu.h"

#include "algorithm/pid.h"
#include "algorithm/filter.h"
#include "algorithm/imu_fusion.h"

#include "pid_parameters.h"
#include "chassis.h"

#include <stdlib.h>
#include <string.h>

//#define ANGLE_DELTA(A) if(A > ANGLE_RANGE/2) A-=ANGLE_RANGE; \
//									else if(A < -ANGLE_RANGE/2) A+=ANGLE_RANGE;

static pid_struct_t pitch_pid[2], yaw_pid[2];
static pid_struct_t pitch_imu_pid[2], yaw_imu_pid[2];

RAM_PERSIST int16_t pitch_ang_ref;
RAM_PERSIST int16_t yaw_ang_ref;

RAM_PERSIST gimbal_state_t gimbal_state;
RAM_PERSIST gimbal_state_t gimbal_last_state;
uint16_t start_time_cnt = 0;

extern CAN_HandleTypeDef GIMBAL_MOTORS_HCAN;

HAL_StatusTypeDef gimbal_ctrl(motor_response_msg_t* motor_data_ptr,uint16_t raw_pitch_ang, uint16_t raw_yaw_ang);
HAL_StatusTypeDef gimbal_imu_ctrl(eular_t* imu_eular, imu_data_fp_t* imu_gyro,uint16_t raw_pitch_ang, uint16_t raw_yaw_ang);
gimbal_state_t gimbal_smooth_reset(motor_response_msg_t* motor_data_ptr);

void gimbal_init(uint8_t reset_all_flag){
	
	memcpy(&pitch_pid[ANG_LOOP],&pitch_pid_ang_loop,sizeof(pid_struct_t));
	memcpy(&pitch_pid[RPM_LOOP],&pitch_pid_rpm_loop,sizeof(pid_struct_t));
	
	memcpy(&yaw_pid[ANG_LOOP],&yaw_pid_ang_loop,sizeof(pid_struct_t));
	memcpy(&yaw_pid[RPM_LOOP],&yaw_pid_rpm_loop,sizeof(pid_struct_t));
	
	memcpy(&pitch_imu_pid[ANG_LOOP],&pitch_imu_pid_ang_loop,sizeof(pid_struct_t));
	memcpy(&pitch_imu_pid[RPM_LOOP],&pitch_imu_pid_rpm_loop,sizeof(pid_struct_t));
	
	memcpy(&yaw_imu_pid[ANG_LOOP],&yaw_imu_pid_ang_loop,sizeof(pid_struct_t));
	memcpy(&yaw_imu_pid[RPM_LOOP],&yaw_imu_pid_rpm_loop,sizeof(pid_struct_t));

	if(reset_all_flag){
		pitch_ang_ref = GIMBAL_PITCH_CENTER;
		yaw_ang_ref = GIMBAL_YAW_CENTER;
		start_time_cnt = 0;
		gimbal_state = GIMBAL_UNRESET;
	}
}

void gimbal_control_loop(rc_ctrl_t* rc_recv){
	if(get_motor_rx_flags(GIMBAL_MOTORS_MASKS) == GIMBAL_MOTORS_MASKS){
		clear_motor_rx_flags(GIMBAL_MOTORS_MASKS);
		if(rc_recv->rc.switch_right == RC_SW_DOWN){

			start_time_cnt = 0;
			gimbal_state = GIMBAL_UNRESET;
			gimbal_last_state = GIMBAL_UNRESET;
			reset_chassis_angle();
			shut_motors_output(&(GIMBAL_MOTORS_HCAN),M6020_ID_EXTEND);
			
		}else if(rc_recv->rc.switch_right == RC_SW_MID || rc_recv->rc.switch_right == RC_SW_UP){
			motor_response_msg_t* motor_ptr = get_motor_data_ptr();
			if(gimbal_state != GIMBAL_UNRESET && gimbal_state != GIMBAL_RESETING){
				
				if(is_ext_imu_offline()){
					if(gimbal_last_state != GIMBAL_MOTOR_CTRL){
						pitch_ang_ref = motor_ptr[CAN_GM6020_PITCH_ID].raw_angle;
						yaw_ang_ref = motor_ptr[CAN_GM6020_YAW_ID].raw_angle;
					}
					gimbal_state = GIMBAL_MOTOR_CTRL;
				}else{
					if(gimbal_last_state != GIMBAL_IMU_CTRL){
						pitch_ang_ref = (int16_t)ext_eular.pitch;	
						yaw_ang_ref = (int16_t)ext_eular.yaw;
					}
					gimbal_state = GIMBAL_IMU_CTRL;
				}
				
				if(!is_rc_offline()){
					if(rc_recv->rc.switch_right == RC_SW_UP){
						pitch_ang_ref += (int16_t)((float)rc_recv->mouse.y *(GIMBAL_MOVE_SPEED*0.001f));
						yaw_ang_ref += (int16_t)((float)rc_recv->mouse.x *(GIMBAL_MOVE_SPEED*0.001f));
					}else{
						pitch_ang_ref += -(int16_t)((float)rc_recv->rc.ch3 * GIMBAL_MOVE_SPEED / 660);
						yaw_ang_ref += -(int16_t)((float)rc_recv->rc.ch2 * GIMBAL_MOVE_SPEED / 660);
					}
				}
				
				LIMIT_MIN_MAX(pitch_ang_ref, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

				if(yaw_ang_ref < 0) yaw_ang_ref += ANGLE_RANGE;
				else if(yaw_ang_ref > ANGLE_RANGE) yaw_ang_ref -= ANGLE_RANGE;
				
				if(gimbal_state == GIMBAL_IMU_CTRL){
					gimbal_imu_ctrl(&ext_eular,&ext_imu_data,pitch_ang_ref, yaw_ang_ref);
				}else if(gimbal_state == GIMBAL_MOTOR_CTRL){
					gimbal_ctrl(motor_ptr,pitch_ang_ref, yaw_ang_ref);
				}
			}else{
				gimbal_state = gimbal_smooth_reset(motor_ptr);
			}
			gimbal_last_state = gimbal_state;
		}else{
			shut_motors_output(&(GIMBAL_MOTORS_HCAN),M6020_ID_EXTEND);
		}
	}
}

uint32_t is_gimbal_reset(void){ return (gimbal_state != GIMBAL_UNRESET && gimbal_state != GIMBAL_RESETING); }

/*云台缓启动，使云台保持与底盘同向*/
gimbal_state_t gimbal_smooth_reset(motor_response_msg_t* motor_data_ptr){
	int16_t delta_ang_pitch,delta_ang_yaw;
	int16_t delta_ang_pitch_ref,delta_ang_yaw_ref;
	
	if(motor_data_ptr == NULL) return GIMBAL_RESETED;

	motor_response_msg_t pitch_motor = motor_data_ptr[CAN_GM6020_PITCH_ID];
	motor_response_msg_t yaw_motor = motor_data_ptr[CAN_GM6020_YAW_ID];

	if(gimbal_state == GIMBAL_RESETING){
		delta_ang_pitch_ref = get_delta_ang(GIMBAL_PITCH_CENTER,pitch_ang_ref);
		delta_ang_yaw_ref = get_delta_ang(GIMBAL_YAW_CENTER,yaw_ang_ref);

		pitch_ang_ref += CLAMP(delta_ang_pitch_ref,SOFT_START_SPEED);
		yaw_ang_ref += CLAMP(delta_ang_yaw_ref,SOFT_START_SPEED);

		gimbal_ctrl(motor_data_ptr, pitch_ang_ref, yaw_ang_ref);
		
		delta_ang_pitch = get_delta_ang(GIMBAL_PITCH_CENTER,pitch_motor.raw_angle);
		delta_ang_yaw = get_delta_ang(GIMBAL_YAW_CENTER,yaw_motor.raw_angle);
		
		if(start_time_cnt < SOFT_START_MAX_CNT){
			start_time_cnt++;
			if(abs(delta_ang_pitch) <= SOFT_START_DEADBAND 
				&& abs(delta_ang_yaw) <= SOFT_START_DEADBAND){
				return GIMBAL_RESETED;
			}else return GIMBAL_RESETING;
		}else return GIMBAL_RESETING;
		//else return GIMBAL_RESETED;
	}else{
		pitch_ang_ref = pitch_motor.raw_angle;
		yaw_ang_ref = yaw_motor.raw_angle;
		return GIMBAL_RESETING;
	}
}
/*纯电机控制云台*/
HAL_StatusTypeDef gimbal_ctrl(motor_response_msg_t* motor_data_ptr
							,uint16_t raw_pitch_ang, uint16_t raw_yaw_ang){
	motor_transmit_msg_t tx_msg;
	int16_t delta_ang_pitch,delta_ang_yaw;
	int16_t current_yaw_angle;
	
	if(motor_data_ptr == NULL) return HAL_ERROR;

	motor_response_msg_t pitch_motor = motor_data_ptr[CAN_GM6020_PITCH_ID];
	motor_response_msg_t yaw_motor = motor_data_ptr[CAN_GM6020_YAW_ID];
	
	memset(tx_msg.D,0,sizeof(motor_transmit_msg_t));
	
	delta_ang_pitch = get_delta_ang(raw_pitch_ang,pitch_motor.raw_angle);
	
	//current_yaw_angle = yaw_motor.raw_angle;
	current_yaw_angle = get_gimbal_angle(motor_data_ptr,&motor_data_ptr[CAN_GM6020_YAW_ID]);
	delta_ang_yaw = get_delta_ang(raw_yaw_ang,current_yaw_angle);
	
	//CAN_GM6020_PITCH_ID start from 4 (id-0x201)
	float pitch_current_rpm = iir_filter_2(pitch_motor.speed_rpm, GIMBAL_PITCH_FILTER2_RPM_CH);
	//任意电机在发送数据包中的位置可以用原始ID对4求余获得
	tx_msg.D[(CAN_GM6020_PITCH_ID % 0x04)] = (int16_t)pid_dual_loop(pitch_pid,delta_ang_pitch, pitch_current_rpm);

	//chassis follow gimbal stablize
	//directions: chassis w_z + ,yaw speed_rpm -, wz uses ground as reference system
	//Yaw轴旋转转速，以下情况为6020电机电源口在底盘上
	float yaw_current_rpm = iir_filter_2(yaw_motor.speed_rpm, GIMBAL_YAW_FILTER2_RPM_CH)
									- get_current_chassis_wz();

	tx_msg.D[(CAN_GM6020_YAW_ID % 0x04)] = (int16_t)pid_dual_loop(yaw_pid, delta_ang_yaw, yaw_current_rpm);
	
	return set_motor_output(&(GIMBAL_MOTORS_HCAN),&tx_msg,M6020_ID_EXTEND);
	
}

/*IMU控制云台*/
HAL_StatusTypeDef gimbal_imu_ctrl(eular_t* imu_eular, imu_data_fp_t* imu_gyro,
						uint16_t raw_pitch_ang, uint16_t raw_yaw_ang){
	motor_transmit_msg_t tx_msg;
	int16_t delta_ang_pitch,delta_ang_yaw;
	int16_t current_yaw_angle;
	motor_response_msg_t* motor_data_ptr = get_motor_data_ptr();
	motor_response_msg_t yaw_motor = motor_data_ptr[CAN_GM6020_YAW_ID];
	
	if(imu_gyro == NULL || imu_eular == NULL) return HAL_ERROR;
	

	memset(tx_msg.D,0,sizeof(motor_transmit_msg_t));
	
	delta_ang_pitch = get_delta_ang(raw_pitch_ang,(int16_t)imu_eular->pitch);		
							
	current_yaw_angle = (int16_t)imu_eular->yaw;
	delta_ang_yaw = get_delta_ang(raw_yaw_ang,current_yaw_angle);
	
	//CAN_GM6020_PITCH_ID start from 4 (id-0x201)
	//float pitch_current_rpm = iir_filter_2(pitch_motor.speed_rpm, GIMBAL_PITCH_FILTER2_RPM_CH);
	float pitch_current_rpm = imu_gyro->gy*30/PI;

	tx_msg.D[(CAN_GM6020_PITCH_ID % 0x04)] = (int16_t)pid_dual_loop(pitch_imu_pid,delta_ang_pitch, pitch_current_rpm);

	//chassis follow gimbal stablize
	//directions: chassis w_z + ,yaw speed_rpm -, wz uses ground as reference system
	float yaw_current_rpm = iir_filter_2(yaw_motor.speed_rpm, GIMBAL_YAW_FILTER2_RPM_CH) - get_current_chassis_wz();;
	//float yaw_current_rpm = imu_gyro->gz*30/PI;

	tx_msg.D[(CAN_GM6020_YAW_ID % 0x04)] = (int16_t)pid_dual_loop(yaw_imu_pid, delta_ang_yaw, yaw_current_rpm);
	
	return set_motor_output(&(GIMBAL_MOTORS_HCAN),&tx_msg,M6020_ID_EXTEND);
	
}


