#include "robot_tests.h"

#include "../chassis.h"
#include "../gimbal.h"
#include "../shoot.h"
#include "../base_drv/motor_ctrl.h"
#include "../base_drv/chassis_base.h"
#include "../base_drv/motor_ctrl.h"
#include "../base_drv/rc.h"

#include "../algorithm/kalman.h"
#include "../algorithm/pid.h"
#include "../algorithm/util.h"

#include "../Src/mpu_imu/bsp_imu.h"

#include <math.h>
#include <string.h>


kalman_filter_t vx_est, vy_est;
volatile chassis_state_t cur_chassis_state;
volatile chassis_state_t cur_chassis_acc;
volatile int16_t gimal_ang;

extern chassis_wheels_state_t current_wheels;
extern CAN_HandleTypeDef hcan1;
chassis_wheels_state_t cur_wheels_acc;

typedef enum{
	GIMBAL_TEST_START = 0,
	GIMBAL_UP_TEST,
	GIMBAL_DOWN_TEST,
	GIMBAL_TEST_STUCK,
	GIMBAL_TEST_DONE
}gimbal_test_state_t;

typedef struct{
	float RF_x,RF_y;
	float LF_x,LF_y;
	float LB_x,LB_y;
	float RB_x,RB_y;
}chassis_wheel_vec_t;

static uint16_t shoot_check_time_cnt;
static gimbal_test_state_t gimbal_test_state = GIMBAL_TEST_START;
float acc_x, acc_y, acc_z;
float v_x_est, v_y_est,drift_vx, drift_rate;
float vx_output_f, vy_output_f;

extern CAN_HandleTypeDef SHOOT_MOTORS_HCAN;

#define D_T 0.0025f

void init_robot_test(void){
	//acc 220ug/rtHz * 460Hz = 4718.47ug = 0.046241 m/s^2
	kalman_init(&vx_est,0.01,0.008,1);
	kalman_init(&vy_est,0.01,0.008,1);
	kalman_set(&vx_est,0.0f);
	kalman_set(&vy_est,0.0f);
}

#define ROBOT_WEIGHT 18.2f
#define MOTOR_K_T 0.3f

void update_chassis_test_data(void){
	UNUSED(cur_chassis_state);
	UNUSED(gimal_ang);
	UNUSED(acc_x);
	UNUSED(acc_y);
	UNUSED(acc_z);

	cur_chassis_state = mecanum_forward(&current_wheels);
	
	acc_x = -mpu_data.ay * 8.0f *9.7833f / (float)((INT16_MAX)+1);
	acc_y = -mpu_data.ax * 8.0f *9.7833f / (float)((INT16_MAX)+1);
	acc_z = mpu_data.az * 8.0f *9.7833f / (float)((INT16_MAX)+1);

	cur_chassis_acc.v_x = get_acc(cur_chassis_state.v_x,0)/D_T;
	cur_chassis_acc.v_y = get_acc(cur_chassis_state.v_y,1)/D_T;

	vx_output_f = ((acc_x - cur_chassis_acc.v_x) * ROBOT_WEIGHT *(D_WHEEL/2))/MOTOR_K_T /20.0f*C620_OUTPUT_MAX;
	vy_output_f = ((acc_y - cur_chassis_acc.v_y) * ROBOT_WEIGHT *(D_WHEEL/2))/MOTOR_K_T /20.0f*C620_OUTPUT_MAX;
	
	//cur_chassis_acc.v_x = get_acc(cur_chassis_state.v_x,0)/D_T;
	
//	if(fabsf(cur_chassis_state.v_x) < 0.08f) {
//		drift_vx = 0.0f;
//		drift_rate = 0.0f;
//	}else{
//		drift_vx += (cur_chassis_acc.v_x - acc_x)*D_T;
//		if(fabsf(cur_chassis_state.v_x) > 0.2f)
//			drift_rate = drift_vx / (cur_chassis_state.v_x);
//		else drift_rate = 0.0f;
//	}
//	if(fabsf(cur_chassis_state.v_x) > 0.08f) 
//		drift_rate = (cur_chassis_acc.v_x - acc_x)*D_T / cur_chassis_state.v_x;
//	else
//		drift_rate = 0.0f;
	//v_x_est = kalman_update(&vx_est,cur_chassis_state.v_x,acc_x,0.0025f);
	//v_x_est += acc_x*0.0025f;
	
	//v_y_est = kalman_update(&vy_est,cur_chassis_state.v_y,acc_y,0.0025f);
}

#define COM_HEIGHT 0.15f

void chassis_wheels_realloc(chassis_wheel_vec_t* w_v, chassis_wheels_state_t* ratio){
	float real_acc_x = (acc_x - cur_chassis_acc.v_x);
	float real_acc_y = (acc_y - cur_chassis_acc.v_y);
	float total_acc_xy = sqrtf(real_acc_x*real_acc_x + real_acc_y* real_acc_y);
	
	float slope_tan = total_acc_xy / acc_z;
	
	float unit_acc_x = real_acc_x / total_acc_xy;
	float unit_acc_y = real_acc_y / total_acc_xy;
	
	ratio->M_RF = (w_v->RF_x * unit_acc_x + w_v->RF_y * unit_acc_y) - COM_HEIGHT*slope_tan;
	ratio->M_LF = (w_v->LF_x * unit_acc_x + w_v->LF_y * unit_acc_y) - COM_HEIGHT*slope_tan;
	ratio->M_LB = (w_v->LB_x * unit_acc_x + w_v->LB_y * unit_acc_y) - COM_HEIGHT*slope_tan;
	ratio->M_RB = (w_v->RB_x * unit_acc_x + w_v->RB_y * unit_acc_y) - COM_HEIGHT*slope_tan;
	
	float max_ratio = (ratio->M_RF), min_ratio = (ratio->M_RF);
	if(max_ratio < (ratio->M_LF)) max_ratio = ratio->M_LF;
	if(min_ratio > (ratio->M_LF)) min_ratio = ratio->M_LF;
	
	if(max_ratio < (ratio->M_LB)) max_ratio = ratio->M_LB;
	if(min_ratio > (ratio->M_LB)) min_ratio = ratio->M_LB;
	
	if(max_ratio < (ratio->M_RB)) max_ratio = ratio->M_RB;
	if(min_ratio > (ratio->M_RB)) min_ratio = ratio->M_RB;
	
	float clamp_range = (max_ratio - min_ratio)/4;
	
	ratio->M_RF /= clamp_range;
	ratio->M_LF /= clamp_range;
	ratio->M_RB /= clamp_range;
	ratio->M_LB /= clamp_range;
}

float vx_realloc_ratio = 0.5;
float vy_realloc_ratio = 0.5;

 HAL_StatusTypeDef chassis_advance_ctrl(pid_struct_t pPID[3],
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
	
	tmp[MOTOR_RF_ID] *= 2*vx_realloc_ratio;
	tmp[MOTOR_LF_ID] *= 2*vx_realloc_ratio;
	tmp[MOTOR_RB_ID] *= 2*(1-vx_realloc_ratio);
	tmp[MOTOR_LB_ID] *= 2*(1-vx_realloc_ratio);

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



void gimbal_range_check(void){
	motor_transmit_msg_t tx_msg;
	memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
	rc_ctrl_t* rc = get_rc_data_ptr();
	motor_response_msg_t* motor_rep = get_motor_data_ptr();
	
	switch(gimbal_test_state){
		case GIMBAL_TEST_START:
			gimbal_test_state = GIMBAL_UP_TEST;
			break;
		case GIMBAL_UP_TEST:
			if(motor_rep[CAN_GM6020_PITCH_ID].raw_angle < GIMBAL_PITCH_MIN + 50){
				gimbal_test_state = GIMBAL_DOWN_TEST;
			}
			tx_msg.D[(CAN_GM6020_PITCH_ID % 0x04)] = -7500;
			break;
		case GIMBAL_DOWN_TEST:
			if(motor_rep[CAN_GM6020_PITCH_ID].raw_angle > GIMBAL_PITCH_MAX - 50){
				gimbal_test_state = GIMBAL_TEST_DONE;
			}
			tx_msg.D[(CAN_GM6020_PITCH_ID % 0x04)] = 2500;
			break;
		case GIMBAL_TEST_DONE:
			break;
		case GIMBAL_TEST_STUCK:
			break;
	}
	
	if(rc->rc.switch_right == RC_SW_MID){
		if(rc->rc.last_switch_right == RC_SW_DOWN) gimbal_test_state = GIMBAL_TEST_START;
		set_motor_output(&(GIMBAL_MOTORS_HCAN),&tx_msg,M6020_ID_EXTEND);
	}else{
		memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
		gimbal_test_state = GIMBAL_TEST_DONE;
	}
}

//void reset_check_time(void){check_time_cnt = 1;}

void shoot_check(void){
	motor_transmit_msg_t tx_msg;
	rc_ctrl_t* rc = get_rc_data_ptr();
	motor_response_msg_t* motor_rep = get_motor_data_ptr();
	memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
	if(shoot_check_time_cnt < 30 && shoot_check_time_cnt != 0){
		tx_msg.D[(CAN_SHOOT_LEFT_ID % 0x04)] = -1500;
		tx_msg.D[(CAN_SHOOT_RIGHT_ID % 0x04)] = 1500;
		shoot_check_time_cnt++;
	}else{
		shoot_check_time_cnt = 0;
	}
	set_motor_output(&(SHOOT_MOTORS_HCAN),&tx_msg,C620_ID_EXTEND);
	
	if(rc->rc.switch_left == RC_SW_MID && rc->rc.last_switch_left == RC_SW_DOWN){
		if(motor_rep[CAN_SHOOT_LEFT_ID].speed_rpm < 300 && motor_rep[CAN_SHOOT_RIGHT_ID].speed_rpm < 300)
		shoot_check_time_cnt = 1;
	}
}

