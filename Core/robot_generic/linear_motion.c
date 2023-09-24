#include "linear_motion.h"

#include "base_drv/drv_conf.h"

#ifdef LINEAR_MOTION_MOTORS_NUM
#if LINEAR_MOTION_MOTORS_NUM >= MOTOR_NUM
#error "LINEAR_MOTION_MOTORS_NUM is larger than total motor num"
#endif
#else
#endif

extern CAN_HandleTypeDef LINEAR_MOTORS_HCAN;

linear_motion lm_motors[LINEAR_MOTION_MOTORS_NUM];

motion_state lift_motors_reset(pid_struct_t* mPID,uint16_t speed){
	uint16_t current_ang = lm_motors[LIFT_MOTOR_LEFT_ID].motor->raw_angle;

	int16_t current_rpm = lm_motors[LIFT_MOTOR_LEFT_ID].motor->speed_rpm;

	int16_t delta_raw_ang = current_ang - lm_motors[LIFT_MOTOR_LEFT_ID].last_raw_ang;
	
	if(current_rpm > 0 && delta_raw_ang < 0){
		delta_raw_ang += RAW_ANGLE_RANGE;
	}else if(current_rpm < 0 && delta_raw_ang > 0){
		delta_raw_ang -= RAW_ANGLE_RANGE;
	}
	lm_motors[LIFT_MOTOR_LEFT_ID].last_raw_ang = current_ang;
	if(current_rpm == 0 && delta_raw_ang == 0 ){
		lm_motors[LIFT_MOTOR_LEFT_ID].current_pos = 0;
		return RESETED;

	}else{
		motor_transmit_msg_t tx_msg;
		
		tx_msg.D[LIFT_MOTOR_LEFT_ID] = (int16_t)pid_calc(mPID, speed,current_rpm);
		tx_msg.D[LIFT_MOTOR_RIGHT_ID] = tx_msg.D[LIFT_MOTOR_LEFT_ID];

		set_motor_output(&(LINEAR_MOTORS_HCAN),&tx_msg,C620_ID_EXTEND);
		return RESETING;
	}
}
	


motion_state lift_motors_move(pid_struct_t* mPID,uint16_t pos){
	uint16_t current_ang = lm_motors[LIFT_MOTOR_LEFT_ID].motor->raw_angle;

	int16_t current_rpm = lm_motors[LIFT_MOTOR_LEFT_ID].motor->speed_rpm;

	int16_t delta_raw_ang = current_ang - lm_motors[LIFT_MOTOR_LEFT_ID].last_raw_ang;

	if(current_rpm > 0 && delta_raw_ang < 0){
		delta_raw_ang += RAW_ANGLE_RANGE;
	}else if(current_rpm < 0 && delta_raw_ang > 0){
		delta_raw_ang -= RAW_ANGLE_RANGE;
	}
	lm_motors[LIFT_MOTOR_LEFT_ID].current_pos += delta_raw_ang;
	// > or < ?
	if(pos > lm_motors[LIFT_MOTOR_LEFT_ID].MAX_RANGE) pos = lm_motors[LIFT_MOTOR_LEFT_ID].MAX_RANGE;
	motor_transmit_msg_t tx_msg;
	
	tx_msg.D[LIFT_MOTOR_LEFT_ID] = (int16_t)pid_calc(mPID, pos, lm_motors[LIFT_MOTOR_LEFT_ID].current_pos);
	tx_msg.D[LIFT_MOTOR_RIGHT_ID] = tx_msg.D[LIFT_MOTOR_LEFT_ID];

	set_motor_output(&(LINEAR_MOTORS_HCAN),&tx_msg,C620_ID_EXTEND);

	if(current_rpm == 0  && delta_raw_ang > MIN_ERR) {
		return STUCKED;
	}else{
		return READY;
	}
}

