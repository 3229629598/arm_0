#include "motor_tests.h"

#include "utils/mem_log.h"
#include "../base_drv/motor_ctrl.h"
#include "../base_drv/drv_conf.h"
#include "../base_drv/rc.h"
#include "../algorithm/pid.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

char* motor_id_msg = "motor: 0 spin";

void identify_motors(void){
	motor_response_msg_t* motor_data = get_motor_data_ptr();
	for(uint32_t i = 0; i < MOTOR_NUM; i++){
		if(abs(motor_data[i].speed_rpm) > RPM_THRESOLD){
			motor_id_msg[7] = '0' + i;
			mem_log_print(motor_id_msg,'I');
		}
	}
}

void motor_spin(int16_t rpm_ref){
	pid_struct_t test_pid;
	motor_transmit_msg_t tx_msg;
	motor_response_msg_t* motor_rx = get_motor_data_ptr();
	memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
	pid_init(&test_pid,80,0,0,0,16384,0);

	//HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_0);
	
	if(get_motor_rx_flags(0xFFFF) != 0){
		clear_motor_rx_flags(0xFFFF);
		tx_msg.D[0] = (int16_t)pid_calc(&test_pid,rpm_ref,motor_rx[4].speed_rpm);
		set_motor_output(&hcan1,&tx_msg,M6020_ID_BASE);
	}
}

void motor_set_angle(uint16_t ang_ref){
	pid_struct_t test_pid;
	motor_transmit_msg_t tx_msg;
	motor_response_msg_t* motor_rx = get_motor_data_ptr();
	memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
	pid_init(&test_pid,20,0,0,0,16384,0);

	//HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_0);
	if(get_motor_rx_flags(0xFFFF) != 0){
		clear_motor_rx_flags(0xFFFF);
		float delta = get_delta_ang(ang_ref,motor_rx[4].raw_angle);
		tx_msg.D[0] = (int16_t)pid_calc(&test_pid,delta,0);
		set_motor_output(&hcan1,&tx_msg,M6020_ID_BASE);
	}
}

void motor_vibrate(void){
	static int d_out = 0;
	motor_transmit_msg_t tx_msg;
	motor_response_msg_t* motor_rx = get_motor_data_ptr();
	memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
	
	rc_ctrl_t* rc = get_rc_data_ptr();
	
	if(d_out == 0){
		d_out = 1;
		tx_msg.D[(CAN_SHOOT_RIGHT_ID % 0x04)] = 3000;
		tx_msg.D[(CAN_SHOOT_LEFT_ID % 0x04)] = 3000;
	}else{
		d_out = 0;
		tx_msg.D[(CAN_SHOOT_LEFT_ID % 0x04)] = -3000;
		tx_msg.D[(CAN_SHOOT_RIGHT_ID % 0x04)] = -3000;
	}
	if(rc->rc.switch_right == RC_SW_MID){
		if(get_motor_rx_flags(0xFFFF) != 0){
			clear_motor_rx_flags(0xFFFF);
			set_motor_output(&hcan2,&tx_msg,C620_ID_EXTEND);
		}
	}else{
		memset(&tx_msg,0,sizeof(motor_transmit_msg_t));
		set_motor_output(&hcan2,&tx_msg,C620_ID_EXTEND);
	}
	
}

