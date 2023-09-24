#include "robot.h"

#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void robot_init(void){
	rc_recv_dma_init();
//	referee_recv_dma_init();
	motor_data_init();	
	can_user_init(&hcan1);
	can_user_init(&hcan2);
	arm_init();
	HAL_TIM_Base_Start_IT(&htim3);
}

void robot_loop(void){
	rc_ctrl_t* rc_data = get_rc_data_ptr();
	update_rc_last_key();
	inc_rc_rx_lost();
	arm_loop(rc_data);
//	inc_referee_rx_lost();

}
