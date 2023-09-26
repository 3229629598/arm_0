#ifndef arm_h
#define arm_h

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "can.h"
#include "base_drv/rc.h"
#include "algorithm/crc.h"
#include "algorithm/pid.h"
#include "algorithm/util.h"
#include "base_drv/drv_uart.h"
#include "base_drv/motor_ctrl.h"
#include "base_drv/drv_conf.h"
#include "pid_parameters.h"
#include "string.h"
#include "math.h"
#include "usbd_cdc_if.h"

#define Fixed 0
#define Prismatic 1
#define Revolute 2

typedef struct
{
	uint8_t joint_type;
	
	float min_pos;
	float max_pos;
	float pos_range;
	float r;

	uint8_t boot_flag;
	
	uint8_t use_reset;
	float reset_current;
	float reset_threshold;
	uint16_t reset_cnt;
	uint16_t cnt;
	
	struct
	{
		float goal_pos;
		float goal_val;
		float cur_pos;
		float cur_val_sum;
		float cur_val;
		int16_t cur_rpm;
		int16_t cur_current;
		float last_val;
		uint8_t stuck_check;
	}state;
	
	pid_struct_t pid[2];
	
}joint;

joint joint_init(uint8_t joint_type,float min_pos,float max_pos,float r,int16_t reset_current,float reset_threshold,uint16_t reset_cnt,pid_struct_t pid_ptr_ang,pid_struct_t pid_ptr_rpm);

void arm_init(void);
void arm_loop(rc_ctrl_t* rc_data);

#define e 2.71828182845904523536f

#endif
