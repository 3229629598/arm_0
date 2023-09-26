#include "arm.h"

joint joint_str[joint_num];
motor_response_msg_t* motor_ptr_a;

uint8_t use_moveit=1;
uint8_t use_custom_ctrl=0;
uint8_t use_key_ctrl=1;

uint8_t trajectory_begin;
uint16_t tx_cnt;
uint16_t tx_psc=10;
tx_bag tx_data;
uint8_t arm_request;

motor_transmit_msg_t arm_can_tx1;
motor_transmit_msg_t arm_can_tx2;

float motor_to_joint_pos(float val,float r)
{
  return val/8192.0f/REDUCTION_RATIO*2*PI*r;
}

float joint_to_motor_ang(float pos,float r)
{
  return 8192.0f*REDUCTION_RATIO/(2*PI*r)*pos;
}

float exponential_decay(float s,float s_max,float v_max,float v_min)
{
	return (v_max-v_min)*pow(e,1.0f/(1.0f-s_max/s))+v_min;
}

joint joint_init(uint8_t joint_type,float min_pos,float max_pos,float r,int16_t reset_current,float reset_threshold,uint16_t reset_cnt,pid_struct_t pid_ptr_ang,pid_struct_t pid_ptr_rpm)
{
	joint joint_str;
	
	joint_str.joint_type=joint_type;
	
	joint_str.min_pos=min_pos;
	joint_str.max_pos=max_pos;
	joint_str.pos_range=max_pos-min_pos;
	joint_str.r=r;
	
	joint_str.use_reset=1;
	joint_str.reset_current=reset_current;
	joint_str.reset_threshold=reset_threshold;
	joint_str.reset_cnt=reset_cnt;
	
	INIT_PID_STRUCT(joint_str.pid[0],pid_ptr_ang);
	INIT_PID_STRUCT(joint_str.pid[1],pid_ptr_rpm);
	
	joint_str.boot_flag=1;
	
	if(joint_type==Revolute)
	{
		joint_str.r=1;
	}
	
	return joint_str;
}

void arm_init(void)
{
	motor_ptr_a=get_motor_data_ptr();
	joint_str[0]=joint_init(Prismatic, 0, 0.2, 0.01, -300, 0.005, 300, arm_ang_pid_val, arm_rpm_pid_val);
}

void arm_loop(rc_ctrl_t* rc_data)
{
	for(int i=0;i<joint_num;i++)
	{
		joint_str[i].state.cur_val=motor_ptr_a[i].raw_angle;
		
		if(joint_str[i].boot_flag)
		{
			joint_str[i].state.last_val=joint_str[i].state.cur_val;			
			joint_str[i].state.cur_val_sum=joint_to_motor_ang(joint_str[i].min_pos,joint_str[i].r);
			joint_str[i].state.goal_pos=joint_str[i].min_pos;
			pid_reset(&joint_str[i].pid[0]);
			pid_reset(&joint_str[i].pid[1]);
			if(motor_ptr_a[i].tempture)
			{
				joint_str[i].boot_flag=0;
			}
		}
		joint_str[i].state.cur_current=motor_ptr_a[i].current;
		joint_str[i].state.cur_rpm=motor_ptr_a[i].speed_rpm;
		joint_str[i].state.cur_val_sum+=get_delta_ang(joint_str[i].state.cur_val,joint_str[i].state.last_val);
		joint_str[i].state.last_val=joint_str[i].state.cur_val;
		joint_str[i].state.cur_pos=motor_to_joint_pos(joint_str[i].state.cur_val_sum,joint_str[i].r);
	}
	
	if(use_moveit)
	{
		if(rx_data.flag)
		{
			trajectory_begin=1;
		}
		if(trajectory_begin)
		{
			for(int i=0;i<joint_num;i++)
			{
				joint_str[i].state.goal_pos=rx_data.joint_goal[i];
			}
		}
		
		if(use_custom_ctrl)
		{
			
		}
		
		arm_request=0xff;
		if(tx_cnt==0)
		{
			tx_data.header=tx_header;
			for(int i=0;i<joint_num;i++)
			{
				tx_data.joint_position[i]=joint_str[i].state.cur_pos;
			}
			tx_data.arm_request=arm_request;
			Append_CRC16_Check_Sum((uint8_t*)&tx_data,tx_len);	
			CDC_Transmit_FS((uint8_t*)&tx_data,tx_len);
		}
		tx_cnt++;
		tx_cnt%=tx_psc;
	}
	
	if(use_key_ctrl)
	{
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin))
		{
			trajectory_begin=0;
			joint_str[0].state.goal_pos+=joint_str[0].pos_range/3/tim3_f;
			LIMIT_MIN_MAX(joint_str[0].state.goal_pos,joint_str[0].min_pos,joint_str[0].max_pos);
		}
	}

	for(int i=0;i<joint_num;i++)
	{
		if(joint_str[i].state.cur_current>(joint_str[i].pid[1].out_max*0.9f) && abs(joint_str[i].state.cur_rpm)<10)
		{
			joint_str[i].state.stuck_check=1;
			joint_str[i].state.goal_pos=joint_str[i].state.cur_pos;
			trajectory_begin=0;
		}
		if(joint_str[i].use_reset && joint_str[i].state.cur_pos<joint_str[i].reset_threshold && joint_str[i].cnt<=joint_str[i].reset_cnt)
		{
			if(joint_str[i].cnt<joint_str[i].reset_cnt)
			{
				joint_str[i].cnt++;
				if(i<4)
				{
					arm_can_tx1.D[i]=joint_str[i].reset_current;
				}
				else
				{
					arm_can_tx2.D[i%4]=joint_str[i].reset_current;
				}
				if(joint_str[i].state.cur_rpm==0)
				{
					joint_str[i].cnt=joint_str[i].reset_cnt;
				}
			}
			if(joint_str[i].cnt==joint_str[i].reset_cnt)
			{
				memset(&joint_str[i].state,0,sizeof(joint_str[i].state));
				joint_str[i].cnt++;
				joint_str[i].boot_flag=1;
			}
		}
		else
		{
			joint_str[i].state.goal_val=joint_to_motor_ang(joint_str[i].state.goal_pos,joint_str[i].r);
			if(i<4)
			{
				arm_can_tx1.D[i]=pid_dual_loop(joint_str[i].pid,joint_str[i].state.goal_val-joint_str[i].state.cur_val_sum,joint_str[i].state.cur_rpm);
			}
			else
			{
				arm_can_tx2.D[i%4]=pid_dual_loop(joint_str[i].pid,joint_str[i].state.goal_val-joint_str[i].state.cur_val_sum,joint_str[i].state.cur_rpm);
			}
			if(joint_str[i].state.cur_pos>joint_str[i].reset_threshold)
			{
				joint_str[i].cnt=0;
			}
		}
	}
	set_motor_output(&CHASSIS_MOTORS_HCAN,&arm_can_tx1,C620_ID_BASE);
	set_motor_output(&CHASSIS_MOTORS_HCAN,&arm_can_tx2,C620_ID_EXTEND);
}
