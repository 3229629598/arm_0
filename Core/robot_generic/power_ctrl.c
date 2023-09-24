#include "power_ctrl.h"

#include "base_drv/drv_conf.h"
#include "base_drv/super_cap.h"
#include "base_drv/referee.h"

#include "pid_parameters.h"

#include <math.h>
#include <string.h>

static uint8_t low_voltage_flag = 0;
static float cap_voltage_peak = 0;

extern CAN_HandleTypeDef SUPER_CAP_CAN_HANDLE;

pid_struct_t power_limit_pid;
pid_struct_t power_cap_pid;

void power_ctrl_init(chassis_power_lim_t* chassis_power_lim){
	//pid_init(&power_limit_pid, 4.0f,0.25f,0.3f,C620_OUTPUT_MAX/350,C620_OUTPUT_MAX/100,0);
	//pid_init(&power_cap_pid, 4.9f,0.3f,0.0f,C620_OUTPUT_MAX/350,C620_OUTPUT_MAX/100,0);
	memcpy(&power_limit_pid,&power_limit_pid_init_val,sizeof(pid_struct_t));
	memcpy(&power_cap_pid,&power_cap_pid_init_val,sizeof(pid_struct_t));

	//保持以下初始化值等同于不限制功率
	chassis_power_lim->max_total_out = 4.0f*C620_OUTPUT_MAX;
	chassis_power_lim->vxy_output_max = chassis_vx_pid_init_val.out_max;
	chassis_power_lim->wz_output_max = chassis_wz_pid_init_val.out_max;
	chassis_power_lim->vxy_ratio = 1.0f;
	chassis_power_lim->wz_ratio = 1.0f;
}

float pid_limit_low(pid_struct_t* m_pid, float min,float limit_target,float current){
	float limit_output = 1.0f;
	if(current < min){
		limit_output = pid_calc(m_pid, limit_target,current);
	}else{
		if(m_pid->i_out > m_pid->ki){
			m_pid->i_out -= I_RESTORE_RATE * m_pid->ki;
			limit_output = m_pid->i_out;
		}else limit_output= 1;
	}
	return limit_output;
}
/*
功率限制函数，适用于存在功率限制
计算输出为输出衰减倍率(ratio)和输出最大值，其中可以通过use_buffer指定两者状态
抑制超级电容状态：该状态希望维持超级电容电压，使其不下降，
				抑制状态下限幅将按照裁判系统给出的最大功率进行限制，削平起步时的功率尖峰，同时当电压低于上一次的峰值时将用PID计算衰减率限制输出
释放超级电容状态：该状态允许超级电容电压下降至额定值12V（C620电调在12V以下工作状态异常）
				释放状态下输出不作限幅，当电压低于警戒值（12V以上的某个值）时开始如同抑制状态一样限制输出，确保工作状态正常
*/
void power_ctrl(chassis_power_lim_t* chassis_power_lim,uint16_t max_power,uint8_t use_buffer){
	float result_ratio = 1;
	//uint16_t max_power = (uint16_t)(game_robot_status.chassis_power_limit);
	float max_out_voltage, max_out_current;
		
	if(max_power < 40 || max_power > 150) max_power = 50;
	
	max_out_current = ((max_power/cap_data.cap_voltage)/20.0f*C620_OUTPUT_MAX);
	if(max_out_current > MAX_CURRENT_LIMIT) max_out_current = MAX_CURRENT_LIMIT;

	//max_power -= 3;
	if(power_heat_data.chassis_power_buffer < 15.0f) max_power -= 3;
	//else if(use_buffer == 2) max_power += (PowerHeatData.chassis_power_buffer-15)*0.5;
	
	if((uint16_t)cap_data.power_set != max_power){
		if(max_power > MAX_POWER_OUT) max_power = MAX_POWER_OUT;
		set_cap_power(&SUPER_CAP_CAN_HANDLE,max_power);
	}
	
	if(cap_data.cap_voltage < LOW_VOLTAGE_THESOLD_L 
		&& cap_voltage_peak < VOLTAGE_LOW_PEAK_THESOLD) low_voltage_flag = 1;
	else if(cap_data.cap_voltage > LOW_VOLTAGE_THESOLD_H) low_voltage_flag = 0;
	
	if(use_buffer != 0 || low_voltage_flag){
		//use cap buffer, limit the power output when cap voltage is low
		//使用超级电容电压，允许下降
		result_ratio = pid_limit_low(&power_cap_pid,
							LOW_VOLTAGE_THESOLD_H,LOW_VOLTAGE_THESOLD_H,
										cap_data.cap_voltage);
		cap_voltage_peak = VOLTAGE_LOW_PEAK_THESOLD;

		if(low_voltage_flag){
			chassis_power_lim->max_total_out = max_out_current * 4.0f * OVERLOAD_RATIO;
			chassis_power_lim->vxy_output_max = max_out_current * 2.0f * OVERLOAD_RATIO;
		}else{
			chassis_power_lim->max_total_out = 4*C620_OUTPUT_MAX;
			chassis_power_lim->vxy_output_max = 2*C620_OUTPUT_MAX;
		}

		result_ratio = fabsf(result_ratio);
	}else{ 
		//don't use cap buffer,maintain voltage
		//抑制模式，保持电容电压，等同于不使用电容
		if(cap_data.cap_voltage > cap_voltage_peak) 
			cap_voltage_peak =cap_data.cap_voltage;
		
		max_out_voltage = (cap_data.input_voltage - 2.0f)*0.95f;
		
		if(cap_voltage_peak > max_out_voltage) 
			cap_voltage_peak = max_out_voltage;
		
		result_ratio = pid_limit_low(&power_limit_pid,cap_voltage_peak
									,cap_voltage_peak,cap_data.cap_voltage);
		result_ratio = fabsf(result_ratio);
		
		//chassis_power_lim->vxy_output_max = max_out_current;
		chassis_power_lim->max_total_out = max_out_current * 4.0f * OVERLOAD_RATIO;
		chassis_power_lim->vxy_output_max = max_out_current * 2.0f * OVERLOAD_RATIO;
	}
	
	if(result_ratio < 1) result_ratio = 1;

	chassis_power_lim->vxy_ratio = result_ratio;
	if(result_ratio > WZ_LIMIT_THESOLD) 
		chassis_power_lim->wz_ratio = result_ratio/WZ_LIMIT_THESOLD;
	else chassis_power_lim->wz_ratio = 1;
}

