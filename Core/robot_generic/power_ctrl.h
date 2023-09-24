#ifndef _POW_CTRL_H
#define _POW_CTRL_H

#include "base_drv/chassis_base.h"
#include "algorithm/pid.h"

//超级电容最大输出(W)
#define MAX_POWER_OUT 135
//最大输出电流限制(16384进制->20A)
#define MAX_CURRENT_LIMIT 3000

//低压保护触发电压下限(V)
#define LOW_VOLTAGE_THESOLD_L 12.0f
//低压保护触发电压上限(V)
#define LOW_VOLTAGE_THESOLD_H 13.8f
//电容抑制模式最低峰值电压(V)
#define VOLTAGE_LOW_PEAK_THESOLD 12.0f

//换算出的最大电流可调节的倍率
#define OVERLOAD_RATIO 1.8f
//底盘自旋速度限制开始的限制衰减率(相比于平移速度衰减率)
#define WZ_LIMIT_THESOLD 5
//单向PID恢复速率，越小恢复越慢越不容易出现抖动，但抑制响应越慢
#define I_RESTORE_RATE 0.6f

void power_ctrl_init(chassis_power_lim_t* chassis_power_lim);
void power_ctrl(chassis_power_lim_t* chassis_power_lim,uint16_t max_power,uint8_t use_buffer);

#endif
