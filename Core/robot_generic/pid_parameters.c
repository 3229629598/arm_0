#include "pid_parameters.h"

#include "base_drv/drv_conf.h"
/*PID参数文件*/
//chassis pid
//------------------------------------------------------------------------------------------
const pid_struct_t wheels_pid_init_val={
	.kp = 40.0f,
	.ki = 0.1f,
	.kd = 0.0f,
	.i_max = C620_OUTPUT_MAX/10,
	.out_max = C620_OUTPUT_MAX,
	.k_deadband = 0.0f
};

const pid_struct_t arm_ang_pid_val ={
	.kp = 0.5f,
	.ki = 0.0f,
	.kd = 0.0f,
	.i_max = C620_OUTPUT_MAX/5,
	.out_max = C620_OUTPUT_MAX,
	.k_deadband = 0.0f
};

const pid_struct_t arm_rpm_pid_val ={
	.kp = 10.0f,
	.ki = 0.05f,
	.kd = 1.0f,
	.i_max = C620_OUTPUT_MAX/5,
	.out_max = C620_OUTPUT_MAX,
	.k_deadband = 0.0f
};
