#ifndef _PID_PARA_H
#define _PID_PARA_H

#include "algorithm/pid.h"
#include <string.h>

extern const pid_struct_t wheels_pid_init_val;

extern const pid_struct_t arm_ang_pid_val;
extern const pid_struct_t arm_rpm_pid_val;

#define INIT_PID_STRUCT(p,c) memcpy(&(p),&(c),sizeof(pid_struct_t));

#endif

