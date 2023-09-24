#ifndef _ROBOT_INC_H
#define _ROBOT_INC_H

#include "stm32f4xx_hal.h"
#include "base_drv/rc.h"
#include "base_drv/drv_can.h"
#include "usart.h"
#include "arm.h"
#include "base_drv/referee.h"
#include "base_drv/drv_conf.h"

void robot_init(void);
void robot_loop(void);

#endif
