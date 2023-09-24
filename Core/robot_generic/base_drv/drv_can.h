#ifndef _DRV_CAN_H
#define _DRV_CAN_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if MOTOR_CAN_ENABLE == 1

HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef* hcan);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif

#endif

