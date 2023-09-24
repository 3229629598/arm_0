#ifndef _EXT_IMU_H
#define _EXT_IMU_H

#include "drv_conf.h"
#include HAL_INCLUDE

#include "../algorithm/imu_fusion.h"

#define EXT_IMU_RX_MAX_LOST 5

extern eular_t ext_eular;
extern imu_data_fp_t ext_imu_data;

HAL_StatusTypeDef parse_imu_data(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer);

void inc_ext_imu_rx_lost(void);
uint32_t is_ext_imu_offline(void);


#endif


