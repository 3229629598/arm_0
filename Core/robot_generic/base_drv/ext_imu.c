#include "ext_imu.h"

#if USE_EXT_CAN_IMU == 1

#include <string.h>

eular_t ext_eular;
imu_data_fp_t ext_imu_data;

static uint16_t ext_imu_rx_lost = EXT_IMU_RX_MAX_LOST;

/**
  * @struct imu_can_t
  * @brief the feedback data struct of imu
  */
typedef __packed struct{
	int16_t raw_yaw_eular; //0.1°
	int16_t raw_pitch_eular; //0.01°
	int16_t raw_yaw_gyo;// 0.1°/s
	int16_t raw_pitch_gyo;// 0.1°/s
}imu_can_t;

imu_can_t imu_raw_data;


/**
  * @brief parse can data from imu can board and store it
  * @param[in]  rx_header  the CAN header
  * @param[in]  rx_buffer  raw data(8 bytes) received
  * @return HAL_OK if the id is match otherwise HAL_ERROR
  */
HAL_StatusTypeDef parse_imu_data(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer){
	if(rx_header->StdId == IMU_CAN_RX_ID){
		memcpy(&imu_raw_data,rx_buffer,CAN_DATA_LEN);
		ext_eular.yaw = ((float)imu_raw_data.raw_yaw_eular / 10.0f + 180.0f)*ANGLE_RANGE/360; //yaw
		ext_eular.pitch = ((float)imu_raw_data.raw_pitch_eular / 100.0f + 180.0f)*ANGLE_RANGE/360; //pitch
		ext_imu_data.gz = ((float) imu_raw_data.raw_yaw_gyo /10.0f)*PI/180; //yaw
		ext_imu_data.gy = -((float) imu_raw_data.raw_pitch_gyo /10.0f)*PI/180; //Pitch

		ext_imu_rx_lost = 0;
		return HAL_OK;
	}else return HAL_ERROR;
}

/**
  * @brief  increment of ext_imu_rx_lost counter, should be call after process of data
  * @return none
  */
void inc_ext_imu_rx_lost(void){
	if(ext_imu_rx_lost < EXT_IMU_RX_MAX_LOST) ext_imu_rx_lost++;
}

/**
  * @brief  check if external imu module is offline
  * @return 0 for external imu module online, others for offline
  */
uint32_t is_ext_imu_offline(void){ return ext_imu_rx_lost >= EXT_IMU_RX_MAX_LOST; }


#endif
