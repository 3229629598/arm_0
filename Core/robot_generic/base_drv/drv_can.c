#include "drv_can.h"

#if MOTOR_CAN_ENABLE == 1

#include "motor_ctrl.h"
#include "super_cap.h"
#include "ext_imu.h"

#define SINGLE_FILTER(can, num, id, fifo) if(hcan->Instance == (can)){ 						\
										can_filter.FilterBank = (num); 						\
										can_filter.FilterIdHigh = (id) << 5;				\
										can_filter.FilterIdLow  = 0;						\
										can_filter.FilterMaskIdHigh = 0;					\
										can_filter.FilterMaskIdLow  = 0;					\
										can_filter.FilterFIFOAssignment = fifo;	\
										HAL_CAN_ConfigFilter(hcan, &can_filter);			\
										} 

/**
  * @brief  can fifo 0 rx callback, get motor feedback info
  * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @note this is a weak function in HAL
  * @return None
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_buffer[CAN_DATA_LEN];
	//判断读取FIFO0是否有效
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer) == HAL_OK){
		#if USE_EXT_CAN_IMU == 1
      //处理外置IMU响应数据
			if(rx_header.StdId == IMU_CAN_RX_ID){
				parse_imu_data(&rx_header,rx_buffer);
			}else{
				parse_motor_data(hcan->Instance,&rx_header,rx_buffer);
			}
		#else
      //处理电机数据
			parse_motor_data(hcan->Instance,&rx_header,rx_buffer);
		#endif
	}
}

/**
  * @brief  can fifo 1 rx callback, get motor feedback info
  * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @return None
  * @note this is a weak function in HAL
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_buffer[CAN_DATA_LEN];
  //判断读取FIFO1是否有效
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_buffer) == HAL_OK){
		
	#if USE_SUPER_CAP == 1
    //处理超级电容响应数据
		if(rx_header.StdId == CAP_RESPONSE_ID){
			parse_cap_data(&rx_header,rx_buffer);
		}else{
			parse_motor_data(hcan->Instance,&rx_header,rx_buffer);
		}
  #else
    parse_motor_data(hcan->Instance,&rx_header,rx_buffer);
  #endif
	}
}

/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @return HAL_OK if success otherwise HAL_ERROR
  */
/*
  CAN 相关过滤器初始化，目的是使CAN总线的数据能均匀进入两个FIFO，防止缓冲区满丢数据
  FIFO0:0x201-0x204, 0x209, 0x20A,0x20B
  FIFO1:0x205-0x208, 外置IMU,超级电容
  可以通过宏SINGLE_FILTER仿照外置IMU的例子配置单个过滤器，此时每个过滤器分别单独占用不同的FilterBank
*/
HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef* hcan){
  CAN_FilterTypeDef  can_filter;

  //CAN_ID_STD = 0x0,CAN_RTR_DATA=0x0
  can_filter.FilterMode =  CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 0) << 5;//32位ID，高16位
  can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 1) << 5;//低16位
  can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 2) << 5;
  can_filter.FilterMaskIdLow  = (MOTOR_ID_OFFSET + 3) << 5;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // assign to fifo0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;

  if(hcan->Instance == CAN2){
       can_filter.FilterBank = 14;
  }else{
       can_filter.FilterBank = 0;
  }


  if(HAL_CAN_ConfigFilter(hcan, &can_filter) == HAL_OK){ // init can filter
  	//no error HAL_CAN_ERROR_NOT_INITIALIZED
  	can_filter.FilterBank = can_filter.FilterBank + 1;
  	can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 4) << 5;//32位ID，高16位
  	can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 5) << 5;//低16位
  	can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 6) << 5;
  	can_filter.FilterMaskIdLow  = (MOTOR_ID_OFFSET + 7) << 5;
  	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  	HAL_CAN_ConfigFilter(hcan, &can_filter);
	  
	can_filter.FilterBank = can_filter.FilterBank + 1;
  	can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 8) << 5;//32位ID，高16位
  	can_filter.FilterIdLow  = (MOTOR_ID_OFFSET + 9) << 5;//低16位
  	can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 10) << 5;
  	can_filter.FilterMaskIdLow  = 0;
  	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  	HAL_CAN_ConfigFilter(hcan, &can_filter);

    #if USE_SUPER_CAP == 1
	  SINGLE_FILTER(SUPER_CAP_USE_CAN,CAP_CAN_FILTER_NUM,CAP_RESPONSE_ID, CAN_FILTER_FIFO1);
    #endif
	
	#if USE_EXT_CAN_IMU == 1
		SINGLE_FILTER(IMU_USE_CAN,IMU_CAN_FILTER_NUM,IMU_CAN_RX_ID, CAN_FILTER_FIFO0);
	#endif

  	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can rx interrupt
  	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	__HAL_CAN_ENABLE_IT(hcan,CAN_IT_BUSOFF);
	
  	return HAL_CAN_Start(hcan);                          // start can
  }else{
  	return  HAL_ERROR;
  }
}

#endif

