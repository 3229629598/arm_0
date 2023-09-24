#include "motor_ctrl.h"

#if MOTOR_CAN_ENABLE == 1

#include <string.h> 

motor_response_msg_t motor_data[MOTOR_NUM];
//static motor_transmit_msg_t mMsg;

//bit fields to record motors receive or not
/*这是用于记录电机接收数据的变量，每个二进制位代表一个电机是否收到数据*/
RAM_PERSIST static uint16_t can_rx_flags;


/**
  * @brief clear all data and rx flags
  * @return none
  */
void motor_data_init(void){
	memset(motor_data, 0, sizeof(motor_response_msg_t)*MOTOR_NUM);
  	can_rx_flags = 0x0;
}

/**
  * @brief transfer raw data to pubilc struct
  * @param[in]  rx_header  the CAN header
  * @param[in]  rx_buffer  raw data(8 bytes) received
  * @return HAL_OK if success otherwise HAL_ERROR
  */
	/*处理电机数据函数，会在CAN接收中断的回调函数内被调用*/
HAL_StatusTypeDef parse_motor_data(CAN_TypeDef* can_ptr, CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer){
	uint32_t motor_id = rx_header->StdId - MOTOR_ID_OFFSET;
	#ifdef CAN_1_2_DIV
		if(can_ptr == HIGHER_ID_CAN) motor_id += CAN_1_2_DIV;
	#endif
	if(motor_id < MOTOR_NUM){
		motor_data[motor_id].raw_angle = ((rx_buffer[0] << 8) | rx_buffer[1]);
		motor_data[motor_id].speed_rpm = ((rx_buffer[2] << 8) | rx_buffer[3]);
		motor_data[motor_id].current = ((rx_buffer[4] << 8) | rx_buffer[5]);
		/*
		((uint32_t*)(&motor_data))[0]=__REV16(((uint32_t*)(rx_buffer))[0]);
		((uint32_t*)(&motor_data))[1]=__REV16(((uint32_t*)(rx_buffer))[1]);
		//motor_data.current = ((rx_buffer[4] << 8) | rx_buffer[5]);*/
		motor_data[motor_id].tempture = rx_buffer[6];
		
		//数据是否有效
		if(motor_data[motor_id].raw_angle < ANGLE_RANGE){
			//将对应电机的标志位置1
			can_rx_flags = can_rx_flags | (uint16_t)(0x01 << motor_id); //update rx flag
			return HAL_OK;
		}else{ //error data
			//将对应电机的标志位清零
			can_rx_flags = can_rx_flags & ~(uint16_t)(0x01 << motor_id); //clear invaild rx flag
			return HAL_ERROR;
		}
		
	}else return HAL_ERROR;
}



/**
  * @brief set the motor voltage
  * @param[in]  hcan    the CAN handler to transmit raw data
  * @param[in]  tx_msg   the pointer to voltage or current data struct (4 motor data per struct)
  * @param[in]  id      choose from C620_ID_BASE,C620_ID_EXTEND,M6020_ID_BASE,M6020_ID_EXTEND
  * @return HAL_OK if success otherwise HAL_ERROR
  */
HAL_StatusTypeDef set_motor_output(CAN_HandleTypeDef* hcan,motor_transmit_msg_t* tx_msg, uint32_t id){
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_buffer[CAN_DATA_LEN];
	//uint8_t* pTxMsg = (uint8_t*)tx_msg;
	uint32_t* ptx_msg = (uint32_t*)tx_msg;
	uint32_t can_mailbox;
	
	//endian convert
	/*
	for(uint8_t i=0;i<4;i++){
		tx_buffer[2*i+1]=ptx_msg[2*i];
		tx_buffer[2*i]=ptx_msg[2*i+1];
	}*/
	//此时直接调用ARM指令调转大小端
	*(uint32_t*)(&tx_buffer[0])=__REV16(ptx_msg[0]);
	*(uint32_t*)(&tx_buffer[4])=__REV16(ptx_msg[1]);
	tx_header.DLC = CAN_DATA_LEN;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = id;
	tx_header.TransmitGlobalTime = DISABLE;
	

	#if CAN_TX_TIMEOUT != 0
	uint32_t tickstart = HAL_GetTick();
	while(HAL_CAN_IsTxMessagePending(hcan,CAN_TX_MAILBOX0)){
		if(HAL_GetTick() - tickstart  > CAN_TX_TIMEOUT){
			HAL_CAN_AbortTxRequest(hcan,CAN_TX_MAILBOX0);
			break;
		}
	}
	#endif

	//异步发送
	return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer,&can_mailbox);
}

/**
  * @brief shut the motor output to 0
  * @param[in]  hcan  the CAN handler to transmit raw data
  * @param[in]  id    choose from C620_ID_BASE,C620_ID_EXTEND,M6020_ID_BASE,M6020_ID_EXTEND
  * @return HAL_OK if success otherwise HAL_ERROR
  */
HAL_StatusTypeDef shut_motors_output(CAN_HandleTypeDef* hcan, uint32_t id){
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_buffer[CAN_DATA_LEN];
	uint32_t can_mailbox;
	
	memset(tx_buffer ,0 ,CAN_DATA_LEN);

	tx_header.DLC = CAN_DATA_LEN;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = id;
	tx_header.TransmitGlobalTime = DISABLE;

	#if CAN_TX_TIMEOUT != 0
	uint32_t tickstart = HAL_GetTick();
	while(HAL_CAN_IsTxMessagePending(hcan,CAN_TX_MAILBOX0)){
		if(HAL_GetTick() - tickstart  > CAN_TX_TIMEOUT){
			HAL_CAN_AbortTxRequest(hcan,CAN_TX_MAILBOX0);
			break;
		}
	}
	#endif

	return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer,&can_mailbox);
}

/**
  * @brief  get a copy of motor response data
  * @param[in]  hcan  the CAN handler to receive motor data
  * @param[in]  m_id  the specific motor id
  * @return a copy of data in motor_response_msg_t
  */
motor_response_msg_t get_motor_data(CAN_HandleTypeDef* hcan, uint32_t m_id){
	motor_response_msg_t tmp;
	memset(&tmp, 0, sizeof(motor_transmit_msg_t));
	if(m_id >= MOTOR_NUM) return tmp;
	//关中断防止读取操作中发生中断使读取数据无效
	__HAL_CAN_DISABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	__HAL_CAN_DISABLE_IT(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
	tmp = motor_data[m_id];
	__HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	__HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);
	return tmp;
}

/**
  * @brief  get the pointer of motor response data
  * @return pointer of motor_response_msg_t
  */
motor_response_msg_t* get_motor_data_ptr(void){
	return motor_data;
}


/**
  * @brief  get the specific motors rx flags by masks
  * @param[in]  masks: the mask (bit set to 1) to get the flags
  * @return the desired flags 
  */
uint16_t get_motor_rx_flags(uint16_t masks){
	//通过掩码获取对应电机组的接收情况
	return (can_rx_flags & masks);
}

/**
  * @brief  clear the specific motors rx flags by masks
  * @param[in]  masks: the mask (bit set to 1) to clear the flags
  * @return none
  * @note use 0xFFFFF to clear all rx flags   
  */
void clear_motor_rx_flags(uint16_t masks){
	can_rx_flags = can_rx_flags & (~masks); // clear rx flags
}

#endif

