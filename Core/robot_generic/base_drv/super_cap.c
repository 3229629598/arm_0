#include "super_cap.h"

#if USE_SUPER_CAP == 1

#include <string.h>

//setup super cap default value
RAM_PERSIST cap_data_t cap_data = {
	.input_voltage = 24.0f,
	.cap_voltage = 21.9f,
	.power_set = 50
};

static uint16_t cap_rx_lost = SUPER_CAP_RX_MAX_LOST;

extern CAN_HandleTypeDef SUPER_CAP_CAN_HANDLE;

/**
  * @brief parse can data from super cap board and store it
  * @param[in]  rx_header  the CAN header
  * @param[in]  rx_buffer  raw data(8 bytes) received
  * @return HAL_OK if the id is match otherwise HAL_ERROR
  */
HAL_StatusTypeDef parse_cap_data(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer){
	uint16_t* power_data = (uint16_t*)rx_buffer;
	if(rx_header->StdId == CAP_RESPONSE_ID){
		cap_data.input_voltage = (float)power_data[0] / 100.0f;
		cap_data.cap_voltage = (float)power_data[1] / 100.0f;
		cap_data.input_current = (float)power_data[2] / 100.0f;
		cap_data.power_set = (float)power_data[3] / 100.0f;
		
		cap_rx_lost = 0;
		return HAL_OK;
	}else return HAL_ERROR;
}

/**
  * @brief  get a copy of super cap response data
  * @param[in]  hcan  the CAN handler to receive super cap response data
  * @return a copy of data in cap_data_t
  */
cap_data_t get_cap_data(void){
	cap_data_t tmp;
	//__HAL_CAN_DISABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	__HAL_CAN_DISABLE_IT(&(SUPER_CAP_CAN_HANDLE),CAN_IT_RX_FIFO1_MSG_PENDING);
	tmp = cap_data;
	//__HAL_CAN_ENABLE_IT(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	__HAL_CAN_ENABLE_IT(&(SUPER_CAP_CAN_HANDLE),CAN_IT_RX_FIFO1_MSG_PENDING);
	return tmp;
}


/**
  * @brief set the super cap power limit
  * @param[in]  hcan  the CAN handler to transmit raw data
  * @param[in]  set_power  the expected power limit
  * @return HAL_OK if transmission was success otherwise HAL_ERROR
  */
HAL_StatusTypeDef set_cap_power(CAN_HandleTypeDef* hcan, uint16_t set_power){
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_buffer[CAN_DATA_LEN];
	uint32_t can_mailbox;
	
	memset(tx_buffer ,0 ,CAN_DATA_LEN);

	tx_header.DLC = CAN_DATA_LEN;
	tx_header.IDE = CAN_ID_STD;
  	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = CAP_TRANSMIT_ID;
	tx_header.TransmitGlobalTime = DISABLE;

	set_power = set_power *100;
	tx_buffer[0] = (uint8_t)(set_power >> 8);
	tx_buffer[1] = (uint8_t)(set_power & 0xff);

	return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_buffer,&can_mailbox);
}

/**
  * @brief  increment of cap_rx_lost counter, should be call after process of data
  * @return none
  */
void inc_cap_rx_lost(void){
	if(cap_rx_lost < SUPER_CAP_RX_MAX_LOST) cap_rx_lost++;
}

/**
  * @brief  check if super cap module is offline
  * @return 0 for super cap module online, others for offline
  */
uint32_t is_cap_offline(void){ return cap_rx_lost >= SUPER_CAP_RX_MAX_LOST; }

#endif

