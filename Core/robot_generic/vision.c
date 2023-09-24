#include "vision.h"

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE

#include <string.h>

#include "base_drv/referee.h"
#include "base_drv/motor_ctrl.h"

#include "algorithm/crc.h"

#include "shoot.h"

#if VISION_INF_USB == 0
#include "base_drv/drv_uart.h"

int32_t vision_frame_offset = 0;
int32_t vision_receive_size = 0;

uint8_t vision_rx_buffer[VISION_CTRL_FRAME_LEN*2];

extern UART_HandleTypeDef VISION_UART_HANDLE;
#else

#include "../Inc/usbd_cdc_if.h"

#endif

uint16_t vision_rx_lost = VISION_RX_LOST_MAX;
uint8_t vision_req_flag = 0;
vision_req_t vision_request;
vision_ctrl_t vision_control_data;

uint8_t current_task_mode = MODE_AUTO_AIM;
uint8_t armour_state = NONARMOR;
float current_pitch, current_yaw;
float vision_pitch, vision_yaw;



/**
  * @brief      return the self color for vision
  * @return     0 for red, others for blue
  */
uint8_t is_red_or_blue(void){
	return (game_robot_status.robot_id > 100);
}

void set_task_mode(uint8_t tm){ current_task_mode = tm; }

uint8_t get_vision_cmd(void){ return vision_control_data.command;}

void set_vision_req_ang(float pitch, float yaw){
	current_pitch = pitch;
	current_yaw = yaw;
}

uint8_t parse_vision_data(uint8_t* buf, uint16_t len){
	if(buf[0] == VISION_UART_CTRL_SOF && len == VISION_CTRL_FRAME_LEN &&
		Verify_CRC16_Check_Sum(&buf[0], VISION_CTRL_FRAME_LEN)){
		memcpy((uint8_t*)&vision_control_data, buf, VISION_CTRL_FRAME_LEN);
		
		if(vision_control_data.command == GET_ANGLE){
			vision_req_flag = 1;
		}else if(vision_control_data.command == REQUEST_TRANS){
			vision_pitch = vision_control_data.cmd_pitch;
			vision_yaw = vision_control_data.cmd_yaw;
			armour_state = REQUEST_TRANS;
		}else{
			armour_state = vision_control_data.command;
		}
		return VISION_DATA_NOERR;
	}else return VISION_DATA_ERR;
}

#if VISION_INF_USB == 0
HAL_StatusTypeDef vision_uart_dma_recv_init(void){
	vision_rx_lost = VISION_RX_LOST_MAX;
		
	__HAL_UART_ENABLE_IT(&(VISION_UART_HANDLE),UART_IT_IDLE);
	return uart_recv_dma_init(&(VISION_UART_HANDLE),vision_rx_buffer,VISION_CTRL_FRAME_LEN*2);
}

void vision_uart_idle_handler(UART_HandleTypeDef* huart){
	if(huart->Instance == VISION_UART && __HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE)){
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		DMA_Stream_TypeDef* uhdma = huart->hdmarx->Instance;
		vision_receive_size = (int32_t)(2*VISION_CTRL_FRAME_LEN - uhdma->NDTR) - vision_frame_offset;
		if(vision_receive_size == VISION_CTRL_FRAME_LEN || vision_receive_size == -VISION_CTRL_FRAME_LEN){
			if(parse_vision_data(&vision_rx_buffer[vision_frame_offset],VISION_CTRL_FRAME_LEN) == VISION_DATA_NOERR){
				vision_rx_lost = 0;
			}
			vision_frame_offset = (int32_t)(vision_frame_offset == 0)*VISION_CTRL_FRAME_LEN;
		}else if(vision_frame_offset != 0){
			//some bytes lost, reset DMA buffer
			CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			__HAL_DMA_DISABLE(huart->hdmarx);
			vision_frame_offset = 0;
			uhdma->NDTR = (uint32_t)(VISION_CTRL_FRAME_LEN*2);
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_CLEAR_FLAG(huart->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(huart->hdmarx));
			__HAL_DMA_ENABLE(huart->hdmarx);
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		}
	}
}
#else

void vision_usb_rx_handler(uint8_t* buf, uint32_t rx_len){
  	if(rx_len >= VISION_CTRL_FRAME_LEN) parse_vision_data(buf,VISION_CTRL_FRAME_LEN);
}

#endif
/**
  * @brief  send upper data
  * @return none
  */
void send_vision_request(void){
	vision_request.f_sof = VISION_UART_REQ_SOF;
	float shoot_speed = 15.0f;

	vision_request.self_color = is_red_or_blue();
	vision_request.task_mode = current_task_mode;
	vision_request.send_pitch = current_pitch;
	vision_request.send_yaw = current_yaw;
	
	if(vision_control_data.task_mode == MODE_SMALL_BUFF ||
		vision_control_data.task_mode == MODE_LARGE_BUFF){
		shoot_speed = 25.0f;
	}else{
		shoot_speed = game_robot_status.shooter_id1_17mm_speed_limit * SHOOT_SPEED_DEC;
		if(shoot_speed < DEFAULT_SHOOT_SPEED * SHOOT_SPEED_DEC) 
			shoot_speed = DEFAULT_SHOOT_SPEED * SHOOT_SPEED_DEC;
	}
	vision_request.shoot_speed[0] = (uint8_t)shoot_speed;
	vision_request.shoot_speed[1] = ((uint8_t)(shoot_speed*10.0f)) % 10;
	
	vision_request._adjust = 0;
	vision_request.crc16 = Get_CRC16_Check_Sum((uint8_t*)&vision_request
													,sizeof(vision_req_t) - 2, CRC16_INIT);
	#if VISION_INF_USB == 0
		uart_send(&(VISION_UART_HANDLE),(uint8_t*)&vision_request, sizeof(vision_req_t));
	#else
		CDC_Transmit_FS((uint8_t*)&vision_request, sizeof(vision_req_t));
	#endif
}

/**
  * @brief  check if vision upper is offline
  * @return 0 for  vision upper online, others for offline
  */
uint32_t is_vision_offline(void){ return vision_rx_lost >= VISION_RX_LOST_MAX; }


/**
  * @brief  increment of vision_rx_lost counter, should be call after process of all rc data
  * @return none
  */
void inc_vision_rx_lost(void){
	if(vision_rx_lost < VISION_RX_LOST_MAX) vision_rx_lost++;
}

/**
  * @brief  check vision request flag
  * @return 1 for vision upper request, others for not receive
  * @note calling this function will set vision_rx_flag to zero
  */
uint8_t is_vision_req(void){
	uint8_t rx = vision_req_flag;
	vision_req_flag = 0;
	return rx;
}


/**
  * @brief  get the current armour state
  * @return one of the value in e_ctrl_cmd
  */
uint8_t get_armour_state(void){ return armour_state; }

