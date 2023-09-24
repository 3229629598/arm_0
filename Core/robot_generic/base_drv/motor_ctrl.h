#ifndef _MOTOR_CTRL_H
#define _MOTOR_CTRL_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if MOTOR_CAN_ENABLE == 1
//-----------------------------

//receive id
#define MOTOR_ID_OFFSET 0x201U

//transmit data stdid
#define C620_ID_BASE    0x200U
#define C620_ID_EXTEND  0x1ffU
#define M6020_ID_BASE   0x1ffU
#define M6020_ID_EXTEND 0x2ffU

/**
  * @struct motor_response_msg_t
  * @brief the struct that stores motor data receive from CAN
  */
typedef struct{
	uint16_t raw_angle;  ///< the ogrinal angle (range from 0 to 8191)
	int16_t speed_rpm;   ///< the ogrinal spinning speed (in rpm)
	int16_t current;     ///< the torque current(16 bit)
	uint8_t tempture;    ///< the tempture
}motor_response_msg_t;


/**
  * @struct motor_transmit_msg_t
  * @brief the struct stores motor data for transmission
  */
typedef struct{
	int16_t D[4]; ///< the array of 4 16bit data
}motor_transmit_msg_t;



void motor_data_init(void);

HAL_StatusTypeDef parse_motor_data(CAN_TypeDef* can_ptr, CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer);

HAL_StatusTypeDef set_motor_output(CAN_HandleTypeDef* hcan,motor_transmit_msg_t* TxMsg, uint32_t id);
HAL_StatusTypeDef shut_motors_output(CAN_HandleTypeDef* hcan, uint32_t id);

motor_response_msg_t get_motor_data(CAN_HandleTypeDef* hcan, uint32_t m_id);
motor_response_msg_t* get_motor_data_ptr(void);
uint16_t get_motor_rx_flags(uint16_t masks);
void clear_motor_rx_flags(uint16_t masks);

static inline int16_t get_delta_ang(int16_t ang1, int16_t ang2){
	return (ang1 - ang2 + ANGLE_RANGE*3/2) % ANGLE_RANGE- ANGLE_RANGE/2;
}

#endif

#endif

