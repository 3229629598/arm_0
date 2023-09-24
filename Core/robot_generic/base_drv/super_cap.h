#ifndef _SUPER_CAP_H
#define _SUPER_CAP_H

#include "drv_conf.h"
#include HAL_INCLUDE

#if USE_SUPER_CAP == 1

#define CAP_TRANSMIT_ID 0x210
#define CAP_RESPONSE_ID 0x211

#define SUPER_CAP_RX_MAX_LOST 20

/**
  * @struct cap_data_t
  * @brief the feedback data struct of super cap
  */
typedef struct
{
	float input_voltage;  ///< the input voltage 
	float cap_voltage;    ///< super cap(same as output voltage) voltage
	float input_current;  ///< the input current
	float power_set;      ///< target power set by user
}cap_data_t;

HAL_StatusTypeDef parse_cap_data(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_buffer);

HAL_StatusTypeDef set_cap_power(CAN_HandleTypeDef* hcan, uint16_t set_power);
cap_data_t get_cap_data(void);

void inc_cap_rx_lost(void);
uint32_t is_cap_offline(void);

extern cap_data_t cap_data;

#endif 

#endif
