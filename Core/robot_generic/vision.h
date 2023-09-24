#ifndef _VISION_H
#define _VISION_H

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE

typedef __packed struct{
	uint8_t f_sof;
	float send_pitch;
	float send_yaw;
	uint8_t shoot_speed[2];
	uint8_t self_color;
	uint8_t task_mode;
	uint8_t _adjust;
	uint16_t crc16;
}vision_req_t;

typedef __packed struct{
	uint8_t f_sof;
	uint8_t command;
	uint8_t shoot_mode;
	uint8_t task_mode;
	float cmd_pitch;
	float cmd_yaw;
	uint8_t buff_status;
	uint8_t _reserved;
	uint16_t crc16;
}vision_ctrl_t;

#define VISION_INF_USB 0

#define VISION_UART_REQ_SOF 0x5A
#define VISION_UART_CTRL_SOF 0xAF
#define VISION_CTRL_FRAME_LEN (sizeof(vision_ctrl_t))

#define VISION_DATA_NOERR 0x00
#define VISION_DATA_ERR 0xFF

#define VISION_RX_LOST_MAX 30


enum e_ctrl_cmd {
    NONARMOR = 0xAA,
    GET_ANGLE = 0xBB,
    REQUEST_TRANS = 0xCC,
    NONARMOR_SLOW = 0xDD
};

enum e_buff_status {
    STATE_NONE = 0xEE,
    STATE_SMALL_BUF = 0xAA,
    STATE_LARGE_BUF = 0xBB
};

enum e_task_mode {
    MODE_AUTO_AIM = 0,
    MODE_SMALL_BUFF = 1,
    MODE_LARGE_BUFF = 2
};

enum e_shoot_mode {
    NO_FIRE = 0xDD,
    FIRE = 0xFF
};

extern vision_ctrl_t vision_control_data;

#if VISION_INF_USB == 0
#define VISION_UART_HANDLE huart1
#define VISION_UART USART1

void vision_uart_idle_handler(UART_HandleTypeDef* huart);
HAL_StatusTypeDef vision_uart_dma_recv_init(void);

#else

void vision_usb_rx_handler(uint8_t* buf, uint32_t rx_len);
#endif


void set_task_mode(uint8_t tm);
uint8_t get_vision_cmd(void);
void set_vision_req_ang(float pitch, float yaw);

void send_vision_request(void);

uint32_t is_vision_offline(void);
void inc_vision_rx_lost(void);

uint8_t get_armour_state(void);
uint8_t is_vision_req(void);

#endif
