#ifndef _BUS_DETECT_H
#define _BUS_DETECT_H

#include "base_drv/drv_conf.h"
#include HAL_INCLUDE

void detect_loop(void);

uint32_t is_motor_online(uint32_t m_id);
uint32_t is_referee_sys_online(void);
uint32_t is_module_online(uint16_t mask);

void mpu_nx_protect(void);

inline uint32_t RCC_FLAG(uint32_t flags);

#define IS_POR (RCC_FLAG(RCC_CSR_PORRSTF))
#define IS_IWDG_RST (RCC_FLAG(RCC_CSR_IWDGRSTF))

#if _DEBUG == 1
#define FAULT_HANDLER __BKPT(0xbe);
#else
#define FAULT_HANDLER __set_FAULTMASK(1);NVIC_SystemReset();
#endif

#endif

