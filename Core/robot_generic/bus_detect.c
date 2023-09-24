#include "bus_detect.h"

//#include "dwt.h"
#include "base_drv/motor_ctrl.h"
#include "base_drv/rc.h"
#include "base_drv/referee.h"

//电机/遥控离线管理，CAN错误管理，保护栈溢出，复位管理/看门狗模块管理

static uint16_t motor_frame_lost[MOTOR_NUM];
static uint32_t can_err_code;

#define MOTOR_MAX_LOST 50

void detect_loop(void){
	//collect all status data
	//can_err_code = HAL_CAN_GetState(hcan);
	//HAL_CAN_ERROR_RX_FOV0 overrun
	//HAL_CAN_ERROR_TX_ALST0
	//HAL_CAN_ERROR_TX_TERR0 
	//HAL_CAN_ERROR_NONE   
	//HAL_CAN_ERROR_BOF 
	uint8_t rc_rx_flag = is_rc_offline();
	uint16_t motor_rx_flags = get_motor_rx_flags(0xFFFF); //get all motors flags

	for(int i=0; i < MOTOR_NUM; i++){
		if((motor_rx_flags >> i) & 0x1
			|| can_err_code != HAL_CAN_ERROR_NONE){

			if(can_err_code == HAL_CAN_ERROR_BOF){
				motor_frame_lost[i] = MOTOR_MAX_LOST + 2;
			}else{
        if(motor_frame_lost[i] < MOTOR_MAX_LOST + 2)
				  motor_frame_lost[i]++;
			}
      //clear_motor_rx_flags(0x01 << i);
		}else{
			motor_frame_lost[i] = 0;
		}
	}

  


  //if(can_err_code == HAL_CAN_ERROR_BOF) HAL_CAN_Start(hcan);

  //is_module_online(CHASSIS_MOTORS_MASKS)
  //is_module_online(GIMBAL_MOTORS_MASKS)
  //is_module_online(SHOOT_MOTORS_MASKS)

}

uint32_t is_motor_online(uint32_t m_id){
	if(m_id >= MOTOR_NUM) return 0;
	return (motor_frame_lost[m_id] < MOTOR_MAX_LOST);
}


uint32_t is_module_online(uint16_t mask){
  return (get_motor_rx_flags(mask) == mask);
}

// void can_detect_bof(CAN_HandleTypeDef* hcan){
  
// }

/*内存保护，将栈设为不可执行*/
void mpu_nx_protect(void){
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* 禁止 MPU */
    HAL_MPU_Disable();

    /* 配置SRAM的MPU属性为不可执行 */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x20000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /*使能 MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


// void dwt_write_watch(uint32_t addr, uint32_t size) {
//     if(size > 0x800) return;
//   /*
//   const size_t num_comparators = (A_DWT->CTRL>>28) & 0xF;
//   if (comp_id > num_comparators) return;
//   dwt_config_t *config = &(A_DWT->COMP_CONFIG[comp_id]);
//   config->COMP = addr;
//   config->MASK = size;
//   // set list since this will enable the comparator
//   config->FUNCTION = 0x6; //0x5 for read, 0x6 for write
//   */
//     DWT->FUNCTION0 = 0x0;
//     SET_BIT(CoreDebug->DEMCR,CoreDebug_DEMCR_TRCENA_Msk);
// 	//CoreDebug_DEMCR_DWTENA_Msk
//     SET_BIT(DWT->CTRL,0x01);
//     DWT->COMP0 = addr;
//     DWT->MASK0 = size;
//     DWT->FUNCTION0 = 0x6; //0x5 for read, 0x6 for write
//     __DSB();
// }

// void disable_dwt(void){
// 	DWT->FUNCTION0 = 0x0;
// 	CLEAR_BIT(CoreDebug->DEMCR,CoreDebug_DEMCR_TRCENA_Msk);
// 	__DSB();
// }


// FUNCTION        Evaluation Performed        Event Generated
// 0b0000 (0x0)    None                        Comparator is Disabled
// 0b0100 (0x4)    Instruction Fetch           PC watchpoint debug event
// 0b0101 (0x5)    Read/Load Data Access       Watchpoint debug event
// 0b0110 (0x6)    Write/Store Data Access     Watchpoint debug event
// 0b0111 (0x7)    Read or Write Data Access   Watchpoint debug event


// #define SECT_START(_name_)                _name_##$$Base
// #define SECT_END(_name_)                  _name_##$$Limit
// extern const uint32_t SECT_START(STACK);  //end of the stack
// //extern uint32_t __initial_sp;
// //#define MAX_STACK_SIZE 0x400
// //sp_addr = (uint32_t)(&__initial_sp);

// void enable_stack_protect(uint32_t detect_size){
//     //setup an area in end of stack(low address) in DWT, it will send a debug event when written,
//     //the halt eventally lead to a watchdog reset
//     //a scatter load after reset may trigger DWT, please put stack segement into non-init segment
//     uint32_t addr =  (uint32_t)(&SECT_START(STACK)) + detect_size;
//     dwt_write_watch(addr,detect_size);
// }



/*
获取复位类型
READ_BIT(RCC->CSR, Flags) == Flags
Flags: 
RCC_CSR_LPWRRSTF   < Low-Power reset flag 
RCC_CSR_PINRSTF    < PIN reset flag 
RCC_CSR_PORRSTF    < POR/PDR reset flag 
RCC_CSR_SFTRSTF    < Software Reset flag 
RCC_CSR_IWDGRSTF   < Independent Watchdog reset flag 
RCC_CSR_WWDGRSTF   < Window watchdog reset flag 
RCC_CSR_BORRSTF    < BOR reset flag (Brown-out reset,欠压复位)

*/
inline uint32_t RCC_FLAG(uint32_t flags){
    return (READ_BIT(RCC->CSR, flags) == flags);
}

//MPU Manage
//void HAL_MPU_ConfigRegion  ( MPU_Region_InitTypeDef *  MPU_Init ) 
//void HAL_MPU_Enable  ( uint32_t  MPU_Control )

//Manual Reset
//__set_FAULTMASK(1); // 关闭所有中断
//NVIC_SystemReset(); // 复位

