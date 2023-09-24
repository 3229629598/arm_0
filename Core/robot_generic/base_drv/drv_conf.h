#ifndef _DRV_CONF_H
#define _DRV_CONF_H

#define HAL_INCLUDE "stm32f4xx_hal.h"///< define HAL header

/*
RAM_PERSIST宏为变量内存持久化的修饰符
某些关键变量在非上电复位时需要保持，一般初始化代码内在复位后会把所有变量清空，该修饰符可阻止编译器清空变量
未在KEIL中的sct文件增加无初始化段，或者启动代码内没有判断复位类型而不处理变量初始化全部清空，该宏无效可注释掉
*/
//#define RAM_PERSIST __attribute__((section("NO_INIT"),zero_init))  ///< marco to avoid variable initialized at reset

#ifndef RAM_PERSIST
#define RAM_PERSIST ///< disabled RAM_PERSIST
#endif

#define PI (3.14159265358979323846f)
#define _DEBUG 1	///< enable debug function

#define USE_RC_UART 1         ///< select if we use remote control uart(0 or 1)

#if USE_RC_UART == 1
/** @name remote control config
  * @note Usage: \n
  *       1.add rc_uart_idle_handle(&huartx) to stm32fxx_it.c(before IRQ_Handler) \n
  *       2.MX_USARTX_UART_Init() \n
  *       3.rc_recv_dma_init() \n
  * @{
  */
#define RC_UART USART1			///< select which uart remote control dbus uses (USARTx)
#define RC_UART_HANDLE huart1	///< select which uart handle remote control dbus uses (huartx)
/** @} remote control config */ 

#endif

#define USE_VT_RC_UART 0
/** @name VT remote control config
  * @note Usage: \n
  *       1.add rc_uart_idle_handle(&huartx) to stm32fxx_it.c(before IRQ_Handler) \n
  *       2.MX_USARTX_UART_Init() \n
  *       3.vt_rc_recv_dma_init() \n
  * @{
  */
#if USE_VT_RC_UART == 1
  #define VT_RC_UART USART1
  #define VT_RC_UART_HANDLE huart1
#endif

#define USE_REFEREE_UART 0      ///< select if we use referee user uart(0 or 1)
#if USE_REFEREE_UART == 1
/** @name referee system receive config
  * @note Usage: \n
  *       1.add referee_uart_idle_handle(&huartx) to stm32fxx_it.c (before IRQ_Handler) \n
  *       2.MX_USARTX_UART_Init() \n
  *       3.referee_recv_dma_init()\n
  * @{
  */
#define REFEREE_UART USART6			///< select which uart referee system feedback uses (USARTx)
#define REFEREE_UART_HANDLE huart6	///< select which uart handle referee system feedback uses (huartx)\
#define REFEREE_ASYNC_SEND 0        ///< select if referee send use uart_send_async(0 or 1)

/** @} referee system receive config */ 

#endif

//----------------------------------------------------------------

#define MOTOR_CAN_ENABLE 1      ///< select if we use CAN to receive motor date(0 or 1)
/** @name motor control config
  * @brief configure motors parameters
  * @note Usage: \n
  *       1.MX_CANX_Init() \n
  *       2.can_user_init(&hcanx); \n
  *       3.motor_data_init(); (clear motors data if needed) \n
  * @{
  */

/*定义电机总数量以预先分配缓冲区*/
#define MOTOR_NUM 16u			///< total motor numbers

/*
CAN_1_2_DIV 用于将两个CAN上ID重复的电机分离
当接收到HIGHER_ID_CAN上的电机数据时，将会把原始的电机ID对应的下标增加CAN_1_2_DIV后保存在缓冲区中，即将
HIGHER_ID_CAN上的电机ID后移HIGHER_ID_CAN个位置
*/
/** @name can_1_2_div
  * @brief motor id < CAN_1_2_ID_DIV = motors on HIGHER_ID_CAN
  *        motor id >= CAN_1_2_ID_DIV = motors on another CAN(still start from 0x201)
  *        undefine the constant to disable the feature
  * @{
  */
//#define CAN_1_2_DIV 4
//#define HIGHER_ID_CAN CAN1

/** @} can_1_2_div */

/** @name motor id list defines
  * @brief define index in the array motor_data
  * @{
  */
/*此处为电机的用户定义，需要根据机器人上的电机修改
该ID对应的是电机数据缓冲区的下标*/
#define CAN_M3508_RF_ID 0
#define CAN_M3508_LF_ID 1
#define CAN_M3508_LB_ID 2
#define CAN_M3508_RB_ID 3

#define joint1_can 8
#define joint2_can 9
#define joint3_can 10
#define joint4_can 11
#define joint5_can 12
#define joint6_can 13
/** @} motor id list defines */

/*CAN等待发送完成超时，单位毫秒，设为0即无阻塞的发送*/
#define CAN_TX_TIMEOUT 0		///< timeout in ms, set to 0 to disable

#define C620_OUTPUT_MAX 16384	///< C620 maximum output(range:-20~20A)
#define C610_OUTPUT_MAX 10000
#define GM6020_OUTPUT_MAX 30000	///< GM6020 maximum output
#define ANGLE_RANGE 8192		///< range of motor raw angle(13bit data)
#define CAN_DATA_LEN 8			///< can frame length

/** @} motor control config */

//-----------------------------------------------------------------

/** @name chasssis parameter configure
  * @note mecanum wheels name L=left,R=right,F=front,B=back \n
  *       motor id & position \n
  *       1             0 \n
  *       2             3 \n
  * @{
  */
#define CHASSIS_MOTORS_HCAN hcan1	///< select which CAN handle chassis control uses (hcanx)
#define arm_can hcan2
#define tim3_f 1000

/** @name chassis motor id list defines
  * @brief define index in the array motor_data
  * @{
  */
#define MOTOR_RF_ID CAN_M3508_RF_ID
#define MOTOR_LF_ID CAN_M3508_LF_ID
#define MOTOR_LB_ID CAN_M3508_LB_ID
#define MOTOR_RB_ID CAN_M3508_RB_ID
/** @} chassis motor id list defines */

/*底盘电机CAN接收标志位掩码*/
#define CHASSIS_MOTORS_MASKS 0x0Fu 			///< 4 motors masks of CAN rx flag
//#define CHASSIS_MOTORS_MASKS (0x01u << MOTOR_LF_ID)|(0x01u << MOTOR_LF_ID)| \
//                             (0x01u << MOTOR_LB_ID)|(0x01u << MOTOR_RB_ID) ///< 4 motors masks of CAN rx flag

/*云台电机CAN接收标志位掩码*/
//#define GIMBAL_MOTORS_MASKS 0x30
#define PITCH_RX_MASK (0x01u << CAN_GM6020_PITCH_ID)
#define YAW_RX_MASK (0x01u << CAN_GM6020_YAW_ID)
#define GIMBAL_MOTORS_MASKS (PITCH_RX_MASK | YAW_RX_MASK)   ///< gimbal motors masks of CAN rx flag

/*发射电机CAN接收标志位掩码*/
//#define SHOOT_MOTORS_MASKS (0x01u << CAN_BULLET_PUSHER_ID) | \
//                  (0x01u << CAN_SHOOT_LEFT_ID) | (0x01u << CAN_SHOOT_RIGHT_ID)
#define SHOOT_MOTORS_MASKS 0x70u   ///< shoot motors masks of CAN rx flag

/*是否为舵轮底盘（实验性功能，未测试）*/
#define USE_SWERVE_CHASSIS 0

/*底盘机械参数定义，
两者分别为左或右前轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
和左或右后轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
*/
#define KXY_FRONT 0.354f					///< front wheels X+Y(distance,m)
#define KXY_BACK  0.354f					///< bask wheels X+Y(distance,m)

/*底盘自旋校准因子，用于校正直接底盘逆向解算出的转速的比例误差
该参数直接影响无IMU的小陀螺时云台的漂移大小*/
#define K_CHASSIS_CAL 0.9514f

#if USE_SWERVE_CHASSIS == 1
#define K_SWERVE 0.3f
#define SWERVE_SPIN_ANG 45
#endif

/*3508的减速比，约为19:1*/
#define REDUCTION_RATIO (3591.0f/187.0f)	///< reduction ratio of M3508

#define reduction_ratio_2006 36.0f

/*麦克纳姆轮的尺寸*/
//reference: https://www.robomaster.com/zh-CN/products/components/detail/135
#define D_WHEEL (0.1525f)					///< diameter of mecanum wheel in meter
#define C_WHEEL (PI*D_WHEEL)			///< circumference of wheel in meter

/** @} chasssis parameter configure */

//--------------------------------------------------------------------------------
/** @name super cap can config
  * @{
  */
#define USE_SUPER_CAP 0             ///< super cap module switch
#define SUPER_CAP_USE_CAN CAN1      ///< select CAN1 or CAN2
#define SUPER_CAP_CAN_HANDLE hcan1  ///< select which can handle super cap data receive uses (hcanx)
#define CAP_CAN_FILTER_NUM 12       ///< if uses CAN1, filter number should be 0 ~ 13. if uses CAN2 filter number should be 14 ~ 27

/** @} super cap can config */

/** @name external can imu config
  * @{
  */
#define USE_EXT_CAN_IMU 0			///< external imu module switch
#define IMU_CAN_RX_ID 0x0008		///< imu can stdid
#define IMU_USE_CAN CAN2			///< select CAN1 or CAN2
#define IMU_CAN_FILTER_NUM 26		///< if uses CAN1, filter number should be 0 ~ 13. if uses CAN2 filter number should be 14 ~ 27

/** @} external can imu config */

#endif

