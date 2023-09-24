#ifndef _REFEREE_CONF_H
#define _REFEREE_CONF_H

#define REFEREE_SYS_VERSION 2021

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define	LEN_HEADER    5U        //֡ͷ��
#define	LEN_CMDID     2U        //�����볤��
#define	LEN_TAIL      2U	      //֡βCRC16

#define	SOF_BYTE			0xA5U     //֡ͷ

#define	CMDID_OFFSET	5U
#define	CMDID_H				6U
#define	CMDID_L				5U

/* RFID������ */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)

/**
  * @enum CmdID
  * @brief ������ID,�����жϽ��յ���ʲô����
  */
typedef enum
{ 
	ID_game_state       						= 0x0001,///< ����״̬����
	ID_game_result 	   							= 0x0002,///< �����������
	ID_game_robot_HP   							= 0x0003,///< ���������˴������
	ID_dart_status								= 0x0004,///< ����״̬
	ID_event_data  								= 0x0101,///< �����¼����� 
	ID_supply_projectile_action   				= 0x0102,///< ���ز���վ������ʶ����
	ID_supply_projectile_booking 				= 0x0103,///< ���ز���վԤԼ�ӵ�����
	ID_refee_alert								= 0x0104,///< ���о������ݣ����淢������
	ID_dart_countdown							= 0x0105,///< ���ڷ���ڵ���ʱ��1Hz���ڷ���
	ID_game_robot_state    						= 0x0201,///< ������״̬����
	ID_power_heat_data    						= 0x0202,///< ʵʱ������������
	ID_game_robot_pos        					= 0x0203,///< ������λ������
	ID_buff_musk								= 0x0204,///< ��������������
	ID_aerial_robot_energy						= 0x0205,///< ���л���������״̬����
	ID_robot_hurt								= 0x0206,///< �˺�״̬����
	ID_shoot_data								= 0x0207,///< ʵʱ�������
	ID_bullet_remaining_num						= 0x0208,///< ����ʣ�෢����
	ID_RFID_status								= 0x0209,///< ������RFID״̬
	ID_dart_client_data							= 0x020A,///< ���ڻ����˿ͻ���ָ������
	ID_custom_data								= 0x0301
} CmdID;

typedef __packed struct{
    uint8_t  sof;
    uint16_t data_len;
    uint8_t  seq;
    uint8_t  crc8;
}frame_header_t;

/* ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz */
typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ����������ݣ�0x0002������Ƶ�ʣ������������� */
typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

/* ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz */
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* �˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ�����л����� */
typedef __packed struct
{
    uint8_t F1_zone_status : 1;
    uint8_t F1_zone_buff_debuff_status : 3;
    uint8_t F2_zone_status : 1;
    uint8_t F2_zone_buff_debuff_status : 3;
    uint8_t F3_zone_status : 1;
    uint8_t F3_zone_buff_debuff_status : 3;
    uint8_t F4_zone_status : 1;
    uint8_t F4_zone_buff_debuff_status : 3;
    uint8_t F5_zone_status : 1;
    uint8_t F5_zone_buff_debuff_status : 3;
    uint8_t F6_zone_status : 1;
    uint8_t F6_zone_buff_debuff_status : 3;
    uint16_t red1_bullet_left;
    uint16_t red2_bullet_left;
    uint16_t blue1_bullet_left;
    uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

/* �����¼����ݣ�0x0101������Ƶ�ʣ�1Hz */
typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������ı����, ���ͷ�Χ������������ */
typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ��������淢������ */
typedef __packed struct
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz���ڷ��ͣ����ͷ�Χ������������ */
typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz */
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz */
typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz */
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz */
typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz */
typedef __packed struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������� */
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� */
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�10Hz���ڷ��ͣ����л����˷��� */
typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* ������RFID״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ������ */
typedef __packed struct
{
    uint32_t rfid_status;
} ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������ */
typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

static uint8_t cmdid_len_table[4][11] = {{0, sizeof(ext_game_status_t), sizeof(ext_game_result_t), sizeof(ext_game_robot_HP_t), 0, sizeof(ext_ICRA_buff_debuff_zone_status_t), 0, 0, 0, 0, 0},
                                         {0, sizeof(ext_event_data_t), sizeof(ext_supply_projectile_action_t), 0, sizeof(ext_referee_warning_t), sizeof(ext_dart_remaining_time_t), 0, 0, 0, 0, 0},
                                         {0, sizeof(ext_game_robot_status_t), sizeof(ext_power_heat_data_t), sizeof(ext_game_robot_pos_t), sizeof(ext_buff_t), sizeof(aerial_robot_energy_t), sizeof(ext_robot_hurt_t), sizeof(ext_shoot_data_t), sizeof(ext_bullet_remaining_t), sizeof(ext_rfid_status_t), sizeof(ext_dart_client_cmd_t)},
											 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
                                         //{0, 0, sizeof(ext_student_interactive_header_data_t), 0, 0, 0, 0, 0, 0, 0, 0}};


#endif
