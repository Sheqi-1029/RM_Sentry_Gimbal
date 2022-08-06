#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "sys.h"
#include "data_fifo.h"
#include "protocol.h"

#define UART_RX_DMA_SIZE      (1024)
#define DMA_BUFFER_SIZE 			(1024)

#define UART_TX_SIGNAL      ( 1 << 2 )
#define UART_IDLE_SIGNAL    ( 1 << 1 )


typedef struct{
	USART_TypeDef *USARTx;
	
	DMA_TypeDef *rx_DMAx;
	uint32_t rx_Stream;
	DMA_Stream_TypeDef *rx_dma_stream;
	
	DMA_TypeDef *tx_DMAx;
	uint32_t tx_Stream;
	DMA_Stream_TypeDef *tx_dma_stream;
	
	u32 rx_usart_dma_memory_base_address_0,rx_usart_dma_memory_base_address_1,rx_usart_dma_buffer_size;
	
	u8 tx_finish_flag;
	osThreadId *task;
}usart_param_struct;

//���������������������������������������������������������״����ϵͳ���Բ��֡���������������������������������������������������������������������������������������//
typedef struct{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
	float target_state_point;
}lidar_send_struct;

/*********************************************************************************	
 * 														Э�鲿�֡�������Э��
 *********************************************************************************/

#define DN_REG_ID    0xA5  //����ϵͳͨ��
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //����֡
#define CRC_LEN      2    //CRC16У��
#define INTERACTIVE_HEADER_LEN 6 //����֡ͷ

#define PROTOCAL_FRAME_MAX_SIZE  200


/** 
  * @brief  ֡ͷ����
  */
typedef __packed struct
{
  uint8_t  sof;							//�̶�ֵ0xA5
  uint16_t data_length;			//���ݶγ���
  uint8_t  seq;							//�����
  uint8_t  crc8;						//֡ͷУ��
} frame_header_t;

/** 
  * @brief  ������趨��
  */
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

/** 
  * @brief  �����ж�����
  */
typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct
{
  DMA_Stream_TypeDef *dma_stream;
  fifo_s_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

/*********************************************************************************	
 * 														Э�鲿�֡��������ϵͳͨ��
 *********************************************************************************/

#define JUDGE_FIFO_BUFLEN 500

/** 
  * @brief  ����ϵͳ������
  */
typedef enum
{
//�Ӳ���ϵͳ����
  GAME_STATE_ID          = 0x0001,		//����״̬���ݣ�1HZ
  GAME_RESULT_ID         = 0x0002,		//����������ݣ�������������
  GAME_SURVIVORS_ID      = 0x0003,		//������Ѫ�����ݣ�1HZ
	DART_STATUS_ID				 = 0X0004,		//���ڷ���״̬,���ڷ������
  EVENT_DATA_ID          = 0x0101,		//�����¼����ݣ��¼��ı����
  SUPPLY_ACTION_ID       = 0x0102,		//����վ������ʶ�������ı����
  SUPPLY_BOOKING_ID      = 0x0103,		//����վԤԼ�ӵ����Կ���δ����
	REFEREE_WARNING_ID		 = 0X0104,		//���о�����Ϣ,���淢������
	DART_REMAINING_ID			 = 0X0105, 		//���ڷ���ڵ���ʱ,1Hz 
  GAME_ROBOT_STATE_ID    = 0x0201,		//����������״̬��10HZ
  POWER_HEAT_ID          = 0x0202,		//ʵʱ�����������ݣ�50HZ
	ROBOT_POS_ID           = 0x0203,	  //������λ�ã�10HZ
	BUFF_MUSK_ID           = 0x0204,	  //���������棬״̬�ı����
	AERIAL_ROBOT_ENERGY_ID = 0x0205,	  //���л���������״̬
	ROBOT_HURT_ID          = 0x0206,	  //�˺�״̬���˺���������
	SHOOT_DATA_ID          = 0x0207,	  //ʵʱ�����Ϣ���������
	BULLET_REMAIN_ID			 = 0X0208,		//�ӵ�������1Hz ���ڷ��ͣ����л������Լ��ڱ����������ط���
	RFID_STATUS_ID				 = 0X0209,		//������ RFID ״̬1Hz�����ͷ�Χ����һ�����ˡ�
	DART_CMD_ID						 = 0X020A,		//���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
	CUSTOM_CONTROLLER_ID	 = 0X0302,		//�Զ���������������ݽӿڣ��ͻ��˴�������
	MINIMAP_DATA_ID				 = 0X0303,		//�ͻ���С��ͼ�������ݣ���������
	KEYBORD_DATA_ID				 = 0X0304,		//���������Ϣ��ͼ�����ڷ���
	
//�����ϵͳ���� �������˼�ͨ��
	STU_INTERACTIVE_ID							=0x0301,     //�������ݽ�����Ϣ
	STU_STU_DATA_ID									=0x0211,   //�����˼�ͨ������ID��0x0201~0x02FF)
	STU_CUSTOM_DELETE_PICTURE_ID		=0x0100,		//��ͻ���ɾ��ͼ������ID
	STU_CUSTOM_ONE_PICTURE_ID				=0x0101,		//��ͻ���һ��ͼ������ID
	STU_CUSTOM_TWO_PICTURE_ID				=0x0102,		//��ͻ�������ͼ������ID
	STU_CUSTOM_FIVE_PICTURE_ID			=0x0103,		//��ͻ������ͼ������ID
	STU_CUSTOM_SEVEN_PICTURE_ID			=0x0104,		//��ͻ����߸�ͼ������ID

//ͨ��IDȷ��
	RED_HERO_ID            =0x0001,    //�췽Ӣ�ۻ�����ID
	RED_ENGINEER_ID        =0x0002,
	RED_INFANTRY3_ID       =0x0003,
	RED_INFANTRY4_ID       =0x0004,
	RED_INFANTRY5_ID       =0x0005,
	RED_AERIAL_ID          =0x0006,
	RED_SENTRY_ID          =0x0007,
	BLUE_HERO_ID           =0x0065,
	BLUE_ENGINEER_ID       =0x0066,
	BLUE_INFANTRY3_ID      =0x0067,
	BLUE_INFANTRY4_ID      =0x0068,
	BLUE_INFANTRY5_ID      =0x0069,
	BLUE_AERIAL_ID         =0x006A,
	BLUE_SENTRY_ID         =0x006B,
  RED_HERO_CUSTOM_ID     =0x0101,    //�췽Ӣ�ۻ����˿ͻ���ID
	RED_ENGINEER_CUSTOM_ID =0x0102,
	RED_INFANTRY3_CUSTOM_ID=0x0103,
	RED_INFANTRY4_CUSTOM_ID=0x0104,
	RED_INFANTRY5_CUSTOM_ID=0x0105,
	RED_AERIAL_CUSTOM_ID   =0x0106,
	BLUE_HERO_CUSTOM_ID    =0x0165,
	BLUE_ENGINEER_CUSTOM_ID=0x0166,
	BLUE_INFANTRY3_CUSTOM_ID=0x0167,
	BLUE_INFANTRY4_CUSTOM_ID=0x0168,
	BLUE_INFANTRY5_CUSTOM_ID=0x0169,
	BLUE_AERIAL_CUSTOM_ID   =0x016A,

} judge_data_id_e;

/** 
  * @brief  game information structures definition(0x0001)
  */
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
} ext_game_state_t;

/** 
  * @brief  game result(0x0002)
  */
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;

/** 
  * @brief  (0x0003)
  */
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
/** 
  * @brief  (0x0004)
  */
typedef __packed struct
{
uint8_t dart_belong;
uint16_t stage_remaining_time;
} ext_dart_status_t;  
/** 
  * @brief  (0x0101)
  */
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;


/** 
  * @brief  (0x0102)
  */
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;



/** 
  * @brief  (0x0104)
  */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;
/** 
  * @brief  (0x0105)
  */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;
/** 
  * @brief  (0x0201)
  */
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


/** 
  * @brief  (0x0202)
  */
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
/** 
  * @brief  (0x0203)
  */
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

/** 
  * @brief  (0x0204)
  */
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_musk_t;
/** 
  * @brief  (0x0205)
  */
typedef __packed struct
{
 uint8_t attack_time;
} aerial_robot_energy_t;
/** 
  * @brief  (0x0206)
  */
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/** 
  * @brief  (0x0207)
  */
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;
/** 
  * @brief  (0x0208)
  */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
/** 
  * @brief  (0x0209)
  */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;
/** 
  * @brief  (0x020A)
  */
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint8_t first_dart_speed;
	uint8_t second_dart_speed;
	uint8_t third_dart_speed;
	uint8_t fourth_dart_speed;
	uint16_t last_dart_launch_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;
/** 
  * @brief  student custom data(0x0301)
  */
typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;


/** 
  * @brief  student student data(0x0301)---(0x0201~0x02FF)
ѧ�������˼�ͨ��
  */
typedef __packed struct
{
uint8_t data[16];//�ֽڴ�СС��113
} robot_interactive_data_t;

/** 
  * @brief  student student data conjunction
  */
typedef __packed struct
{
 ext_student_interactive_header_data_t   student_interactive_header_data;
 robot_interactive_data_t                robot_interactive_data;
}ext_student_student_data_t;
/** 
  * @brief  student custom delete picture(0x0301)---(0x0100)
  */
typedef __packed struct
{
uint8_t operate_type;//0 �ղ��� 1 ɾ��ͼ�� 2 ɾ������
uint8_t layer;//ͼ����0-9
}ext_client_custom_graphic_delete_t;
/** 
  * @brief  student custom graph data
  */
typedef __packed struct
{
	uint8_t graphic_name[3];//ͼ����
	uint32_t operate_type:3;//ͼ�β��� 1����2�޸�3ɾ��
	uint32_t graphic_type:3;//ͼ������
	uint32_t layer:4;//ͼ����0-9
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;//��ֹ�Ƕ�
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;//��С��뾶
	uint32_t end_x:11;
	uint32_t end_y:11;
} graphic_data_struct_t;//��һ��ͼ�Σ�һ��ͼ�Ρ���ͨ��Э��25ҳ

/** 
  * @brief  student custom draw one picture(0x0301)---(0x0101)
  */
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct;
}ext_client_custom_graphic_single_t;
/** 
  * @brief  student custom draw two picture(0x0301)---(0x0102)
  */
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
/** 
  * @brief  student custom draw five picture(0x0301)---(0x0103)
  */
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
/** 
  * @brief  student custom draw seven picture(0x0301)---(0x0104)
  */
typedef __packed struct
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
/** 
  * @brief costum_interactive_data_t(0x0302)
  */
typedef __packed struct
{
uint8_t data[30];
} ext_custum_interactive_data_t;
/** 
  * @brief Interactive information of small map(0x0303)
  */
typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_minimap_t;
/** 
  * @briefPicture transmission remote control information(0x0304)
  */
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_interactive_information_t;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  ext_game_state_t                      game_state_data;               //0x0001
  ext_game_result_t                     game_result_data;              //0x0002
  ext_game_robot_HP_t            				ext_game_robot_HP_t;     			 //0x0003
	ext_dart_status_t											dart_status_data;							 //0x0004
  ext_event_data_t                      event_data;                    //0x0101
  ext_supply_projectile_action_t        supply_projectile_action_data; //0x0102
	ext_referee_warning_t									referee_warning_data;					 //0x0104
	ext_dart_remaining_time_t							dart_remaining_data;					 //0x0105
  ext_game_robot_status_t               game_robot_state_data;        //0x0201
  ext_power_heat_data_t                 power_heat_data;               //0x0202
	ext_game_robot_pos_t                  game_robot_pos_data;           //0x0203
	ext_buff_musk_t                       buff_musk_data;                //0x0204
	aerial_robot_energy_t                 aerial_robot_energy_data;      //0x0205
	ext_robot_hurt_t                      robot_hurt_data;               //0x0206
	ext_shoot_data_t                      shoot_data;                    //0x0207
	ext_student_student_data_t            student_student_data;
	ext_bullet_remaining_t								bullet_reamining_data;				 //0x0208
	ext_rfid_status_t											rfid_status_data;							 //0x0209
	ext_dart_client_cmd_t									dart_cmd_data;								 //0x020A
	ext_custum_interactive_data_t					custom_interactive_data;			 //0x0302
	ext_minimap_t													mini_map_data;						 //0x0303
	ext_interactive_information_t					interactive_information_data;			 //0x0304
} receive_judge_t;

void communicate_param_init(void);

void judge_unpack_task(void);
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, ext_student_interactive_header_data_t student_interactive_header);
void UserInteractiveData(void);
void StuInteractiveData(void);
void robot_communicate_send_data(uint16_t data_cmd_id, uint16_t send_ID, uint16_t receiver_ID, uint8_t* p_data, uint16_t  len1);

#endif
