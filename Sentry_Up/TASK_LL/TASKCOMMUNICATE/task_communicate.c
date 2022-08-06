#include "task_communicate.h"
#include "cmsis_os.h"
#include "string.h"
#include "math.h"
#include "bsp_usart.h"
#include "stdbool.h"
#include "driver_remote.h"
#include "freertostask.h"
#include "task_remote.h"
#include "driver_gimbal.h"

#define PIE (3.1415926f)

//变量定义
/* communicate task static parameter */
/* judge system receive data fifo and buffer*/
static osMutexId judge_rxdata_mutex;
static fifo_s_t  judge_rxdata_fifo;
static uint8_t   judge_rxdata_buf[JUDGE_FIFO_BUFLEN];
/* judge system send data fifo and buffer*/
static osMutexId judge_txdata_mutex;
static fifo_s_t  judge_txdata_fifo;
static uint8_t   judge_txdata_buf[JUDGE_FIFO_BUFLEN];
/* judge system dma receive data object */
static uart_dma_rxdata_t judge_rx_obj;
/* unpack object */
static unpack_data_t judge_unpack_obj;

uint8_t judge_dma_rxbuff[2][UART_RX_DMA_SIZE];

extern TaskHandle_t judge_unpack_task_t;
ext_student_interactive_header_data_t student_interactive_header;


//----------------------------------------------外设初始化-----------------------------------------------------------

usart_param_struct judgement_usart;
void communicate_param_init()
{
	//judgement_param_init
	judgement_usart.USARTx = UART5;
	
	judgement_usart.rx_DMAx = DMA1;
	judgement_usart.rx_Stream = LL_DMA_STREAM_0;
	judgement_usart.rx_dma_stream=DMA1_Stream0;

	judgement_usart.tx_DMAx = DMA1;
	judgement_usart.tx_Stream = LL_DMA_STREAM_7;
	judgement_usart.tx_dma_stream=DMA1_Stream7;
	
	judgement_usart.rx_usart_dma_memory_base_address_0 = (u32)judge_dma_rxbuff[0];
	judgement_usart.rx_usart_dma_memory_base_address_1 = (u32)judge_dma_rxbuff[1];
	judgement_usart.rx_usart_dma_buffer_size = DMA_BUFFER_SIZE;
	judgement_usart.task = &judge_unpack_task_t;
	judgement_usart.tx_finish_flag = 1;
	
}

void judgement_uart_init(void)
{
		/* create the judge_rxdata_mutex mutex  */
  osMutexDef(judge_rxdata_mutex);
  judge_rxdata_mutex = osMutexCreate(osMutex(judge_rxdata_mutex));
  
  /* create the judge_txdata_mutex mutex  */
  osMutexDef(judge_txdata_mutex);
  judge_txdata_mutex = osMutexCreate(osMutex(judge_txdata_mutex));
	
	/* judge data fifo init */
  fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_FIFO_BUFLEN, judge_rxdata_mutex);
  fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_FIFO_BUFLEN, judge_txdata_mutex);
	
	/* initial judge data dma receiver object */
  judge_rx_obj.dma_stream = judgement_usart.rx_dma_stream;
  judge_rx_obj.data_fifo = &judge_rxdata_fifo;
  judge_rx_obj.buff_size = UART_RX_DMA_SIZE;
  judge_rx_obj.buff[0] = judge_dma_rxbuff[0];
  judge_rx_obj.buff[1] = judge_dma_rxbuff[1];
	
  /* initial judge data unpack object */
  judge_unpack_obj.data_fifo = &judge_rxdata_fifo;
  judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
  judge_unpack_obj.index = 0;
  judge_unpack_obj.data_len = 0;
  judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
	
	usart_communicate_config(judgement_usart);

}



//----------------------------------------------协议处理-----------------------------------------------------------


/** 
  * @brief  分数据，增加协议包改这里
  */

//裁判系统数据接收
/* data send (forward) */
/* data receive */
receive_judge_t judge_rece_mesg;

/**
  * @brief    get judgement system message
  */
extern  enum 		REMOTESTATE RemoteControlMode;
extern  u8 			Do_heat_calibration;
				u8		  SupplyingFlag;
extern  u8      DeathModeFlag;
				float   SuperCVoltage=0;
extern  float 	SmallGim_Heat_User;
extern  float 	BigGimHeat_User;
int16_t 				SmalBulletSped = 0;
int16_t 				BigBulletSped	 = 0;
int16_t 				SmallBulletShotNum = 0;
int16_t 				BigBulletShotNum   		= 0;//----------用来计算大子弹消耗数量
int16_t 				BigBulletShotNumLast  = 0;
float           BigBulletActualSpeed  = 0;
u8    					BigBulletIncreFlag		= 0;
u8              BigBulletIncreFlag_2  = 0;
u8 							NoBigBulletFlag				= 0;
u8							AimingSequence[8]			={0};//瞄准顺序
extern float TargetBigBulletSpeed;//射击初速度上限

//-----------------------------------------给裁判系统打包发送的标准格式-----------------------------------------------------//
void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);
	
  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  
  switch (cmd_id)
  {
    case GAME_STATE_ID:
      memcpy(&judge_rece_mesg.game_state_data, data_addr, data_length);
    break;

    case GAME_RESULT_ID:
      memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
    break;
    case GAME_SURVIVORS_ID://机器人血量数据
      memcpy(&judge_rece_mesg.ext_game_robot_HP_t, data_addr, data_length);
    break;
    case DART_STATUS_ID:
      memcpy(&judge_rece_mesg.dart_status_data, data_addr, data_length);
    break;
		case EVENT_DATA_ID:
			memcpy(&judge_rece_mesg.event_data, data_addr, data_length);
		break;
    case SUPPLY_ACTION_ID:
      memcpy(&judge_rece_mesg.supply_projectile_action_data, data_addr, data_length);
			if(judge_rece_mesg.supply_projectile_action_data.supply_robot_id == judge_rece_mesg.game_robot_state_data.robot_id)
			{//如果是我在补弹
				SupplyingFlag = 1;
			}
			else
			{//如果不是我在补弹
				SupplyingFlag = 0;
			}
    break;
		case REFEREE_WARNING_ID://裁判系统警告ID
      memcpy(&judge_rece_mesg.referee_warning_data, data_addr, data_length);
		break;
		case DART_REMAINING_ID://飞镖发射口计时
      memcpy(&judge_rece_mesg.dart_remaining_data, data_addr, data_length);
		break;
    case GAME_ROBOT_STATE_ID://比赛机器人状态，10HZ
      memcpy(&judge_rece_mesg.game_robot_state_data, data_addr, data_length);
			#if USE_JUDEG_UPDATE
			Do_heat_calibration = 1;
			TargetBigBulletSpeed=judge_rece_mesg.game_robot_state_data.shooter_id1_42mm_speed_limit;//输入子弹初速度上限
			#endif//第一次接收到机器人信息才会定时进行热量校准
    break;
    
    case POWER_HEAT_ID://实时功率热量数据，50HZ
      memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);
    break;
		case ROBOT_POS_ID://机器人位置，10HZ
      memcpy(&judge_rece_mesg.game_robot_pos_data, data_addr, data_length);
    break;
		case BUFF_MUSK_ID://机器人增益，状态改变后发送
      memcpy(&judge_rece_mesg.buff_musk_data, data_addr, data_length);
    break;
	  case AERIAL_ROBOT_ENERGY_ID:
      memcpy(&judge_rece_mesg.aerial_robot_energy_data, data_addr, data_length);
    break;
		case ROBOT_HURT_ID://伤害状态，伤害发生后发送
      memcpy(&judge_rece_mesg.robot_hurt_data, data_addr, data_length);
    break;
    case SHOOT_DATA_ID://实时射击信息，射击后发送
      memcpy(&judge_rece_mesg.shoot_data, data_addr, data_length);
			#if USE_JUDEG_UPDATE
				switch(judge_rece_mesg.shoot_data.bullet_type)
				{
					case 1:
						SmallBulletShotNum++;
						SmalBulletSped = (int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 1000.0f);
						SmallGim_Heat_User += judge_rece_mesg.shoot_data.bullet_speed;//小子弹用户热量更新
						break;
					case 2:
						BigBulletShotNum++;
						if(BigBulletShotNum   != BigBulletShotNumLast)
						{
							BigBulletShotNumLast = BigBulletShotNum;
							BigBulletIncreFlag   = 1;//用于反馈发弹
							BigBulletIncreFlag_2 = 1;//用于摩擦轮速度调整
//							if(!DeathModeFlag)
//							{
//								BigGimHeat_User   += 100;
//							}
						}
						BigBulletActualSpeed = judge_rece_mesg.shoot_data.bullet_speed*1000;
						BigBulletSped = (int16_t)(judge_rece_mesg.shoot_data.bullet_speed * 1000.0f);
						break;
					default:
						break;
				}
			#endif
    break;
		case STU_INTERACTIVE_ID://交互数据接收信息
      memcpy(&judge_rece_mesg.student_student_data, data_addr, data_length);
    break;
		case BULLET_REMAIN_ID://剩余子弹信息
      memcpy(&judge_rece_mesg.bullet_reamining_data, data_addr, data_length);
		break;
		case RFID_STATUS_ID://机器人RFID状态
      memcpy(&judge_rece_mesg.rfid_status_data, data_addr, data_length);
		break;
		case DART_CMD_ID://飞镖机器人客户端指令数据
      memcpy(&judge_rece_mesg.dart_cmd_data, data_addr, data_length);
		break;
		case CUSTOM_CONTROLLER_ID://自定义控制器交互数据接口
      memcpy(&judge_rece_mesg.custom_interactive_data, data_addr, data_length);
		break;
		case MINIMAP_DATA_ID://客户端小地图交互数据
      memcpy(&judge_rece_mesg.mini_map_data, data_addr, data_length);
		break;
		case KEYBORD_DATA_ID://键盘鼠标信息
      memcpy(&judge_rece_mesg.interactive_information_data, data_addr, data_length);
		break;
		
    default:
    break;
  }

		LostCounterFeed(9);

}



//----------------------------------------------打包与解包-----------------------------------------------------------
/** 
  * @brief  解包
  */
	
void unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
  uint8_t byte = 0;
  
  while ( fifo_used_count(p_obj->data_fifo) )
  {
    byte = fifo_s_get(p_obj->data_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == HEADER_LEN)
        {
          if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  

      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
          {
             if (sof == DN_REG_ID)
            {
              judgement_data_handler(p_obj->protocol_packet);
            }   
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

//for debug
int dma_write_len = 0;
int fifo_overflow = 0;

void dma_buffer_to_unpack_buffer(uart_dma_rxdata_t *dma_obj, uart_it_type_e it_type)
{
  int16_t  tmp_len;
  uint8_t  current_memory_id;
  uint16_t remain_data_counter;
  uint8_t  *pdata = dma_obj->buff[0];
  
  get_dma_memory_msg(dma_obj->dma_stream, &current_memory_id, &remain_data_counter);
  
  if (UART_IDLE_IT == it_type)
  {
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size*2 - remain_data_counter;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size - remain_data_counter;
    }
  }
  else if (UART_DMA_FULL_IT == it_type)
  {
#if 0
    if (current_memory_id)
    {
      dma_obj->write_index = dma_obj->buff_size;
    }
    else
    {
      dma_obj->write_index = dma_obj->buff_size*2;
    }
#endif
  }
  
  if (dma_obj->write_index < dma_obj->read_index)
  {
    dma_write_len = dma_obj->buff_size*2 - dma_obj->read_index + dma_obj->write_index;
    
    tmp_len = dma_obj->buff_size*2 - dma_obj->read_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = 0;
    
    tmp_len = dma_obj->write_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = dma_obj->write_index;
  }
  else
  {
    dma_write_len = dma_obj->write_index - dma_obj->read_index;
    
    tmp_len = dma_obj->write_index - dma_obj->read_index;
    if (tmp_len != fifo_s_puts(dma_obj->data_fifo, &pdata[dma_obj->read_index], tmp_len))
      fifo_overflow = 1;
    else
      fifo_overflow = 0;
    dma_obj->read_index = (dma_obj->write_index) % (dma_obj->buff_size*2);
  }
}


/** 
  * @brief  打包
  */

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len,uint8_t *tx_buf, ext_student_interactive_header_data_t student_interactive_header)
{
  //memset(tx_buf, 0, 100);
  //static uint8_t seq;
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN+INTERACTIVE_HEADER_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  
  p_header->sof          = DN_REG_ID;
  p_header->data_length  = len + 6;
  p_header->seq          = 0;
  //p_header->seq          = seq++;
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
	memcpy(&tx_buf[HEADER_LEN + CMD_LEN], (uint8_t*)&student_interactive_header,INTERACTIVE_HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN+INTERACTIVE_HEADER_LEN], p_data, len);
  append_crc16_check_sum(tx_buf, frame_length);
  
  return tx_buf;
}

uint32_t send_packed_fifo_data(fifo_s_t *pfifo, uint8_t sof)
{


  uint8_t  tx_buf[JUDGE_FIFO_BUFLEN];
  uint32_t fifo_count = fifo_used_count(pfifo);
  
  if (fifo_count)
  {
    fifo_s_gets(pfifo, tx_buf, fifo_count);
    
    if (sof == DN_REG_ID)
			usart_dma_send(&judgement_usart, (u32)tx_buf, fifo_count);
    else
      return 0;
  }
  
  return fifo_count;
}

  uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

//data_packet_pack(INFANTRY_SHOOT_MODE_ID, (uint8_t *)&gimbalmode,sizeof(gimbalmode), UP_REG_ID);

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len,ext_student_interactive_header_data_t student_interactive_header)
{
  
  uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN+INTERACTIVE_HEADER_LEN;   //5+2+4+2

  
  protocol_packet_pack(cmd_id, p_data, len, tx_buf, student_interactive_header);
  
  /* use mutex operation */
  fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);
 
    return ;
}

//----------------------------------------雷达数据打包---------------------------------------//
uint8_t* lidar_protocol_packet_pack(uint16_t cmd_id, lidar_send_struct lidar_data, uint8_t *tx_buf, ext_student_interactive_header_data_t student_interactive_header)
{
  //memset(tx_buf, 0, 100);
  //static uint8_t seq;
  
	uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN;
  //uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN + INTERACTIVE_HEADER_LEN;
  frame_header_t *p_header = (frame_header_t*)tx_buf;
  
  p_header->sof          = DN_REG_ID;
  p_header->data_length  = 14;
  p_header->seq          = 0;
  //p_header->seq          = seq++;
  memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(tx_buf, HEADER_LEN);
	memcpy(&tx_buf[HEADER_LEN + CMD_LEN], (uint8_t*)&student_interactive_header,INTERACTIVE_HEADER_LEN);
  memcpy(&tx_buf[HEADER_LEN + CMD_LEN+INTERACTIVE_HEADER_LEN], (uint8_t*)&lidar_data, 14);
	//memcpy(&tx_buf[HEADER_LEN + CMD_LEN], (uint8_t*)&lidar_data,14);
  append_crc16_check_sum(tx_buf, frame_length);
  
  return tx_buf;
}

void lidar_data_pack(uint16_t cmd_id, lidar_send_struct lidar_data, ext_student_interactive_header_data_t student_interactive_header)
{
	//uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN + INTERACTIVE_HEADER_LEN;  //雷达发送数据固定长度为14，标志头长度+命令长度+交互长度
	uint16_t frame_length = HEADER_LEN + CMD_LEN + 14 + CRC_LEN;
	
	lidar_protocol_packet_pack(cmd_id,lidar_data,tx_buf,student_interactive_header);
	
	fifo_s_puts(&judge_txdata_fifo, tx_buf, frame_length);

}

//********************************车间通信发送代码**********************************//
void robot_communicate_send_data(uint16_t data_cmd_id, uint16_t send_ID, uint16_t receiver_ID, uint8_t* p_data, uint16_t  len1)
{
	ext_student_interactive_header_data_t robot_interactive;
	robot_interactive.data_cmd_id=data_cmd_id;
	robot_interactive.send_ID=send_ID;
	robot_interactive.receiver_ID=receiver_ID;
	data_packet_pack(STU_INTERACTIVE_ID,p_data,len1,robot_interactive);
}


//----------------------------------------------Task-----------------------------------------------------------

//————————————————————————————————车间通信————————————————————————————————————//
//uint8_t   data[3] = {1,2,3};
//uint8_t   *p_data=data;
//uint16_t  len=3;
//ext_student_interactive_header_data_t robot_interactive;

//void robot_communicate_init()
//{
//	p_data=data;
//  len=3;
//	robot_interactive.data_cmd_id=0x02F0;
//	robot_interactive.send_ID=BLUE_INFANTRY4_ID;
//	robot_interactive.receiver_ID=BLUE_HERO_ID;
//}


//————————————————————————————————雷达发送数据初始化————————————————————————————————————//

//lidar_send_struct lidar_data;
//ext_student_interactive_header_data_t lidar_robot_interactive;
//void lidar_send_init()
//{
//	lidar_robot_interactive.data_cmd_id  = 0x02F0;
//	lidar_robot_interactive.receiver_ID  = 0x016A;
//	lidar_robot_interactive.send_ID      = 0x006D;
//	
//	lidar_data.target_robot_id    = RED_HERO_ID;
//	lidar_data.target_position_x  = 5.1;
//	lidar_data.target_position_y  = 5.1;
//	lidar_data.target_state_point = 0;
//}




void judge_unpack_task(void)
{
  osEvent event;
  
  /* open judge uart receive it */
	judgement_uart_init();
  
  while (1)
  {
    event = osSignalWait(UART_TX_SIGNAL | UART_IDLE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      //receive judge data puts fifo
      if (event.value.signals & UART_IDLE_SIGNAL)
      {
        dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
        unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);
      }
      
      //send data to judge system
      if (event.value.signals & UART_TX_SIGNAL)
      {
				//画线，标明方向
        //send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID);
				//机器人数据交互
				//robot_communicate_send_data(0x02F0,BLUE_INFANTRY4_ID,BLUE_HERO_ID,p_data,3);
				//雷达发送位置消息
				//lidar_send_init();
				//lidar_data_pack(0x0301,lidar_data,lidar_robot_interactive);
      }
      
    }
  }
}

extern EncoderDataStruct  EncoderDataSave;
ext_client_custom_graphic_delete_t 	Delete_Graph;
ext_client_custom_graphic_single_t 	Single_Graph;
ext_client_custom_graphic_double_t 	Double_Graph;
ext_client_custom_graphic_five_t 		Five_Graph;
ext_client_custom_graphic_seven_t 	Seven_Graph;

u16 BodyStartX   = 800;
u16 BodyStartY   = 540;
u16 BodyEndX 		 = 0;
u16 BodyEndY 		 = 0;
u16 Length   = 50;
float Alpha  = 0;

void StuInteractiveData(void)
{
		static u8 GraphFlag1 = 0;
		if(GraphFlag1 == 0)
		{
			Single_Graph.grapic_data_struct.operate_type = 1;
			GraphFlag1 = 1;
		}
		else
			Single_Graph.grapic_data_struct.operate_type = 2;
		
		Single_Graph.grapic_data_struct.graphic_type = 0;	
		strcpy(Single_Graph.grapic_data_struct.graphic_name,"CB");
		
		Alpha = (YAW_INIT_VALUE_SET	-	EncoderDataSave.Yaw)*PIE*2;
		BodyEndX = BodyStartX - Length*sin(Alpha);
		BodyEndY = BodyStartY + Length*cos(Alpha);
		
		Single_Graph.grapic_data_struct.start_x      = BodyStartX;
		Single_Graph.grapic_data_struct.start_y      = BodyStartY;
		Single_Graph.grapic_data_struct.end_x				 = BodyEndX;
		Single_Graph.grapic_data_struct.end_y				 = BodyEndY;
		Single_Graph.grapic_data_struct.color 			 = 2;
		Single_Graph.grapic_data_struct.layer				 = 8;
		Single_Graph.grapic_data_struct.width        = 6;
		
		student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;
		if(judge_rece_mesg.game_robot_state_data.robot_id == BLUE_HERO_ID)
		{
			student_interactive_header.send_ID     = BLUE_HERO_ID;
			student_interactive_header.receiver_ID = BLUE_HERO_CUSTOM_ID;
		}
		if(judge_rece_mesg.game_robot_state_data.robot_id == RED_HERO_ID)
		{
			student_interactive_header.send_ID     = RED_HERO_ID;
			student_interactive_header.receiver_ID = RED_HERO_CUSTOM_ID;
		}
		data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *)&Single_Graph,sizeof(Single_Graph), student_interactive_header);
		osSignalSet(judge_unpack_task_t, UART_TX_SIGNAL);
	

}
