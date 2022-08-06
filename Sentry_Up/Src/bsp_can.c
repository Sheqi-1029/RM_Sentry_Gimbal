#include "bsp_can.h"
#include "driver_chassis.h"
#include "BSPConfig.h"
#include "task_gimbal.h"
#include "task_lostcounter.h"
#include "driver_gimbal.h"
#include "driver_feedmotor.h"

static CAN_TxHeaderTypeDef Tx1Message;
static CAN_TxHeaderTypeDef Tx2Message;

void CanInit(void)
{
  UserCan1FilterConfig();													//CAN1:pitch，yaw，feed
  UserCan2FilterConfig();													//CAN2:板间通讯，3508摩擦轮
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
}

unsigned char UserCan1FilterConfig()
{
	CAN_FilterTypeDef  CAN1_FilerConf;

	CAN1_FilerConf.FilterIdHigh=CAN1_FILTER_ID_FMI2<<5;     //32位ID
  CAN1_FilerConf.FilterIdLow=CAN1_FILTER_ID_FMI0<<5;
  CAN1_FilerConf.FilterMaskIdHigh=CAN1_FILTER_ID_FMI3<<5; //32位MASK
  CAN1_FilerConf.FilterMaskIdLow=CAN1_FILTER_ID_FMI1<<5;
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  CAN1_FilerConf.FilterBank=0;          //过滤器0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器0
  CAN1_FilerConf.SlaveStartFilterBank=14;
	
	
	CAN1_FilerConf.FilterIdHigh=CAN1_FILTER_ID_FMI6<<5;     //32位ID
  CAN1_FilerConf.FilterIdLow=CAN1_FILTER_ID_FMI4<<5;
  CAN1_FilerConf.FilterMaskIdHigh=CAN1_FILTER_ID_FMI7<<5; //32位MASK
  CAN1_FilerConf.FilterMaskIdLow=CAN1_FILTER_ID_FMI5<<5;
  CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  CAN1_FilerConf.FilterBank=1;          //过滤器0
  CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
  CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
  CAN1_FilerConf.FilterActivation=ENABLE; //激活滤波器1
  CAN1_FilerConf.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK) return 1;//滤波器初始化

	__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0消息挂起中断允许.	  

	return 0;
}

unsigned char UserCan2FilterConfig()
{
	CAN_FilterTypeDef  CAN2_FilerConf;
	
	CAN2_FilerConf.FilterIdHigh=CAN2_FILTER_ID_FMI2<<5;     //32位ID
  CAN2_FilerConf.FilterIdLow=CAN2_FILTER_ID_FMI0<<5;
  CAN2_FilerConf.FilterMaskIdHigh=CAN2_FILTER_ID_FMI3<<5; //32位MASK
  CAN2_FilerConf.FilterMaskIdLow=CAN2_FILTER_ID_FMI1<<5;
  CAN2_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0<<5;//过滤器0关联到FIFO0
  CAN2_FilerConf.FilterBank=14;          //过滤器0
  CAN2_FilerConf.FilterMode=CAN_FILTERMODE_IDLIST;
  CAN2_FilerConf.FilterScale=CAN_FILTERSCALE_16BIT;
  CAN2_FilerConf.FilterActivation=ENABLE; //激活滤波器0
  CAN2_FilerConf.SlaveStartFilterBank=14;
	
	if(HAL_CAN_ConfigFilter(&hcan2,&CAN2_FilerConf)!=HAL_OK) return 1;//滤波器初始化

	__HAL_CAN_ENABLE_IT(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0消息挂起中断允许.	  
	return 0;	
}


#define CAN_RFxR_RFOMx_RELEASE (0x0020U)
#define CAN_RFxR_FULL_FLAG (0x0008U)
#define CAN_RFxR_OVERRUN_FLAG (0x0010U)
#define CAN_RFxR_FMPx_MASK (0x0003U)
#define CAN_IER_FMP0 (0x0002U)

uint8_t Data[8];
CAN_RxHeaderTypeDef RxMessage;
void LL_CAN_Receive(CAN_HandleTypeDef* hcan)
{
	uint32_t tmp1 = 0U, tmp2 = 0U;
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FULL_FLAG) == CAN_RFxR_FULL_FLAG);
	tmp2 = ((hcan->Instance->RF0R & CAN_RFxR_OVERRUN_FLAG) == CAN_RFxR_OVERRUN_FLAG);
  if(tmp1 || tmp2)
		hcan->Instance->RF0R = CAN_RFxR_FULL_FLAG | CAN_RFxR_OVERRUN_FLAG;
	
	tmp1 = ((hcan->Instance->RF0R & CAN_RFxR_FMPx_MASK) != 0U);
  tmp2 = ((hcan->Instance->IER & CAN_IER_FMP0) == CAN_IER_FMP0);
  
  if(tmp1 && tmp2)
  {	
		RxMessage.IDE = (uint8_t)0x04 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
		if (RxMessage.IDE == CAN_ID_STD)
		{
			RxMessage.StdId = 0x000007FFU & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR >> 21U);
		}
		else
		{
			RxMessage.ExtId = 0x1FFFFFFFU & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR >> 3U);
		}
		RxMessage.RTR = (uint8_t)0x02 & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
		RxMessage.DLC = (uint8_t)0x0F & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR;
		RxMessage.Timestamp = (CAN_RDT0R_TIME & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR) >> CAN_RDT0R_TIME_Pos;
		RxMessage.FilterMatchIndex = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDTR >> 8U);
		
		
		Data[0] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR;
		Data[1] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 8U);
		Data[2] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 16U);
		Data[3] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDLR >> 24U);
		Data[4] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR;
		Data[5] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 8U);
		Data[6] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 16U);
		Data[7] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RDHR >> 24U);
		
		hcan->Instance->RF0R = CAN_RFxR_RFOMx_RELEASE;
		HAL_CAN_RxFifo0MsgPendingCallback(hcan);
		
	}
}

extern GimbalMotorStruct	YawMotor,PitchMotor,Delivery,Friction_left,Friction_right;
extern FeedMotorStruct	 	FeedMotor,FeedMotor_UP;

uint16_t blood_judge,bullet_judge;
//extern int heat;
int heat_last=0,heat_receive;

u8 CANChassisrecbuf_A[8];
u8 CANChassisrecbuf_B[8];
u8 CAN2_downyawset[8];
u8 Chassis_rebuf[8];
extern Judgement_data judgement_data; //裁判系统接收数据
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	u8 i;
	if(hcan==&hcan2)
	{
		__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
		LostCounterFeed(RxMessage.FilterMatchIndex);
   switch(RxMessage.StdId)
	 {
//      case 0x0250:
//				for(i=0;i<8;i++)
//				{
//					CANChassisrecbuf_A[i] = Data[i];
//				}
//				
//				judgement_data.game_mode = (u8)(CANChassisrecbuf_A[0]);
//				judgement_data.armour_hurt = (u8)(CANChassisrecbuf_A[1]);
//				judgement_data.bulletnum = (short)((CANChassisrecbuf_A[2]<<8)|CANChassisrecbuf_A[3]);
//				judgement_data.heat1 = (short)((CANChassisrecbuf_A[4]<<8)|CANChassisrecbuf_A[5]);
//				judgement_data.qianshaozhan_blood = (short)((CANChassisrecbuf_A[6]<<8)|CANChassisrecbuf_A[7]);
//				break;
//			case 0x0251:
//				for(i=0;i<8;i++)
//				{
//					CANChassisrecbuf_B[i] = Data[i];
//				}
//				
//				judgement_data.chassis_dis = (short)((CANChassisrecbuf_B[0]<<8)|CANChassisrecbuf_B[1]);
//				judgement_data.chassis_Sp = (u8)(CANChassisrecbuf_B[2]);
//				judgement_data.chassis_speed = (short)((CANChassisrecbuf_B[3]<<8)|CANChassisrecbuf_B[4]);
//				judgement_data.bullet_bool = (u8)(CANChassisrecbuf_B[5]);
//				judgement_data.bullet_speed = (int)(CANChassisrecbuf_B[6]);

//				break;
//			case 0x0205:
//				for(i=0;i<8;i++)
//				{
//					PitchMotor.CANReceiveMessege[i] = Data[i];
//				}
//				LostCounterFeed(GIMBAL_MOTOR_PITCH);
//				break;
//			case 0x0203:
//				for(i=0;i<8;i++)
//				{
//					FeedMotor.ReceiveMessege[i] = Data[i];
//				}
//				LostCounterFeed(FEEDMOTOR_LOST_COUNT);
//				break;
//			case 0x0204:
//				for(i=0;i<8;i++)
//				{
//					FeedMotor_UP.ReceiveMessege[i] = Data[i];
//				}
//				LostCounterFeed(FEEDMOTORUP_LOST_COUNT);
//				break;

				
//			case 0x0252:
//				for(i=0;i<8;i++)
//				{
//					CAN2_downyawset[i] = Data[i];
//				}
//			break;	
				
			case 0x0230:
				for(i=0;i<8;i++)
				{
					Chassis_rebuf[i] = Data[i];
				}
				judgement_data.bullet_bool = Chassis_rebuf[3];
				judgement_data.game_mode = Chassis_rebuf[4];
				LostCounterFeed(GIMBAL_MOTOR_YAW);
			break;
				
			default:break;
	 }
			
//		for(i=0;i<8;i++)
//		{
//			ChassisMotor[RxMessage.FilterMatchIndex].SpeedReceiveMessege[i] = Data[i];
//		}
		
	}else if(hcan==&hcan1)
	{
		__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		switch(RxMessage.StdId)
		{	
			case 0x0207:
				for(i=0;i<8;i++)
				{
					PitchMotor.CANReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_PITCH);
				break;
			case 0x0206:
				for(i=0;i<8;i++)
				{
					FeedMotor.ReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(FEEDMOTOR_LOST_COUNT);
				break;
			case 0x0208:
				for(i=0;i<8;i++)
				{
					YawMotor.CANReceiveMessege[i] = Data[i];
				}
				LostCounterFeed(GIMBAL_MOTOR_YAW);
			break;
			default:
				break;
		}
	}
}


u8 CAN1_Send_Msg(u8* msg,u8 len,int Id)
{
    u16 i=0;
	  uint8_t Data[8];
    Tx1Message.StdId=Id;        //标准标识符
    Tx1Message.ExtId=0x12;        //扩展标识符(29位)
    Tx1Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx1Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx1Message.DLC=len;
    for(i=0;i<len;i++)
			Data[i]=msg[i];
		if(HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,Data,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK) return 1;     //发送
    return 0;		
}

u8 CAN2_Send_Msg(u8* msg,u8 len,int Id)
{
    u16 i=0;
		uint8_t Data[8];
    Tx2Message.StdId=Id;        //标准标识符
    Tx2Message.ExtId=0x12;        //扩展标识符(29位)
    Tx2Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx2Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx2Message.DLC=len;
    for(i=0;i<len;i++)
			Data[i]=msg[i];
		if(HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,Data,(uint32_t *)CAN_TX_MAILBOX0)!=HAL_OK) return 1;     //发送
    return 0;		
}
