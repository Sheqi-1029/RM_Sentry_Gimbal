#include "FreeRTOS.h"
#include "task.h"
#include "task_remote.h"
#include "math.h"
#include "driver_remote.h"
#include "driver_gimbal.h"
#include "driver_friction.h"
#include "task_feedmotor.h"
#include "task_gimbal.h"
#include "task_chassis.h"
#include "bsp_can.h"

extern RemoteDataUnion RemoteData;
extern GimbalMotorStruct	YawMotor,PitchMotor,Delivery,Friction_left,Friction_right;;

RemoteDataPortStruct RemoteModeProcessData(RemoteDataProcessedStruct	RemoteDataReceive)
{
	RemoteDataPortStruct	RemoteDataPortTemp={0};
		
	RemoteDataPortTemp.PitchIncrement	=		RemoteDataReceive.Channel_1;
	RemoteDataPortTemp.YawIncrement		=	-	RemoteDataReceive.Channel_0;
	
	return RemoteDataPortTemp;
}

RemoteDataPortStruct	RemoteDataPort;

u8 Can2_DownSentry_SendMessage[8];
//下云台通信协议：0x0923
//[0][1] 左摇杆横；[2][3]z左摇杆纵；[4][5]小拨杆；[6] lost_flag; [7]待定;

u8 RemoteTaskControl()
{
	taskENTER_CRITICAL();           //进入临界区	
	
	//Step	1	:	Receive remote raw data from buffer
	RemoteDataProcessedStruct	RemoteDataReceive;
	RemoteDataReceive=RemoteDataProcess(RemoteData);
	Can2_DownSentry_SendMessage[0] = (u8)((((RemoteData.RemoteDataProcessed.RCValue.Ch2_h<<2)&0xff00)>>8)&0x00ff);
	Can2_DownSentry_SendMessage[1] = (u8)((((RemoteData.RemoteDataProcessed.RCValue.Ch2_h<<2)&0xfffc)//0xfffc=1111 1111 1111 1100
	                                         |RemoteData.RemoteDataProcessed.RCValue.Ch2_l)&0x00ff);
	Can2_DownSentry_SendMessage[2] = (u8)((RemoteData.RemoteDataProcessed.RCValue.Ch3>>8)&0x00ff);
	Can2_DownSentry_SendMessage[3] = (u8)(RemoteData.RemoteDataProcessed.RCValue.Ch3&0x00ff);
	Can2_DownSentry_SendMessage[4] = RemoteData.RemoteDataProcessed.RCValue.s1;
	Can2_DownSentry_SendMessage[5] = RemoteData.RemoteDataProcessed.RCValue.s2;
	Can2_DownSentry_SendMessage[7] = 0;
	
	CAN2_Send_Msg(Can2_DownSentry_SendMessage,8,0x0250);
	
	//Step	2	:	Judge Validity
	if(RemoteDataReceive.FlagValidity)
	{	
		//Step	3	：Process	remote data	and	Save into RemoteDataPort
		RemoteDataPort	=	RemoteModeProcessData(RemoteDataReceive);
		taskEXIT_CRITICAL();            //退出临界区
		return 0; 
	}
	
	taskEXIT_CRITICAL();            //退出临界区
	return 1;
	

}
