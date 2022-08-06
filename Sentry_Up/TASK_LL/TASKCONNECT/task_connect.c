#include "task_connect.h"
#include "driver_connect.h"
#include "bsp_can.h"
#include "driver_remote.h"
#include "driver_vision.h"
#include "usbd_cdc_if.h"
/****************************************************
*			与工控机的通信协议
*			ChipType:	STM32F405RGTx
*			Date:			2019.12.15
*												ZHR.
*****************************************************/

/****************************************************
*			与工控机的通信协议具体格式
      使用的定时器：time7
      通信间隔：20ms
			每20ms会处理一次视觉发送的数据，并且会发送一次数据给工控机
      视觉给电控
      开头 0xff
      角度1（pitch）    32位   
      角度2 （yaw）   32位
      命令状态   16位
      扩展   32位 不开启全发0
      结尾0xff




      电控给视觉
      开头0xff
      云台位置，包括pitch16位. yaw16位，
      云台移动速度，包括vx16位.vy16位.vz16位   
      命令状态8位 
      扩展32位 不开启全发0 
      结尾0xff





      命令状态
      哨兵  1、巡逻  2、追踪  3、开火   

*			
*			
*****************************************************/

//**********************处理完的接收数据，直接取用*******************
//float x_receive;//经过归一化的pitch角度变化量,取值在-1和1之间
//float y_receive;//经过归一化的yaw角度变化量，取值在-1和1之间
//float z_receive;
u8 command_receive;//命令状态
extern RemoteDataProcessedStruct RemoteData;
float delta_pitch_receive;
float delta_yaw_receive;
float delay;
Point3fStruct Point3f;
//********************************************************************

//*************************发送的数据********************************
//0xff（开头）
uint16_t yaw_send;
uint16_t blood_judge;
uint16_t bullet_judge;
uint16_t radar_judge;
//0xff（结尾）
//**********************************************************************
int send_result=0;//发送成功会反馈0，发送失败反馈1
extern int receive_flag;
u8 Can2SendMessege[8];
u8 send_command;
extern int yawtemp,pitchtemp; 
int speed_receive;
float v_parallel,v_vertical;
float pitchangel,yawangel;
static uint8_t PCRXBuf[100];
extern uint8_t UsbRxBuf[100];
extern uint8_t UsbTxBuf[100];

void ReceiveData(uint8_t *Buffer)
{
	float *_x, *_y,*_z;uint8_t *n;
	unsigned int hor=0,ver=0,z=0,m=0;
	//每将个uint8_t加进去，就将其左移，使得最低的位为0000，然后就可以继续把uint8_t加进去
	for(int i=3;i>=0;i--)//yaw
	{
		hor+=Buffer[i];
		if (i!=0)hor=hor<<8;
	}
	//同上
	for(int i=7;i>=4;i--)//pitch
	{
		ver+=Buffer[i];
		if (i!=4)ver=ver<<8;
	}
	for(int i=11;i>=8;i--)//ser
	{
		z+=Buffer[i];
		if (i!=8)z=z<<8;
	}
	for(int i=13;i>=12;i--)
	{

		m+=Buffer[i];
		if (i!=12)m=m<<8;
	}
	//强制转换
	_x=((float*)&hor);
	_y=((float*)&ver);
	_z=((float*)&z);
	n=((uint8_t*)&m);
		//////////视觉数据接收////////////////
	
	
		Point3f.y=*_y;
		Point3f.x=*_x;
		Point3f.z=*_z;
	  command_receive=(u8)(UsbRxBuf[12]);
}

void ConnectTask(void)
{
	//u8 *command_receive_p;
	//command_receive_p = &command_receive;	
	if(receive_flag)
	{
		float *px,*py,*pz;
		px=&Point3f.x;
		py=&Point3f.y;
		driver_connect_receive(px,
                         py,
		                     &Point3f.z,
                         &command_receive);//接收数据处理
		//ReceiveData(UsbRxBuf);
		receive_flag=0;
	}
	/*if(Point3f.z==0)
	{
		delta_pitch_receive=0;
		delta_yaw_receive=0;
	}
	else
	{
		PreFransform(&yawangel,&pitchangel,pitchtemp,yawtemp,speed_receive,&v_parallel,&v_vertical);
	Transform(Point3f,pitchangel//pitch 的位置
	,yawangel//yaw 位置
		, &delta_pitch_receive//增量
			, &delta_yaw_receive//增量
				,&delay//子弹出枪口到命中的时间
					, v_parallel//横向移动速度，向右为正，毫米每秒
						, (v_vertical+2800));//FOWARD是子弹速度+前进分量
	}*/
  

  send_result=driver_connect_send(blood_judge,bullet_judge,radar_judge);//发送，发送失败反馈1
	
	
	Can2SendMessege[0] = send_command;
	CAN2_Send_Msg(Can2SendMessege,8,0x250);
}


