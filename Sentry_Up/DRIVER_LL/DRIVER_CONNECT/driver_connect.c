#include "driver_connect.h"
#include "usbd_cdc_if.h"
#include "driver_gimbal.h"
#include "math.h"
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
      开头1 0x5A
			开头2 0xA5
      角度1（pitch）    32位   
      角度2 （yaw）   32位
      命令状态   16位
      扩展   16位 不开启全发0
      结尾0xff




      电控给视觉
      开头0xff
      云台位置，包括pitch16位. yaw16位，
      云台移动速度，包括vx16位.vy16位.vz16位   
      命令状态8位 
      扩展32位 不开启全发0 
      结尾0xff





      命令状态（推荐）：步兵：1、手瞄 2、自瞄 3、打符
      哨兵  1、巡逻  2、开火  3、反导   
      
*			
*****************************************************/
extern uint8_t UsbRxBuf[100];
//uint8_t UsbTxBuf[100];
float pitch_tem,yaw_tem;
extern GimbalMotorStruct	YawMotor,PitchMotor;

void driver_connect_receive(float *x_receive,
                            float *y_receive,
                            float *z_receive,
                            u8 *command_receive)
{
//	if((UsbRxBuf[0]&0xff)!=0xff||(UsbRxBuf[13]&0xff)!=0xff)//数据发生错误,无法使用
//	{
//		return;
//	}
//	else//数据接收正确，进行数据拆分
	if (((UsbRxBuf[0]&0xff)!=0x5A)||((UsbRxBuf[1]&0xff)!=0xA5))
	{
		return;
	}
	else
	{
    *x_receive = (float)((UsbRxBuf[5]&0xff) | ((UsbRxBuf[4]&0xff)<<8) | ((UsbRxBuf[3]&0xff)<<16) | ((UsbRxBuf[2]&0xff)<<24));
    
		*y_receive = (float)((UsbRxBuf[9]) | (UsbRxBuf[8]<<8) | (UsbRxBuf[7]<<16) | (UsbRxBuf[6]<<24));
		
		*command_receive=(short)((UsbRxBuf[10]) | (UsbRxBuf[9]<<8));
	}
}

int driver_connect_send(
	                       uint16_t blood_judge,
	                       uint16_t bullet_judge,
												 uint16_t radar_judge)
{//数据打包，准备发送

	/*UsbTxBuf[0]=0xA3;
	UsbTxBuf[1]=0x3A;
	UsbTxBuf[2]=0x0010>>8;UsbTxBuf[3]=0x0010&0xff;
	
	UsbTxBuf[4]=0x0020>>8;UsbTxBuf[5]=0x0020&0xff;
	
	UsbTxBuf[6]=0x0030>>8;UsbTxBuf[7]=0x0030&0xff;
	
	UsbTxBuf[8]=0x0040>>8;UsbTxBuf[9]=0x0040&0xff;
	
	UsbTxBuf[10]=0x0050>>8;UsbTxBuf[11]=0x0050&0xff;
	
	UsbTxBuf[12]=0x60;
  UsbTxBuf[16]=0xff;
  int result=1;
//	int send_times=0;//发送次数，若多次发送都失败则反馈回失败，result=1
//	while(result||send_times>=3)
//	{
	  result= CDC_Transmit_FS(UsbTxBuf,8);//发送函数
//		send_times++;
//	}
	if(result==0)//检查是否发送失败
	{
		return 1;
	}
	else return 0;*/
}