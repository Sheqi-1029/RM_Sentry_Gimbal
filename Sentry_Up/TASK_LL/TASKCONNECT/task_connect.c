#include "task_connect.h"
#include "driver_connect.h"
#include "bsp_can.h"
#include "driver_remote.h"
#include "driver_vision.h"
#include "usbd_cdc_if.h"
/****************************************************
*			�빤�ػ���ͨ��Э��
*			ChipType:	STM32F405RGTx
*			Date:			2019.12.15
*												ZHR.
*****************************************************/

/****************************************************
*			�빤�ػ���ͨ��Э������ʽ
      ʹ�õĶ�ʱ����time7
      ͨ�ż����20ms
			ÿ20ms�ᴦ��һ���Ӿ����͵����ݣ����һᷢ��һ�����ݸ����ػ�
      �Ӿ������
      ��ͷ 0xff
      �Ƕ�1��pitch��    32λ   
      �Ƕ�2 ��yaw��   32λ
      ����״̬   16λ
      ��չ   32λ ������ȫ��0
      ��β0xff




      ��ظ��Ӿ�
      ��ͷ0xff
      ��̨λ�ã�����pitch16λ. yaw16λ��
      ��̨�ƶ��ٶȣ�����vx16λ.vy16λ.vz16λ   
      ����״̬8λ 
      ��չ32λ ������ȫ��0 
      ��β0xff





      ����״̬
      �ڱ�  1��Ѳ��  2��׷��  3������   

*			
*			
*****************************************************/

//**********************������Ľ������ݣ�ֱ��ȡ��*******************
//float x_receive;//������һ����pitch�Ƕȱ仯��,ȡֵ��-1��1֮��
//float y_receive;//������һ����yaw�Ƕȱ仯����ȡֵ��-1��1֮��
//float z_receive;
u8 command_receive;//����״̬
extern RemoteDataProcessedStruct RemoteData;
float delta_pitch_receive;
float delta_yaw_receive;
float delay;
Point3fStruct Point3f;
//********************************************************************

//*************************���͵�����********************************
//0xff����ͷ��
uint16_t yaw_send;
uint16_t blood_judge;
uint16_t bullet_judge;
uint16_t radar_judge;
//0xff����β��
//**********************************************************************
int send_result=0;//���ͳɹ��ᷴ��0������ʧ�ܷ���1
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
	//ÿ����uint8_t�ӽ�ȥ���ͽ������ƣ�ʹ����͵�λΪ0000��Ȼ��Ϳ��Լ�����uint8_t�ӽ�ȥ
	for(int i=3;i>=0;i--)//yaw
	{
		hor+=Buffer[i];
		if (i!=0)hor=hor<<8;
	}
	//ͬ��
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
	//ǿ��ת��
	_x=((float*)&hor);
	_y=((float*)&ver);
	_z=((float*)&z);
	n=((uint8_t*)&m);
		//////////�Ӿ����ݽ���////////////////
	
	
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
                         &command_receive);//�������ݴ���
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
	Transform(Point3f,pitchangel//pitch ��λ��
	,yawangel//yaw λ��
		, &delta_pitch_receive//����
			, &delta_yaw_receive//����
				,&delay//�ӵ���ǹ�ڵ����е�ʱ��
					, v_parallel//�����ƶ��ٶȣ�����Ϊ��������ÿ��
						, (v_vertical+2800));//FOWARD���ӵ��ٶ�+ǰ������
	}*/
  

  send_result=driver_connect_send(blood_judge,bullet_judge,radar_judge);//���ͣ�����ʧ�ܷ���1
	
	
	Can2SendMessege[0] = send_command;
	CAN2_Send_Msg(Can2SendMessege,8,0x250);
}


