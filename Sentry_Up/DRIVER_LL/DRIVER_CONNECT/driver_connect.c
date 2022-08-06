#include "driver_connect.h"
#include "usbd_cdc_if.h"
#include "driver_gimbal.h"
#include "math.h"
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
      ��ͷ1 0x5A
			��ͷ2 0xA5
      �Ƕ�1��pitch��    32λ   
      �Ƕ�2 ��yaw��   32λ
      ����״̬   16λ
      ��չ   16λ ������ȫ��0
      ��β0xff




      ��ظ��Ӿ�
      ��ͷ0xff
      ��̨λ�ã�����pitch16λ. yaw16λ��
      ��̨�ƶ��ٶȣ�����vx16λ.vy16λ.vz16λ   
      ����״̬8λ 
      ��չ32λ ������ȫ��0 
      ��β0xff





      ����״̬���Ƽ�����������1������ 2������ 3�����
      �ڱ�  1��Ѳ��  2������  3������   
      
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
//	if((UsbRxBuf[0]&0xff)!=0xff||(UsbRxBuf[13]&0xff)!=0xff)//���ݷ�������,�޷�ʹ��
//	{
//		return;
//	}
//	else//���ݽ�����ȷ���������ݲ��
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
{//���ݴ����׼������

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
//	int send_times=0;//���ʹ���������η��Ͷ�ʧ��������ʧ�ܣ�result=1
//	while(result||send_times>=3)
//	{
	  result= CDC_Transmit_FS(UsbTxBuf,8);//���ͺ���
//		send_times++;
//	}
	if(result==0)//����Ƿ���ʧ��
	{
		return 1;
	}
	else return 0;*/
}