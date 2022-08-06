#ifndef __TASK_VISION_H__
#define __TASK_VISION_H__
#include "sys.h"

#define VISION_DATA_LENGTH (12)

typedef struct 
{
	float exp_yaw;
	float exp_pitch;
}Gimbal_Msg;            //Ԥ��ṹ��

void sendMsgToPC(void);
u8 VisionReceiveControlTask(void);
void init_func();
void clear_array();
float get_flying_time(float distance);
Gimbal_Msg Get_ExpPosition(float real_enemy_yaw_angle,float real_enemy_pitch_angle, float time_,float distance);
	
typedef struct
{
	float ReceiveYaw;
	float ReceivePitch;
	
	float ReceiveDistanceRaw;
	float ReceiveDistance;
	
	float ReceiveYaw_last;
	float ReceivePitch_last;
	float ReceiveDistance_last;
	
	float time;
	float DeltaT;
	
	u8 Command;
	
//	u8 		update_flag;
//	u8    YawUpdate;
//	u8  	PitchUpdate;
//	
//	short SendPitchLocation;
//	short SendYawLocation;
//	short Sendvx;
//	short Sendvy;
//	short Sendvz;

	float Change_yaw;
	float Change_pitch;
	
	float Time_bullet_fly;
	
}VisionDataStruct;


typedef union
{
	uint8_t buffer[13];
	struct state
	{
		float pitch;  
		float yaw;
		float bullet;   //����
		//float motion;   //�����˶�
		uint8_t status; //״̬
	}state;
}UsbSendMsg;

typedef struct
{
	float kx_0,kx_1;
	float ky_0,ky_1;
	float kz_0,kz_1;
}K_Lowpass_Point;

typedef struct
{
	float der;
	float eer;
	float lasteer;
	float Out;
	float kd;
}Der;


void usbDataReceiveHandle(void);


void VisionControlTask(void);

void Vision_kalman_init(void);         //������������ʼ��
void Vision_xy_kalman_dispose(void);   //������x,y��������
void Vision_resV_kalman_dispose(void); //�������ٶȴ���
void Vision_yaw_kalman_dispose(void);  //������yaw����
void Vision_z_kalman_dispose(void);    //������z����

void Shijielimit_deal(void);           //�ӽ����ƴ���  
void Pre_interupt_wsol(void);
#endif

