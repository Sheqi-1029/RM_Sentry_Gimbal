/*----------------------------------------------//
|       title:�ڱ���̨��������									 |
|				date: 2021.3                             |
|															CH.                |
//----------------------------------------------*/
#ifndef __TASK_GIMBAL_H__
#define __TASK_GIMBAL_H__
#include "driver_gimbal.h"
#include "sys.h"

typedef enum
{
	GIMBAL_TRACK_ARMOR			=0,			//�Զ�׷��ģʽ
	GIMBAL_PATROL						=1,			//��̨Ѳ��ģʽ
	GIMBAL_REMOTE_CTRL			=2,			//ң����ģʽ
	GIMBAL_RELAX						=3,			//ֹͣ����ģʽ
	GIMBAL_KEEP							=4,			//��̨���ֲ���
	GIMBAL_FIRE             =5,     //�Զ�����ģʽ
	GIMBAL_SEARCH_FRONT			=6,     //ǰ��Ѳ��
	GIMBAL_SEARCH_BACK			=7,     //���Ѳ��
}GimbalModeEnum;

typedef struct
{
	GimbalModeEnum GimbalMode;
	GimbalSetLocationStruct	GimbalSetLocationDataTemp;
	int8_t YawPartolDirection;
}GimbalCtrlStruct;

typedef struct  // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

typedef struct
{
		//short blood;
	  u8 game_mode;
	  
		u8 front_hurt;
	  u8 back_hurt;
	
	  u8 armour_hurt;
		short bulletnum;//ʣ���ӵ���
		short heat1;//����1
	  short heat2;//����2
		u8 bullet_bool;//�ж��Ƿ�ɷ���
	
		int bullet_speed;//�ӵ�����
	
	  short qianshaozhan_blood;
		
		short chassis_dis;    //����
		u8 chassis_Sp;        //�ٶ�����
	  short chassis_speed;  //�ٶȾ���

	
}	Judgement_data;

void GimbalControlTask(void); //�߼�����

//�������
void GimbalDriverTask(void);  //��������
void kalmanFilterInit(void);
void GimbalUseEncoder(u8 isUseEncoder_Yaw,u8 isUseEncoder_Pitch);


//ģʽ���
GimbalModeEnum GetGimbalMode(void);//ģʽѡ���л�
void SetGimbalMode(GimbalModeEnum Mode);//ģʽģʽ
void Gimbal_relax(void);//ֹͣ����ģʽ����
void Gimbal_remote_ctrl(void);//ң����ģʽ����
void Gimbal_patrol(void);//Ѳ��ģʽ
void Gimbal_search_front(void);//ǰ����ģʽ
void Gimbal_search_behind(void);//������ģʽ
void Gimbal_track_armor(void);//�Զ�׷��ģʽ����
void Gimbal_fire(void);//�Զ�����ģʽ����



//��������
void ClearPatrolCout(void);//�Զ���ʱ���������
void Flag_zero_init(void);//��ر�־λ��ʼ��

#endif
