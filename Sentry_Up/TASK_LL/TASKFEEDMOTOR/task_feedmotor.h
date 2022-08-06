/****************************************************
*			Title:		TaskFeedMotor
*			ChipType:	STM32F405RGT6
*			Version:	1.2.3
*			Date:			2017.09.23
*												LD.
*****************************************************/

#ifndef __TASKFEEDMOTOR_H
#define __TASKFEEDMOTOR_H	 
#include "sys.h"

typedef enum
{
	FEEDMOTOR_INIT          	=0,
	FEEDMOTOR_REMOTE_CTRL			=1,			//ң��	
	FEEDMOTOR_HIGH_FREQ 			=2,			//�������������
	FEEDMOTOR_LOW_FREQ				=3,			//Զ�����������
	FEEDMOTOR_STOP						=4,			//ֹͣģʽ
	FEEDMOTOR_FILL_UP					=5,    	//��ǰ��������
	FEEDMOTOR_RELAX						=6,
}FeedMotorModeEnum;

typedef struct
{
	FeedMotorModeEnum FeedMotorMode;
	u8 shoot_freq;
	float SetLocation;						//�趨λ��
	u8 FeedMotorEnable;						//�Ƿ�ʹ�����
	int ShootFreqCount;						//����
	unsigned char DeverseLocked;	//��ת��ת��־λ
	u8 is_fill_up;							//�������־λ
}FeedMotorCtrlStruct;

typedef struct{
		float GunEnergy;							//ǹ������
		u8 IsValid;										//��Ч��
}EnergyInfoStruct;



void FeedMotorControlLogic(void);//��������߼�
void FeedMotor_UpControlLogic(void);//�ϲ��������߼�
//�����������
void LockedMotorDetectionAndProcessed(void);//��ת
void LockedUPMotorDetectionAndProcessed(void);//�ϲ�����ת
void FeedMotorLocationUpdate(unsigned char ShootNumber);	//�ӵ�����
void FeedMotorUPLocationUpdate(unsigned char ShootNumber);//�ϲ����ӵ�����
void FeedMotorKeepShooting(int Freq);//�����ӵ�����Ƶ��ģʽ����
void FeedMotorUPKeepShooting(int Freq);//�����ӵ�����Ƶ��ģʽ����
void FeedMotorCountShooting(int Num);//�ӵ���������ģʽ����
int FeedFitFreqcal(void);//�ӵ�����Ƶ����Ӧ�Լ���

//��������
u8 IsBulletLimitForceClose(void);
void ReceiveGunEnergyMsg(void);
void LostGunEnergyMsg(void);
float GetGunEnergy(void);
void GunEnergyCoolingCalc(void);
void AddGunEnergy(void);
void JudgementGunEnergyUpdate(void);
int  GunEnergyJudge(void);

//ģʽ���
FeedMotorModeEnum GetFeedMotorMode(void);//����ģʽ
void SetFeedMotorMode(FeedMotorModeEnum Mode); //ģʽ��ֵ


void FeedMotor_InitGame(void);//��ʼ����е��λ��������ʱ��
void FeedMotor_remote_ctrl(void);//ң����ģʽ����
void FeedMotor_fill_up(void);//��ǰ��������
void FeedMotor_high_freq(void);//��Ƶ�Զ������
void FeedMotor_low_freq(void);//��Ƶ�Զ������

void FeedMotor_UP_InitGame(void);														       //��ʼ����е��λ(������ʱ)
void FeedMotor_UP_remote_ctrl(void);													  	 //ң�ط���ģʽ
void FeedMotor_UP_high_freq(void);                                 //��Ƶ�Զ�������
void FeedMotor_UP_low_freq(void);                                  //��Ƶ�Զ�������

void ClearTimeDartle(void);//������������



void CalcAVR_bulletspeed(void);//��ֵ�˲�����
float Get_AVR_bulletspeed(void);//

#endif
