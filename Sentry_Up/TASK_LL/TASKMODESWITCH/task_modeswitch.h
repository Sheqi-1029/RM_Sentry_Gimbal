#ifndef _MODESWITCH_H
#define _MODESWITCH_H
#include "sys.h"
#include "BSPconfig.h"
#include "stdbool.h"


typedef struct
{
	int  Front;
	int  Back;
}ArmourHurtStruct;

void updateBulletRemaining();

void General_Control(void);//�ܿ��߼�����

void AutoShootControl(void);//�Զ��������


//�����ж�
int IsGameStart(void);//���������ж�		
int BulletNum_Full(void); //�ӵ�ʣ�����ж�
int Qianshaozhan_Save(void);//ǰ��վѪ���ж�
void Armour_bool(void);//װ���ܴ���ж�


void modeSwitchCalc();


#endif
