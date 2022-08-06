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

void General_Control(void);//总控逻辑代码

void AutoShootControl(void);//自动发射控制


//控制判断
int IsGameStart(void);//比赛进程判断		
int BulletNum_Full(void); //子弹剩余数判断
int Qianshaozhan_Save(void);//前哨站血量判断
void Armour_bool(void);//装甲受打击判断


void modeSwitchCalc();


#endif
