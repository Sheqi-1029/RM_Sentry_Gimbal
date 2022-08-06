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
	FEEDMOTOR_REMOTE_CTRL			=1,			//遥控	
	FEEDMOTOR_HIGH_FREQ 			=2,			//近距离高速连发
	FEEDMOTOR_LOW_FREQ				=3,			//远距离低速连发
	FEEDMOTOR_STOP						=4,			//停止模式
	FEEDMOTOR_FILL_UP					=5,    	//赛前填满弹道
	FEEDMOTOR_RELAX						=6,
}FeedMotorModeEnum;

typedef struct
{
	FeedMotorModeEnum FeedMotorMode;
	u8 shoot_freq;
	float SetLocation;						//设定位置
	u8 FeedMotorEnable;						//是否使能射击
	int ShootFreqCount;						//计数
	unsigned char DeverseLocked;	//堵转反转标志位
	u8 is_fill_up;							//填补弹道标志位
}FeedMotorCtrlStruct;

typedef struct{
		float GunEnergy;							//枪口热量
		u8 IsValid;										//有效性
}EnergyInfoStruct;



void FeedMotorControlLogic(void);//发射控制逻辑
void FeedMotor_UpControlLogic(void);//上拨弹控制逻辑
//发射驱动相关
void LockedMotorDetectionAndProcessed(void);//堵转
void LockedUPMotorDetectionAndProcessed(void);//上拨弹堵转
void FeedMotorLocationUpdate(unsigned char ShootNumber);	//子弹发射
void FeedMotorUPLocationUpdate(unsigned char ShootNumber);//上拨弹子弹拨动
void FeedMotorKeepShooting(int Freq);//发射子弹连发频率模式控制
void FeedMotorUPKeepShooting(int Freq);//发射子弹连发频率模式控制
void FeedMotorCountShooting(int Num);//子弹连发数量模式控制
int FeedFitFreqcal(void);//子弹发射频率适应性计算

//热量控制
u8 IsBulletLimitForceClose(void);
void ReceiveGunEnergyMsg(void);
void LostGunEnergyMsg(void);
float GetGunEnergy(void);
void GunEnergyCoolingCalc(void);
void AddGunEnergy(void);
void JudgementGunEnergyUpdate(void);
int  GunEnergyJudge(void);

//模式相关
FeedMotorModeEnum GetFeedMotorMode(void);//返回模式
void SetFeedMotorMode(FeedMotorModeEnum Mode); //模式赋值


void FeedMotor_InitGame(void);//初始化机械对位（降低延时）
void FeedMotor_remote_ctrl(void);//遥控器模式控制
void FeedMotor_fill_up(void);//赛前填满弹道
void FeedMotor_high_freq(void);//高频自动挡射击
void FeedMotor_low_freq(void);//低频自动档射击

void FeedMotor_UP_InitGame(void);														       //初始化机械对位(降低延时)
void FeedMotor_UP_remote_ctrl(void);													  	 //遥控发射模式
void FeedMotor_UP_high_freq(void);                                 //高频自动挡补弹
void FeedMotor_UP_low_freq(void);                                  //低频自动挡补弹

void ClearTimeDartle(void);//连发记数清零



void CalcAVR_bulletspeed(void);//均值滤波弹速
float Get_AVR_bulletspeed(void);//

#endif
