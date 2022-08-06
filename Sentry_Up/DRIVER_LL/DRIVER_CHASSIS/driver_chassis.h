/****************************************************
*			Title:		Chassis
*			ChipType:	STM32F405RGT6
*			Version:	1.0.7
*			Date:			2017.09.14
*												LD.
*****************************************************/

#ifndef DRIVER_CHASSIS_H
#define DRIVER_CHASSIS_H

#include "sys.h"
#include "BSPconfig.h"
#include "pid.h"

#if CONFIG_USE_CHASSIS
//电机数据结构体
typedef struct
{
	struct{
		float	SetSpeed;
		float	Speed;
	}Speed;
	
	u8 SpeedReceiveMessege[8];
	
	PID PIDSpeed;
	
}ChassisMotorStruct;

typedef struct
{
	float SetSpeedX;
	float SetSpeedY;
	float Spin;
}ChassisSpeedMessegePort;

extern ChassisMotorStruct ChassisMotor[4];


void ChassisInit(void);
void ChassisControl(ChassisSpeedMessegePort ChassisSpeed);
void ChassisFollowInit(void);
void SetChassisFollowRefTargetValue(void);
void SetChassisFollowRef(float SetLocation);
void ChassisFollowCalculate(ChassisSpeedMessegePort *ChassisSpeed);
void ChassisShakeCalculate(ChassisSpeedMessegePort *ChassisSpeed);
void ChassisChangeFollow(ChassisSpeedMessegePort *ChassisSpeed);

#endif

#endif
