/****************************************************
*			Title:		Chassis
*			ChipType:	STM32F405RGT6
*			Version:	1.0.7
*			Date:			2017.09.14
*												LD.
*****************************************************/

#ifndef TASK_CHASSIS_H
#define TASK_CHASSIS_H
#include "sys.h"
void ChassisControlTask(void);

typedef enum
{
	NOTFOLLOW=0,
	FOLLOW=1,
	SHAKE=2,
	CHANGEFOLLOW=3,
}ChassisControlModeEnum;

void ChassisSetSpeed(float SpeedX,float SpeedY,float Spin);


#endif
