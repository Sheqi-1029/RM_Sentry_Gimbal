#ifndef TASK_LED_H
#define TASK_LED_H
#include "sys.h"

#define LED_G PCout(2)
#define LED_R PCout(3)

#define LED_ON 		(0)
#define LED_OFF		(1)

//LED指示灯描述，编号越小，优先级越高
typedef enum{
	LED_LOST_PITCH 				= 0,
	LED_LOST_YAW 					= 1,
	LED_LOST_FEEDMOTOR		=	2,
	LED_LOST_FRICTION			= 3,
	LED_LOST_SIGLE_GYRO		=	4,
	LED_LOST_JUDGE				=	5,
	LED_LOST_USB					=	6,
	LED_TEST_MODE					= 7,
	LED_PERFECT						=	8,
}LedModeEnum;

void LedControlTask();
void SetLedMode(LedModeEnum Mode);
LedModeEnum GetLedMode();

#endif
