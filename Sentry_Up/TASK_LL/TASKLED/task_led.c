#include "task_led.h"
#include "cmsis_os.h"

#define LED_TMINKLE_TIME 	(180)
#define LED_SLEEP_TIME		(800)

LedModeEnum LedMode = LED_PERFECT;

void LedTwinkleCtrl(u8 times)
{
		for(u8 i=0; i< times ;i++)
		{
				LED_R =	LED_OFF;
				osDelay(LED_TMINKLE_TIME);
				LED_R =	LED_ON;
				osDelay(LED_TMINKLE_TIME);
		}
		LED_R =	LED_OFF;
		osDelay(LED_SLEEP_TIME);
}

void LedControlTask()
{

	switch(LedMode)
	{
		case LED_LOST_PITCH:
			LED_G = LED_OFF;
			LedTwinkleCtrl(1);
			break;
		
		case LED_LOST_YAW:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(2);
			break;
		
		case LED_LOST_FEEDMOTOR:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(3);
			break;
		
		case LED_LOST_FRICTION:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(4);
			break;
		
		case LED_LOST_SIGLE_GYRO:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(5);
			break;
		
		case LED_LOST_JUDGE:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(6);
			break;
		
		case LED_LOST_USB:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(7);
			break;
		
		case LED_TEST_MODE:
			LED_G = LED_OFF;	
			LedTwinkleCtrl(8);
			break;
		
		case LED_PERFECT:
			LED_R = LED_OFF;	
			LED_G=!LED_G;
			osDelay(500);
			break;
		
		default:
			break;
	}
}

//----------------------------------------------LED模式切换接口----------------------------------------------------------
void SetLedMode(LedModeEnum Mode)
{
	LedMode = Mode;
}
LedModeEnum GetLedMode()
{
	return LedMode;
}

