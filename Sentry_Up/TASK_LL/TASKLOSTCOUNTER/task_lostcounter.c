/****************************************************
*			Title:		LOSTCounter
*			ChipType:	STM32F405RGT6
*			Version:	1.2.0
*			Date:			2017.09.20
*												LD.
*****************************************************/


#include "task_lostcounter.h"

#define LOST_COUNTER_TIME_MS (50)
//#define MODE_SWITCH_TIME_MS (50)

u32 LostCounterCountNumber[NUMBERS_OF_COUNT];

//extern LostCounterStruct lost_counter;

void LostCounterInit()
{
	u8 i;
	
	for(i=0;i<NUMBERS_OF_COUNT;i++)
	{
		LostCounterCountNumber[i]=1000;
		//lost_counter.count_number[i]=1000;
	}
}

int lose_flag=0;//Ò£¿ØÆ÷¶ªÊ§Êý¾Ý²¼¶û·û
int close_flag=0;//Ò£¿ØÆ÷¹Ø¿Ø²¼¶û·û
extern u8 Can2_DownSentry_SendMessage[8];//[6] lose_flag;

void LostCounterFeed(u8 i)
{
	LostCounterCountNumber[i]=0;
	//lost_counter.count_number[i]=0;
}

void LostCounterControl(u16 SystemErrorStatus)
{
	if((SystemErrorStatus>>REMOTE_LOST_COUNT)&1) //Ò£¿ØÆ÷¶ªÊý¾Ý
	{
		lose_flag=0;
	}
	else
	{
		lose_flag=1;
	}
	Can2_DownSentry_SendMessage[6] = 1 - lose_flag;
}

u16 LostCounterCount()
{
	u8 i;
	u16 SystemErrorStatus=0;

	/********************************************************************************************
	SystemErrorStatus:
		15  |   14  |   13  |   12  |   11  |   10		|			9			|   	8		   |
				|       |       |       |       |					|	JUDGEMENT	|   VISION	 |
---------------------------------------------------------------------------------------------
		7		|		6			|   5   |   4   	|   3   	|   2   		|   1   	|   	0  		|
REMOTE	|	FEED	  |	YAW		|  PITCH  | MOTOR-3	|  MOTOR-2  | MOTOR-1 |		MOTOR-0	|
********************************************************************************************/
	
	//lost_counter.system_error_status = 0;
	
	for(i=0;i<NUMBERS_OF_COUNT;i++)
	{
		LostCounterCountNumber[i]++;
		//lost_counter.count_number[i]++;
	}	
	if(LostCounterCountNumber[GIMBAL_MOTOR_PITCH]>(GIMBAL_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< GIMBAL_MOTOR_PITCH;
	
	if(LostCounterCountNumber[GIMBAL_MOTOR_YAW]>(GIMBAL_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< GIMBAL_MOTOR_YAW;
	
	if(LostCounterCountNumber[FEEDMOTOR_LOST_COUNT]>(FEEDMOTOR_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< FEEDMOTOR_LOST_COUNT;
	
	if(LostCounterCountNumber[REMOTE_LOST_COUNT]>(REMOTE_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< REMOTE_LOST_COUNT;
	
	if(LostCounterCountNumber[VISION_LOST_COUNT]>(VISION_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< VISION_LOST_COUNT;
	
	if(LostCounterCountNumber[JUDGEMENT_LOST_COUNT]>(JUDGEMENT_LOST_TOLERANCE_MS/LOST_COUNTER_TIME_MS))
		SystemErrorStatus |=	1<< JUDGEMENT_LOST_COUNT;
	
	LostCounterControl(SystemErrorStatus);
	
	return SystemErrorStatus;
	
}

u32* GetLostCounterData()
{
	return LostCounterCountNumber;
}



