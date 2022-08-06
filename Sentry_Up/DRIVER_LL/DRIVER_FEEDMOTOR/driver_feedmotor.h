#ifndef __DRIVERFEEDMOTOR_H
#define __DRIVERFEEDMOTOR_H	 
#include "sys.h"
#include "BSPconfig.h"
#include "pid.h"

typedef struct
{
	struct{
		float	SetSpeed;
		float	Speed;
	}Speed;
	
	struct{
		float	SetLocation;
		float	Location;
	}Location;
	
	u8 ReceiveMessege[8];
	
	PID PIDSpeed;
	PID PIDLocation;
	float Current;
}FeedMotorStruct;

#if CONFIG_USE_FEEDMOTOR
		void FeedMotorInit(void);
		void FeedMotorSpeedDataUpdate(void)	;
		void FeedMotorLocationDataUpdate(void);
		void FeedMotorDataUpdate(void);
		void MotorLocationControlLogic(void);
		void SetFeedMotorSetLocation(float SetLocation);
		void FeedMotorPIDClear(void);
		float GetFeedMotorLocation(void);
    float GetFeedMotorSetLocation(void);
		float GetFeedMotorSpeed(void);	
		float GetFeedMotorpeedOut(void);
		void FeedMotorSet(u8 FeedMotorEnable);
		void MotorSpeedControlLogic(void);
		void SetFeedMotorSetSpeed(float SetSpeed);
		void SetFeedMotorSpeedOut(float SpeedOut);
		void FeedMotorLocationClear(void);
		float GetFeedMotorLocationError(void);
		float GetFeedMotorCurrent(void);
		float GetFeedMotorUPLocation(void);
		float GetFeedMotorUPSetLocation(void);
		float GetFeedMotorUPSpeed(void);
		float GetFeedMotorUPLocationError(void);
		float GetFeedMotorUPCurrent(void);
    void SetFeedMotorUPSetSpeed(float SetSpeed);
    void SetFeedMotorUPSetLocation(float SetLocation);
#endif


#endif	
