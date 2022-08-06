#include "driver_feedmotor.h"
#include "pid.h"
#include "math.h"
#include "bsp_can.h"

FeedMotorStruct FeedMotor={0},FeedMotor_UP = {0};

#define FEEDMOTOR_SPEED_KP	(1.6f)
#define FEEDMOTOR_SPEED_KI	(0)
#define FEEDMOTOR_SPEED_KD	(0)
#define FEEDMOTOR_SPEED_AP	(0.8f)
#define FEEDMOTOR_SPEED_BP	(1.6f)
#define FEEDMOTOR_SPEED_CP	(0.4f)

#define FEEDMOTOR_LOCATION_KP	(0.23f)
#define FEEDMOTOR_LOCATION_KI	(0)
#define FEEDMOTOR_LOCATION_KD	(0.4f)
#define FEEDMOTOR_LOCATION_AP	(0.28f)
#define FEEDMOTOR_LOCATION_BP	(0.6f)
#define FEEDMOTOR_LOCATION_CP	(0.05f)

#define FEEDMOTOR_UP_SPEED_KP	(0.6f)
#define FEEDMOTOR_UP_SPEED_KI	(0)
#define FEEDMOTOR_UP_SPEED_KD	(0)
#define FEEDMOTOR_UP_SPEED_AP	(0.3f)
#define FEEDMOTOR_UP_SPEED_BP	(0.6f)
#define FEEDMOTOR_UP_SPEED_CP	(0.01f)

#define FEEDMOTOR_UP_LOCATION_KP	(0.25f)
#define FEEDMOTOR_UP_LOCATION_KI	(0)
#define FEEDMOTOR_UP_LOCATION_KD	(0.0f)
#define FEEDMOTOR_UP_LOCATION_AP	(0.1f)
#define FEEDMOTOR_UP_LOCATION_BP	(0.0f)
#define FEEDMOTOR_UP_LOCATION_CP	(0.0f)
int draw_feed_loca;
int draw_feed_setloca;

//--------------------------------------拨弹电机初始化-----------------------------------------//
void FeedMotorInit(void)
{
	FeedMotor.PIDSpeed.OutMax=0.8;
	FeedMotor.PIDSpeed.OutMin=-0.8;
	FeedMotor.PIDSpeed.calc=&PidCalc;
	FeedMotor.PIDSpeed.clear=&PidClear;
	FeedMotor.PIDSpeed.clear(&FeedMotor.PIDSpeed);
	
	FeedMotor.PIDLocation.OutMax=1;
	FeedMotor.PIDLocation.OutMin=-1;
	FeedMotor.PIDLocation.calc=&PidCalc;
	FeedMotor.PIDLocation.clear=&PidClear;
	FeedMotor.PIDLocation.clear(&FeedMotor.PIDLocation);
	
	FeedMotor.PIDSpeed.Kp=FEEDMOTOR_SPEED_KP;
	FeedMotor.PIDSpeed.Ki=FEEDMOTOR_SPEED_KI;
	FeedMotor.PIDSpeed.Kd=FEEDMOTOR_SPEED_KD;
	FeedMotor.PIDSpeed.Ap=FEEDMOTOR_SPEED_AP;
	FeedMotor.PIDSpeed.Bp=FEEDMOTOR_SPEED_BP;
	FeedMotor.PIDSpeed.Cp=FEEDMOTOR_SPEED_CP;
	
	FeedMotor.PIDLocation.Kp=FEEDMOTOR_LOCATION_KP;
	FeedMotor.PIDLocation.Ki=FEEDMOTOR_LOCATION_KI;
	FeedMotor.PIDLocation.Kd=FEEDMOTOR_LOCATION_KD;
	FeedMotor.PIDLocation.Ap=FEEDMOTOR_LOCATION_AP;
	FeedMotor.PIDLocation.Bp=FEEDMOTOR_LOCATION_BP;
	FeedMotor.PIDLocation.Cp=FEEDMOTOR_LOCATION_CP;
	
	FeedMotor_UP.PIDSpeed.OutMax=0.8;
	FeedMotor_UP.PIDSpeed.OutMin=-0.8;
	FeedMotor_UP.PIDSpeed.calc=&PidCalc;
	FeedMotor_UP.PIDSpeed.clear=&PidClear;
	FeedMotor_UP.PIDSpeed.clear(&FeedMotor_UP.PIDSpeed);
	
	FeedMotor_UP.PIDLocation.OutMax=1;
	FeedMotor_UP.PIDLocation.OutMin=-1;
	FeedMotor_UP.PIDLocation.calc=&PidCalc;
	FeedMotor_UP.PIDLocation.clear=&PidClear;
	FeedMotor_UP.PIDLocation.clear(&FeedMotor_UP.PIDLocation);
	
	FeedMotor_UP.PIDSpeed.Kp=FEEDMOTOR_UP_SPEED_KP;
	FeedMotor_UP.PIDSpeed.Ki=FEEDMOTOR_UP_SPEED_KI;
	FeedMotor_UP.PIDSpeed.Kd=FEEDMOTOR_UP_SPEED_KD;
	FeedMotor_UP.PIDSpeed.Ap=FEEDMOTOR_UP_SPEED_AP;
	FeedMotor_UP.PIDSpeed.Bp=FEEDMOTOR_UP_SPEED_BP;
	FeedMotor_UP.PIDSpeed.Cp=FEEDMOTOR_UP_SPEED_CP;
	
	FeedMotor_UP.PIDLocation.Kp=FEEDMOTOR_UP_LOCATION_KP;
	FeedMotor_UP.PIDLocation.Ki=FEEDMOTOR_UP_LOCATION_KI;
	FeedMotor_UP.PIDLocation.Kd=FEEDMOTOR_UP_LOCATION_KD;
	FeedMotor_UP.PIDLocation.Ap=FEEDMOTOR_UP_LOCATION_AP;
	FeedMotor_UP.PIDLocation.Bp=FEEDMOTOR_UP_LOCATION_BP;
	FeedMotor_UP.PIDLocation.Cp=FEEDMOTOR_UP_LOCATION_CP;
}

//--------------------------------------拨弹编码器速度返回-------------------------------//
void FeedMotorSpeedDataUpdate(void)							
{
	FeedMotor.Speed.Speed=((int16_t)((FeedMotor.ReceiveMessege[2]<<8)|FeedMotor.ReceiveMessege[3]));
	FeedMotor.Speed.Speed = FeedMotor.Speed.Speed/9600;
	
	FeedMotor_UP.Speed.Speed=((int16_t)((FeedMotor_UP.ReceiveMessege[2]<<8)|FeedMotor_UP.ReceiveMessege[3]));
	FeedMotor_UP.Speed.Speed = FeedMotor_UP.Speed.Speed/9600;
}
//--------------------------------------拨弹编码器转矩返回-------------------------------//
void FeedMotorCurrentDataUpdate(void)		
{
	FeedMotor.Current = ((int16_t)((FeedMotor.ReceiveMessege[4]<<8)|FeedMotor.ReceiveMessege[5]));
	FeedMotor_UP.Current = ((int16_t)((FeedMotor_UP.ReceiveMessege[4]<<8)|FeedMotor_UP.ReceiveMessege[5]));
}

//--------------------------------------拨弹编码器位置返回-------------------------------//
float LocationDataLast=0;
int LocationCount=0;
float Location_UPDataLast=0;
int Location_UPCount=0;

void FeedMotorLocationDataUpdate(void)
{
	//发射拨弹
	float LocationDataTemp;

	LocationDataTemp = ((int16_t)((FeedMotor.ReceiveMessege[0]<<8)|FeedMotor.ReceiveMessege[1]));
	LocationDataTemp = LocationDataTemp/8191;
	
	if			(LocationDataTemp	-	LocationDataLast	>	0.50f)
		LocationCount--;
	else if	(LocationDataTemp	-	LocationDataLast	<	-0.50f)
		LocationCount++;
	
	LocationDataLast 	= LocationDataTemp;
	
	FeedMotor.Location.Location	=	LocationDataTemp + LocationCount;
	//上拨弹
	float LocationDataTemp_UP;

	LocationDataTemp_UP = ((int16_t)((FeedMotor_UP.ReceiveMessege[0]<<8)|FeedMotor_UP.ReceiveMessege[1]));
	LocationDataTemp_UP = LocationDataTemp_UP/8191;
	
	if			(LocationDataTemp_UP	-	Location_UPDataLast	>	0.50f)
		Location_UPCount--;
	else if	(LocationDataTemp_UP	-	Location_UPDataLast	<	-0.50f)
		Location_UPCount++;
	
	Location_UPDataLast 	= LocationDataTemp_UP;
	
	FeedMotor_UP.Location.Location	=	LocationDataTemp_UP + Location_UPCount;
}
//--------------------------------------拨弹编码器位置清零-------------------------------//
void FeedMotorLocationClear(void)
{
		LocationDataLast = 0;
		LocationCount = 0;
		FeedMotor.Location.Location = 0;
		
		Location_UPDataLast = 0;
		Location_UPCount = 0;
		FeedMotor_UP.Location.Location = 0;
}
//--------------------------------------拨弹编码器数据更新-------------------------------//
void FeedMotorDataUpdate()
{
		FeedMotorSpeedDataUpdate();
		FeedMotorLocationDataUpdate();
		FeedMotorCurrentDataUpdate();
}
//---------------------------------------赋值电机pid结构体-------------------------------//
void MotorLocationControlLogic(void)
{
	u8 Can2FeedMotorSendMessege[8] = {0};
	
	FeedMotor.PIDLocation.Ref	=	FeedMotor.Location.SetLocation;
	FeedMotor.PIDLocation.Fdb	=	FeedMotor.Location.Location;
	
	FeedMotor.PIDLocation.calc(&FeedMotor.PIDLocation);
	
	FeedMotor.Speed.SetSpeed	=	FeedMotor.PIDLocation.Out;
	
	FeedMotor.PIDSpeed.Ref	=	FeedMotor.Speed.SetSpeed;
	FeedMotor.PIDSpeed.Fdb	=	FeedMotor.Speed.Speed;
	
	FeedMotor.PIDSpeed.calc(&FeedMotor.PIDSpeed);
	
	//Can2FeedMotorSendMessege[4]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))>>8;
	//Can2FeedMotorSendMessege[5]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))&0x00ff;

	FeedMotor_UP.PIDLocation.Ref	=	FeedMotor_UP.Location.SetLocation;
	FeedMotor_UP.PIDLocation.Fdb	=	FeedMotor_UP.Location.Location;
	
	FeedMotor_UP.PIDLocation.calc(&FeedMotor_UP.PIDLocation);
	
	FeedMotor_UP.Speed.SetSpeed	=	FeedMotor_UP.PIDLocation.Out;
	
	FeedMotor_UP.PIDSpeed.Ref	=	FeedMotor_UP.Speed.SetSpeed;
	FeedMotor_UP.PIDSpeed.Fdb	=	FeedMotor_UP.Speed.Speed;
	
	FeedMotor_UP.PIDSpeed.calc(&FeedMotor_UP.PIDSpeed);
	
	
	draw_feed_loca = (int)((FeedMotor.Location.Location-LocationCount) * 8191);
	draw_feed_setloca = (int)((FeedMotor.Location.SetLocation-LocationCount) * 8191);
	
	
#if DEBUG_USE_GIMBALMOTOR_CANSEND
	//CAN2_Send_Msg(Can2FeedMotorSendMessege,8,0x1ff);
#endif
}

//-----------------------------------（无用的函数）-------------------------------------//
void MotorSpeedControlLogic(void)
{
	FeedMotor.PIDSpeed.Ref	=	FeedMotor.Speed.SetSpeed;
	FeedMotor.PIDSpeed.Fdb	=	FeedMotor.Speed.Speed;
	
	FeedMotor.PIDSpeed.calc(&FeedMotor.PIDSpeed);
}
//--------------------------------赋值电机速度-------------------------------//
void SetFeedMotorSetSpeed(float SetSpeed)
{
	FeedMotor.Speed.SetSpeed = SetSpeed;
}
//--------------------------------赋值电机位置-------------------------------//
void SetFeedMotorSetLocation(float SetLocation)
{
	FeedMotor.Location.SetLocation = SetLocation;
}
//--------------------------------赋值电机速度-------------------------------//
void SetFeedMotorUPSetSpeed(float SetSpeed)
{
	FeedMotor_UP.Speed.SetSpeed = SetSpeed;
}
//--------------------------------赋值电机位置-------------------------------//
void SetFeedMotorUPSetLocation(float SetLocation)
{
	FeedMotor_UP.Location.SetLocation = SetLocation;
}
//--------------------------------pid结构体参数清空-------------------------------//
void FeedMotorPIDClear(void)
{
	FeedMotor.PIDSpeed.clear(&FeedMotor.PIDSpeed);
	FeedMotor.PIDLocation.clear(&FeedMotor.PIDLocation);
	FeedMotor_UP.PIDSpeed.clear(&FeedMotor_UP.PIDSpeed);
	FeedMotor_UP.PIDLocation.clear(&FeedMotor_UP.PIDLocation);
}
//--------------------------位置取值----------------------------//
float GetFeedMotorLocation(void)
{
	return FeedMotor.Location.Location;
}
float GetFeedMotorSetLocation(void)
{
	return FeedMotor.Location.SetLocation;
}

float GetFeedMotorSpeed(void)
{
	return FeedMotor.Speed.Speed;
}

float GetFeedMotorLocationError(void)
{
		return FeedMotor.PIDLocation.Err;
}

float GetFeedMotorCurrent(void)
{
		return FeedMotor.Current;
}

float GetFeedMotorUPLocation(void)
{
	return FeedMotor_UP.Location.Location;
}

float GetFeedMotorUPSetLocation(void)
{
	return FeedMotor_UP.Location.SetLocation;
}

float GetFeedMotorUPSpeed(void)
{
	return FeedMotor_UP.Speed.Speed;
}

float GetFeedMotorUPLocationError(void)
{
		return FeedMotor_UP.PIDLocation.Err;
}

float GetFeedMotorUPCurrent(void)
{
		return FeedMotor_UP.Current;
}
