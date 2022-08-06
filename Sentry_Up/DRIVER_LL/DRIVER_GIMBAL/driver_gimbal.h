/****************************************************
*			Title:		��̨����
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#ifndef __DRIVER_GIMBAL_H__
#define __DRIVER_GIMBAL_H__
#include "sys.h"
#include "pid.h"
#include "BSPconfig.h"
#define PITCH_INIT_VALUE PITCH_INIT_VALUE_SET		//0.65
#define PITCH_TEMPSET (PITCH_INIT_VALUE>0.5f ? 1.0f:0.0f)
#define YAW_INIT_VALUE YAW_INIT_VALUE_SET 		//0.25
#define YAW_TEMPSET	(YAW_INIT_VALUE>0.5f ? 1.0f:0.0f)

#define PITCH_MAX_VALUE (PITCH_INIT_VALUE+0.059f)
#define PITCH_MIN_VALUE (PITCH_INIT_VALUE-0.029f)

#define YAW_MAX_VALUE (YAW_INIT_VALUE+0.42f)
#define YAW_MIN_VALUE (YAW_INIT_VALUE-0.42f)

void GimbalInit(void);
void GimbalReturnToInitLocation(u8 IfPitch,u8 IfYaw);
void GimbalPIDLocationClear(u8 IfPitch,u8 IfYaw);

typedef struct					//����ٶ����ֵ
{
	float	SetSpeed;
	float Speed;
}SpeedStruct;						//�ٶȽṹ�嶨��

typedef struct					//���λ�����ֵ
{
	float	SetLocation;
	float Location;
}LocationStruct;				//λ�ýṹ�嶨��

typedef struct
{
	LocationStruct	Location;
	PID	PIDLocation;
	SpeedStruct	Speed;
	PID	PIDSpeed;
	u8 CANReceiveMessege[8];
}GimbalMotorStruct;

typedef struct
{
	float Yaw;
	float Pitch;
}GyroDataStruct;

typedef struct
{
	float Yaw;
	float Pitch;
	float yawspeed;
	float pitchspeed;
}EncoderDataStruct;

typedef struct
{
	float PitchSetLocation;
	float YawSetLocation;

	u8 FlagPitchUseEncoder;
	u8 FlagYawUseEncoder;
}GimbalSetLocationStruct;

float GetGimbalError(void);
float GetYawEncoderValue(void);
float GetYawGyroValue(void);
void GimbalPIDClear(void);
void GimbalDataInit(void);
void PitchDataInit(void);

void GimbalControlTask(void);
void GyroAndEncoderDataGet(void);
EncoderDataStruct YawPitchEncoderDataUpdate(void);
GyroDataStruct YawPitchGyroDataUpdate(void);
void GimbalDataInput(GimbalSetLocationStruct GimbalData);
void GimbalControlCalculateAndSend(void);
void GyroAndEncoderDataGet(void);					//������ֵ��ʱ���£�1ms
void GimbalSpeedDataUpdate(void);									//��̨�ٶȸ���

float GetYawLocation();
float GetPitchLocation();
float getYawSetLocation();
float getPitchSetLocation();
float GetYawLocationRaw();
float GetYawSpeedRaw();

void kalman_speed_dispose(void);
#endif