#ifndef __TASK_VISION_H__
#define __TASK_VISION_H__
#include "sys.h"

#define VISION_DATA_LENGTH (12)

typedef struct 
{
	float exp_yaw;
	float exp_pitch;
}Gimbal_Msg;            //预测结构体

void sendMsgToPC(void);
u8 VisionReceiveControlTask(void);
void init_func();
void clear_array();
float get_flying_time(float distance);
Gimbal_Msg Get_ExpPosition(float real_enemy_yaw_angle,float real_enemy_pitch_angle, float time_,float distance);
	
typedef struct
{
	float ReceiveYaw;
	float ReceivePitch;
	
	float ReceiveDistanceRaw;
	float ReceiveDistance;
	
	float ReceiveYaw_last;
	float ReceivePitch_last;
	float ReceiveDistance_last;
	
	float time;
	float DeltaT;
	
	u8 Command;
	
//	u8 		update_flag;
//	u8    YawUpdate;
//	u8  	PitchUpdate;
//	
//	short SendPitchLocation;
//	short SendYawLocation;
//	short Sendvx;
//	short Sendvy;
//	short Sendvz;

	float Change_yaw;
	float Change_pitch;
	
	float Time_bullet_fly;
	
}VisionDataStruct;


typedef union
{
	uint8_t buffer[13];
	struct state
	{
		float pitch;  
		float yaw;
		float bullet;   //射速
		//float motion;   //底盘运动
		uint8_t status; //状态
	}state;
}UsbSendMsg;

typedef struct
{
	float kx_0,kx_1;
	float ky_0,ky_1;
	float kz_0,kz_1;
}K_Lowpass_Point;

typedef struct
{
	float der;
	float eer;
	float lasteer;
	float Out;
	float kd;
}Der;


void usbDataReceiveHandle(void);


void VisionControlTask(void);

void Vision_kalman_init(void);         //卡尔曼参数初始化
void Vision_xy_kalman_dispose(void);   //卡尔曼x,y初步处理
void Vision_resV_kalman_dispose(void); //卡尔曼速度处理
void Vision_yaw_kalman_dispose(void);  //卡尔曼yaw处理
void Vision_z_kalman_dispose(void);    //卡尔曼z处理

void Shijielimit_deal(void);           //视界限制处理  
void Pre_interupt_wsol(void);
#endif

