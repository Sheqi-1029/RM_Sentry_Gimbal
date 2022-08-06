#ifndef __INTERFACE_PACER_H
#define __INTERFACE_PACER_H
#include "sys.h"
#include "data_channel_judge_system.h"


//整车自检信息
typedef __packed struct
{
	//摩擦轮状态
	u8 shooter_status;
	//wifi状态
	u8 wifi_status;
	//陀螺仪状态
	u8 gyroscope_status;
	//pitch轴电机状态
	u8 pitch_status;
	//Yaw轴电机状态
	u8 yaw_status;
	//遥控器状态
	u8 controller_status;
	//底盘电机4状态
	u8 chassis4_status;
	//底盘电机3状态
	u8 chassis3_status;
	//底盘电机2状态
	u8 chassis2_status;
	//底盘电机1状态
	u8 chassis1_status;
}Self_Check_TypeDef;


//整车调试信息
typedef __packed struct{
	//1.功率
	float power;
	//2.陀螺仪数据
	float gyroscope_yaw;
	float gyroscope_pitch;
	//3.lostcounter数据
	//lostcounter数据：遥控器
	int lost_counter_controller;
	//lostcounter数据：四底盘
	int lost_counter_chassis_1;
	int lost_counter_chassis_2;
	int lost_counter_chassis_3;
	int lost_counter_chassis_4;
	//lostcounter数据：云台
	int lost_counter_cloud_deck_yaw;
	int lost_counter_cloud_deck_pitch;
	//lostcounter数据：陀螺仪
	int lost_counter_gyroscope;
	//4.码盘数据
	//码盘数据：yaw
	float encoding_disk_yaw;
	//码盘数据：pitch
	float encoding_disk_pitch;
}Debug_Vehicle_Data;
typedef union{
	char byte_data[52];
	Debug_Vehicle_Data debug_vehicle_data;
}Debug_Vehicle_Data_Union;


#endif
