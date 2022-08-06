/*----------------------------------------------//
|       title:哨兵云台控制任务									 |
|				date: 2021.3                             |
|															CH.                |
//----------------------------------------------*/
#ifndef __TASK_GIMBAL_H__
#define __TASK_GIMBAL_H__
#include "driver_gimbal.h"
#include "sys.h"

typedef enum
{
	GIMBAL_TRACK_ARMOR			=0,			//自动追踪模式
	GIMBAL_PATROL						=1,			//云台巡逻模式
	GIMBAL_REMOTE_CTRL			=2,			//遥控器模式
	GIMBAL_RELAX						=3,			//停止运行模式
	GIMBAL_KEEP							=4,			//云台保持不动
	GIMBAL_FIRE             =5,     //自动开火模式
	GIMBAL_SEARCH_FRONT			=6,     //前侧巡逻
	GIMBAL_SEARCH_BACK			=7,     //后侧巡逻
}GimbalModeEnum;

typedef struct
{
	GimbalModeEnum GimbalMode;
	GimbalSetLocationStruct	GimbalSetLocationDataTemp;
	int8_t YawPartolDirection;
}GimbalCtrlStruct;

typedef struct  // speed_calc_data_t
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

typedef struct
{
		//short blood;
	  u8 game_mode;
	  
		u8 front_hurt;
	  u8 back_hurt;
	
	  u8 armour_hurt;
		short bulletnum;//剩余子弹数
		short heat1;//热量1
	  short heat2;//热量2
		u8 bullet_bool;//判断是否可发射
	
		int bullet_speed;//子弹射速
	
	  short qianshaozhan_blood;
		
		short chassis_dis;    //距离
		u8 chassis_Sp;        //速度正负
	  short chassis_speed;  //速度距离

	
}	Judgement_data;

void GimbalControlTask(void); //逻辑控制

//驱动相关
void GimbalDriverTask(void);  //驱动控制
void kalmanFilterInit(void);
void GimbalUseEncoder(u8 isUseEncoder_Yaw,u8 isUseEncoder_Pitch);


//模式相关
GimbalModeEnum GetGimbalMode(void);//模式选择切换
void SetGimbalMode(GimbalModeEnum Mode);//模式模式
void Gimbal_relax(void);//停止运行模式控制
void Gimbal_remote_ctrl(void);//遥控器模式控制
void Gimbal_patrol(void);//巡逻模式
void Gimbal_search_front(void);//前搜索模式
void Gimbal_search_behind(void);//后搜索模式
void Gimbal_track_armor(void);//自动追踪模式控制
void Gimbal_fire(void);//自动开火模式控制



//辅助函数
void ClearPatrolCout(void);//自动挡时间计数清零
void Flag_zero_init(void);//相关标志位初始化

#endif
