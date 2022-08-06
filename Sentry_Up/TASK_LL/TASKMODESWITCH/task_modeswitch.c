#include "task_modeswitch.h"
#include "task_gimbal.h"
#include "task_chassis.h"
#include "task_feedmotor.h"
#include "task_remote.h"
#include "driver_remote.h"
#include "driver_friction.h"
#include "task_communicate.h"
#include "task_vision.h"
#include "BSPConfig.h"
#include "stdbool.h"
#include "task_led.h"
#include "math.h"
#include "task_friction.h"
#include "task_lostcounter.h"

//裁判系统比赛/调试运用模式
#define JUDGEMENTGAME  (0)//比赛1 调试0


//远距离目标：5m以上
#define FAR_TARGET_DISTANCE		(5000)

//子弹决策档位
#define BULLET_FULLNUM         (200)//剩余充足判断标准
#define BULLET_FULL            (2)//剩余充足
#define BULLET_LOW             (1)//剩余不够
#define BULLET_EMPTY           (0)//没有子弹

//前哨站血量决策档位  总血量2000
#define QIANSHAO_RISKNUM          (1200)//安全临界血量
#define QIANSHAO_SAVE              (1)//安全
#define QIANSHAO_RISK              (0)//危险

//遥控器命令档位
#define REMOTE_COMMAND_VOID        (3)//空挡
#define REMOTE_COMMAND_CTRL        (2)//手动挡
#define REMOTE_COMMAND_AUTO        (1)//自动挡

//模块离线判断
#define MODE_SWITCH_TIME_MS (5)

//视觉命令
#define VISION_PATROL        (3)//巡逻
#define VISION_TRACK         (1)//跟踪
#define VISION_FIRE          (2)//打击


//--------------Significant整体变量集合-----------------//

extern LostCounterStruct lost_counter;
extern RemoteDataUnion RemoteData; //遥控器命令
extern RemoteDataPortStruct	RemoteDataPort;//遥控器通道命令

extern VisionDataStruct VisionData; //视觉数据
extern Judgement_data judgement_data; //裁判系统接收数据
extern u8 Chassis_rebuf[8];//底盘数据接收；



ArmourHurtStruct IsArmourHurt;//前后装甲受伤判断

u8 Remote_command;//遥控器主命令
u8 Sentry_comdata;//车间通信指令

int Bool_isgame,//比赛进程判断
		Bool_bulletnum,//子弹剩余数判断
	  Bool_qianshaozhan;//前哨站血量判断

extern u8 Can2ChassisSendMessage[8];

//-------------------------------------*********总体逻辑控制工作********-------------------------------------//
void General_Control(void)
{
	Remote_command = RemoteData.RemoteDataProcessed.RCValue.s2;
	Sentry_comdata = Chassis_rebuf[2];
	Bool_isgame = IsGameStart();              //比赛进程判断
	Bool_bulletnum = BulletNum_Full();        //子弹剩余数判断
	Bool_qianshaozhan = Qianshaozhan_Save();  //前哨站血量判断
	Armour_bool();
	
	//空挡
	if(Remote_command == REMOTE_COMMAND_VOID) 
	{
		SetGimbalMode(GIMBAL_RELAX);
		SetFeedMotorMode(FEEDMOTOR_INIT);
		
		Can2ChassisSendMessage[0] = 0x01;
	}
	
	//手动挡
	else if(Remote_command == REMOTE_COMMAND_CTRL) 
	{
		SetGimbalMode(GIMBAL_REMOTE_CTRL);
		SetFeedMotorMode(FEEDMOTOR_REMOTE_CTRL);
		
		if (RemoteData.RemoteDataProcessed.RCValue.Ch3 >=1100)
		{
			Can2ChassisSendMessage[0] = 0x00;
		}
		else
		{
			Can2ChassisSendMessage[0] = 0x01;
		}
	}
	
	//自动挡
	else if(Remote_command == REMOTE_COMMAND_AUTO)
	{
		Can2ChassisSendMessage[0] = 0x00;
		//不在比赛离线或正在比赛
		if ((judgement_data.game_mode == 0)||(judgement_data.game_mode == 4))
		{					
			if((VisionData.Command == VISION_PATROL)
				||(VisionData.Command == 0))
			{
//          switch(Sentry_comdata)
//					{
//						case 0://常态巡逻	
							SetGimbalMode(GIMBAL_PATROL);
//						break;
//						case 1://左侧巡逻
//							SetGimbalMode(GIMBAL_SEARCH_FRONT);
//						break;
//						case 2://右侧巡逻
//							SetGimbalMode(GIMBAL_SEARCH_BACK);
//						break;
//						default:
//							SetGimbalMode(GIMBAL_PATROL);
//						break;	
//					}
				SetFeedMotorMode(FEEDMOTOR_INIT);
			}
			if(VisionData.Command == VISION_TRACK)
			{
				SetGimbalMode(GIMBAL_TRACK_ARMOR);
				if(Getfriction_Full()==1)
					SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
				else SetFeedMotorMode(FEEDMOTOR_INIT);
			}
			if(VisionData.Command == VISION_FIRE)
			{
				SetGimbalMode(GIMBAL_FIRE);
				#if (JUDGEMENTGAME == 1)
				AutoShootControl();    //自动射击控制
				#endif
				
				#if (JUDGEMENTGAME == 0)
				if(Getfriction_Full()==1)
					SetFeedMotorMode(FEEDMOTOR_HIGH_FREQ);
				else SetFeedMotorMode(FEEDMOTOR_INIT);
				#endif
			}
		}

		//场前三分钟
		if (judgement_data.game_mode == 1)
		{
			if(VisionData.Command == VISION_PATROL)
			{
				SetGimbalMode(GIMBAL_PATROL);
			  SetFeedMotorMode(FEEDMOTOR_INIT);
	  	}
			if((VisionData.Command == VISION_TRACK)||(VisionData.Command == VISION_FIRE))
			{
				SetGimbalMode(GIMBAL_TRACK_ARMOR);
				SetFeedMotorMode(FEEDMOTOR_INIT);
			}
		}
	}
}


//-------------------------------------------比赛进程判断----------------------------------------------
//1，比赛中
//0，非比赛中
int IsGameStart(void)							
{
	if(judgement_data.game_mode == 0)  //丢失裁判系统信息，认为在比赛中（断联模拟）
	{
			return 1;
	}
	else
	{
	if( judgement_data.game_mode == 4)			
			return 1;
		else
			return 0;
	}
}

//-------------------------------------------装甲打击判断-----------------------------------------------
void Armour_bool(void)
{
	if(judgement_data.game_mode == 0)  //丢失裁判系统信息，认为在比赛中（断联模拟）
	{
			IsArmourHurt.Front = 0;
		  IsArmourHurt.Back = 0;
	}
	else
	{
		if (judgement_data.armour_hurt == 0x00)
		{
			IsArmourHurt.Front = 0;
		  IsArmourHurt.Back = 0;
		}
		if (judgement_data.armour_hurt == 0x10)
		{
			IsArmourHurt.Front = 1;
		  IsArmourHurt.Back = 0;
		}
		if (judgement_data.armour_hurt == 0x01)
		{
			IsArmourHurt.Front = 0;
		  IsArmourHurt.Back = 1;
		}
	}
}


//-------------------------------------------决策参数判断-----------------------------------------------
int BulletNum_Full(void) //子弹标准 剩余200
{
	if (judgement_data.game_mode == 0)  //丢失裁判系统信息，认为在比赛中（断联模拟）
	{
	    return BULLET_LOW;
  }
	else 
	{
		if (judgement_data.bulletnum >= BULLET_FULLNUM)
			return BULLET_FULL;
	  if ((judgement_data.bulletnum < BULLET_FULLNUM) &&( judgement_data.bulletnum != 0))
			return BULLET_LOW;
	  if (judgement_data.bulletnum == 0)
			return BULLET_EMPTY;
	}
}
int Qianshaozhan_Save(void)//前哨站血量标准 剩余1200
{
	if (judgement_data.game_mode == 0)  //丢失裁判系统信息，认为在比赛中（断联模拟）
	{
	    return QIANSHAO_RISK;
  }
	else 
	{
		if (judgement_data.qianshaozhan_blood >= QIANSHAO_RISKNUM)
			return QIANSHAO_SAVE;
	  if (judgement_data.qianshaozhan_blood < QIANSHAO_RISKNUM)
			return QIANSHAO_RISK;
	}
}

//-------------------------------------------自动发射控制------------------------------------------------
/*
         \子弹数      充足>200         不够<200 
前哨站血量

                    远  低频           远  不打
	血多       
                    近  低频           近  低频

										远  不打           远  不打
血少/死亡     
                    近  高频           近  低频

*/
void AutoShootControl(void)
{
	if (Bool_isgame == 1)
	{
		//子弹充足 前哨站安全
		if ((Bool_bulletnum == BULLET_FULL)&&(Bool_qianshaozhan == QIANSHAO_SAVE))
		{
			SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
		}
		//子弹不足 前哨站安全
		if ((Bool_bulletnum == BULLET_LOW)&&(Bool_qianshaozhan == QIANSHAO_SAVE))
		{
			if(VisionData.ReceiveDistanceRaw < FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
			}
			else if(VisionData.ReceiveDistanceRaw >= FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_INIT);
			}
		}
		//子弹充足 前哨站危险
		if ((Bool_bulletnum == BULLET_FULL)&&(Bool_qianshaozhan == QIANSHAO_RISK))
		{
			if(VisionData.ReceiveDistanceRaw < FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_HIGH_FREQ);
			}
			else if(VisionData.ReceiveDistanceRaw >= FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_INIT);
			}
		}
		//子弹不足 前哨站危险
		if ((Bool_bulletnum == BULLET_LOW)&&(Bool_qianshaozhan == QIANSHAO_RISK))
		{
			if(VisionData.ReceiveDistanceRaw < FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
			}
			else if(VisionData.ReceiveDistanceRaw >= FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_INIT);
			}
		}
		//没子弹
		if (Bool_bulletnum == BULLET_EMPTY)
		{
			SetFeedMotorMode(FEEDMOTOR_INIT);
		}
	}
	else                       //丢失裁判系统信息，认为在比赛中，始终发射
	{
		if(VisionData.ReceiveDistanceRaw < FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
			}
			else if(VisionData.ReceiveDistanceRaw >= FAR_TARGET_DISTANCE && VisionData.ReceiveDistanceRaw != 0)
			{
					SetFeedMotorMode(FEEDMOTOR_HIGH_FREQ);
			}
	}
}

