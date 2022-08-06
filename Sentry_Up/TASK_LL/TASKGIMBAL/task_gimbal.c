/*----------------------------------------------//
|       title:哨兵云台控制任务									 |
|				date: 2021.3                             |
|															CH.                |
//----------------------------------------------*/
#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "task_connect.h"
#include "driver_feedmotor.h"
#include "task_remote.h"
#include "driver_remote.h"
#include "math.h"
#include "BSPconfig.h"
#include "kalman_filter.h"
#include "BSPConfig.h"
//#include "task_communicate.h"
#include "task_feedmotor.h"
#include "ramp.h"
#include "task_vision.h"
#include "bsp_can.h"

//数学
#define PIE (3.1415926) 

//遥控器命令档位
#define REMOTE_COMMAND_VOID        (3)//空挡
#define REMOTE_COMMAND_CTRL        (2)//手动挡
#define REMOTE_COMMAND_AUTO        (1)//自动挡
//遥控器灵敏度
#define YAW_REMOTE_SENSITIVE 			(0.0005f)
#define PITCH_REMOTE_SENSITIVE 		(0.001f)
//遥控器通道相关设定
#define MID_CTRL              (1024.0)  //ch通道中间值
#define DIFFERENCE_CTRL       (1684.0 - 364.0)  //ch通道最值差

//编码器&陀螺仪切换
#define ENCODER_USE                (1)//编码器
#define GYRO_USE                   (0)//陀螺仪

//YAW巡逻
#define YAW_PATROL_SENSITIVE 				(0.0002f)//

//PITCH巡逻
#define PITCH_PATROL_PERIOD_MS 		(700)//周期
#define PITCH_PATROL_ANGLE 				(0.026f)//振幅（低点：0.512  高点：0.564）
#define PITCH_PATROL_CENTER				(0.538f)//中心点

//YAW巡逻0
#define YAW_PATROL_PERIOD_MS 		  (3000)//周期
#define YAW_PATROL_ANGLE 					(0.2f)//振幅（低点：0.66  高点：0.76）
#define YAW_PATROL_CENTER					(YAW_INIT_VALUE)//中心点

//巡逻模式2之YAW轴跳跃位置
#define PITCH_PATROL_2_MS					(500)
#define YAW_PATROL_STEP						(0.15f)
#define YAW_STAY_TIME_MS					(600)

//装甲受击打模式切换
#define FIND_ENEMY_TIME_MS				(3000)
//#define YAW_TURN_SENSITIVE				(0.00005f)

//自动打击参数接口
#define MAX_SEQUENCE_LENGTH				(30)
#define VISION_DATA_DELAY_MS			(20)

#define SPEED_THRESHOLD 					(10.0f)
#define	SPEED_CALC_SENSITIVE 			(1000)

GimbalCtrlStruct GimbalCtrl = {	.GimbalMode = GIMBAL_RELAX , 
																.GimbalSetLocationDataTemp={PITCH_INIT_VALUE,YAW_INIT_VALUE,1,0} , 
																.YawPartolDirection = 1};
															//控制器	
															
//-------------卡尔曼滤波相关变量设定-------------------//																
kalman_filter_init_t yaw_kalman_filter_para = {
	.xhat_data = {0, 0},
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {20, 0, 0, 2000}
};

kalman_filter_init_t pitch_kalman_filter_para = 
{
	.xhat_data = {PITCH_INIT_VALUE, 0},
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {100, 0, 0, 2000}// 5000越小越贴近测量值
};

kalman_filter_t yaw_kalman_filter={0};
kalman_filter_t pitch_kalman_filter={0};
																
															
//--------------Significant整体变量集合-----------------//

u8 remote_command;//遥控器命令指令
extern int close_flag;//遥控器关控布尔符
extern RemoteDataPortStruct	RemoteDataPort;//遥控器通道命令
u8 Initial_flag;//初次开控识别布尔符
extern RemoteDataUnion RemoteData; //遥控器命令
extern VisionDataStruct VisionData; //视觉数据

int PitchPatrolCount=0;//自动挡时间计数_pitch
int YawPatrolCount=0;//自动挡时间计时_yaw
int YawPointArmourCount=0;//自动挡前后装甲时间计数_yaw
float watch_yaw_pat;
Judgement_data judgement_data; //裁判系统接收数据

float Yaw_initial_get;//yaw第一时间获取位置值
float Pitch_initial_get;//pitch第一时间获取位置值

float Yaw_init_center = 0.25;//yaw摆正位置
float Yaw_Base_Location;
int Yaw_init_center_flag = 1;//yaw摆正位置获取标识符

int Gimbal_flag = 1;//初始化字符

u8 Can_down_tofire[8]={0,0,0,0,0,0,0,0};//集火传递
//-------------------------------------*********云台逻辑控制工作********-------------------------------------//
void GimbalControlTask()	
{	
	
	//卡尔曼滤波参数初始化
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	
	//模式命令
	//remote_command = RemoteData.RemoteDataProcessed.RCValue.s2;
	//Gimbal_mode = GetGimbalMode();//
	//GimbalCtrl.GimbalMode = GetGimbalMode();
	watch_yaw_pat = GetYawLocationRaw();
	//Gimbal
	GimbalUseEncoder(ENCODER_USE,ENCODER_USE);//编码器or陀螺仪位置环取值设定(yaw,pitch)
	
	//FeedMotor
	FeedMotorDataUpdate();
	MotorLocationControlLogic();

	//if(GetGimbalError()<0.2f)
	//{
		if(GimbalCtrl.GimbalMode == GIMBAL_RELAX)	
			Gimbal_relax();                                      //停止运行模式控制
		if(GimbalCtrl.GimbalMode == GIMBAL_REMOTE_CTRL)	
			Gimbal_remote_ctrl();  															 //遥控器模式控制
		if(GimbalCtrl.GimbalMode == GIMBAL_PATROL) 
			{ Gimbal_patrol();                                     
		    Can_down_tofire[0] = 0;}  //巡逻模式
		if(GimbalCtrl.GimbalMode == GIMBAL_SEARCH_FRONT) 
			Gimbal_search_front(); 															 //前(左)侧巡逻
		if(GimbalCtrl.GimbalMode == GIMBAL_SEARCH_BACK) 
			Gimbal_search_behind(); 														 //后(右)侧巡逻
		if(GimbalCtrl.GimbalMode == GIMBAL_TRACK_ARMOR)	
			{	Gimbal_track_armor();  
				Can_down_tofire[0] = 0;}                           //自动追踪模式控制
		if(GimbalCtrl.GimbalMode == GIMBAL_FIRE) 
			{	Gimbal_fire();    
				Can_down_tofire[0] = 1;}		//自动开火模式控制
	  ClearPatrolCout();
		GimbalDriverTask();
	//}
		if(	Can_down_tofire[0] == 0) for(int i=1;i<8;i++) Can_down_tofire[i] = 0;
		
		//CAN2_Send_Msg(Can_down_tofire,8,0x0255);//集火发送
				
		if(Gimbal_flag) Gimbal_flag = 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>云台驱动相关函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//--------------------------------------------------云台底层驱动---------------------------------------------//
void GimbalDriverTask(void)
{
	GimbalDataInput(GimbalCtrl.GimbalSetLocationDataTemp);	//云台位置速度赋值，设定值更新
	GimbalSpeedDataUpdate();									//云台位置速度读取更新
	GimbalControlCalculateAndSend();					//云台位置环计算
}

//----------------------------------------------------yaw限位----------------------------------------------//
void YawLocationLimit(void)		//无滑环YAW轴上下限限制
{
	if(GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation > Yaw_Base_Location+0.42f)
		GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation = Yaw_Base_Location+0.42f;
	else if(GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation < Yaw_Base_Location-0.42f)
		GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation = Yaw_Base_Location-0.42f;
}

//----------------------------------------------------pitch限位----------------------------------------------//
void PitchLocationLimit(void)		//PITCH轴上下限限制
{
	if(GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation > PITCH_MAX_VALUE)
		GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_MAX_VALUE;
	else if(GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation < PITCH_MIN_VALUE)
		GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_MIN_VALUE;
}

//----------------------------------------------------yaw控制赋值--------------------------------------------//
void YawSetLocationValueChange(float Yaw)
{
	GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation	=	Yaw;
	YawLocationLimit();
}

//---------------------------------------------------pitch控制赋值-------------------------------------------//
void PitchSetLocationValueChange(float Pitch)
{
	GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation	=	Pitch;	
	PitchLocationLimit();
}
//------------------------------------------yaw轴编码器（非陀螺仪）使用模式判定------------------------------//
void GimbalUseEncoder(u8 isUseEncoder_Yaw,u8 isUseEncoder_Pitch)
{
	GimbalCtrl.GimbalSetLocationDataTemp.FlagYawUseEncoder	=	isUseEncoder_Yaw;
	GimbalCtrl.GimbalSetLocationDataTemp.FlagPitchUseEncoder = isUseEncoder_Pitch;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>云台模式相关函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//-------------------------------------------------模式切换----------------------------------------------------//
/*模式逻辑：遥控器左拨杆：下：空挡；
            遥控器左拨杆：中：手动挡；
						遥控器左拨杆：上：自动挡：
						{
							视觉命令：1：巡逻；
							视觉命令：2：追踪；
							视觉命令：3：开火；
						}	   
---------------------------------------*/
//extern u8 command_receive;//视觉命令状态
u8 comm_rec = 1;//脱机试验命令状态符
GimbalModeEnum GetGimbalMode(void)  //返回模式
{
	return GimbalCtrl.GimbalMode;
}
void SetGimbalMode(GimbalModeEnum Mode) //模式赋值
{
		GimbalCtrl.GimbalMode = Mode;
}
	
//--------------------------------------------停运模式控制---------------------------------------------//
int ctrl_initial_flag;// 手动挡初始化斜坡标识
int patrol_initial_flag;//自动挡初始化斜坡标识
ramp_t ctrl_initial_ramp = RAMP_GEN_DAFAULT;// 手动挡初始化斜坡
ramp_t patrol_initial_ramp = RAMP_GEN_DAFAULT;//自动挡初始化斜坡
void Gimbal_relax(void)
{
	close_flag = 0;
	
	Flag_zero_init();
	Yaw_initial_get = GetYawLocation();

	if (Yaw_init_center>0.5f)
		Yaw_Base_Location=Yaw_initial_get>Yaw_init_center-0.5f ?  Yaw_init_center:Yaw_init_center-1;
	else 
		Yaw_Base_Location=Yaw_initial_get>Yaw_init_center+0.5f ?  Yaw_init_center+1:Yaw_init_center;
	YawSetLocationValueChange(Yaw_Base_Location);
//	if (Yaw_init_center_flag )
//	{
//		Yaw_init_center = GetYawLocation();
//		Yaw_init_center_flag = 0;
//	}
}

//-------------------------------------------遥控器模式控制--------------------------------------------//
float remote_yaw,remote_pitch;  //遥控器相对位置
extern int YawEncoderCount;
void Gimbal_remote_ctrl(void)
{
	close_flag = 1;
	if (Initial_flag)
	{
		//GimbalDataInit();
		Yaw_initial_get = GetYawLocation();
		Pitch_initial_get = GetPitchLocation();
		
		Initial_flag = 0;
	}
	if(ctrl_initial_flag)
	{
		if(ctrl_initial_ramp.calc(&ctrl_initial_ramp) < 1)
		{
			//YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (YAW_INIT_VALUE + YawEncoderCount  - Yaw_initial_get));//滑环
			YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));//无滑环
			PitchSetLocationValueChange(Pitch_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (PITCH_INIT_VALUE - Pitch_initial_get));
		}
		else
		{
			//YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (YAW_INIT_VALUE + YawEncoderCount - Yaw_initial_get));//滑环
			YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));//无滑环
			PitchSetLocationValueChange(Pitch_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (PITCH_INIT_VALUE - Pitch_initial_get));
			ctrl_initial_flag = 0;
		}
	}
	else
	{
		remote_pitch = PITCH_REMOTE_SENSITIVE * RemoteDataPort.PitchIncrement;
		remote_yaw = YAW_REMOTE_SENSITIVE 	* RemoteDataPort.YawIncrement;
		PitchSetLocationValueChange(getPitchSetLocation()-remote_pitch);
		YawSetLocationValueChange(getYawSetLocation()+remote_yaw);
		
	}
}
//----------------------------------------------巡逻模式-----------------------------------------------//
float patrol_pitch,patrol_yaw;
ramp_t patrol_armour_ramp = RAMP_GEN_DAFAULT;
void Gimbal_patrol(void)
{
	close_flag = 1; 
	#if 1
	if (Initial_flag)
	{
		//PitchDataInit();
		
		Yaw_initial_get = GetYawLocation();
		if (Yaw_init_center>0.5f)
			Yaw_Base_Location=Yaw_initial_get>Yaw_init_center-0.5f ?  Yaw_init_center:Yaw_init_center-1;
		else 
			Yaw_Base_Location=Yaw_initial_get>Yaw_init_center+0.5f ?  Yaw_init_center+1:Yaw_init_center;
		
		//YawSetLocationValueChange(Yaw_initial_get);
		patrol_initial_flag = 1;
   	patrol_initial_ramp.init(&patrol_initial_ramp,1200);
		Pitch_initial_get = GetPitchLocation();
		if(Yaw_initial_get!=0)
			Initial_flag = 0;
	}
	if(patrol_initial_flag)
	{
		if(patrol_initial_ramp.calc(&patrol_initial_ramp) < 1)
		{
			YawSetLocationValueChange(Yaw_initial_get + patrol_initial_ramp.calc(&patrol_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));
			//YawSetLocationValueChange(Yaw_initial_get);
			PitchSetLocationValueChange(Pitch_initial_get + patrol_initial_ramp.calc(&patrol_initial_ramp) * (PITCH_PATROL_CENTER - Pitch_initial_get));
		}
		else
		{
			YawSetLocationValueChange(Yaw_initial_get + patrol_initial_ramp.calc(&patrol_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));
	   	//YawSetLocationValueChange(Yaw_initial_get);
			PitchSetLocationValueChange(Pitch_initial_get + patrol_initial_ramp.calc(&patrol_initial_ramp) * (PITCH_PATROL_CENTER - Pitch_initial_get));
			patrol_initial_flag = 0;
		}
	}
	else
	{
	#endif
		PitchPatrolCount++;
		patrol_pitch = PITCH_PATROL_CENTER + PITCH_PATROL_ANGLE*sin( 2*PIE/PITCH_PATROL_PERIOD_MS * PitchPatrolCount );
		PitchSetLocationValueChange(patrol_pitch);
		#if 0  //yaw轴匀速圆周巡逻 
		patrol_yaw = getYawSetLocation() + YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 0  //yaw轴前侧慢巡
		if (GetYawLocationRaw() >= 0.39f  && GetYawLocationRaw() <= 0.89f) //前侧装甲范围内
			patrol_yaw = getYawSetLocation() + YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		else patrol_yaw = getYawSetLocation() + 1.8*YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 0  //yaw轴前侧巡
		YawPatrolCount++;
		patrol_yaw = Yaw_init_center + YAW_PATROL_ANGLE*sin( 2*PIE/YAW_PATROL_PERIOD_MS * YawPatrolCount );
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 1 //yaw无滑环装甲前巡
		float sensitiveyaw;
		if(( GetYawLocationRaw() <= 0.59f && GetYawLocationRaw() >=0.00f)
				||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.91f))   //前侧装甲范围内  
		{
				sensitiveyaw = YAW_PATROL_SENSITIVE;
				patrol_armour_ramp.init(&patrol_armour_ramp,500);
				YawPointArmourCount++;
				if( YawPointArmourCount >= 5000 )
					YawPointArmourCount = 1;//非归零重置
		}					
		else if( (GetYawLocationRaw() >= 0.75f && GetYawLocationRaw() <= 0.91f)) //右后小侧
		{
				GimbalCtrl.YawPartolDirection = 1;
				if( YawPointArmourCount > 0 )							
					sensitiveyaw = 2*YAW_PATROL_SENSITIVE * patrol_armour_ramp.calc(&patrol_armour_ramp) - YAW_PATROL_SENSITIVE;//斜坡减加速							
				else
					sensitiveyaw = YAW_PATROL_SENSITIVE;
		}
		else if((GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >= 0.59f))					//左后小侧
		{
				GimbalCtrl.YawPartolDirection = -1;
				if( YawPointArmourCount > 0 )							
					sensitiveyaw = 2*YAW_PATROL_SENSITIVE * patrol_armour_ramp.calc(&patrol_armour_ramp) - YAW_PATROL_SENSITIVE;//斜坡减加速							
				else
					sensitiveyaw = YAW_PATROL_SENSITIVE;
		}
		YawSetLocationValueChange(getYawSetLocation() + sensitiveyaw * GimbalCtrl.YawPartolDirection);

		#endif
	}
	
}
//---------------------------------------------前(左)搜索模式-----------------------------------------------//
int YawPointArmourCount_FL;
ramp_t search_armour_ramp = RAMP_GEN_DAFAULT;
void Gimbal_search_front(void)
{
		float sensitiveyaw;
	//if(GetGimbalError()<0.2f)
		//	{	
					PitchPatrolCount ++;
					GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_PATROL_CENTER + PITCH_PATROL_ANGLE*sin( 2*PI/PITCH_PATROL_PERIOD_MS * PitchPatrolCount );
					if(( GetYawLocationRaw() <= 0.59f && GetYawLocationRaw() >=0.25f)) 						//前(左)侧范围内  
					{
						  sensitiveyaw = YAW_PATROL_SENSITIVE;
							search_armour_ramp.init(&search_armour_ramp,1000);
							YawPointArmourCount_FL++;
							if( YawPointArmourCount_FL >= 5000 )
							{
								//YawPointArmourCount_FL = 0;//非归零重置
								SetGimbalMode(GIMBAL_PATROL);
							}
					}					
					else if( (GetYawLocationRaw() <= 0.25f && GetYawLocationRaw() >= 0.0f)
					||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.75f)) //左-右
					{
							GimbalCtrl.YawPartolDirection = 1;
							if( YawPointArmourCount_FL > 0 )							
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;//斜坡减加速							
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
					else if( (GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >= 0.59f)) //左-左
					{
							GimbalCtrl.YawPartolDirection = -1;
							if( YawPointArmourCount_FL > 0 )							
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;//斜坡减加速							
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
			    YawSetLocationValueChange(getYawSetLocation() + sensitiveyaw * GimbalCtrl.YawPartolDirection);
			//}

}
//---------------------------------------------后(右)搜索模式-----------------------------------------------//
int YawPointArmourCount_BR;
void Gimbal_search_behind(void)
{
	float sensitiveyaw;
	//if(GetGimbalError()<0.2f)
		//	{	
					PitchPatrolCount ++;
					GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_PATROL_CENTER + PITCH_PATROL_ANGLE*sin( 2*PI/PITCH_PATROL_PERIOD_MS * PitchPatrolCount );
				 
					if( (GetYawLocationRaw() <= 0.25f && GetYawLocationRaw() >= 0.0f)
					||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.91f))	//后(右)侧范围内
					{
							search_armour_ramp.init(&search_armour_ramp,100);
							sensitiveyaw = YAW_PATROL_SENSITIVE;
							YawPointArmourCount_BR++;
//							if( YawPointArmourCount > FIND_ENEMY_TIME_MS )
//								SetGimbalMode(GIMBAL_PATROL);
							if( YawPointArmourCount_BR >= 5000 )
							{
								//YawPointArmourCount_BR = 1;//非归零重置
								SetGimbalMode(GIMBAL_PATROL);
							}
					}					
					else if( (GetYawLocationRaw() <= 0.91f && GetYawLocationRaw() >= 0.75f))     //右—右
					{
							GimbalCtrl.YawPartolDirection = 1;
							if( YawPointArmourCount_BR > 0 )
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
					else if(( GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >=0.25f))     //右-左
					{
							GimbalCtrl.YawPartolDirection = -1;
							if( YawPointArmourCount_BR > 0 )
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
				YawSetLocationValueChange(getYawSetLocation() + sensitiveyaw * GimbalCtrl.YawPartolDirection);
				
			//}
}

//------------------------------------------自动追踪模式控制-------------------------------------------//
float vision_pitch,vision_pitch_last1,vision_pitch_last2;
float vision_yaw,vision_yaw_last1,vision_yaw_last2;
//float PITCH_SIGHT_AREA=0.00005,YAW_SIGHT_AREA=-0.00003;
float PITCH_PIAN_fdistance = 0  ;//pitch 距离相关强行偏置
float YAW_PIAN_fdistance;//yaw 距离相关强行偏置
int DRAW_X,DRAW_Y;
float Command_X,Command_Y;//用于命令的XY相对值



float k1 = 0.1, k2 = 0.2, k3 = 0.7;
//float PITCH_SIGHT_AREA,YAW_SIGHT_AREA;

void Gimbal_track_armor(void)
{
	close_flag = 1;
	Flag_zero_init();
	vision_pitch = VisionData.Change_pitch;// * PITCH_SIGHT_AREA ;
	vision_yaw = VisionData.Change_yaw;// * YAW_SIGHT_AREA ;
	
	PitchSetLocationValueChange(getPitchSetLocation() + vision_pitch);
	YawSetLocationValueChange(getYawSetLocation()+vision_yaw );
 
	#if 1
		DRAW_Y = (int)((getPitchSetLocation() + vision_pitch  - GetPitchLocation())*8191.0f);
		DRAW_X = (int)((getYawSetLocation() + vision_yaw  - GetYawLocation())*8191.0f);
	#endif
	
	Command_X = (float)((getYawSetLocation() + vision_yaw  - GetYawLocation())*100.0f);
  Command_Y	= (float)((getPitchSetLocation() + vision_pitch  - GetPitchLocation())*100.0f);
	
}
//------------------------------------------自动开火模式控制-------------------------------------------//
short Yaw_delivery;//传递归一化值乘1000
short Pitch_delivery;//传递角度值乘10
short Dis_delivery;//传递距离
float U_zero_deg =  0.662;//0.662 pitch零点值

void Gimbal_fire(void)
{
	close_flag = 1;
	Flag_zero_init();
	patrol_initial_flag = 0;Initial_flag = 0;
	Gimbal_track_armor();
	
	Yaw_delivery = (short)(GetYawLocation()*1000.0f);
	Pitch_delivery = (short)((-GetPitchLocation()+U_zero_deg)*360.0*10.0f); 
	Dis_delivery = (short)(VisionData.ReceiveDistanceRaw);
	
	Can_down_tofire[1] = (u8)((Yaw_delivery>>8)&0x00ff);
	Can_down_tofire[2] = (u8)(Yaw_delivery&0x00ff);
	Can_down_tofire[3] = (u8)((Pitch_delivery>>8)&0x00ff);
	Can_down_tofire[4] = (u8)(Pitch_delivery&0x00ff);
	Can_down_tofire[5] = (u8)((Dis_delivery>>8)&0x00ff);
	Can_down_tofire[6] = (u8)(Dis_delivery&0x00ff);
	
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>辅助相关函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//-----------------------------------------自动挡时间计数清零------------------------------------------//
void ClearPatrolCout(void)
{
	if(GimbalCtrl.GimbalMode != GIMBAL_PATROL && GimbalCtrl.GimbalMode != GIMBAL_SEARCH_FRONT 
																						&& GimbalCtrl.GimbalMode != GIMBAL_SEARCH_BACK )
	{
		YawPatrolCount = 0;
		PitchPatrolCount = 0;
	}
	if(GimbalCtrl.GimbalMode != GIMBAL_PATROL && GimbalCtrl.GimbalMode != GIMBAL_SEARCH_FRONT 
																						&& GimbalCtrl.GimbalMode != GIMBAL_SEARCH_BACK)
		YawPointArmourCount=0;
}
//-----------------------------------------相关标志位初始化------------------------------------------//
void Flag_zero_init()
{
	Initial_flag = 1;
	ctrl_initial_flag = 1;
	patrol_initial_flag = 1;
	ctrl_initial_ramp.init(&ctrl_initial_ramp,2000);
	patrol_initial_ramp.init(&patrol_initial_ramp,2000);
}

