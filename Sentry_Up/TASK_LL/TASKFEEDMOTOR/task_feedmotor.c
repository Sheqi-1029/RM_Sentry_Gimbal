#include "driver_feedmotor.h"
#include "task_feedmotor.h"
#include "math.h"
#include "task_remote.h"
#include "driver_remote.h"
#include "task_vision.h"
#include "driver_vision.h"
#include "task_gimbal.h"
#include "driver_gimbal.h"
#include "task_communicate.h"

//同云台播弹形式
#define DOUBLEFEED   (0) //1-两拨弹；0-无两拨弹


//裁判系统热量比赛运用模式
#define JUDGEMENTGAME  (1)

//热量控制参数
#define HEAT_UPPER_LIMIT  (320.0f)//热量极限
#define COOLlNG_RATIO   	(100.0f)//每秒冷却
#define SHOOT_SPEED     	(30.0f)//发射速度
#define SPARE_HEAT        (20.0f)//多余热量
#define HEAT_PER_SHOOT    (10.0f)//单发加热量

//手动挡拨弹档位
#define LOADED_SHOOT      (2)//上膛
#define BURSTS_SHOOT      (3)//点射
#define DARTLE_SHOOT      (1)//连发


//拨弹参数
#define ENCODER_LINE (36.0f)//减速比
#define SHOOT_NUMBER (10)//发射拨弹拨盘齿数
#define SHOOT_ONCE_LOCATION ( -ENCODER_LINE/SHOOT_NUMBER*1.0f )//一次拨弹位移

#define UP_NUMBER (8)//上拨弹拨盘齿数
#define UP_ONCE_LOCATION ( ENCODER_LINE/UP_NUMBER*1.0f )//一次拨弹位移

#define CTRL_SetDartlePerNum   (5.0f)//手动挡测试发射频率
#define CTRL_SetDartleTime     (1/(CTRL_SetDartlePerNum*0.004))//手动挡测试发射周期 0.004为任务周期

#define Count_SetDartleTime    (CTRL_SetDartleTime) //数量模式拨弹周期

//机械延时初始化位置
#define FEED_INIT_BEST_LOCA  (0.00f)

//堵转回转拨盘格数
#define MOTOR_RETURN_RATE (0.5f)

//视觉相关
#define FAR_TARGET_DISTANCE   (1000)

float SetLocation;

//--------------Significant整体变量集合-----------------//
extern RemoteDataUnion RemoteData; //遥控器命令
extern u8 Initial_flag;//初次开控识别布尔符
int feedmotor_init_flag = 1;//初始化机械对位标识符1
int feedmotor_init_up_flag = 1;//初始化机械对位标识符2
int Flag_shoot = 0;          //射击标识符
int remote_shoot;        //遥控器发射命令

short Shoot_Heat;     //读取射击热量
int  Shoot_test_Heat;  //自控射击热量
int Heat_count;//热量计时

int 		TimeCount=0;								//用来产生0.1s时段供堵转使用
int 		TimeCountUP=0;								//用来产生0.1s时段供堵转使用
int     TimeDartle=0;               //用来产生时段供连发使用
int     TimeDartleUP=0;               //用来产生时段供上连发使用
float 	Current[10]={0,0,0,0,0,0,0,0,0,0};
float   CurrentUP[10]={0,0,0,0,0,0,0,0,0,0};
float  CurrentMean;
float  CurrentMeanUP;

FeedMotorCtrlStruct FeedMotorCtrl;//拨弹电机控制器
FeedMotorCtrlStruct FeedMotor_UpCtrl;//上拨弹电机控制器

extern VisionDataStruct   VisionData;
extern receive_judge_t judge_rece_mesg; //原直接接收裁判系统数据
extern Judgement_data judgement_data; //裁判系统接收数据

int count_bullet_flag = 0;//数量模式开启标志位
int num_count = 0;//发射个数统计

float bullet_speed[10] = {23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2};
float Bullet_speedAVR = 23.2;

//-------------------------------------*********拨弹逻辑控制工作********-------------------------------------//
//---------------------------------发射总控逻辑-----------------------------------------//
void FeedMotorControlLogic()
{	
	LockedMotorDetectionAndProcessed();
	#if  JUDGEMENTGAME
	JudgementGunEnergyUpdate();
	#endif
	FeedMotorCtrl.FeedMotorMode=GetFeedMotorMode();
	
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_INIT)
		FeedMotor_InitGame();														       //初始化机械对位(降低延时)
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL)
		FeedMotor_remote_ctrl();													  	 //遥控发射模式
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_FILL_UP)
		FeedMotor_fill_up();                                   //赛前填满弹道模式
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ)
		FeedMotor_high_freq();                                 //高频自动挡射击
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		FeedMotor_low_freq();                                  //低频自动档射击
	
	
	//ClearTimeDartle();
	
}

#if DOUBLEFEED
//---------------------------------上拨弹总控逻辑-----------------------------------------//
int bullet_num_detec_flag = 1;//监测子弹数标志位
int bullet_num_detec;//临时子弹数
int bullet_max_num_temp = 500;//临时最高值

void FeedMotor_UpControlLogic() 
{	
	LockedUPMotorDetectionAndProcessed();
	
	FeedMotor_UpCtrl.FeedMotorMode=GetFeedMotorMode();
	
//	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ
//		||FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ)
//		{
//			if (bullet_num_detec_flag == 1)
//			{
//				bullet_num_detec = bullet_max_num_temp - judgement_data.bulletnum;
//				if (bullet_max_num_temp >= 50)
//				{
//					bullet_max_num_temp -= 50;
//					bullet_num_detec_flag = 0;
//					count_bullet_flag = 0;
//				}
//			}
//			else
//			{
//				FeedMotorCountShooting(50);
//				if (count_bullet_flag == 1)
//					bullet_num_detec_flag =1;
//			}
//		}			
	//ClearTimeDartle();
	
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_INIT)
		FeedMotor_UP_InitGame();														       //初始化机械对位(降低延时)
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL)
		FeedMotor_UP_remote_ctrl();													  	 //遥控发射模式
	
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ)
		FeedMotor_UP_high_freq();                                 //高频自动挡射击
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		FeedMotor_UP_low_freq();                                  //低频自动档射击
	
}
#endif

//----------------------------------------堵转检测算法--------------------------------------------//
void LockedMotorDetectionAndProcessed(void)		//发射堵转
{

	//更新电流
	for(int i=0;i<9;i++)
	{
			Current[i]=Current[i+1];
	}
	Current[9]=GetFeedMotorCurrent();
	
	//堵转检测
	if(FeedMotorCtrl.DeverseLocked==1)//发生堵转  1为堵转 0没有堵转
	{
			if(TimeCount<25)			//0.1s一个周期，虽然我也不知道为什么freertos里要4ms一次任务
			  TimeCount++; 
			else
			{
				TimeCount = 0;
				FeedMotorCtrl.DeverseLocked = 0;
			}
	}
	else//没有堵转
	{	
			CurrentMean=(fabs(Current[0])+fabs(Current[1])+fabs(Current[2])+fabs(Current[3])
									+fabs(Current[4])+fabs(Current[5])+fabs(Current[6])+fabs(Current[7])
									+fabs(Current[8])+fabs(Current[9]))/10.0f; 
			
			if( fabs(CurrentMean) > 9500.0f)
			{
				FeedMotorCtrl.SetLocation =  GetFeedMotorLocation() + MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//反转
				SetFeedMotorSetLocation(  FeedMotorCtrl.SetLocation );
				FeedMotorCtrl.DeverseLocked = 1;					
			}
			else
				FeedMotorCtrl.DeverseLocked = 0;
	}
	
}

#if DOUBLEFEED
void LockedUPMotorDetectionAndProcessed(void)		//上拨弹堵转
{

	//更新电流
	for(int i=0;i<9;i++)
	{
			CurrentUP[i]=CurrentUP[i+1];
	}
	CurrentUP[9]=GetFeedMotorUPCurrent();
	
	//堵转检测
	if(FeedMotor_UpCtrl.DeverseLocked==1)//发生堵转  1为堵转 0没有堵转
	{
			if(TimeCountUP<25)			//0.1s一个周期，虽然我也不知道为什么freertos里要4ms一次任务
			  TimeCountUP++; 
			else
			{
				TimeCountUP = 0;
				FeedMotor_UpCtrl.DeverseLocked = 0;
			}
	}
	else//没有堵转
	{	
			CurrentMeanUP=(fabs(CurrentUP[0])+fabs(CurrentUP[1])+fabs(CurrentUP[2])+fabs(CurrentUP[3])
									+fabs(CurrentUP[4])+fabs(CurrentUP[5])+fabs(CurrentUP[6])+fabs(CurrentUP[7])
									+fabs(CurrentUP[8])+fabs(CurrentUP[9]))/10.0f; 
			
			if( fabs(CurrentMeanUP) > 9000.0f)
			{
				FeedMotor_UpCtrl.SetLocation =  GetFeedMotorUPLocation() + MOTOR_RETURN_RATE * UP_ONCE_LOCATION;	//反转
				SetFeedMotorUPSetLocation(  FeedMotor_UpCtrl.SetLocation );
				FeedMotor_UpCtrl.DeverseLocked = 1;					
			}
			else
				FeedMotor_UpCtrl.DeverseLocked = 0;
	}
	
}
#endif

//----------------------------------------子弹发射------------------------------------------------//
void FeedMotorLocationUpdate(unsigned char ShootNumber)
{
		if(FeedMotorCtrl.DeverseLocked == 0)	//不堵转
		{
			#if  JUDGEMENTGAME
			if( GunEnergyJudge() )	//不超热量
			{
			#endif
				if( fabs(GetFeedMotorLocationError()) < fabs(SHOOT_ONCE_LOCATION) )	//误差控制
				{
						FeedMotorCtrl.SetLocation = GetFeedMotorSetLocation() - SHOOT_ONCE_LOCATION * ShootNumber;
						SetFeedMotorSetLocation( FeedMotorCtrl.SetLocation );
					  Shoot_test_Heat += HEAT_PER_SHOOT;
					
				}
			#if  JUDGEMENTGAME
			}
			#endif
		}
}

#if DOUBLEFEED
//-----------------------------------------子弹拨动-----------------------------------------------//
void FeedMotorUPLocationUpdate(unsigned char ShootNumber)
{
		if(FeedMotor_UpCtrl.DeverseLocked == 0)	//不堵转
		{
			
				if( fabs(GetFeedMotorUPLocationError()) < fabs(UP_ONCE_LOCATION) )	//误差控制
				{
						FeedMotor_UpCtrl.SetLocation = GetFeedMotorUPSetLocation() - UP_ONCE_LOCATION * ShootNumber;
						SetFeedMotorUPSetLocation( FeedMotor_UpCtrl.SetLocation );
				}
			
		}
}
#endif

//------------发射拨弹子弹连发频率模式控制-----------------//
void FeedMotorKeepShooting(int Freqtime)
{
	
	if (TimeDartle<Freqtime)
				TimeDartle++;
			else
			{
				TimeDartle = 0;
				FeedMotorLocationUpdate(1);
			}
}

#if DOUBLEFEED
//------------补弹拨弹子弹连发频率模式控制-----------------//
void FeedMotorUPKeepShooting(int Freqtime)
{
	
	if (TimeDartleUP<Freqtime)
				TimeDartleUP++;
			else
			{
				TimeDartleUP = 0;
				FeedMotorUPLocationUpdate(1);
			}
}
#endif


//------------子弹连发数量模式控制-----------------//
#if DOUBLEFEED
void FeedMotorCountShooting(int Num)
{
	if (~count_bullet_flag)
	{
		if(num_count < Num )
		{
			if (TimeDartleUP<Count_SetDartleTime)
		
						TimeDartleUP++;
					else
					{
						TimeDartleUP = 0;
						FeedMotorUPLocationUpdate(1);
						num_count ++;
					}
		}else if(num_count == Num) 
		{
			TimeDartleUP = 0;
			count_bullet_flag = 1;
		}
	}
}
#endif

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>热量计算相关函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
EnergyInfoStruct EnergyInfo ;
u8 ForceCloseFlag = 0;//发射热量标识符

void JudgementGunEnergyUpdate()
{
	
	if (judgement_data.game_mode == 0) //丢失裁判系统信息，认为在比赛中，始终发射
	{
		if (Heat_count < 25) 
		{
			Heat_count ++;
		}
		else
		{
			Heat_count = 0;
			if ((Shoot_test_Heat - COOLlNG_RATIO/10.0f) >= 0)
			{
				Shoot_test_Heat -= COOLlNG_RATIO/10.0f;
			}
		}
		EnergyInfo.GunEnergy = Shoot_test_Heat;
	}
	else
	{
	  Shoot_Heat = judgement_data.heat1;
		EnergyInfo.GunEnergy = Shoot_Heat;
	}
		//}
}

//1，允许发射
//0，不允许发射
int GunEnergyJudge()   
{
		if( HEAT_UPPER_LIMIT - EnergyInfo.GunEnergy >= SHOOT_SPEED + SPARE_HEAT)
			return 1;
		else
			return 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>拨弹模式相关函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//----------------------------------------------模式切换-----------------------------------------------//
FeedMotorModeEnum GetFeedMotorMode(void)
{
		return FeedMotorCtrl.FeedMotorMode;
}
void SetFeedMotorMode(FeedMotorModeEnum Mode) //模式赋值
{
		FeedMotorCtrl.FeedMotorMode = Mode;
}
//-------------------------------------初始化机械对位（降低延时）--------------------------------------//
float feedmotorloca_temp;
void FeedMotor_InitGame(void)
{
	float best_loca_err = 1.0;
	int best_loca_k = 0;

	if (feedmotor_init_flag)
	{
		feedmotorloca_temp = GetFeedMotorLocation();
		feedmotor_init_flag = 0;
	}
//	for (int i=0;i<SHOOT_NUMBER;i++)
//	{
//		float Loca_err = fabs(feedmotorloca_temp - (FEED_INIT_BEST_LOCA+SHOOT_ONCE_LOCATION*i));
//		if (Loca_err < best_loca_err)
//		{
//			best_loca_err = Loca_err;
//			best_loca_k = i;
//		}
//	}
//	FeedMotorCtrl.SetLocation = FEED_INIT_BEST_LOCA+SHOOT_ONCE_LOCATION*best_loca_k;
	FeedMotorCtrl.SetLocation = feedmotorloca_temp;
	SetFeedMotorSetLocation(FeedMotorCtrl.SetLocation);
}	

#if DOUBLEFEED
float feedmotorloca_up_temp;
void FeedMotor_UP_InitGame(void)
{
	if (feedmotor_init_up_flag)
	{
		feedmotorloca_up_temp = GetFeedMotorUPLocation();
		feedmotor_init_up_flag = 0;
	}
	FeedMotor_UpCtrl.SetLocation = feedmotorloca_up_temp;
	SetFeedMotorUPSetLocation(FeedMotor_UpCtrl.SetLocation);
}	
#endif

//-------------------------------------------遥控器模式控制--------------------------------------------//
void FeedMotor_remote_ctrl(void)
{
	feedmotor_init_flag = 1;
	remote_shoot = RemoteData.RemoteDataProcessed.RCValue.s1; 
	
	//----------------发射准备----------------//
	if(remote_shoot == LOADED_SHOOT) Flag_shoot = 1;
	//------------------点射------------------//				
	if(remote_shoot == BURSTS_SHOOT) 
	{
		if((Initial_flag == 0)&&(Flag_shoot == 1))
		{
			FeedMotorLocationUpdate(1);
		}
		Flag_shoot = 0;
	}
	//------------------连发------------------//
	if(remote_shoot == DARTLE_SHOOT)
	{
		Flag_shoot = 2;
		if(Initial_flag == 1) Flag_shoot = 0;
		if((Initial_flag == 0)&&(Flag_shoot == 2))
			FeedMotorKeepShooting(CTRL_SetDartleTime);
	}
}

#if DOUBLEFEED
int Flag_shoot_up = 0;
void FeedMotor_UP_remote_ctrl(void)
{
	feedmotor_init_up_flag = 1;
	int remote_shoot_up = RemoteData.RemoteDataProcessed.RCValue.s1;
	//----------------发射准备----------------//
	if(remote_shoot_up == LOADED_SHOOT) Flag_shoot_up = 1;
	//------------------点射------------------//				
	if(remote_shoot_up == BURSTS_SHOOT) 
	{
		if((Initial_flag == 0)&&(Flag_shoot_up == 1))
		{
			FeedMotorUPLocationUpdate(1);
		}
		Flag_shoot_up = 0;
	}
	//------------------连发------------------//
	if(remote_shoot_up == DARTLE_SHOOT)
	{
		Flag_shoot_up = 2;
		if(Initial_flag == 1) Flag_shoot_up = 0;
		if((Initial_flag == 0)&&(Flag_shoot_up == 2))
			FeedMotorUPKeepShooting(CTRL_SetDartleTime/2);
	}
}
#endif

//----------------------------------------赛前填满弹道模式控制---------------------------------------//
void FeedMotor_fill_up(void)
{
	
}
//-------------------------------------------高频自动挡射击------------------------------------------//
extern float panduan_Z;
extern Point3fStruct Point3f;//最终值
void FeedMotor_high_freq(void)
{
	feedmotor_init_flag = 1;
	float HighDartlePerNum;
	if((GetPitchLocation()<=0.485)||(GetPitchLocation()>=0.5678)) //大于平角不打  下极限不打
			FeedMotor_InitGame();
	else
	{
		if (Point3f.z < 6500)
			HighDartlePerNum = 10;
		else if((Point3f.z >= 6500)&&(Point3f.z <7500))
			HighDartlePerNum = 8;
		else HighDartlePerNum = 5;
		FeedMotorKeepShooting(1/(HighDartlePerNum*0.004));
	}
}
#if DOUBLEFEED
void FeedMotor_UP_high_freq(void)
{
	feedmotor_init_up_flag = 1;
	float HighDartlePerNum_UP;
	//LowDartlePerNum = calc();
	HighDartlePerNum_UP = 15;
	FeedMotorUPKeepShooting(1/(HighDartlePerNum_UP*0.004));
}
#endif
//-------------------------------------------低频自动挡射击------------------------------------------//
void FeedMotor_low_freq(void)
{
	feedmotor_init_flag = 1;
	float LowDartlePerNum;
	if((GetPitchLocation()<=0.485)||(GetPitchLocation()>=0.5678)) //大于平角不打  下极限不打
			FeedMotor_InitGame();
	else
	{
		if (Point3f.z < (7800))
		{
			LowDartlePerNum = 2;
			FeedMotorKeepShooting(1/(LowDartlePerNum*0.004));
		}
		else 
		{	
			FeedMotor_InitGame();
		}
	}
}
#if 0
void FeedMotor_UP_low_freq(void)
{
	feedmotor_init_up_flag = 1;
	float LowDartlePerNum_UP;
	//LowDartlePerNum = calc();
	LowDartlePerNum_UP = 3;
	FeedMotorUPKeepShooting(1/(LowDartlePerNum_UP*0.004));
}
#endif
//------------------------------------------连发计数清零--------------------------------------------//
void ClearTimeDartle(void)
{
		if(FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL
			&&FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ
			&&FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		TimeDartle = 0;
}

//------------------------------------------弹速计算相关--------------------------------------------//


