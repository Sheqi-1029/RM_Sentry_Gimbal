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

//ͬ��̨������ʽ
#define DOUBLEFEED   (0) //1-��������0-��������


//����ϵͳ������������ģʽ
#define JUDGEMENTGAME  (1)

//�������Ʋ���
#define HEAT_UPPER_LIMIT  (320.0f)//��������
#define COOLlNG_RATIO   	(100.0f)//ÿ����ȴ
#define SHOOT_SPEED     	(30.0f)//�����ٶ�
#define SPARE_HEAT        (20.0f)//��������
#define HEAT_PER_SHOOT    (10.0f)//����������

//�ֶ���������λ
#define LOADED_SHOOT      (2)//����
#define BURSTS_SHOOT      (3)//����
#define DARTLE_SHOOT      (1)//����


//��������
#define ENCODER_LINE (36.0f)//���ٱ�
#define SHOOT_NUMBER (10)//���䲦�����̳���
#define SHOOT_ONCE_LOCATION ( -ENCODER_LINE/SHOOT_NUMBER*1.0f )//һ�β���λ��

#define UP_NUMBER (8)//�ϲ������̳���
#define UP_ONCE_LOCATION ( ENCODER_LINE/UP_NUMBER*1.0f )//һ�β���λ��

#define CTRL_SetDartlePerNum   (5.0f)//�ֶ������Է���Ƶ��
#define CTRL_SetDartleTime     (1/(CTRL_SetDartlePerNum*0.004))//�ֶ������Է������� 0.004Ϊ��������

#define Count_SetDartleTime    (CTRL_SetDartleTime) //����ģʽ��������

//��е��ʱ��ʼ��λ��
#define FEED_INIT_BEST_LOCA  (0.00f)

//��ת��ת���̸���
#define MOTOR_RETURN_RATE (0.5f)

//�Ӿ����
#define FAR_TARGET_DISTANCE   (1000)

float SetLocation;

//--------------Significant�����������-----------------//
extern RemoteDataUnion RemoteData; //ң��������
extern u8 Initial_flag;//���ο���ʶ�𲼶���
int feedmotor_init_flag = 1;//��ʼ����е��λ��ʶ��1
int feedmotor_init_up_flag = 1;//��ʼ����е��λ��ʶ��2
int Flag_shoot = 0;          //�����ʶ��
int remote_shoot;        //ң������������

short Shoot_Heat;     //��ȡ�������
int  Shoot_test_Heat;  //�Կ��������
int Heat_count;//������ʱ

int 		TimeCount=0;								//��������0.1sʱ�ι���תʹ��
int 		TimeCountUP=0;								//��������0.1sʱ�ι���תʹ��
int     TimeDartle=0;               //��������ʱ�ι�����ʹ��
int     TimeDartleUP=0;               //��������ʱ�ι�������ʹ��
float 	Current[10]={0,0,0,0,0,0,0,0,0,0};
float   CurrentUP[10]={0,0,0,0,0,0,0,0,0,0};
float  CurrentMean;
float  CurrentMeanUP;

FeedMotorCtrlStruct FeedMotorCtrl;//�������������
FeedMotorCtrlStruct FeedMotor_UpCtrl;//�ϲ������������

extern VisionDataStruct   VisionData;
extern receive_judge_t judge_rece_mesg; //ԭֱ�ӽ��ղ���ϵͳ����
extern Judgement_data judgement_data; //����ϵͳ��������

int count_bullet_flag = 0;//����ģʽ������־λ
int num_count = 0;//�������ͳ��

float bullet_speed[10] = {23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2,23.2};
float Bullet_speedAVR = 23.2;

//-------------------------------------*********�����߼����ƹ���********-------------------------------------//
//---------------------------------�����ܿ��߼�-----------------------------------------//
void FeedMotorControlLogic()
{	
	LockedMotorDetectionAndProcessed();
	#if  JUDGEMENTGAME
	JudgementGunEnergyUpdate();
	#endif
	FeedMotorCtrl.FeedMotorMode=GetFeedMotorMode();
	
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_INIT)
		FeedMotor_InitGame();														       //��ʼ����е��λ(������ʱ)
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL)
		FeedMotor_remote_ctrl();													  	 //ң�ط���ģʽ
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_FILL_UP)
		FeedMotor_fill_up();                                   //��ǰ��������ģʽ
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ)
		FeedMotor_high_freq();                                 //��Ƶ�Զ������
	if (FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		FeedMotor_low_freq();                                  //��Ƶ�Զ������
	
	
	//ClearTimeDartle();
	
}

#if DOUBLEFEED
//---------------------------------�ϲ����ܿ��߼�-----------------------------------------//
int bullet_num_detec_flag = 1;//����ӵ�����־λ
int bullet_num_detec;//��ʱ�ӵ���
int bullet_max_num_temp = 500;//��ʱ���ֵ

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
		FeedMotor_UP_InitGame();														       //��ʼ����е��λ(������ʱ)
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL)
		FeedMotor_UP_remote_ctrl();													  	 //ң�ط���ģʽ
	
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ)
		FeedMotor_UP_high_freq();                                 //��Ƶ�Զ������
	if (FeedMotor_UpCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		FeedMotor_UP_low_freq();                                  //��Ƶ�Զ������
	
}
#endif

//----------------------------------------��ת����㷨--------------------------------------------//
void LockedMotorDetectionAndProcessed(void)		//�����ת
{

	//���µ���
	for(int i=0;i<9;i++)
	{
			Current[i]=Current[i+1];
	}
	Current[9]=GetFeedMotorCurrent();
	
	//��ת���
	if(FeedMotorCtrl.DeverseLocked==1)//������ת  1Ϊ��ת 0û�ж�ת
	{
			if(TimeCount<25)			//0.1sһ�����ڣ���Ȼ��Ҳ��֪��Ϊʲôfreertos��Ҫ4msһ������
			  TimeCount++; 
			else
			{
				TimeCount = 0;
				FeedMotorCtrl.DeverseLocked = 0;
			}
	}
	else//û�ж�ת
	{	
			CurrentMean=(fabs(Current[0])+fabs(Current[1])+fabs(Current[2])+fabs(Current[3])
									+fabs(Current[4])+fabs(Current[5])+fabs(Current[6])+fabs(Current[7])
									+fabs(Current[8])+fabs(Current[9]))/10.0f; 
			
			if( fabs(CurrentMean) > 9500.0f)
			{
				FeedMotorCtrl.SetLocation =  GetFeedMotorLocation() + MOTOR_RETURN_RATE * SHOOT_ONCE_LOCATION;	//��ת
				SetFeedMotorSetLocation(  FeedMotorCtrl.SetLocation );
				FeedMotorCtrl.DeverseLocked = 1;					
			}
			else
				FeedMotorCtrl.DeverseLocked = 0;
	}
	
}

#if DOUBLEFEED
void LockedUPMotorDetectionAndProcessed(void)		//�ϲ�����ת
{

	//���µ���
	for(int i=0;i<9;i++)
	{
			CurrentUP[i]=CurrentUP[i+1];
	}
	CurrentUP[9]=GetFeedMotorUPCurrent();
	
	//��ת���
	if(FeedMotor_UpCtrl.DeverseLocked==1)//������ת  1Ϊ��ת 0û�ж�ת
	{
			if(TimeCountUP<25)			//0.1sһ�����ڣ���Ȼ��Ҳ��֪��Ϊʲôfreertos��Ҫ4msһ������
			  TimeCountUP++; 
			else
			{
				TimeCountUP = 0;
				FeedMotor_UpCtrl.DeverseLocked = 0;
			}
	}
	else//û�ж�ת
	{	
			CurrentMeanUP=(fabs(CurrentUP[0])+fabs(CurrentUP[1])+fabs(CurrentUP[2])+fabs(CurrentUP[3])
									+fabs(CurrentUP[4])+fabs(CurrentUP[5])+fabs(CurrentUP[6])+fabs(CurrentUP[7])
									+fabs(CurrentUP[8])+fabs(CurrentUP[9]))/10.0f; 
			
			if( fabs(CurrentMeanUP) > 9000.0f)
			{
				FeedMotor_UpCtrl.SetLocation =  GetFeedMotorUPLocation() + MOTOR_RETURN_RATE * UP_ONCE_LOCATION;	//��ת
				SetFeedMotorUPSetLocation(  FeedMotor_UpCtrl.SetLocation );
				FeedMotor_UpCtrl.DeverseLocked = 1;					
			}
			else
				FeedMotor_UpCtrl.DeverseLocked = 0;
	}
	
}
#endif

//----------------------------------------�ӵ�����------------------------------------------------//
void FeedMotorLocationUpdate(unsigned char ShootNumber)
{
		if(FeedMotorCtrl.DeverseLocked == 0)	//����ת
		{
			#if  JUDGEMENTGAME
			if( GunEnergyJudge() )	//��������
			{
			#endif
				if( fabs(GetFeedMotorLocationError()) < fabs(SHOOT_ONCE_LOCATION) )	//������
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
//-----------------------------------------�ӵ�����-----------------------------------------------//
void FeedMotorUPLocationUpdate(unsigned char ShootNumber)
{
		if(FeedMotor_UpCtrl.DeverseLocked == 0)	//����ת
		{
			
				if( fabs(GetFeedMotorUPLocationError()) < fabs(UP_ONCE_LOCATION) )	//������
				{
						FeedMotor_UpCtrl.SetLocation = GetFeedMotorUPSetLocation() - UP_ONCE_LOCATION * ShootNumber;
						SetFeedMotorUPSetLocation( FeedMotor_UpCtrl.SetLocation );
				}
			
		}
}
#endif

//------------���䲦���ӵ�����Ƶ��ģʽ����-----------------//
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
//------------���������ӵ�����Ƶ��ģʽ����-----------------//
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


//------------�ӵ���������ģʽ����-----------------//
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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>����������غ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
EnergyInfoStruct EnergyInfo ;
u8 ForceCloseFlag = 0;//����������ʶ��

void JudgementGunEnergyUpdate()
{
	
	if (judgement_data.game_mode == 0) //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�ʼ�շ���
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

//1��������
//0����������
int GunEnergyJudge()   
{
		if( HEAT_UPPER_LIMIT - EnergyInfo.GunEnergy >= SHOOT_SPEED + SPARE_HEAT)
			return 1;
		else
			return 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>����ģʽ��غ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//----------------------------------------------ģʽ�л�-----------------------------------------------//
FeedMotorModeEnum GetFeedMotorMode(void)
{
		return FeedMotorCtrl.FeedMotorMode;
}
void SetFeedMotorMode(FeedMotorModeEnum Mode) //ģʽ��ֵ
{
		FeedMotorCtrl.FeedMotorMode = Mode;
}
//-------------------------------------��ʼ����е��λ��������ʱ��--------------------------------------//
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

//-------------------------------------------ң����ģʽ����--------------------------------------------//
void FeedMotor_remote_ctrl(void)
{
	feedmotor_init_flag = 1;
	remote_shoot = RemoteData.RemoteDataProcessed.RCValue.s1; 
	
	//----------------����׼��----------------//
	if(remote_shoot == LOADED_SHOOT) Flag_shoot = 1;
	//------------------����------------------//				
	if(remote_shoot == BURSTS_SHOOT) 
	{
		if((Initial_flag == 0)&&(Flag_shoot == 1))
		{
			FeedMotorLocationUpdate(1);
		}
		Flag_shoot = 0;
	}
	//------------------����------------------//
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
	//----------------����׼��----------------//
	if(remote_shoot_up == LOADED_SHOOT) Flag_shoot_up = 1;
	//------------------����------------------//				
	if(remote_shoot_up == BURSTS_SHOOT) 
	{
		if((Initial_flag == 0)&&(Flag_shoot_up == 1))
		{
			FeedMotorUPLocationUpdate(1);
		}
		Flag_shoot_up = 0;
	}
	//------------------����------------------//
	if(remote_shoot_up == DARTLE_SHOOT)
	{
		Flag_shoot_up = 2;
		if(Initial_flag == 1) Flag_shoot_up = 0;
		if((Initial_flag == 0)&&(Flag_shoot_up == 2))
			FeedMotorUPKeepShooting(CTRL_SetDartleTime/2);
	}
}
#endif

//----------------------------------------��ǰ��������ģʽ����---------------------------------------//
void FeedMotor_fill_up(void)
{
	
}
//-------------------------------------------��Ƶ�Զ������------------------------------------------//
extern float panduan_Z;
extern Point3fStruct Point3f;//����ֵ
void FeedMotor_high_freq(void)
{
	feedmotor_init_flag = 1;
	float HighDartlePerNum;
	if((GetPitchLocation()<=0.485)||(GetPitchLocation()>=0.5678)) //����ƽ�ǲ���  �¼��޲���
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
//-------------------------------------------��Ƶ�Զ������------------------------------------------//
void FeedMotor_low_freq(void)
{
	feedmotor_init_flag = 1;
	float LowDartlePerNum;
	if((GetPitchLocation()<=0.485)||(GetPitchLocation()>=0.5678)) //����ƽ�ǲ���  �¼��޲���
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
//------------------------------------------������������--------------------------------------------//
void ClearTimeDartle(void)
{
		if(FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_REMOTE_CTRL
			&&FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_HIGH_FREQ
			&&FeedMotorCtrl.FeedMotorMode==FEEDMOTOR_LOW_FREQ)
		TimeDartle = 0;
}

//------------------------------------------���ټ������--------------------------------------------//


