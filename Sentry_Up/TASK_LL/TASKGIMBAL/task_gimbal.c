/*----------------------------------------------//
|       title:�ڱ���̨��������									 |
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

//��ѧ
#define PIE (3.1415926) 

//ң�������λ
#define REMOTE_COMMAND_VOID        (3)//�յ�
#define REMOTE_COMMAND_CTRL        (2)//�ֶ���
#define REMOTE_COMMAND_AUTO        (1)//�Զ���
//ң����������
#define YAW_REMOTE_SENSITIVE 			(0.0005f)
#define PITCH_REMOTE_SENSITIVE 		(0.001f)
//ң����ͨ������趨
#define MID_CTRL              (1024.0)  //chͨ���м�ֵ
#define DIFFERENCE_CTRL       (1684.0 - 364.0)  //chͨ����ֵ��

//������&�������л�
#define ENCODER_USE                (1)//������
#define GYRO_USE                   (0)//������

//YAWѲ��
#define YAW_PATROL_SENSITIVE 				(0.0002f)//

//PITCHѲ��
#define PITCH_PATROL_PERIOD_MS 		(700)//����
#define PITCH_PATROL_ANGLE 				(0.026f)//������͵㣺0.512  �ߵ㣺0.564��
#define PITCH_PATROL_CENTER				(0.538f)//���ĵ�

//YAWѲ��0
#define YAW_PATROL_PERIOD_MS 		  (3000)//����
#define YAW_PATROL_ANGLE 					(0.2f)//������͵㣺0.66  �ߵ㣺0.76��
#define YAW_PATROL_CENTER					(YAW_INIT_VALUE)//���ĵ�

//Ѳ��ģʽ2֮YAW����Ծλ��
#define PITCH_PATROL_2_MS					(500)
#define YAW_PATROL_STEP						(0.15f)
#define YAW_STAY_TIME_MS					(600)

//װ���ܻ���ģʽ�л�
#define FIND_ENEMY_TIME_MS				(3000)
//#define YAW_TURN_SENSITIVE				(0.00005f)

//�Զ���������ӿ�
#define MAX_SEQUENCE_LENGTH				(30)
#define VISION_DATA_DELAY_MS			(20)

#define SPEED_THRESHOLD 					(10.0f)
#define	SPEED_CALC_SENSITIVE 			(1000)

GimbalCtrlStruct GimbalCtrl = {	.GimbalMode = GIMBAL_RELAX , 
																.GimbalSetLocationDataTemp={PITCH_INIT_VALUE,YAW_INIT_VALUE,1,0} , 
																.YawPartolDirection = 1};
															//������	
															
//-------------�������˲���ر����趨-------------------//																
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
  .R_data = {100, 0, 0, 2000}// 5000ԽСԽ��������ֵ
};

kalman_filter_t yaw_kalman_filter={0};
kalman_filter_t pitch_kalman_filter={0};
																
															
//--------------Significant�����������-----------------//

u8 remote_command;//ң��������ָ��
extern int close_flag;//ң�����ؿز�����
extern RemoteDataPortStruct	RemoteDataPort;//ң����ͨ������
u8 Initial_flag;//���ο���ʶ�𲼶���
extern RemoteDataUnion RemoteData; //ң��������
extern VisionDataStruct VisionData; //�Ӿ�����

int PitchPatrolCount=0;//�Զ���ʱ�����_pitch
int YawPatrolCount=0;//�Զ���ʱ���ʱ_yaw
int YawPointArmourCount=0;//�Զ���ǰ��װ��ʱ�����_yaw
float watch_yaw_pat;
Judgement_data judgement_data; //����ϵͳ��������

float Yaw_initial_get;//yaw��һʱ���ȡλ��ֵ
float Pitch_initial_get;//pitch��һʱ���ȡλ��ֵ

float Yaw_init_center = 0.25;//yaw����λ��
float Yaw_Base_Location;
int Yaw_init_center_flag = 1;//yaw����λ�û�ȡ��ʶ��

int Gimbal_flag = 1;//��ʼ���ַ�

u8 Can_down_tofire[8]={0,0,0,0,0,0,0,0};//���𴫵�
//-------------------------------------*********��̨�߼����ƹ���********-------------------------------------//
void GimbalControlTask()	
{	
	
	//�������˲�������ʼ��
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	
	//ģʽ����
	//remote_command = RemoteData.RemoteDataProcessed.RCValue.s2;
	//Gimbal_mode = GetGimbalMode();//
	//GimbalCtrl.GimbalMode = GetGimbalMode();
	watch_yaw_pat = GetYawLocationRaw();
	//Gimbal
	GimbalUseEncoder(ENCODER_USE,ENCODER_USE);//������or������λ�û�ȡֵ�趨(yaw,pitch)
	
	//FeedMotor
	FeedMotorDataUpdate();
	MotorLocationControlLogic();

	//if(GetGimbalError()<0.2f)
	//{
		if(GimbalCtrl.GimbalMode == GIMBAL_RELAX)	
			Gimbal_relax();                                      //ֹͣ����ģʽ����
		if(GimbalCtrl.GimbalMode == GIMBAL_REMOTE_CTRL)	
			Gimbal_remote_ctrl();  															 //ң����ģʽ����
		if(GimbalCtrl.GimbalMode == GIMBAL_PATROL) 
			{ Gimbal_patrol();                                     
		    Can_down_tofire[0] = 0;}  //Ѳ��ģʽ
		if(GimbalCtrl.GimbalMode == GIMBAL_SEARCH_FRONT) 
			Gimbal_search_front(); 															 //ǰ(��)��Ѳ��
		if(GimbalCtrl.GimbalMode == GIMBAL_SEARCH_BACK) 
			Gimbal_search_behind(); 														 //��(��)��Ѳ��
		if(GimbalCtrl.GimbalMode == GIMBAL_TRACK_ARMOR)	
			{	Gimbal_track_armor();  
				Can_down_tofire[0] = 0;}                           //�Զ�׷��ģʽ����
		if(GimbalCtrl.GimbalMode == GIMBAL_FIRE) 
			{	Gimbal_fire();    
				Can_down_tofire[0] = 1;}		//�Զ�����ģʽ����
	  ClearPatrolCout();
		GimbalDriverTask();
	//}
		if(	Can_down_tofire[0] == 0) for(int i=1;i<8;i++) Can_down_tofire[i] = 0;
		
		//CAN2_Send_Msg(Can_down_tofire,8,0x0255);//������
				
		if(Gimbal_flag) Gimbal_flag = 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>��̨������غ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//--------------------------------------------------��̨�ײ�����---------------------------------------------//
void GimbalDriverTask(void)
{
	GimbalDataInput(GimbalCtrl.GimbalSetLocationDataTemp);	//��̨λ���ٶȸ�ֵ���趨ֵ����
	GimbalSpeedDataUpdate();									//��̨λ���ٶȶ�ȡ����
	GimbalControlCalculateAndSend();					//��̨λ�û�����
}

//----------------------------------------------------yaw��λ----------------------------------------------//
void YawLocationLimit(void)		//�޻���YAW������������
{
	if(GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation > Yaw_Base_Location+0.42f)
		GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation = Yaw_Base_Location+0.42f;
	else if(GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation < Yaw_Base_Location-0.42f)
		GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation = Yaw_Base_Location-0.42f;
}

//----------------------------------------------------pitch��λ----------------------------------------------//
void PitchLocationLimit(void)		//PITCH������������
{
	if(GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation > PITCH_MAX_VALUE)
		GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_MAX_VALUE;
	else if(GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation < PITCH_MIN_VALUE)
		GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_MIN_VALUE;
}

//----------------------------------------------------yaw���Ƹ�ֵ--------------------------------------------//
void YawSetLocationValueChange(float Yaw)
{
	GimbalCtrl.GimbalSetLocationDataTemp.YawSetLocation	=	Yaw;
	YawLocationLimit();
}

//---------------------------------------------------pitch���Ƹ�ֵ-------------------------------------------//
void PitchSetLocationValueChange(float Pitch)
{
	GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation	=	Pitch;	
	PitchLocationLimit();
}
//------------------------------------------yaw����������������ǣ�ʹ��ģʽ�ж�------------------------------//
void GimbalUseEncoder(u8 isUseEncoder_Yaw,u8 isUseEncoder_Pitch)
{
	GimbalCtrl.GimbalSetLocationDataTemp.FlagYawUseEncoder	=	isUseEncoder_Yaw;
	GimbalCtrl.GimbalSetLocationDataTemp.FlagPitchUseEncoder = isUseEncoder_Pitch;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>��̨ģʽ��غ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//-------------------------------------------------ģʽ�л�----------------------------------------------------//
/*ģʽ�߼���ң�����󲦸ˣ��£��յ���
            ң�����󲦸ˣ��У��ֶ�����
						ң�����󲦸ˣ��ϣ��Զ�����
						{
							�Ӿ����1��Ѳ�ߣ�
							�Ӿ����2��׷�٣�
							�Ӿ����3������
						}	   
---------------------------------------*/
//extern u8 command_receive;//�Ӿ�����״̬
u8 comm_rec = 1;//�ѻ���������״̬��
GimbalModeEnum GetGimbalMode(void)  //����ģʽ
{
	return GimbalCtrl.GimbalMode;
}
void SetGimbalMode(GimbalModeEnum Mode) //ģʽ��ֵ
{
		GimbalCtrl.GimbalMode = Mode;
}
	
//--------------------------------------------ͣ��ģʽ����---------------------------------------------//
int ctrl_initial_flag;// �ֶ�����ʼ��б�±�ʶ
int patrol_initial_flag;//�Զ�����ʼ��б�±�ʶ
ramp_t ctrl_initial_ramp = RAMP_GEN_DAFAULT;// �ֶ�����ʼ��б��
ramp_t patrol_initial_ramp = RAMP_GEN_DAFAULT;//�Զ�����ʼ��б��
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

//-------------------------------------------ң����ģʽ����--------------------------------------------//
float remote_yaw,remote_pitch;  //ң�������λ��
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
			//YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (YAW_INIT_VALUE + YawEncoderCount  - Yaw_initial_get));//����
			YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));//�޻���
			PitchSetLocationValueChange(Pitch_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (PITCH_INIT_VALUE - Pitch_initial_get));
		}
		else
		{
			//YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (YAW_INIT_VALUE + YawEncoderCount - Yaw_initial_get));//����
			YawSetLocationValueChange(Yaw_initial_get + ctrl_initial_ramp.calc(&ctrl_initial_ramp) * (Yaw_Base_Location - Yaw_initial_get));//�޻���
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
//----------------------------------------------Ѳ��ģʽ-----------------------------------------------//
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
		#if 0  //yaw������Բ��Ѳ�� 
		patrol_yaw = getYawSetLocation() + YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 0  //yaw��ǰ����Ѳ
		if (GetYawLocationRaw() >= 0.39f  && GetYawLocationRaw() <= 0.89f) //ǰ��װ�׷�Χ��
			patrol_yaw = getYawSetLocation() + YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		else patrol_yaw = getYawSetLocation() + 1.8*YAW_PATROL_SENSITIVE * GimbalCtrl.YawPartolDirection;
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 0  //yaw��ǰ��Ѳ
		YawPatrolCount++;
		patrol_yaw = Yaw_init_center + YAW_PATROL_ANGLE*sin( 2*PIE/YAW_PATROL_PERIOD_MS * YawPatrolCount );
		YawSetLocationValueChange(patrol_yaw);
		#endif
		#if 1 //yaw�޻���װ��ǰѲ
		float sensitiveyaw;
		if(( GetYawLocationRaw() <= 0.59f && GetYawLocationRaw() >=0.00f)
				||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.91f))   //ǰ��װ�׷�Χ��  
		{
				sensitiveyaw = YAW_PATROL_SENSITIVE;
				patrol_armour_ramp.init(&patrol_armour_ramp,500);
				YawPointArmourCount++;
				if( YawPointArmourCount >= 5000 )
					YawPointArmourCount = 1;//�ǹ�������
		}					
		else if( (GetYawLocationRaw() >= 0.75f && GetYawLocationRaw() <= 0.91f)) //�Һ�С��
		{
				GimbalCtrl.YawPartolDirection = 1;
				if( YawPointArmourCount > 0 )							
					sensitiveyaw = 2*YAW_PATROL_SENSITIVE * patrol_armour_ramp.calc(&patrol_armour_ramp) - YAW_PATROL_SENSITIVE;//б�¼�����							
				else
					sensitiveyaw = YAW_PATROL_SENSITIVE;
		}
		else if((GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >= 0.59f))					//���С��
		{
				GimbalCtrl.YawPartolDirection = -1;
				if( YawPointArmourCount > 0 )							
					sensitiveyaw = 2*YAW_PATROL_SENSITIVE * patrol_armour_ramp.calc(&patrol_armour_ramp) - YAW_PATROL_SENSITIVE;//б�¼�����							
				else
					sensitiveyaw = YAW_PATROL_SENSITIVE;
		}
		YawSetLocationValueChange(getYawSetLocation() + sensitiveyaw * GimbalCtrl.YawPartolDirection);

		#endif
	}
	
}
//---------------------------------------------ǰ(��)����ģʽ-----------------------------------------------//
int YawPointArmourCount_FL;
ramp_t search_armour_ramp = RAMP_GEN_DAFAULT;
void Gimbal_search_front(void)
{
		float sensitiveyaw;
	//if(GetGimbalError()<0.2f)
		//	{	
					PitchPatrolCount ++;
					GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_PATROL_CENTER + PITCH_PATROL_ANGLE*sin( 2*PI/PITCH_PATROL_PERIOD_MS * PitchPatrolCount );
					if(( GetYawLocationRaw() <= 0.59f && GetYawLocationRaw() >=0.25f)) 						//ǰ(��)�෶Χ��  
					{
						  sensitiveyaw = YAW_PATROL_SENSITIVE;
							search_armour_ramp.init(&search_armour_ramp,1000);
							YawPointArmourCount_FL++;
							if( YawPointArmourCount_FL >= 5000 )
							{
								//YawPointArmourCount_FL = 0;//�ǹ�������
								SetGimbalMode(GIMBAL_PATROL);
							}
					}					
					else if( (GetYawLocationRaw() <= 0.25f && GetYawLocationRaw() >= 0.0f)
					||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.75f)) //��-��
					{
							GimbalCtrl.YawPartolDirection = 1;
							if( YawPointArmourCount_FL > 0 )							
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;//б�¼�����							
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
					else if( (GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >= 0.59f)) //��-��
					{
							GimbalCtrl.YawPartolDirection = -1;
							if( YawPointArmourCount_FL > 0 )							
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;//б�¼�����							
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
			    YawSetLocationValueChange(getYawSetLocation() + sensitiveyaw * GimbalCtrl.YawPartolDirection);
			//}

}
//---------------------------------------------��(��)����ģʽ-----------------------------------------------//
int YawPointArmourCount_BR;
void Gimbal_search_behind(void)
{
	float sensitiveyaw;
	//if(GetGimbalError()<0.2f)
		//	{	
					PitchPatrolCount ++;
					GimbalCtrl.GimbalSetLocationDataTemp.PitchSetLocation = PITCH_PATROL_CENTER + PITCH_PATROL_ANGLE*sin( 2*PI/PITCH_PATROL_PERIOD_MS * PitchPatrolCount );
				 
					if( (GetYawLocationRaw() <= 0.25f && GetYawLocationRaw() >= 0.0f)
					||( GetYawLocationRaw() <= 1.00f && GetYawLocationRaw() >=0.91f))	//��(��)�෶Χ��
					{
							search_armour_ramp.init(&search_armour_ramp,100);
							sensitiveyaw = YAW_PATROL_SENSITIVE;
							YawPointArmourCount_BR++;
//							if( YawPointArmourCount > FIND_ENEMY_TIME_MS )
//								SetGimbalMode(GIMBAL_PATROL);
							if( YawPointArmourCount_BR >= 5000 )
							{
								//YawPointArmourCount_BR = 1;//�ǹ�������
								SetGimbalMode(GIMBAL_PATROL);
							}
					}					
					else if( (GetYawLocationRaw() <= 0.91f && GetYawLocationRaw() >= 0.75f))     //�ҡ���
					{
							GimbalCtrl.YawPartolDirection = 1;
							if( YawPointArmourCount_BR > 0 )
								sensitiveyaw = 2*YAW_PATROL_SENSITIVE * search_armour_ramp.calc(&search_armour_ramp) - YAW_PATROL_SENSITIVE;
							else
								sensitiveyaw = 3.0f*YAW_PATROL_SENSITIVE;
					}
					else if(( GetYawLocationRaw() <= 0.75f && GetYawLocationRaw() >=0.25f))     //��-��
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

//------------------------------------------�Զ�׷��ģʽ����-------------------------------------------//
float vision_pitch,vision_pitch_last1,vision_pitch_last2;
float vision_yaw,vision_yaw_last1,vision_yaw_last2;
//float PITCH_SIGHT_AREA=0.00005,YAW_SIGHT_AREA=-0.00003;
float PITCH_PIAN_fdistance = 0  ;//pitch �������ǿ��ƫ��
float YAW_PIAN_fdistance;//yaw �������ǿ��ƫ��
int DRAW_X,DRAW_Y;
float Command_X,Command_Y;//���������XY���ֵ



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
//------------------------------------------�Զ�����ģʽ����-------------------------------------------//
short Yaw_delivery;//���ݹ�һ��ֵ��1000
short Pitch_delivery;//���ݽǶ�ֵ��10
short Dis_delivery;//���ݾ���
float U_zero_deg =  0.662;//0.662 pitch���ֵ

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

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>������غ���<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

//-----------------------------------------�Զ���ʱ���������------------------------------------------//
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
//-----------------------------------------��ر�־λ��ʼ��------------------------------------------//
void Flag_zero_init()
{
	Initial_flag = 1;
	ctrl_initial_flag = 1;
	patrol_initial_flag = 1;
	ctrl_initial_ramp.init(&ctrl_initial_ramp,2000);
	patrol_initial_ramp.init(&patrol_initial_ramp,2000);
}

