/****************************************************
*			Title:		云台控制
*			Version:	6.1.2
*			Data:			2017.09.24
*												LD.
*****************************************************/
#include "driver_gimbal.h"
#include "bsp_can.h"
#include "math.h"
#include "driver_feedmotor.h"
#include "pid.h"
#include "kalman_filter.h"
#include "driver_friction.h"
#include "driver_hi229um.h"

#define ENCODER_LINE (8191.0)
#define GYROY_OFFSET (0)
#define GYROZ_OFFSET (0)

#define PITCH (0)

#define PITCH_LOCATION_KP (2)				//9
#define PITCH_LOCATION_KI (0.0005)   //0.0001
#define PITCH_LOCATION_KD (3)
#define PITCH_LOCATION_AP (1.8)
#define PITCH_LOCATION_BP (1.4)
#define PITCH_LOCATION_CP (0.2)
#define PITCH_SPEED_KP (20)							
#define PITCH_SPEED_KI (0.0)
#define PITCH_SPEED_KD (0)
#define PITCH_SPEED_AP (15)								
#define PITCH_SPEED_BP (17)								
#define PITCH_SPEED_CP (0.1)	

#define YAW (0)

#define YAW_LOCATION_KP (0)					//1.2
#define YAW_LOCATION_KI (0)
#define YAW_LOCATION_KD (2)
#define YAW_LOCATION_KC (0)
#define YAW_LOCATION_AP (4)				//0.03
#define YAW_LOCATION_BP (5)//0.5
#define YAW_LOCATION_CP (0.1)//1.0
#define YAW_SPEED_KP (0)						//35
#define YAW_SPEED_KI (0)
#define YAW_SPEED_KD (0)	
#define YAW_SPEED_AP (7)//
#define YAW_SPEED_BP (8)//
#define YAW_SPEED_CP (0.1)//

int watch_location;

GimbalMotorStruct	YawMotor,PitchMotor,Delivery,Friction_left,Friction_right;
GimbalSetLocationStruct GimbalSetLocationData;
Pid pitchmotor,yawmotor;
PidParameter pitch,yaw;
EncoderDataStruct	EncoderDataSave;//编码器数据包
GyroDataStruct GyroDataSave;//陀螺仪数据包
short ans_pitch_location;

extern kalman_filter_init_t yaw_kalman_filter_para ;
extern kalman_filter_init_t pitch_kalman_filter_para ;
extern kalman_filter_init_t reset_kalman_filter_para ;

extern kalman_filter_t yaw_kalman_filter;
extern kalman_filter_t pitch_kalman_filter;

extern int lose_flag;//遥控器丢失数据布尔符
extern int close_flag;//遥控器关控布尔符

int Drawspeedf_pitch;
int Drawspeedmotor_pitch;

int Drawsetloca_pitch;
int Drawloca_pitch;
int Drawsetloca_yaw;
int Drawloca_yaw;

extern int Gimbal_flag;
//--------------------------------------云台电机初始化-----------------------------------------//
void GimbalInit(void)
{
	//Yaw轴位置PID初始化
	YawMotor.PIDLocation.OutMax=1;
	YawMotor.PIDLocation.OutMin=-1;
	YawMotor.PIDLocation.calc=&PidCalc;
	YawMotor.PIDLocation.clear=&PidClear;
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	
	YawMotor.PIDSpeed.OutMax=1;
	YawMotor.PIDSpeed.OutMin=-1;
	YawMotor.PIDSpeed.calc=&PidCalc;
	YawMotor.PIDSpeed.clear=&PidClear;
	YawMotor.PIDSpeed.clear(&YawMotor.PIDSpeed);
	
	//Pitch轴位置PID初始化
	PitchMotor.PIDLocation.OutMax=1;
	PitchMotor.PIDLocation.OutMin=-1;
	PitchMotor.PIDLocation.calc=&PidCalc;
	PitchMotor.PIDLocation.clear=&PidClear;
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	
	PitchMotor.PIDSpeed.OutMax=1;
	PitchMotor.PIDSpeed.OutMin=-1;
	PitchMotor.PIDSpeed.calc=&PidCalc;
	PitchMotor.PIDSpeed.clear=&PidClear;
	PitchMotor.PIDSpeed.clear(&PitchMotor.PIDSpeed);
	
	YawMotor.PIDLocation.Kp	= YAW_LOCATION_KP;
	YawMotor.PIDLocation.Ki	= YAW_LOCATION_KI;
	YawMotor.PIDLocation.Kd	= YAW_LOCATION_KD;
	YawMotor.PIDLocation.Kc	= YAW_LOCATION_KC;
	YawMotor.PIDLocation.Ap	=	YAW_LOCATION_AP;
	YawMotor.PIDLocation.Bp	=	YAW_LOCATION_BP;
	YawMotor.PIDLocation.Cp	=	YAW_LOCATION_CP;
	
	YawMotor.PIDSpeed.Kp	=	YAW_SPEED_KP;
	YawMotor.PIDSpeed.Ki	=	YAW_SPEED_KI;
	YawMotor.PIDSpeed.Kd	=	YAW_SPEED_KD;
	YawMotor.PIDSpeed.Ap	=	YAW_SPEED_AP;
	YawMotor.PIDSpeed.Bp	=	YAW_SPEED_BP;
	YawMotor.PIDSpeed.Cp	=	YAW_SPEED_CP;
//////	
	PitchMotor.PIDLocation.Kp	= PITCH_LOCATION_KP;
	PitchMotor.PIDLocation.Ki	= PITCH_LOCATION_KI;
	PitchMotor.PIDLocation.Kd	= PITCH_LOCATION_KD;
	PitchMotor.PIDLocation.Ap	= PITCH_LOCATION_AP;
	PitchMotor.PIDLocation.Bp	= PITCH_LOCATION_BP;
	PitchMotor.PIDLocation.Cp	= PITCH_LOCATION_CP;
	
	PitchMotor.PIDSpeed.Kp	=	PITCH_SPEED_KP;
	PitchMotor.PIDSpeed.Ki	=	PITCH_SPEED_KI;
	PitchMotor.PIDSpeed.Kd	=	PITCH_SPEED_KD;
	PitchMotor.PIDSpeed.Ap = PITCH_SPEED_AP;
	PitchMotor.PIDSpeed.Bp = PITCH_SPEED_BP;
	PitchMotor.PIDSpeed.Cp = PITCH_SPEED_CP;
	

}

void GimbalReturnToInitLocation(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.Location.SetLocation=YAW_INIT_VALUE;
	//if(IfPitch)
		//PitchMotor.Location.SetLocation=PITCH_INIT_VALUE;	
}

void GimbalPIDLocationClear(u8 IfPitch,u8 IfYaw)
{
	if(IfYaw)
		YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
	if(IfPitch)
		PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
}


void GimbalSpeedDataUpdate()									//云台速度值更新
{
	GyroAndEncoderDataGet();
	kalman_speed_dispose();
}

//--------------------------------------------卡尔曼滤波速度处理----------------------------------------//
u8 AutoFlag=0;//卡尔曼初始化Flag

void kalman_speed_dispose(void)
{
	float   yawspeedf,pitchspeedf;
	yawspeedf = EncoderDataSave.yawspeed;
	pitchspeedf = EncoderDataSave.pitchspeed;
	#if 1	//卡尔曼滤波
	if(AutoFlag == 0)
	{		
		yaw_kalman_filter_para.xhat_data[0] = EncoderDataSave.Yaw;
		yaw_kalman_filter_para.xhat_data[1] = yawspeedf;
		pitch_kalman_filter_para.xhat_data[0] = EncoderDataSave.Pitch;
		pitch_kalman_filter_para.xhat_data[1] = pitchspeedf;
		kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
		kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	}
	if(AutoFlag < 10)AutoFlag ++;
	float *pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, EncoderDataSave.Pitch,pitchspeedf);
	float *yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, EncoderDataSave.Yaw, yawspeedf);
	if(!AutoFlag)
	{
		YawMotor.Speed.Speed = yawspeedf;   		
		PitchMotor.Speed.Speed =  pitchspeedf; 
	}
	else
	{
		YawMotor.Speed.Speed	=	yaw_kf_result[1];
		PitchMotor.Speed.Speed=  pitch_kf_result[1]; 
	}			

	#endif
	//图像
	Drawspeedf_pitch = (int)(pitchspeedf*360);
	Drawspeedmotor_pitch = (int)(PitchMotor.Speed.Speed*360); 
	
}

//---------------------------------------陀螺仪更新（看实际值仍需更改）---------------------------------//
extern GyroDataPacketStruct GyroDataPacket;
float YawGyroDataTemp,PitchGyroDataTemp;
float YawGyroDataLast,PitchGyroDataLast;
GyroDataStruct YawPitchGyroDataUpdate()				
{
	GyroDataStruct GyroData;
	float K=0.8;
	
	static int YawGyroCount=0,PitchGyroCount=0;
	get_HI229UMGyroData();
	if ((lose_flag==0)||(close_flag))
	{
		YawGyroCount=0;
	}
	YawGyroDataTemp	=	(GyroDataPacket.yaw + 180.0) /	360.0f;
	
	if			(YawGyroDataTemp	-	YawGyroDataLast	>	0.8f)
	YawGyroCount--;//YawGyroDataLast++;}
	else if	(YawGyroDataTemp	-	YawGyroDataLast	<	-0.8f)
	YawGyroCount++;//YawGyroDataLast--;}
	GyroData.Yaw	=	YawGyroDataTemp*K+YawGyroDataLast*(1-K) + YawGyroCount;
	//GyroData.Yaw = YawGyroDataTemp + YawGyroCount;
	YawGyroDataLast = YawGyroDataTemp ;
	
	PitchGyroDataTemp	=	(GyroDataPacket.roll / 35.0) * 0.08f + 1.11f ;
	if			(PitchGyroDataTemp	-	PitchGyroDataLast	>	0.8f)
		PitchGyroCount--;
	else if	(PitchGyroDataTemp	-	PitchGyroDataLast	<	-0.8f)
		PitchGyroCount++;
	
	if (Gimbal_flag)
	{
		PitchGyroCount = 0;
	}
	
	GyroData.Pitch	=	PitchGyroDataTemp*K+PitchGyroDataLast*(1-K) + PitchGyroCount;
	PitchGyroDataLast= PitchGyroDataTemp;
	
	return GyroData;
	
	
}


//----------------------------------------编码器数据更新---------------------------------//
int pitchspeedtemp0;
short yawtemp,pitchtemp;
int YawEncoderCount=0;//yaw编码器圈数
int PitchEncoderCount=0;//pitch编码器圈数
float YawEncoderDataTemp=YAW_TEMPSET,YawEncoderDataLast=YAW_TEMPSET;
EncoderDataStruct YawPitchEncoderDataUpdate()
{
	EncoderDataStruct EncoderData;
	//static int YawEncoderCount=0;//yaw圈数
	
	static float PitchEncoderDataTemp=PITCH_TEMPSET,PitchEncoderDataLast=PITCH_TEMPSET;
  //static int PitchEncoderCount=0;//pitch圈数
	
	float pitchspeedtemp,yawspeedtemp;
	//yaw位置更新
	yawtemp	=	(short)((YawMotor.CANReceiveMessege[0]<<8)|YawMotor.CANReceiveMessege[1]);
	YawEncoderDataTemp	=	(float)(yawtemp	/	ENCODER_LINE);
	
	//过零点检测
	if			(YawEncoderDataTemp	-	YawEncoderDataLast	>	0.8f)
		YawEncoderCount--;
	else if	(YawEncoderDataTemp	-	YawEncoderDataLast	<	-0.8f)
		YawEncoderCount++;
	
	if (Gimbal_flag)
	{
		YawEncoderCount = 0;
	}
	
	EncoderData.Yaw	=	YawEncoderCount	+	YawEncoderDataTemp;
	YawEncoderDataLast	=	YawEncoderDataTemp;
	
	//pitch位置更新
	pitchtemp	=	(short)((PitchMotor.CANReceiveMessege[0]<<8)|PitchMotor.CANReceiveMessege[1]);
	PitchEncoderDataTemp	=	(float)(pitchtemp	/	ENCODER_LINE);
	
	//过零点检测
	if			(PitchEncoderDataTemp	-	PitchEncoderDataLast	>	0.8f)
		PitchEncoderCount--;
	else if	(PitchEncoderDataTemp	-	PitchEncoderDataLast	<	-0.8f)
		PitchEncoderCount++;
	
	if (Gimbal_flag)
	{
		PitchEncoderCount = 0;
	}
	
	EncoderData.Pitch	=	PitchEncoderCount	+	PitchEncoderDataTemp;
	PitchEncoderDataLast	=	PitchEncoderDataTemp;
	
	//yaw pitch 速度更新
	yawspeedtemp	=	(short int)((YawMotor.CANReceiveMessege[2]<<8)|YawMotor.CANReceiveMessege[3]);
	EncoderData.yawspeed = yawspeedtemp/360.f;
	pitchspeedtemp0	=	(short int)((PitchMotor.CANReceiveMessege[2]<<8)|PitchMotor.CANReceiveMessege[3]);
	EncoderData.pitchspeed = pitchspeedtemp0/360.f;
	return EncoderData;
}

//-----------------------------------------yaw轴数据获取选择---------------------------------//
float GetYawEncoderValue()//返回yaw编码器数据
{
	return	EncoderDataSave.Yaw;
}
float GetYawGyroValue()//返回yaw陀螺仪数据
{
	return	GyroDataSave.Yaw;
}

//--------------------------------------yaw&pitch陀螺仪编码器数据返回-------------------------------//
void GyroAndEncoderDataGet(void)
{
	GyroDataSave = YawPitchGyroDataUpdate();
	EncoderDataSave	=	YawPitchEncoderDataUpdate();
	
	//绘图
	Drawloca_pitch = (int)(EncoderDataSave.Pitch * 8191 - 5000);
	
	
}

void YawSetLocationValueChange(float Yaw);
void PitchSetLocationValueChange(float Pitch);

//---------------------------------------------赋值电机pid结构体-------------------------------//
void GimbalDataInput(GimbalSetLocationStruct GimbalData)
{
	static u8 FlagPitchUseEncoderTemp=1,FlagYawUseEncoderTemp=1;
	
	//绘图
	Drawsetloca_pitch = (int)(GimbalData.PitchSetLocation * 8191 - 5000);
	Drawsetloca_yaw = (int)(YawMotor.Location.SetLocation * 8191);
	Drawloca_yaw = (int)(YawMotor.Location.Location * 8191);
	
	GimbalSetLocationData.YawSetLocation	=	GimbalData.YawSetLocation;        
	GimbalSetLocationData.PitchSetLocation=	GimbalData.PitchSetLocation;      
	
	//云台电机设定值给定
	PitchMotor.Location.SetLocation	=	GimbalData.PitchSetLocation;            //上层pitch设定值赋值于驱动层
	YawMotor.Location.SetLocation	=	GimbalData.YawSetLocation; 								//上层yaw设定值赋值于驱动层
	
	if(GimbalData.FlagPitchUseEncoder)
		PitchMotor.Location.Location	=	EncoderDataSave.Pitch;
	else
		PitchMotor.Location.Location	=	GyroDataSave.Pitch;
	
	if(GimbalData.FlagYawUseEncoder)
		YawMotor.Location.Location	=	EncoderDataSave.Yaw;
	else
		YawMotor.Location.Location	=	GyroDataSave.Yaw;
	
	if(FlagPitchUseEncoderTemp!=GimbalData.FlagPitchUseEncoder)
	{
		PitchMotor.Location.SetLocation	=	PitchMotor.Location.Location;
		PitchSetLocationValueChange(PitchMotor.Location.SetLocation);
	}
	
	if(FlagYawUseEncoderTemp!=GimbalData.FlagYawUseEncoder)
	{
		YawMotor.Location.SetLocation	=	YawMotor.Location.Location;
		YawSetLocationValueChange(YawMotor.Location.SetLocation);
	}
	
	FlagPitchUseEncoderTemp=GimbalData.FlagPitchUseEncoder;
	FlagYawUseEncoderTemp = GimbalData.FlagYawUseEncoder;
		
}

//---------------------------------------------位置环计算发送-----------------------------------//
extern FeedMotorStruct FeedMotor,FeedMotor_UP;
//int watch_yaw,watch_setyaw,watch_location_out,watch_speed_out;
//u8 Can2GimbalSendMessege[8];
u8 Can2ChassisSendMessage[8] = {1,0,0,0,0,0,0,1};
extern u8 CAN2_downyawset[8];//下云台yaw数据
void GimbalControlCalculateAndSend(void)
{	
	u8 Can1GimbalSendMessege[8];
	u8 Can2GimbalSendMessege[8];
	//u8 Can2ChassisSendMessage[8];
	if(lose_flag==0||close_flag==0)//遥控器丢失数据or关控保护，发送0
	{
		int i;
		for(i=0;i<=7;i++)
		{
			Can1GimbalSendMessege[i]=0;
			Can2GimbalSendMessege[i]=0;
			//Can2ChassisSendMessage[i]=0x01;
		}
		CAN1_Send_Msg(Can1GimbalSendMessege,8,0x1ff);
		CAN2_Send_Msg(Can2GimbalSendMessege,8,0x1ff);
		//CAN2_Send_Msg(Can2GimbalSendMessege,8,0x200);
		//CAN1_Send_Msg(Can1GimbalSendMessege,8,0x200);
		//CAN2_Send_Msg(Can2ChassisSendMessage,8,0x220);
    FrictionStop();
	}
	if(lose_flag==1&&close_flag==1)
	{
//  pitchmotor.GRMExpect	=	PitchMotor.Location.SetLocation;
//	pitchmotor.GRMReal = PitchMotor.Location.Location;
//	//watch_location = PitchMotor.Location.Location*10000;
//	PIDControl1(&pitchmotor,&pitch);
//	PitchMotor.Speed.SetSpeed	=	pitchmotor.GRMOut;
	PitchMotor.PIDLocation.Ref	=	PitchMotor.Location.SetLocation;
	PitchMotor.PIDLocation.Fdb	=	PitchMotor.Location.Location;
	
	PitchMotor.PIDLocation.calc(&PitchMotor.PIDLocation);	
	PitchMotor.Speed.SetSpeed	=	PitchMotor.PIDLocation.Out;
	
	PitchMotor.PIDSpeed.Ref	=	PitchMotor.Speed.SetSpeed;
	PitchMotor.PIDSpeed.Fdb	=	PitchMotor.Speed.Speed;
	PitchMotor.PIDSpeed.calc(&PitchMotor.PIDSpeed);
	

	YawMotor.PIDLocation.Ref	=	YawMotor.Location.SetLocation;
	YawMotor.PIDLocation.Fdb	=	YawMotor.Location.Location;
	
	YawMotor.PIDLocation.calc(&YawMotor.PIDLocation);	

	YawMotor.Speed.SetSpeed	=	YawMotor.PIDLocation.Out;

	YawMotor.PIDSpeed.Ref	=	YawMotor.Speed.SetSpeed;
	YawMotor.PIDSpeed.Fdb	=	YawMotor.Speed.Speed;
	
	YawMotor.PIDSpeed.calc(&YawMotor.PIDSpeed);

	if(YawMotor.PIDSpeed.Out > 0.8)//防止跳齿的妥协
	{
		YawMotor.PIDSpeed.Out = 0.8;
	}
	if(YawMotor.PIDSpeed.Out < -0.8)
	{
		YawMotor.PIDSpeed.Out = -0.8;
	}
	
	//yaw
	Can1GimbalSendMessege[6]	=	((s16)(YawMotor.PIDSpeed.Out*30000))>>8;
	Can1GimbalSendMessege[7]	=	((s16)(YawMotor.PIDSpeed.Out*30000))&0x00ff;
	
//	//yaw down 
//	Can2GimbalSendMessege[2] = CAN2_downyawset[2];
//	Can2GimbalSendMessege[3] = CAN2_downyawset[3];
	
	//pitch
	Can1GimbalSendMessege[4]	=	((s16)(PitchMotor.PIDSpeed.Out*30000))>>8;
	Can1GimbalSendMessege[5]	=	((s16)(PitchMotor.PIDSpeed.Out*30000))&0x00ff;	

	//feed
	Can1GimbalSendMessege[2]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))>>8;
	Can1GimbalSendMessege[3]=((int16_t)(FeedMotor.PIDSpeed.Out*32767))&0x00ff;
	
//	Can2GimbalSendMessege[6]=((int16_t)(FeedMotor_UP.PIDSpeed.Out*32767))>>8;
//	Can2GimbalSendMessege[7]=((int16_t)(FeedMotor_UP.PIDSpeed.Out*32767))&0x00ff;

	//Can2ChassisSendMessage[0] = 0x01;
	
#if DEBUG_USE_GIMBALMOTOR_CANSEND
  CAN1_Send_Msg(Can1GimbalSendMessege,8,0x1ff);	
	CAN2_Send_Msg(Can2GimbalSendMessege,8,0x1ff);
	
	//CAN1_Send_Msg(Can1GimbalSendMessege,8,0x200);
	//CAN2_Send_Msg(Can2GimbalSendMessege,8,0x200);
	//CAN2_Send_Msg(Can2ChassisSendMessage,8,0x220);
#endif
  }
}
//-----------------------（警示偏差）-------------------//
float GetGimbalError()
{
	return fabs(YawMotor.PIDLocation.Err)	+	fabs(PitchMotor.PIDLocation.Err);
}
//----------------------位置环ki,kd清零---------------------//
void GimbalPIDClear()
{
	PitchMotor.PIDLocation.clear(&PitchMotor.PIDLocation);
	YawMotor.PIDLocation.clear(&YawMotor.PIDLocation);
}
//----------------------yaw&pitch位置归初始位---------------//
void GimbalDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
	YawMotor.Location.SetLocation = YAW_INIT_VALUE + YawEncoderCount;
}
//------------------------pitch单独归位---------------------//
void PitchDataInit()
{
	PitchMotor.Location.SetLocation = PITCH_INIT_VALUE;
}
//--------------------------位置取值----------------------------//
float GetYawLocation()
{
	return YawMotor.Location.Location;
}

float GetPitchLocation()
{
	return PitchMotor.Location.Location;
}

float getYawSetLocation()
{
	return YawMotor.Location.SetLocation;
}

float getPitchSetLocation()
{
	return PitchMotor.Location.SetLocation;
}
float GetYawLocationRaw()
{
	return (YawMotor.Location.Location - YawEncoderCount);
}
float GetYawSpeedRaw()
{
	return (YawMotor.Speed.Speed);
}