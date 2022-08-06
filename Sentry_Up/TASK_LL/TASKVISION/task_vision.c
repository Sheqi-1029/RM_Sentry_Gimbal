#include "task_vision.h"
#include "usbd_cdc_if.h"
#include "task_remote.h"
#include "task_modeswitch.h"
#include "math.h"
#include "task_communicate.h"
#include "task_gimbal.h"
#include "task_friction.h"
#include "driver_gimbal.h"
#include "driver_vision.h"
#include "math.h"
#include "kalman_filter.h"

#define TARGET_COLOUR (0)   //0:   5:
#define HORIZON_PITCH (0.509)  //水平对应pitch编码器
#define ZERO_YAW   (0.25) //yaw 零位

/****************************************************
*			与工控机的通信协议具体格式
      使用的定时器：time7
      通信间隔：20ms
			每20ms会处理一次视觉发送的数据，并且会发送一次数据给工控机
      视觉给电控
      开头1 0x5A
			开头2 0xA5
      角度1（pitch）    32位   
      角度2 （yaw）   32位
      命令状态   16位
      扩展   16位 不开启全发0
      结尾0xff




      电控给视觉
//      开头1 0x3A
//			开头2 0xA3
      云台位置，包括 yaw16位，pitch16位.
      云台移动速度，包括vx16位.vy16位.vz16位   
      命令状态8位 
      扩展32位 不开启全发0 
      结尾0xff

      命令状态（推荐）：步兵：1、手瞄 2、自瞄 3、打符
      哨兵  1、巡逻  2、开火  3、反导   
      
*			
*****************************************************/
//-------------卡尔曼滤波相关变量设定-------------------//	
kalman_filter_init_t Vision_xy_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 0, 0, 1},//状态转移矩阵
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {10, 0, 0, 10}//不确定方差矩阵
};
kalman_filter_t Vision_xy_kalman_filter={0};

kalman_filter_init_t Vision_yaw_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 1, 0, 1},//状态转移矩阵
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {20, 0, 0, 5000}//不确定方差矩阵
};

kalman_filter_t Vision_yaw_kalman_filter={0};


kalman_filter_init_t Vision_yaw_v_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 100, 0, 1},//状态转移矩阵
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {10, 0, 0, 1}//不确定方差矩阵
};
kalman_filter_t Vision_yaw_v_kalman_filter={0};

kalman_filter_init_t Vision_z_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 0, 0, 1},//状态转移矩阵
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {2000, 0, 0, 1}//不确定方差矩阵
};
kalman_filter_t Vision_z_kalman_filter={0};

//--------------Significant整体变量集合-----------------//
extern u8 VisionRXFlag;   
extern uint8_t  UsbRxBuf[13];
extern uint8_t UsbTxBuf[17];
extern u8 Chassis_rebuf[8];
VisionDataStruct   VisionData;
float Set_ChangePitch,Set_ChangeYaw;
extern PID VisionPitchIncreasement , VisionYawIncreasement ;
Point3fStruct Point_0,//加权一次滤波值
							Point_1,//1次卡尔曼滤波值
              Point3f;//最终值
extern float Command_X,Command_Y;
UsbSendMsg Unionsend;

//速度相关
extern Judgement_data judgement_data; //裁判系统接收数据
short chassis_speed_r,chassis_speed_int = 0;//底盘速度
short chassis_speed_last = 0;
float k_chassis_speed = 0.7;//底盘速度增益系数  正向
float chassis_speed_f;//增益底盘速度

float k_cha_speed = 6.5;//目标速度增益系数  //40.0
float cha_speed_f = 0;//增益目标速度
float yaw_speed_f0,yaw_speed_f,last_yaw_speed_f,//线速度
			yaw_lim_minV = 100;//速度最小值下限幅滤波阈值
//控制延时前馈
float Delay_con_t = 0.0;//控制延时
double Delay_bullet_t = 0.0;
float  Delay_time;//延时 = 控制延时+距离/速度

float k_kal_yaw_v = 0.95;
float kal_w_yaw,//卡尔曼滤波后速度
			kal_q_yaw;//卡尔曼滤波加速度
//基乘系数
float PITCH_SIGHT_AREA=0.00005,YAW_SIGHT_AREA=-0.00003; //弹道模型解算量归一
Der pitchder,yawder;

float parol_k = 5;

float panduan_Z = 0;

float shoot_comarea = 1.5;

K_Lowpass_Point K_Low = {0.007,0.5,0.007,0.2,0.01,0.7};//0.01,0.95,0.01,0.8,0.01,0.1

//卡尔曼标志位
int Auto_vision_flag = 0,
		Auto_vision_flag1 = 0,
		Auto_vision_flag2 = 0,
		Auto_vision_flag3 = 0;

//--------------Speed 提取变量-----------------//
int Scp_kal_w;

//------------------------------------------------------//

void Vision_kalman_init(void)  //卡尔曼参数初始化
{
	mat_init(&Vision_xy_kalman_filter.Q,2,2, Vision_xy_kalman_filter_para.Q_data);
	mat_init(&Vision_xy_kalman_filter.R,2,2, Vision_xy_kalman_filter_para.R_data);
	mat_init(&Vision_yaw_kalman_filter.Q,2,2, Vision_yaw_kalman_filter_para.Q_data);
	mat_init(&Vision_yaw_kalman_filter.R,2,2, Vision_yaw_kalman_filter_para.R_data);
	mat_init(&Vision_yaw_v_kalman_filter.Q,2,2, Vision_yaw_v_kalman_filter_para.Q_data);
	mat_init(&Vision_yaw_v_kalman_filter.R,2,2, Vision_yaw_v_kalman_filter_para.R_data);
	mat_init(&Vision_z_kalman_filter.Q,2,2, Vision_z_kalman_filter_para.Q_data);
	mat_init(&Vision_z_kalman_filter.R,2,2, Vision_z_kalman_filter_para.R_data);
}

void VisionControlTask(void)
{
	Vision_kalman_init();
//	pitchder.kd = 0.0f;
//	yawder.kd = 0.0f;
	if (VisionRXFlag)
	{
		VisionReceiveControlTask();
		VisionRXFlag=0;
	}
	sendMsgToPC();
}

int  DRAW_X0,DRAW_Y0,DRAW_X1,DRAW_Y1,DRAW_XR,DRAW_YR;
int  DRAW_ZR,DRAW_PAN_Z,DRAW_Z0;
int  DRAW_KAL_X,DRAW_KAL_Y;
int  DRAW_V,DRAW_kal_v;
int  DRAW_YAW_V,DRAW_RES_V,DRAW_RES_KAL_V;
extern GimbalCtrl VisionGimbalCtrl;
extern GimbalMotorStruct	YawMotor;
int Scp_yaw_speed_f;

int Draw_chassis_speed;
u8 VisionReceiveControlTask(void)
{
	//---------------------底盘速度接收--------------------
	chassis_speed_r = (short)(((Chassis_rebuf[0]<<8)&0xff00)|Chassis_rebuf[1]);
	chassis_speed_int = 0.7*chassis_speed_r + 0.3*chassis_speed_last;
	k_chassis_speed = (chassis_speed_r>0)?0.6:0.4;
	chassis_speed_f = abs(chassis_speed_int-chassis_speed_last) > 100 ? 0.0f:(float)(chassis_speed_int * k_chassis_speed);
	
	Draw_chassis_speed = (int)(chassis_speed_r*10.0f);
	
	//--------------------工控机速度接收-------------------
	unsigned int hor=0,ver=0,dis=0,com=0;
	for(int i=3;i>=0;i--)
	{
   hor+=UsbRxBuf[i];
   if (i!=0)  hor=hor<<8;    
	}
  for(int i=7;i>=4;i--)
	{
   ver+=UsbRxBuf[i];
   if (i!=4)  ver=ver<<8;
	}
	for(int i=11;i>=8;i--)
	{
   dis+=UsbRxBuf[i];
   if (i!=8)  dis=dis<<8;
	}
	
  VisionData.ReceiveYaw   = ((float*)&hor)[0];
  VisionData.ReceivePitch = ((float*)&ver)[0];
	VisionData.ReceiveDistanceRaw = ((float*)&dis)[0];
	
	#if 1   //观测---------------
		DRAW_XR = (int)((((float*)&hor)[0])*10.0f);
		DRAW_YR = (int)((((float*)&ver)[0])*10.0f);
	  DRAW_ZR = (int)((((float*)&dis)[0])*10.0f);
	#endif
	
	//---低通滤波 去除漏帧 光滑 延迟相位-----------------------
	//计算距离
	float k_z = VisionData.ReceiveDistanceRaw < 0.01 ? K_Low.kz_0 : K_Low.kz_1;
  panduan_Z	= VisionData.ReceiveDistanceRaw * k_z + panduan_Z * (1 - k_z);//一次滤波距离值，体现目标识别视觉稳定性，用于决策击打
	//Point_0.z = VisionData.ReceiveDistanceRaw * k_z + VisionData.ReceiveDistance_last * (1 - k_z);
	
	//近距离不做零阶保持,克服小陀螺，远距离做零阶保持
	if(panduan_Z >6800)
	{
		VisionData.ReceiveDistanceRaw = (VisionData.ReceiveDistanceRaw == 0)?VisionData.ReceiveDistance_last:VisionData.ReceiveDistanceRaw; 
		VisionData.ReceiveYaw = (VisionData.ReceiveYaw == 0) ? VisionData.ReceiveYaw_last : VisionData.ReceiveYaw;  
		VisionData.ReceivePitch = (VisionData.ReceivePitch == 0) ? VisionData.ReceivePitch_last : VisionData.ReceivePitch;
	}
	
	Point_0.z = VisionData.ReceiveDistanceRaw * k_z + VisionData.ReceiveDistance_last * (1 - k_z);
	
	//对0做低通 克服小陀螺
	float k_x = fabs(VisionData.ReceiveYaw) < 0.01 ? K_Low.kx_0 : K_Low.kx_1;;
	Point_0.x = VisionData.ReceiveYaw * k_x + VisionData.ReceiveYaw_last * (1-k_x);

	float k_y = fabs(VisionData.ReceivePitch) < 0.01 ? K_Low.ky_0 : K_Low.ky_1;
	Point_0.y = VisionData.ReceivePitch * k_y + VisionData.ReceivePitch_last * (1-k_y);
	
	//---限幅--------------------------------------------------
	Point_0.x = (Point_0.x > 10000)? 10000:Point_0.x;
	Point_0.y = (Point_0.y > 10000)? 10000:Point_0.y;
	Point_0.x = (Point_0.x < -10000)? -10000:Point_0.x;
	Point_0.y = (Point_0.y < -10000)? -10000:Point_0.y;
	
	#if 1   //观测---------------
		DRAW_PAN_Z = (int)((panduan_Z)*10.0f);
		DRAW_Z0 = (int)((Point_0.z)*10.0f);
		DRAW_X0 = (int)((Point_0.x)*10.0f);
		DRAW_Y0 = (int)((Point_0.y)*10.0f);
	#endif
	
	//---一次滤波值卡尔曼滤波---------------------------------------------
	Vision_xy_kalman_dispose();//卡尔曼滤波 对x,y处理 流畅化
	Vision_z_kalman_dispose();//卡尔曼滤波 对z处理 流畅化
	
	#if 1   //观测---------------
		DRAW_X1 = (int)((Point_1.x)*10.0f);
		DRAW_Y1 = (int)((Point_1.y)*10.0f);
	#endif
	//---目标速度处理-------------------------------------------
	Pre_interupt_wsol();                    //通过分频差分提取目标绝对速度 低通光滑 延迟相位 降幅
	Vision_resV_kalman_dispose();           //卡尔曼处理 提前相位 增幅 Point_1.x 转到 Point3f.x 提取出速度为角速度
	//角速度转线速度
	yaw_speed_f0 = tan(kal_w_yaw/180.0*PI)*Point_1.z;
	//限幅低通滤波
	k_kal_yaw_v = (fabs(yaw_speed_f0)<yaw_lim_minV) ? 0.001:0.95;
	yaw_speed_f = k_kal_yaw_v * yaw_speed_f0 + (1-k_kal_yaw_v)*last_yaw_speed_f;  //预测速度值低通去除高频尖峰
	last_yaw_speed_f = yaw_speed_f;
	#if 1
		Scp_yaw_speed_f = (int)(yaw_speed_f*10.0f);
	#endif
	//k_cha_speed = 10;//(yaw_speed_f<0)?9.5:7;//9.5 7   //计算速度提前量
	cha_speed_f = k_cha_speed * yaw_speed_f;  //算入弹道作为前馈（这一步不知道要不要加，要权衡此处相位提前与在后续卡尔曼里对相位的提前）
	
	//Point_1.y/z 转到 Point3f.y/z 
	Point3f.y = Point_1.y;
	Point3f.z = Point_1.z;
	//---yaw卡尔曼 提前相位 增幅 ------------即Point_1.x不叠加相位，属于纯滞后数据  Point3f.x为相位提前相位 可以进行比较调测
	Vision_yaw_kalman_dispose();  //得到最终Point3f.x的值
		
	//限幅	
	Point3f.x = (Point3f.x > 10000)? 10000:Point3f.x;
	Point3f.y = (Point3f.y > 10000)? 10000:Point3f.y;
	Point3f.z = (Point3f.z > 15000)? 15000:Point3f.z;
	Point3f.x = (Point3f.x < -10000)? -10000:Point3f.x;
	Point3f.y = (Point3f.y < -10000)? -10000:Point3f.y;	
	Point3f.z = (Point3f.z < 0)? 0:Point3f.z;
	
	//---子弹发射延迟处理---------------------------------------
	Delay_bullet_t = VisionData.Time_bullet_fly;
  Delay_time = (float)(Delay_bullet_t + Delay_con_t);
	
	//---视界处理-----------------------------------------------
  //Shijielimit_deal();
	
	//---击打判断-----------------------------------------------
	if (panduan_Z <= parol_k)//VisionData.ReceiveDistanceRaw == 0 )
	{		
			VisionData.Command = 3;
			//数据量清零
			Point_1.x = 0;Point_1.y = 0;Point_1.z = 0;
			Point3f.x = 0;Point3f.y = 0;Point3f.z = 0;
			//带预测卡尔曼标志位清零
			Auto_vision_flag2 = 0;
			Auto_vision_flag1 = 0;
	}
  else if(panduan_Z >= parol_k)
	{	
		if(((Command_X < shoot_comarea && Command_X > -shoot_comarea)
					&&(Command_Y < shoot_comarea && Command_Y > -shoot_comarea))
					||(panduan_Z <= (10000)))
			VisionData.Command = 2;
		if ((Command_X > shoot_comarea || Command_X < -shoot_comarea)
					||(Command_Y > shoot_comarea || Command_Y < -shoot_comarea))
		  VisionData.Command = 1;
	}
	#if 1
	  //DRAW_KAL_X = (int)(yaw_speed_f*100.0f);
		DRAW_KAL_X = (int)(Point3f.x*10.0f);
		//DRAW_KAL_Y = (int)(Point3f.y*10.0f);
	#endif
	
	//pitch 偏置量
	/*
	dis=3000   -0.8f
	dis=6000   -0.5f
	dis=8000   -1.2f
	*/
	if(Point3f.z < 3000.0f) VisionGimbalCtrl.offset_pitch = -0.8f;
	else if((Point3f.z > 3000.0f)&&(Point3f.z < 6000.0f))  VisionGimbalCtrl.offset_pitch = 0.1*(Point3f.z/1000.0f - 3) - 0.8f;
	else if((Point3f.z > 6000.0f)&&(Point3f.z < 8000.0f))  VisionGimbalCtrl.offset_pitch = -0.35*(Point3f.z/1000.0f - 6) - 0.5f;
	else if(Point3f.z > 8000.0f) VisionGimbalCtrl.offset_pitch = -1.2f;
	//yaw 偏置量
	if(Point3f.z < 6000.0f) VisionGimbalCtrl.offset_yaw = 0.3f;
	else if(Point3f.z > 8000.0f) VisionGimbalCtrl.offset_yaw = 0.8f;
	else VisionGimbalCtrl.offset_yaw = 0.25*(Point3f.z/1000.0f - 6) + 0.3;
	
	//---弹道模型---------------------------------------------------
	if (Point3f.z > 200)
	{
		Transform(Point3f,GetPitchLocation(),GetYawLocation(),
							&Set_ChangePitch,&Set_ChangeYaw,&Delay_time,chassis_speed_f+cha_speed_f,0);
  }	
	
	//---接入task_gimbal 通入底层-----------------------------------
  Set_ChangePitch *= PITCH_SIGHT_AREA ;
	Set_ChangeYaw *= YAW_SIGHT_AREA ;
	
	//目标值为增量  已知值为差值 
	//---对Pitch进行PID控制
	VisionPitchIncreasement.Ref=Set_ChangePitch;
		//极性一致
	VisionPitchIncreasement.Fdb=(getPitchSetLocation()-GetPitchLocation())*(Set_ChangePitch);
															//(Set_ChangePitch==0)?
															//0:(getPitchSetLocation()-GetPitchLocation())*(Set_ChangePitch/fabs(Set_ChangePitch));//*Set_ChangePitch
	VisionPitchIncreasement.calc(&VisionPitchIncreasement);
	#if 1 //先行微分
	pitchder.eer = getPitchSetLocation()-GetPitchLocation();
	pitchder.der = pitchder.eer - pitchder.lasteer;
	pitchder.lasteer = pitchder.eer;
	pitchder.Out = pitchder.der * pitchder.kd;
	#endif
	VisionData.Change_pitch=VisionPitchIncreasement.Out + pitchder.Out;
	
	//---对Yaw进行PID控制  
	VisionYawIncreasement.Ref=Set_ChangeYaw;
		//极性一致
	VisionPitchIncreasement.Fdb=(Set_ChangeYaw==0)?
																0:(getYawSetLocation()-GetYawLocation())*(Set_ChangeYaw/fabs(Set_ChangeYaw));//*Set_ChangeYaw
	VisionYawIncreasement.calc(&VisionYawIncreasement);
	#if 1 //先行微分
	yawder.eer = getYawSetLocation()-GetYawLocation();
	yawder.der = yawder.eer - yawder.lasteer;
	yawder.lasteer = yawder.eer;
	yawder.Out = yawder.der * yawder.kd;
	#endif
	VisionData.Change_yaw=VisionYawIncreasement.Out + yawder.Out;
 
	//---上一次数据记录----------------------------------------------
	VisionData.ReceiveYaw_last = Point3f.x;
  VisionData.ReceivePitch_last = Point3f.y;
 	VisionData.ReceiveDistance_last = Point3f.z;
	chassis_speed_last = chassis_speed_int;
	
}
	
extern GimbalMotorStruct	YawMotor,PitchMotor;
float send_pitch,send_yaw,send_bullet,send_motion;

void sendMsgToPC(void)
{
	int sta = 0;
	
  send_yaw = (GetYawLocationRaw() - ZERO_YAW) * 360.0f;
	send_pitch = ( GetPitchLocation() - HORIZON_PITCH) * 360.0f;
	send_bullet = 23;
	send_motion = 0;

	Unionsend.state.pitch = send_pitch;// * 182.0;
	Unionsend.state.yaw = send_yaw;// * 182.0;
	Unionsend.state.bullet = 23;
	//Unionsend.state.motion = judgement_data.chassis_dis / 1000.0f; 
	
    for (int i = 0; i < 4; ++i){                //pitch
        UsbTxBuf[i] = Unionsend.buffer[i];			
    }
    for (int i = 0; i < 4; ++i){                //yaw
				UsbTxBuf[i+4] = Unionsend.buffer[i+4];
		}
		for (int i = 0; i < 4; ++i){                //bullet               
        UsbTxBuf[i+8] = Unionsend.buffer[i+8];
		}
		UsbTxBuf[12] = TARGET_COLOUR;

	CDC_Transmit_FS(UsbTxBuf, 13);
	
}

void Vision_xy_kalman_dispose(void)  //卡尔曼xy滤波器数据处理
{
	//x = [x, y];
	#if 1	//卡尔曼滤波
	if(Auto_vision_flag == 0)
	{		
		Vision_xy_kalman_filter_para.xhat_data[0] = Point_0.x;
		Vision_xy_kalman_filter_para.xhat_data[1] = Point_0.y;
		kalman_filter_init(&Vision_xy_kalman_filter, &Vision_xy_kalman_filter_para);
	}
	if(Auto_vision_flag < 100)Auto_vision_flag ++;
	float *Vision_xy_kf_result = kalman_filter_calc(&Vision_xy_kalman_filter, Point_0.x, Point_0.y);
	if(Auto_vision_flag < 100)
	{
		Point_1.x = Point_0.x;
		Point_1.y = Point_0.y;  	  	
	}
	else
	{
		Point_1.x	=	Vision_xy_kf_result[0];
		Point_1.y = Vision_xy_kf_result[1];
	}
	#endif
}

//----------------------------------------速度提取相关-------------------------------------------------//
float TOP_W_X = 300.0f;
int Pre_cont,//中断分频计数
	  Pre_Cont = 80;//中断分频  延长时间速度效果更明显但延迟更大  
float d_dis_offeset = -400;//测距补偿
extern float Yaw_Base_Location;
short d_Angle_jixing = -1; //极性*系数
float dYaw_angle,     //YAW角 
			d_lastYaw_angle,//上一个yaw角
			dVision_angle,  //视觉角 
			d_Angle,        //绝对角
			d_lastAngle,     //上一次绝对角
			d_lastlastAngle;//上上一次绝对角
float d0_Angle,    //原始绝对角
	    d0_lastAngle,//原始上一次绝对角
			k_d0_Angle = 0.001;
float w_yaw,//解算速度
			q_yaw;//解算加速度
float Tvision_x;               //yaw.x透视投影
float kvision_x = 400.0/5.90f; //yaw.x透视投影系数（实验数据）
float kvision_offx = 0;//40.0f;
float k_yaw = 0.018,k_A_yaw = 1; //滞后 增幅//0.04di 0.02gao
float w_x,w_xlast,k_w_x=0.02;//
float q_x,q_xlast,k_q_x=0.2;//
int Scp_dYaw_angle,Scp_dVision_angle,Scp_d0_Angle,Scp_d_Angle,Scp_w_yaw,Scp_w_x;
void Pre_interupt_wsol(void)
{
	//透视畸变处理
	Tvision_x = (Point_1.z <= 5000) ? Point_1.x-kvision_offx : atan((Point_1.x-kvision_offx)/Point_1.z) / PI *180.0f * kvision_x; 
	//计算yaw电机角度（相对于世界坐标系正前方绝对角度）
	dYaw_angle =(panduan_Z <= parol_k)? 0:((GetYawLocation() - Yaw_Base_Location)*360.f*k_yaw + d_lastYaw_angle * (1-k_yaw))*k_A_yaw;
	d_lastYaw_angle = dYaw_angle;
	//计算视觉角度（相对于视觉坐标系）
	dVision_angle =(Point_1.z <= 0.005)?0: atan(Tvision_x/(Point_1.z+d_dis_offeset))/ PI * 180.0f;
	//计算世界坐标系绝对角度
	d0_Angle = dYaw_angle + d_Angle_jixing*dVision_angle;
	//原始绝对角一次低通滤波得绝对角
	d_Angle = d0_Angle * k_d0_Angle + d0_lastAngle * (1 - k_d0_Angle);
	d0_lastAngle = d_Angle;
	//分频，做计数累计差分 此处每Pre_Cont次计算一次差分
	if(Pre_cont < Pre_Cont) Pre_cont++;
	else 
	{
		//角速度差分
		w_yaw = d_Angle - d_lastAngle;
		//角加速度差分
		q_yaw = d_Angle - 2.0*d_lastAngle + d_lastlastAngle;
		d_lastAngle = d_Angle;
		d_lastlastAngle = d_lastAngle;
		Pre_cont = 0;
	}
	//一次低通滤波
	w_x = w_yaw * k_w_x + w_xlast * (1-k_w_x);
	w_x = ( fabs(w_x) < TOP_W_X ) ? w_x : TOP_W_X;//限幅
	w_xlast = w_x;
	q_x = q_yaw * k_q_x + q_xlast * (1-k_q_x);
	q_xlast = q_x;
	//观测
	Scp_dYaw_angle = (int)(dYaw_angle*10.0f);
	Scp_dVision_angle = (int)(dVision_angle*10.0f);
	Scp_d0_Angle = (int)(d0_Angle*10.0f);
	Scp_d_Angle = (int)(d_Angle*10.0f);
	Scp_w_yaw = (int)(w_yaw*10.0f);
	Scp_w_x = (int)(w_x*10.0f);
}

void Vision_resV_kalman_dispose(void)  //卡尔曼速度数据处理
{
	//x = [v,a]
	#if 1	//卡尔曼滤波
	if(Auto_vision_flag2 == 0)
	{		
		Vision_yaw_v_kalman_filter_para.xhat_data[0] = w_x;
		Vision_yaw_v_kalman_filter_para.xhat_data[1] = q_x;
		kalman_filter_init(&Vision_yaw_v_kalman_filter, &Vision_yaw_v_kalman_filter_para);
	}
	if(Auto_vision_flag2 < 100)Auto_vision_flag2 ++;
	float *Vision_yawV_kf_result = kalman_filter_calc(&Vision_yaw_v_kalman_filter, w_x, q_x);
	if(Auto_vision_flag2 < 100)
	{
		kal_w_yaw = w_x;  	
		Scp_kal_w = (int)(kal_w_yaw*10.0f);
	}
	else
	{
		kal_w_yaw	=	Vision_yawV_kf_result[0];
		Scp_kal_w = (int)(kal_w_yaw*10.0f);
	}
	#endif
}

void Vision_yaw_kalman_dispose(void)  //卡尔曼yaw数据处理(叠加速度)
{
	//x = [p, v];
	float   yawvision_position,yawvision_velocity;
	yawvision_position = Point_1.x;
  yawvision_velocity = yaw_speed_f;
	#if 1	//卡尔曼滤波
	if(Auto_vision_flag1 == 0)
	{		
		Vision_yaw_kalman_filter_para.xhat_data[0] = yawvision_position;
		Vision_yaw_kalman_filter_para.xhat_data[1] = yawvision_velocity;
		kalman_filter_init(&Vision_yaw_kalman_filter, &Vision_yaw_kalman_filter_para);
	}
	if(Auto_vision_flag1 < 100)Auto_vision_flag1 ++;
	float *Vision_yaw_kf_result = kalman_filter_calc(&Vision_yaw_kalman_filter, yawvision_position, yawvision_velocity);
	if(Auto_vision_flag1 < 100)
	{
		Point3f.x = yawvision_position;  	
	}
	else
	{
		Point3f.x	=	Vision_yaw_kf_result[0];
	}
	#endif
}
float disdis;
int Draw_disdis;
void Vision_z_kalman_dispose(void)    //卡尔曼z处理
{
	//x = [z, 0];
	float   dis_z;
	dis_z = Point_0.z;
	#if 1	//卡尔曼滤波
	if(Auto_vision_flag3 == 0)
	{		
		Vision_z_kalman_filter_para.xhat_data[0] = dis_z;
		Vision_z_kalman_filter_para.xhat_data[1] = 0;
		kalman_filter_init(&Vision_z_kalman_filter, &Vision_z_kalman_filter_para);
	}
	if(Auto_vision_flag3 < 500)Auto_vision_flag3 ++;
	float *Vision_z_kf_result = kalman_filter_calc(&Vision_z_kalman_filter, dis_z, 0);
	if(Auto_vision_flag3 < 500)
	{
		disdis = dis_z; 
		Point_1.z = dis_z;
	}
	else
	{
		disdis	=	Vision_z_kf_result[0];
		Point_1.z = Vision_z_kf_result[0];
	}
	Draw_disdis = (int)(disdis);
	#endif
}

//-----------------------------------视界限制处理--------------------------------------------------------//
double limit_dis;//合理限制距离
double dis_shexiangji = 3500;//不同摄像机畸变补偿
void Shijielimit_deal(void)
{
	//整理各角度极限近或远的距离，MATLAB拟合
  //pitch=[0.7 0.699 0.698 0.697 (0.696)min 0.695 0.694 0.693 0.692 0.69 0.688 0.686 0.684 0.68 0.675];
  //dis = [1650 1648 1630 1615 (1596)min 1616 1628 1655 1668 1720 1770 1810 1840 1934 2057];
	float y_loca = GetPitchLocation();
	//float limit_dis;//合理限制距离
	float dH = 1300-800,//轨道高度补偿
		    dH_L;//距离补偿
	dH_L = dH/tan((y_loca - HORIZON_PITCH)*2.0*3.141592);
	limit_dis = (-2214.3*pow(y_loca,2) +3105.2*y_loca -1087.0)*1000 + 1500 + dH_L + dis_shexiangji;
	if ((y_loca>0.568)&&(panduan_Z>limit_dis)) panduan_Z = 4500; 
//		VisionGimbalCtrl.offset_pitch = 0.5;
//	else VisionGimbalCtrl.offset_pitch = 0;
	
}