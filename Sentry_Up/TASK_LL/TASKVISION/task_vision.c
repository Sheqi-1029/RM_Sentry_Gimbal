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
#define HORIZON_PITCH (0.509)  //ˮƽ��Ӧpitch������
#define ZERO_YAW   (0.25) //yaw ��λ

/****************************************************
*			�빤�ػ���ͨ��Э������ʽ
      ʹ�õĶ�ʱ����time7
      ͨ�ż����20ms
			ÿ20ms�ᴦ��һ���Ӿ����͵����ݣ����һᷢ��һ�����ݸ����ػ�
      �Ӿ������
      ��ͷ1 0x5A
			��ͷ2 0xA5
      �Ƕ�1��pitch��    32λ   
      �Ƕ�2 ��yaw��   32λ
      ����״̬   16λ
      ��չ   16λ ������ȫ��0
      ��β0xff




      ��ظ��Ӿ�
//      ��ͷ1 0x3A
//			��ͷ2 0xA3
      ��̨λ�ã����� yaw16λ��pitch16λ.
      ��̨�ƶ��ٶȣ�����vx16λ.vy16λ.vz16λ   
      ����״̬8λ 
      ��չ32λ ������ȫ��0 
      ��β0xff

      ����״̬���Ƽ�����������1������ 2������ 3�����
      �ڱ�  1��Ѳ��  2������  3������   
      
*			
*****************************************************/
//-------------�������˲���ر����趨-------------------//	
kalman_filter_init_t Vision_xy_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 0, 0, 1},//״̬ת�ƾ���
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {10, 0, 0, 10}//��ȷ���������
};
kalman_filter_t Vision_xy_kalman_filter={0};

kalman_filter_init_t Vision_yaw_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 1, 0, 1},//״̬ת�ƾ���
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {20, 0, 0, 5000}//��ȷ���������
};

kalman_filter_t Vision_yaw_kalman_filter={0};


kalman_filter_init_t Vision_yaw_v_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 100, 0, 1},//״̬ת�ƾ���
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {10, 0, 0, 1}//��ȷ���������
};
kalman_filter_t Vision_yaw_v_kalman_filter={0};

kalman_filter_init_t Vision_z_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 0, 0, 1},//״̬ת�ƾ���
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {2000, 0, 0, 1}//��ȷ���������
};
kalman_filter_t Vision_z_kalman_filter={0};

//--------------Significant�����������-----------------//
extern u8 VisionRXFlag;   
extern uint8_t  UsbRxBuf[13];
extern uint8_t UsbTxBuf[17];
extern u8 Chassis_rebuf[8];
VisionDataStruct   VisionData;
float Set_ChangePitch,Set_ChangeYaw;
extern PID VisionPitchIncreasement , VisionYawIncreasement ;
Point3fStruct Point_0,//��Ȩһ���˲�ֵ
							Point_1,//1�ο������˲�ֵ
              Point3f;//����ֵ
extern float Command_X,Command_Y;
UsbSendMsg Unionsend;

//�ٶ����
extern Judgement_data judgement_data; //����ϵͳ��������
short chassis_speed_r,chassis_speed_int = 0;//�����ٶ�
short chassis_speed_last = 0;
float k_chassis_speed = 0.7;//�����ٶ�����ϵ��  ����
float chassis_speed_f;//��������ٶ�

float k_cha_speed = 6.5;//Ŀ���ٶ�����ϵ��  //40.0
float cha_speed_f = 0;//����Ŀ���ٶ�
float yaw_speed_f0,yaw_speed_f,last_yaw_speed_f,//���ٶ�
			yaw_lim_minV = 100;//�ٶ���Сֵ���޷��˲���ֵ
//������ʱǰ��
float Delay_con_t = 0.0;//������ʱ
double Delay_bullet_t = 0.0;
float  Delay_time;//��ʱ = ������ʱ+����/�ٶ�

float k_kal_yaw_v = 0.95;
float kal_w_yaw,//�������˲����ٶ�
			kal_q_yaw;//�������˲����ٶ�
//����ϵ��
float PITCH_SIGHT_AREA=0.00005,YAW_SIGHT_AREA=-0.00003; //����ģ�ͽ�������һ
Der pitchder,yawder;

float parol_k = 5;

float panduan_Z = 0;

float shoot_comarea = 1.5;

K_Lowpass_Point K_Low = {0.007,0.5,0.007,0.2,0.01,0.7};//0.01,0.95,0.01,0.8,0.01,0.1

//��������־λ
int Auto_vision_flag = 0,
		Auto_vision_flag1 = 0,
		Auto_vision_flag2 = 0,
		Auto_vision_flag3 = 0;

//--------------Speed ��ȡ����-----------------//
int Scp_kal_w;

//------------------------------------------------------//

void Vision_kalman_init(void)  //������������ʼ��
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
	//---------------------�����ٶȽ���--------------------
	chassis_speed_r = (short)(((Chassis_rebuf[0]<<8)&0xff00)|Chassis_rebuf[1]);
	chassis_speed_int = 0.7*chassis_speed_r + 0.3*chassis_speed_last;
	k_chassis_speed = (chassis_speed_r>0)?0.6:0.4;
	chassis_speed_f = abs(chassis_speed_int-chassis_speed_last) > 100 ? 0.0f:(float)(chassis_speed_int * k_chassis_speed);
	
	Draw_chassis_speed = (int)(chassis_speed_r*10.0f);
	
	//--------------------���ػ��ٶȽ���-------------------
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
	
	#if 1   //�۲�---------------
		DRAW_XR = (int)((((float*)&hor)[0])*10.0f);
		DRAW_YR = (int)((((float*)&ver)[0])*10.0f);
	  DRAW_ZR = (int)((((float*)&dis)[0])*10.0f);
	#endif
	
	//---��ͨ�˲� ȥ��©֡ �⻬ �ӳ���λ-----------------------
	//�������
	float k_z = VisionData.ReceiveDistanceRaw < 0.01 ? K_Low.kz_0 : K_Low.kz_1;
  panduan_Z	= VisionData.ReceiveDistanceRaw * k_z + panduan_Z * (1 - k_z);//һ���˲�����ֵ������Ŀ��ʶ���Ӿ��ȶ��ԣ����ھ��߻���
	//Point_0.z = VisionData.ReceiveDistanceRaw * k_z + VisionData.ReceiveDistance_last * (1 - k_z);
	
	//�����벻����ױ���,�˷�С���ݣ�Զ��������ױ���
	if(panduan_Z >6800)
	{
		VisionData.ReceiveDistanceRaw = (VisionData.ReceiveDistanceRaw == 0)?VisionData.ReceiveDistance_last:VisionData.ReceiveDistanceRaw; 
		VisionData.ReceiveYaw = (VisionData.ReceiveYaw == 0) ? VisionData.ReceiveYaw_last : VisionData.ReceiveYaw;  
		VisionData.ReceivePitch = (VisionData.ReceivePitch == 0) ? VisionData.ReceivePitch_last : VisionData.ReceivePitch;
	}
	
	Point_0.z = VisionData.ReceiveDistanceRaw * k_z + VisionData.ReceiveDistance_last * (1 - k_z);
	
	//��0����ͨ �˷�С����
	float k_x = fabs(VisionData.ReceiveYaw) < 0.01 ? K_Low.kx_0 : K_Low.kx_1;;
	Point_0.x = VisionData.ReceiveYaw * k_x + VisionData.ReceiveYaw_last * (1-k_x);

	float k_y = fabs(VisionData.ReceivePitch) < 0.01 ? K_Low.ky_0 : K_Low.ky_1;
	Point_0.y = VisionData.ReceivePitch * k_y + VisionData.ReceivePitch_last * (1-k_y);
	
	//---�޷�--------------------------------------------------
	Point_0.x = (Point_0.x > 10000)? 10000:Point_0.x;
	Point_0.y = (Point_0.y > 10000)? 10000:Point_0.y;
	Point_0.x = (Point_0.x < -10000)? -10000:Point_0.x;
	Point_0.y = (Point_0.y < -10000)? -10000:Point_0.y;
	
	#if 1   //�۲�---------------
		DRAW_PAN_Z = (int)((panduan_Z)*10.0f);
		DRAW_Z0 = (int)((Point_0.z)*10.0f);
		DRAW_X0 = (int)((Point_0.x)*10.0f);
		DRAW_Y0 = (int)((Point_0.y)*10.0f);
	#endif
	
	//---һ���˲�ֵ�������˲�---------------------------------------------
	Vision_xy_kalman_dispose();//�������˲� ��x,y���� ������
	Vision_z_kalman_dispose();//�������˲� ��z���� ������
	
	#if 1   //�۲�---------------
		DRAW_X1 = (int)((Point_1.x)*10.0f);
		DRAW_Y1 = (int)((Point_1.y)*10.0f);
	#endif
	//---Ŀ���ٶȴ���-------------------------------------------
	Pre_interupt_wsol();                    //ͨ����Ƶ�����ȡĿ������ٶ� ��ͨ�⻬ �ӳ���λ ����
	Vision_resV_kalman_dispose();           //���������� ��ǰ��λ ���� Point_1.x ת�� Point3f.x ��ȡ���ٶ�Ϊ���ٶ�
	//���ٶ�ת���ٶ�
	yaw_speed_f0 = tan(kal_w_yaw/180.0*PI)*Point_1.z;
	//�޷���ͨ�˲�
	k_kal_yaw_v = (fabs(yaw_speed_f0)<yaw_lim_minV) ? 0.001:0.95;
	yaw_speed_f = k_kal_yaw_v * yaw_speed_f0 + (1-k_kal_yaw_v)*last_yaw_speed_f;  //Ԥ���ٶ�ֵ��ͨȥ����Ƶ���
	last_yaw_speed_f = yaw_speed_f;
	#if 1
		Scp_yaw_speed_f = (int)(yaw_speed_f*10.0f);
	#endif
	//k_cha_speed = 10;//(yaw_speed_f<0)?9.5:7;//9.5 7   //�����ٶ���ǰ��
	cha_speed_f = k_cha_speed * yaw_speed_f;  //���뵯����Ϊǰ������һ����֪��Ҫ��Ҫ�ӣ�ҪȨ��˴���λ��ǰ���ں��������������λ����ǰ��
	
	//Point_1.y/z ת�� Point3f.y/z 
	Point3f.y = Point_1.y;
	Point3f.z = Point_1.z;
	//---yaw������ ��ǰ��λ ���� ------------��Point_1.x��������λ�����ڴ��ͺ�����  Point3f.xΪ��λ��ǰ��λ ���Խ��бȽϵ���
	Vision_yaw_kalman_dispose();  //�õ�����Point3f.x��ֵ
		
	//�޷�	
	Point3f.x = (Point3f.x > 10000)? 10000:Point3f.x;
	Point3f.y = (Point3f.y > 10000)? 10000:Point3f.y;
	Point3f.z = (Point3f.z > 15000)? 15000:Point3f.z;
	Point3f.x = (Point3f.x < -10000)? -10000:Point3f.x;
	Point3f.y = (Point3f.y < -10000)? -10000:Point3f.y;	
	Point3f.z = (Point3f.z < 0)? 0:Point3f.z;
	
	//---�ӵ������ӳٴ���---------------------------------------
	Delay_bullet_t = VisionData.Time_bullet_fly;
  Delay_time = (float)(Delay_bullet_t + Delay_con_t);
	
	//---�ӽ紦��-----------------------------------------------
  //Shijielimit_deal();
	
	//---�����ж�-----------------------------------------------
	if (panduan_Z <= parol_k)//VisionData.ReceiveDistanceRaw == 0 )
	{		
			VisionData.Command = 3;
			//����������
			Point_1.x = 0;Point_1.y = 0;Point_1.z = 0;
			Point3f.x = 0;Point3f.y = 0;Point3f.z = 0;
			//��Ԥ�⿨������־λ����
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
	
	//pitch ƫ����
	/*
	dis=3000   -0.8f
	dis=6000   -0.5f
	dis=8000   -1.2f
	*/
	if(Point3f.z < 3000.0f) VisionGimbalCtrl.offset_pitch = -0.8f;
	else if((Point3f.z > 3000.0f)&&(Point3f.z < 6000.0f))  VisionGimbalCtrl.offset_pitch = 0.1*(Point3f.z/1000.0f - 3) - 0.8f;
	else if((Point3f.z > 6000.0f)&&(Point3f.z < 8000.0f))  VisionGimbalCtrl.offset_pitch = -0.35*(Point3f.z/1000.0f - 6) - 0.5f;
	else if(Point3f.z > 8000.0f) VisionGimbalCtrl.offset_pitch = -1.2f;
	//yaw ƫ����
	if(Point3f.z < 6000.0f) VisionGimbalCtrl.offset_yaw = 0.3f;
	else if(Point3f.z > 8000.0f) VisionGimbalCtrl.offset_yaw = 0.8f;
	else VisionGimbalCtrl.offset_yaw = 0.25*(Point3f.z/1000.0f - 6) + 0.3;
	
	//---����ģ��---------------------------------------------------
	if (Point3f.z > 200)
	{
		Transform(Point3f,GetPitchLocation(),GetYawLocation(),
							&Set_ChangePitch,&Set_ChangeYaw,&Delay_time,chassis_speed_f+cha_speed_f,0);
  }	
	
	//---����task_gimbal ͨ��ײ�-----------------------------------
  Set_ChangePitch *= PITCH_SIGHT_AREA ;
	Set_ChangeYaw *= YAW_SIGHT_AREA ;
	
	//Ŀ��ֵΪ����  ��ֵ֪Ϊ��ֵ 
	//---��Pitch����PID����
	VisionPitchIncreasement.Ref=Set_ChangePitch;
		//����һ��
	VisionPitchIncreasement.Fdb=(getPitchSetLocation()-GetPitchLocation())*(Set_ChangePitch);
															//(Set_ChangePitch==0)?
															//0:(getPitchSetLocation()-GetPitchLocation())*(Set_ChangePitch/fabs(Set_ChangePitch));//*Set_ChangePitch
	VisionPitchIncreasement.calc(&VisionPitchIncreasement);
	#if 1 //����΢��
	pitchder.eer = getPitchSetLocation()-GetPitchLocation();
	pitchder.der = pitchder.eer - pitchder.lasteer;
	pitchder.lasteer = pitchder.eer;
	pitchder.Out = pitchder.der * pitchder.kd;
	#endif
	VisionData.Change_pitch=VisionPitchIncreasement.Out + pitchder.Out;
	
	//---��Yaw����PID����  
	VisionYawIncreasement.Ref=Set_ChangeYaw;
		//����һ��
	VisionPitchIncreasement.Fdb=(Set_ChangeYaw==0)?
																0:(getYawSetLocation()-GetYawLocation())*(Set_ChangeYaw/fabs(Set_ChangeYaw));//*Set_ChangeYaw
	VisionYawIncreasement.calc(&VisionYawIncreasement);
	#if 1 //����΢��
	yawder.eer = getYawSetLocation()-GetYawLocation();
	yawder.der = yawder.eer - yawder.lasteer;
	yawder.lasteer = yawder.eer;
	yawder.Out = yawder.der * yawder.kd;
	#endif
	VisionData.Change_yaw=VisionYawIncreasement.Out + yawder.Out;
 
	//---��һ�����ݼ�¼----------------------------------------------
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
	
    for (int i = 0; i < 4; ++i){                //pitch��
        UsbTxBuf[i] = Unionsend.buffer[i];			
    }
    for (int i = 0; i < 4; ++i){                //yaw��
				UsbTxBuf[i+4] = Unionsend.buffer[i+4];
		}
		for (int i = 0; i < 4; ++i){                //bullet��               
        UsbTxBuf[i+8] = Unionsend.buffer[i+8];
		}
		UsbTxBuf[12] = TARGET_COLOUR;

	CDC_Transmit_FS(UsbTxBuf, 13);
	
}

void Vision_xy_kalman_dispose(void)  //������xy�˲������ݴ���
{
	//x = [x, y];
	#if 1	//�������˲�
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

//----------------------------------------�ٶ���ȡ���-------------------------------------------------//
float TOP_W_X = 300.0f;
int Pre_cont,//�жϷ�Ƶ����
	  Pre_Cont = 80;//�жϷ�Ƶ  �ӳ�ʱ���ٶ�Ч�������Ե��ӳٸ���  
float d_dis_offeset = -400;//��ಹ��
extern float Yaw_Base_Location;
short d_Angle_jixing = -1; //����*ϵ��
float dYaw_angle,     //YAW�� 
			d_lastYaw_angle,//��һ��yaw��
			dVision_angle,  //�Ӿ��� 
			d_Angle,        //���Խ�
			d_lastAngle,     //��һ�ξ��Խ�
			d_lastlastAngle;//����һ�ξ��Խ�
float d0_Angle,    //ԭʼ���Խ�
	    d0_lastAngle,//ԭʼ��һ�ξ��Խ�
			k_d0_Angle = 0.001;
float w_yaw,//�����ٶ�
			q_yaw;//������ٶ�
float Tvision_x;               //yaw.x͸��ͶӰ
float kvision_x = 400.0/5.90f; //yaw.x͸��ͶӰϵ����ʵ�����ݣ�
float kvision_offx = 0;//40.0f;
float k_yaw = 0.018,k_A_yaw = 1; //�ͺ� ����//0.04di 0.02gao
float w_x,w_xlast,k_w_x=0.02;//
float q_x,q_xlast,k_q_x=0.2;//
int Scp_dYaw_angle,Scp_dVision_angle,Scp_d0_Angle,Scp_d_Angle,Scp_w_yaw,Scp_w_x;
void Pre_interupt_wsol(void)
{
	//͸�ӻ��䴦��
	Tvision_x = (Point_1.z <= 5000) ? Point_1.x-kvision_offx : atan((Point_1.x-kvision_offx)/Point_1.z) / PI *180.0f * kvision_x; 
	//����yaw����Ƕȣ��������������ϵ��ǰ�����ԽǶȣ�
	dYaw_angle =(panduan_Z <= parol_k)? 0:((GetYawLocation() - Yaw_Base_Location)*360.f*k_yaw + d_lastYaw_angle * (1-k_yaw))*k_A_yaw;
	d_lastYaw_angle = dYaw_angle;
	//�����Ӿ��Ƕȣ�������Ӿ�����ϵ��
	dVision_angle =(Point_1.z <= 0.005)?0: atan(Tvision_x/(Point_1.z+d_dis_offeset))/ PI * 180.0f;
	//������������ϵ���ԽǶ�
	d0_Angle = dYaw_angle + d_Angle_jixing*dVision_angle;
	//ԭʼ���Խ�һ�ε�ͨ�˲��þ��Խ�
	d_Angle = d0_Angle * k_d0_Angle + d0_lastAngle * (1 - k_d0_Angle);
	d0_lastAngle = d_Angle;
	//��Ƶ���������ۼƲ�� �˴�ÿPre_Cont�μ���һ�β��
	if(Pre_cont < Pre_Cont) Pre_cont++;
	else 
	{
		//���ٶȲ��
		w_yaw = d_Angle - d_lastAngle;
		//�Ǽ��ٶȲ��
		q_yaw = d_Angle - 2.0*d_lastAngle + d_lastlastAngle;
		d_lastAngle = d_Angle;
		d_lastlastAngle = d_lastAngle;
		Pre_cont = 0;
	}
	//һ�ε�ͨ�˲�
	w_x = w_yaw * k_w_x + w_xlast * (1-k_w_x);
	w_x = ( fabs(w_x) < TOP_W_X ) ? w_x : TOP_W_X;//�޷�
	w_xlast = w_x;
	q_x = q_yaw * k_q_x + q_xlast * (1-k_q_x);
	q_xlast = q_x;
	//�۲�
	Scp_dYaw_angle = (int)(dYaw_angle*10.0f);
	Scp_dVision_angle = (int)(dVision_angle*10.0f);
	Scp_d0_Angle = (int)(d0_Angle*10.0f);
	Scp_d_Angle = (int)(d_Angle*10.0f);
	Scp_w_yaw = (int)(w_yaw*10.0f);
	Scp_w_x = (int)(w_x*10.0f);
}

void Vision_resV_kalman_dispose(void)  //�������ٶ����ݴ���
{
	//x = [v,a]
	#if 1	//�������˲�
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

void Vision_yaw_kalman_dispose(void)  //������yaw���ݴ���(�����ٶ�)
{
	//x = [p, v];
	float   yawvision_position,yawvision_velocity;
	yawvision_position = Point_1.x;
  yawvision_velocity = yaw_speed_f;
	#if 1	//�������˲�
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
void Vision_z_kalman_dispose(void)    //������z����
{
	//x = [z, 0];
	float   dis_z;
	dis_z = Point_0.z;
	#if 1	//�������˲�
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

//-----------------------------------�ӽ����ƴ���--------------------------------------------------------//
double limit_dis;//�������ƾ���
double dis_shexiangji = 3500;//��ͬ��������䲹��
void Shijielimit_deal(void)
{
	//������Ƕȼ��޽���Զ�ľ��룬MATLAB���
  //pitch=[0.7 0.699 0.698 0.697 (0.696)min 0.695 0.694 0.693 0.692 0.69 0.688 0.686 0.684 0.68 0.675];
  //dis = [1650 1648 1630 1615 (1596)min 1616 1628 1655 1668 1720 1770 1810 1840 1934 2057];
	float y_loca = GetPitchLocation();
	//float limit_dis;//�������ƾ���
	float dH = 1300-800,//����߶Ȳ���
		    dH_L;//���벹��
	dH_L = dH/tan((y_loca - HORIZON_PITCH)*2.0*3.141592);
	limit_dis = (-2214.3*pow(y_loca,2) +3105.2*y_loca -1087.0)*1000 + 1500 + dH_L + dis_shexiangji;
	if ((y_loca>0.568)&&(panduan_Z>limit_dis)) panduan_Z = 4500; 
//		VisionGimbalCtrl.offset_pitch = 0.5;
//	else VisionGimbalCtrl.offset_pitch = 0;
	
}