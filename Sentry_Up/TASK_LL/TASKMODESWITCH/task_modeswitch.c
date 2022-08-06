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

//����ϵͳ����/��������ģʽ
#define JUDGEMENTGAME  (0)//����1 ����0


//Զ����Ŀ�꣺5m����
#define FAR_TARGET_DISTANCE		(5000)

//�ӵ����ߵ�λ
#define BULLET_FULLNUM         (200)//ʣ������жϱ�׼
#define BULLET_FULL            (2)//ʣ�����
#define BULLET_LOW             (1)//ʣ�಻��
#define BULLET_EMPTY           (0)//û���ӵ�

//ǰ��վѪ�����ߵ�λ  ��Ѫ��2000
#define QIANSHAO_RISKNUM          (1200)//��ȫ�ٽ�Ѫ��
#define QIANSHAO_SAVE              (1)//��ȫ
#define QIANSHAO_RISK              (0)//Σ��

//ң�������λ
#define REMOTE_COMMAND_VOID        (3)//�յ�
#define REMOTE_COMMAND_CTRL        (2)//�ֶ���
#define REMOTE_COMMAND_AUTO        (1)//�Զ���

//ģ�������ж�
#define MODE_SWITCH_TIME_MS (5)

//�Ӿ�����
#define VISION_PATROL        (3)//Ѳ��
#define VISION_TRACK         (1)//����
#define VISION_FIRE          (2)//���


//--------------Significant�����������-----------------//

extern LostCounterStruct lost_counter;
extern RemoteDataUnion RemoteData; //ң��������
extern RemoteDataPortStruct	RemoteDataPort;//ң����ͨ������

extern VisionDataStruct VisionData; //�Ӿ�����
extern Judgement_data judgement_data; //����ϵͳ��������
extern u8 Chassis_rebuf[8];//�������ݽ��գ�



ArmourHurtStruct IsArmourHurt;//ǰ��װ�������ж�

u8 Remote_command;//ң����������
u8 Sentry_comdata;//����ͨ��ָ��

int Bool_isgame,//���������ж�
		Bool_bulletnum,//�ӵ�ʣ�����ж�
	  Bool_qianshaozhan;//ǰ��վѪ���ж�

extern u8 Can2ChassisSendMessage[8];

//-------------------------------------*********�����߼����ƹ���********-------------------------------------//
void General_Control(void)
{
	Remote_command = RemoteData.RemoteDataProcessed.RCValue.s2;
	Sentry_comdata = Chassis_rebuf[2];
	Bool_isgame = IsGameStart();              //���������ж�
	Bool_bulletnum = BulletNum_Full();        //�ӵ�ʣ�����ж�
	Bool_qianshaozhan = Qianshaozhan_Save();  //ǰ��վѪ���ж�
	Armour_bool();
	
	//�յ�
	if(Remote_command == REMOTE_COMMAND_VOID) 
	{
		SetGimbalMode(GIMBAL_RELAX);
		SetFeedMotorMode(FEEDMOTOR_INIT);
		
		Can2ChassisSendMessage[0] = 0x01;
	}
	
	//�ֶ���
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
	
	//�Զ���
	else if(Remote_command == REMOTE_COMMAND_AUTO)
	{
		Can2ChassisSendMessage[0] = 0x00;
		//���ڱ������߻����ڱ���
		if ((judgement_data.game_mode == 0)||(judgement_data.game_mode == 4))
		{					
			if((VisionData.Command == VISION_PATROL)
				||(VisionData.Command == 0))
			{
//          switch(Sentry_comdata)
//					{
//						case 0://��̬Ѳ��	
							SetGimbalMode(GIMBAL_PATROL);
//						break;
//						case 1://���Ѳ��
//							SetGimbalMode(GIMBAL_SEARCH_FRONT);
//						break;
//						case 2://�Ҳ�Ѳ��
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
				AutoShootControl();    //�Զ��������
				#endif
				
				#if (JUDGEMENTGAME == 0)
				if(Getfriction_Full()==1)
					SetFeedMotorMode(FEEDMOTOR_HIGH_FREQ);
				else SetFeedMotorMode(FEEDMOTOR_INIT);
				#endif
			}
		}

		//��ǰ������
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


//-------------------------------------------���������ж�----------------------------------------------
//1��������
//0���Ǳ�����
int IsGameStart(void)							
{
	if(judgement_data.game_mode == 0)  //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�����ģ�⣩
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

//-------------------------------------------װ�״���ж�-----------------------------------------------
void Armour_bool(void)
{
	if(judgement_data.game_mode == 0)  //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�����ģ�⣩
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


//-------------------------------------------���߲����ж�-----------------------------------------------
int BulletNum_Full(void) //�ӵ���׼ ʣ��200
{
	if (judgement_data.game_mode == 0)  //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�����ģ�⣩
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
int Qianshaozhan_Save(void)//ǰ��վѪ����׼ ʣ��1200
{
	if (judgement_data.game_mode == 0)  //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�����ģ�⣩
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

//-------------------------------------------�Զ��������------------------------------------------------
/*
         \�ӵ���      ����>200         ����<200 
ǰ��վѪ��

                    Զ  ��Ƶ           Զ  ����
	Ѫ��       
                    ��  ��Ƶ           ��  ��Ƶ

										Զ  ����           Զ  ����
Ѫ��/����     
                    ��  ��Ƶ           ��  ��Ƶ

*/
void AutoShootControl(void)
{
	if (Bool_isgame == 1)
	{
		//�ӵ����� ǰ��վ��ȫ
		if ((Bool_bulletnum == BULLET_FULL)&&(Bool_qianshaozhan == QIANSHAO_SAVE))
		{
			SetFeedMotorMode(FEEDMOTOR_LOW_FREQ);
		}
		//�ӵ����� ǰ��վ��ȫ
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
		//�ӵ����� ǰ��վΣ��
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
		//�ӵ����� ǰ��վΣ��
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
		//û�ӵ�
		if (Bool_bulletnum == BULLET_EMPTY)
		{
			SetFeedMotorMode(FEEDMOTOR_INIT);
		}
	}
	else                       //��ʧ����ϵͳ��Ϣ����Ϊ�ڱ����У�ʼ�շ���
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

