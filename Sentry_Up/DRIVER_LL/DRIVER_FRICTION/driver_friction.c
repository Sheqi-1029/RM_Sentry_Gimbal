#include "driver_friction.h"
#include "ramp.h"
#include "task_gimbal.h"

#define FRI_FAC_MAX        (680)

#define GAME_TIAOSHIMODE       (0)//0调试，1比赛

#define FRI_FAC_MIN        (800)

#if GAME_TIAOSHIMODE
#define FRI_FAC_MAX_RIGHT  (600)
#define FRI_FAC_MAX_LEFT   (600)
#endif

#if !GAME_TIAOSHIMODE
int FRI_FAC_MAX_RIGHT = 1200;
int FRI_FAC_MAX_LEFT = 1200;
#endif

ramp_t friction_startarmour_ramp = RAMP_GEN_DAFAULT;//右启动斜坡
ramp_t friction_stoparmour_ramp = RAMP_GEN_DAFAULT;//右停止斜坡
int friction_initialrampstart_flag = 1;//启动斜坡标志
int friction_initialrampstop_flag = 1;//停止斜坡标志
int fri_fac_init_right;//右临时初始值
int fri_fac_init_left; //左临时初始值
int fri_fac_right = 500;//右实时跟踪pwm变量
int fri_fac_left = 500;//右实时跟踪pwm变量
int fri_initstart_flag = 1;//启动标识符
int fri_initstop_flag = 1;//停止标识符
extern Judgement_data judgement_data; //裁判系统接收数据

//-------------------------------------------停止函数-----------------------------------------//
void FrictionStop(void)
{
	if ( fri_initstop_flag ==1 )
	{
		fri_fac_init_right = fri_fac_right;
		fri_fac_init_left = fri_fac_left;
		friction_stoparmour_ramp.init(&friction_stoparmour_ramp,1000);
		fri_initstop_flag = 0;
	}
	if (friction_initialrampstop_flag)
	{
		float sensive = friction_stoparmour_ramp.calc(&friction_stoparmour_ramp);
		if ( sensive < 1)
		{
			fri_fac_right = fri_fac_init_right + sensive*(FRI_FAC_MIN - fri_fac_init_right);
			fri_fac_left = fri_fac_init_left + sensive*(FRI_FAC_MIN - fri_fac_init_left);
			FrictionDriver(fri_fac_right,fri_fac_left);
		}
		else
		{
			fri_fac_right = fri_fac_init_right + sensive*(FRI_FAC_MIN - fri_fac_init_right);
			fri_fac_left = fri_fac_init_left + sensive*(FRI_FAC_MIN - fri_fac_init_left);
			FrictionDriver(fri_fac_right,fri_fac_left);
			friction_initialrampstop_flag = 0;
		}
	}
}

//-------------------------------------------启动函数-----------------------------------------//
void FrictionStart(void)
{
	if ( fri_initstart_flag ==1 )
	{
		fri_fac_init_right = fri_fac_right;
		fri_fac_init_left = fri_fac_left;
		friction_startarmour_ramp.init(&friction_startarmour_ramp,1000);
		fri_initstart_flag = 0;
	}
	if (friction_initialrampstart_flag)
	{
		float sensive = friction_startarmour_ramp.calc(&friction_startarmour_ramp);
		if (sensive < 1)
		{
			fri_fac_right = fri_fac_init_right + sensive*(FRI_FAC_MAX_RIGHT - fri_fac_init_right);
			fri_fac_left = fri_fac_init_left + sensive*(FRI_FAC_MAX_LEFT - fri_fac_init_left);
			FrictionDriver(fri_fac_right,fri_fac_left);
		}
		else
		{
			fri_fac_right = fri_fac_init_right + sensive*(FRI_FAC_MAX_RIGHT - fri_fac_init_right);
			fri_fac_left = fri_fac_init_left + sensive*(FRI_FAC_MAX_LEFT - fri_fac_init_left);
			FrictionDriver(fri_fac_right,fri_fac_left);
			friction_initialrampstart_flag = 0;
		}
	}
}
//-------------------------------------------驱动函数-----------------------------------------//
void FrictionDriver(int FRAR,int FRAL) 
{
	LL_TIM_OC_SetCompareCH1(TIM2, FRAR);
	LL_TIM_OC_SetCompareCH2(TIM2, FRAL);
}

extern u8 Initial_flag;//初次开控识别布尔符

u8 test_judge_bullet;
//-------------------------------------------主控函数-----------------------------------------//
void FrictionControl(u8 Judge)
{
	test_judge_bullet = 1 - judgement_data.bullet_bool;
	if(Judge && test_judge_bullet)// && judgement_data.bullet_bool)
	{
		FrictionStart();
		fri_initstop_flag = 1;
		friction_initialrampstop_flag = 1;
	}
	else
	{
		FrictionStop();
		fri_initstart_flag = 1;
		friction_initialrampstart_flag = 1;
	}
}


uint8_t Getfriction_Full(void)
{
	if(friction_startarmour_ramp.out == 1)
		return 1;
	else return 0;
}
