#include "task_friction.h"
#include "task_gimbal.h"
extern int lose_flag;//ң������ʧ���ݲ�����
extern int close_flag;//ң�����ؿز�����
u8 InitFlag=1;
int PWMNUM=500;

int R_fac = 700,
	  L_fac = 700;
extern GimbalCtrlStruct GimbalCtrl;
void FrictionControlTask(void)
{

	if (InitFlag)
	{
//		for (int i =0;i<2;i++)
//		{
//			LL_TIM_OC_SetCompareCH1(TIM2, 800);
//			LL_TIM_OC_SetCompareCH2(TIM2, 800);
//			HAL_Delay(4000);
//			LL_TIM_OC_SetCompareCH1(TIM2, 1200);
//			LL_TIM_OC_SetCompareCH2(TIM2, 1200);
//		}
//		LL_TIM_OC_SetCompareCH1(TIM2, 800);
//		LL_TIM_OC_SetCompareCH2(TIM2, 800);
		InitFlag=0;
	}
	
	
	//�ȴ������ʼ�����Ȼ��ʼ����
	FrictionControl(lose_flag&close_flag);
	
	//Ħ���ֲ�ת����
	//FrictionControl(0);

}