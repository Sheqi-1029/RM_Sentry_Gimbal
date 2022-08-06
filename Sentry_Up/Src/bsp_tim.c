#include "bsp_tim.h"
#include "BSPconfig.h"
#include "stm32f4xx_hal.h"

void UserTim1Config(void)//Ä¦²ÁÂÖ
{
	//TIM1-CH2 CONFIG BSGIN
//	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
//	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//	
//	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
//  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
//  TIM_OC_InitStruct.CompareValue = 999;
//  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
//  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
//  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
//  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
//	
//	LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
//	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
//	LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
//	
//	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//TIM1-CH2 CONFIG END
	
//	TIM1->CCER |=TIM_CCER_CC1E ;
//	TIM1->CCER |=TIM_CCER_CC2E ;
//	TIM1->CCER |=TIM_CCER_CC3E ;
//	TIM1->CCER |=TIM_CCER_CC4E ;
//	
//	LL_TIM_EnableARRPreload(TIM1);
//	LL_TIM_EnableAllOutputs(TIM1);
//	
//	LL_TIM_EnableCounter(TIM1); 
}

void UserTim3Config(void)
{
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_EnableCounter(TIM3);
}

void UserTim8Config(void)
{
	TIM8->CCER |=TIM_CCER_CC3E ;
	TIM8->CCER |=TIM_CCER_CC4E ;

	LL_TIM_EnableARRPreload(TIM8);
	LL_TIM_EnableAllOutputs(TIM8);

	LL_TIM_EnableCounter(TIM8);
}

void UserTim2Config(void)
{
	TIM2->CCER |=TIM_CCER_CC1E ;
	TIM2->CCER |=TIM_CCER_CC2E ;

	LL_TIM_EnableARRPreload(TIM2);
	LL_TIM_EnableAllOutputs(TIM2);

	LL_TIM_EnableCounter(TIM2);
}

void UserTim12Config(void)
{
	TIM12->CCER |=TIM_CCER_CC1E ;
	TIM12->CCER |=TIM_CCER_CC2E ;

	LL_TIM_EnableARRPreload(TIM12);
	LL_TIM_EnableAllOutputs(TIM12);

	LL_TIM_EnableCounter(TIM12);
		LL_TIM_OC_SetCompareCH1(TIM2, 750);
	LL_TIM_OC_SetCompareCH2(TIM2, 750);	
	LL_TIM_OC_SetCompareCH1(TIM12, 750);
	LL_TIM_OC_SetCompareCH2(TIM12, 750);

}