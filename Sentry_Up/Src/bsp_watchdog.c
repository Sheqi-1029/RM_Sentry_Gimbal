/****************************************************
*			Title:		看门狗
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.10.18
*												HH.
*****************************************************/
#include "bsp_watchdog.h"

/**是否使用看门狗
	* 1  是
	* 0  否
*/
#define WWDG_ENABLE 0
#define IWDG_ENABLE 1

/**参数说明：
***独立看门狗
	*IWDG_PR = 0x04：64分频，计算公式:4*2^prer
	*IWDG_RLR = 625 刚好溢出时间为1s
	*溢出时间计算公式:Tout = (4*2^PR)*RLR/40 (ms)
	*如需改变溢出时间，只需要改变PR值或RLR值即可. 默认为1s
***窗口看门狗
	*STM32F405/407下窗口看门狗最长喂狗间隔为59.93ms 
	*目前并不使用
*/
void WatchDogInit(void)
{
	#if CONFIG_USE_IWDG
	/*独立看门狗配置 喂狗间隔1s*/
	IWDG->KR = (uint16_t)0x5555;
	IWDG->PR = (uint16_t)0x04;
	IWDG->RLR = (uint16_t)625;
	IWDG->KR = (uint16_t)0xAAAA;
	IWDG->KR = (uint16_t)0xCCCC;
	#endif
	
	#if CONFIG_USE_WWDG
	/*窗口看门狗配置*/
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, ENABLE);/*窗口看门狗挂在APB1时钟上，频率42MHz*/
	WWDG_SetPrescaler(WWDG_Prescaler_8);/*看门狗实际频率为APB1的8分频*/
	WWDG_SetWindowValue(0x7F);/*窗口看门狗最大延迟时间对应的窗口值*/
	WWDG_Enable(0x7F);/*输入计数初始值 并使能窗口看门狗*/
	#endif
}

void FeedIndependentWatchDog(void)
{
	IWDG->KR = 0xAAAA;
}

//void FeedWindowWatchDog(void)
//{
////	WWDG_SetCounter((uint8_t)0x7F);
//}
