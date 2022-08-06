/****************************************************
*			Title:		内存检测
*			ChipType:	STM32F405RGT6
*			Version:	1.0.7
*			Date:			2017.10.18
*												SFR.
*****************************************************/
#include "FreeRTOS.h"
#include "task.h"



#if INCLUDE_uxTaskGetStackHighWaterMark

extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t LED0Task_Handler;
extern TaskHandle_t CAN1MessegeProcessTask_Handler;
extern TaskHandle_t ChassisTask_Handler;
extern TaskHandle_t FeedMotorTask_Handler;
extern TaskHandle_t RemoteDataUpdateTask_Handler;
extern TaskHandle_t LostCounterTask_Handler;
extern TaskHandle_t MPU6050Task_Handler;
extern TaskHandle_t GimbalTask_Handler;
						
UBaseType_t ChassisTask_Handler_512;
UBaseType_t GimbalTask_Handler_512;
UBaseType_t FeedMotorTask_Handler_512;
UBaseType_t LED0Task_Handler_50;
UBaseType_t RemoteDataUpdateTask_Handler_512;
UBaseType_t LostCounterTask_Handler_64;
UBaseType_t MPU6050Task_Handler_512;
UBaseType_t StartTask_Handler_128;

//测试任务 已将INCLUDE_uxTaskGetStackHighWaterMark置1，测试完要置0;

void HighWater_task(void *pvParameters)
{ 
	while(1)
	{
		ChassisTask_Handler_512=uxTaskGetStackHighWaterMark(ChassisTask_Handler);
		GimbalTask_Handler_512=uxTaskGetStackHighWaterMark(GimbalTask_Handler);
		FeedMotorTask_Handler_512=uxTaskGetStackHighWaterMark(FeedMotorTask_Handler);
		LED0Task_Handler_50=uxTaskGetStackHighWaterMark(LED0Task_Handler);
		RemoteDataUpdateTask_Handler_512=uxTaskGetStackHighWaterMark(RemoteDataUpdateTask_Handler);
		LostCounterTask_Handler_64=uxTaskGetStackHighWaterMark(LostCounterTask_Handler);
		MPU6050Task_Handler_512=uxTaskGetStackHighWaterMark(MPU6050Task_Handler);
		StartTask_Handler_128=uxTaskGetStackHighWaterMark(StartTask_Handler);
	}
}

#endif
