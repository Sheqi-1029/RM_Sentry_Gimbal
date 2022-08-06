/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "sys.h"
#include "led.h"
#include "can.h"
#include "driver_remote.h"
#include "driver_chassis.h"
#include "task_remote.h"
#include "task_chassis.h"
#include "task_lostcounter.h"
#include "task_gimbal.h"
#include "bsp_watchdog.h"
#include "driver_gimbal.h"
#include "task_feedmotor.h"
#include "delay.h"
#include "usbd_cdc_if.h"
#include "task_modeswitch.h"
#include "interface_base.h"
#include "task_wifi.h"
#include "task_connect.h"
#include "driver_connect.h"
#include "driver_friction.h"
#include "task_friction.h"
#include "task_vision.h"
#include "task_communicate.h"
#include "driver_vision.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int lose_connect_flag;
int lose_connect_counter;
extern short Frequency;

osThreadId judge_unpack_task_t;
/* USER CODE END Variables */
osThreadId LED_TaskHandle;
osThreadId FeedMotor_TaskHandle;
osThreadId Remote_TaskHandle;
osThreadId LostCounter_TaskHandle;
osThreadId Gimbal_TaskHandle;
osThreadId ConnectTaskHandle;
osThreadId Friction_TaskHandle;
osThreadId Vision_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void LEDTask(void const * argument);
void FeedMotorTask(void const * argument);
void RemoteTask(void const * argument);
void LostCounterTask(void const * argument);
void GimbalTask(void const * argument);
void Connect_Task(void const * argument);
void FrictionTask(void const * argument);
void VisionTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, LEDTask, osPriorityLow, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of FeedMotor_Task */
  osThreadDef(FeedMotor_Task, FeedMotorTask, osPriorityNormal, 0, 512);
  FeedMotor_TaskHandle = osThreadCreate(osThread(FeedMotor_Task), NULL);

  /* definition and creation of Remote_Task */
  osThreadDef(Remote_Task, RemoteTask, osPriorityRealtime, 0, 2048);
  Remote_TaskHandle = osThreadCreate(osThread(Remote_Task), NULL);

  /* definition and creation of LostCounter_Task */
  osThreadDef(LostCounter_Task, LostCounterTask, osPriorityLow, 0, 64);
  LostCounter_TaskHandle = osThreadCreate(osThread(LostCounter_Task), NULL);

  /* definition and creation of Gimbal_Task */
  osThreadDef(Gimbal_Task, GimbalTask, osPriorityHigh, 0, 1024);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

  /* definition and creation of ConnectTask */
  osThreadDef(ConnectTask, Connect_Task, osPriorityIdle, 0, 128);
  ConnectTaskHandle = osThreadCreate(osThread(ConnectTask), NULL);

  /* definition and creation of Friction_Task */
  osThreadDef(Friction_Task, FrictionTask, osPriorityIdle, 0, 128);
  Friction_TaskHandle = osThreadCreate(osThread(Friction_Task), NULL);

  /* definition and creation of Vision_Task */
  osThreadDef(Vision_Task, VisionTask, osPriorityHigh, 0, 128);
  Vision_TaskHandle = osThreadCreate(osThread(Vision_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_LEDTask */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used 
  * @retval None
  */

//unsigned char	send_buffer[]="Hello World!";




extern u8 init_Wifi_flag;
SemaphoreHandle_t xSemaphore;//wifi的二值信号


/* USER CODE END Header_LEDTask */
void LEDTask(void const * argument)
{

  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */
  for(;;)
  {
		LED0=!LED0;
    osDelay(500);
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_FeedMotorTask */
/**
* @brief Function implementing the FeedMotor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FeedMotorTask */
void FeedMotorTask(void const * argument)
{
  /* USER CODE BEGIN FeedMotorTask */
  /* Infinite loop */
  for(;;)
  {
		FeedMotorControlLogic();
		//FeedMotor_UpControlLogic();   
		osDelay(4);
  }
  /* USER CODE END FeedMotorTask */
}

/* USER CODE BEGIN Header_RemoteTask */
/**
* @brief Function implementing the Remote_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteTask */
void RemoteTask(void const * argument)
{
  /* USER CODE BEGIN RemoteTask */
	osEvent event;
	RemoteInit();
  /* Infinite loop */
  for(;;)
  {
		
		event = osSignalWait(REMOTE_UART_RX_SIGNAL,osWaitForever);
		if(event.status==osEventSignal)									
		{
			if(event.value.signals & REMOTE_UART_RX_SIGNAL)//收到遥控器信号量
			{
         if(!RemoteTaskControl())
			 {
				 LostCounterFeed(REMOTE_LOST_COUNT);
			 }
			}
		}
		
		
		osDelay(1);
  }
  /* USER CODE END RemoteTask */
}

/* USER CODE BEGIN Header_LostCounterTask */
/**
* @brief Function implementing the LostCounter_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LostCounterTask */
void LostCounterTask(void const * argument)
{
  /* USER CODE BEGIN LostCounterTask */
  /* Infinite loop */
  for(;;)
  {
		LostCounterControl(LostCounterCount());
	  FeedIndependentWatchDog();
    osDelay(50);
  }
  /* USER CODE END LostCounterTask */
}

/* USER CODE BEGIN Header_GimbalTask */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask */
void GimbalTask(void const * argument)
{
  /* USER CODE BEGIN GimbalTask */
	/*while(GetLostCounterData()[GIMBAL_MOTOR_PITCH]>GIMBAL_LOST_TOLERANCE_MS)
	{
		vTaskDelay(1);								//精准控制时间离散节奏
	}

	while(GetLostCounterData()[GIMBAL_MOTOR_YAW]>GIMBAL_LOST_TOLERANCE_MS)
	{
		vTaskDelay(1);								//精准控制时间离散节奏
	}*/
  /* Infinite loop */
		VisionInit();

  for(;;)
  {
				General_Control();  //总控
		VisionControlTask();
		GimbalControlTask();
    osDelay(1);
  }
  /* USER CODE END GimbalTask */
}

/* USER CODE BEGIN Header_Connect_Task */
/**
* @brief Function implementing the ConnectTask thread.
* @param argument: Not used
* @retval None
*/
extern  int time_1ms;
int time_count=0;
int pwm_count=0;
/* USER CODE END Header_Connect_Task */
void Connect_Task(void const * argument)
{
  /* USER CODE BEGIN Connect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  
  /* USER CODE END Connect_Task */
}

/* USER CODE BEGIN Header_FrictionTask */
/**
* @brief Function implementing the Friction_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FrictionTask */
void FrictionTask(void const * argument)
{
  /* USER CODE BEGIN FrictionTask */
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableAllOutputs(TIM2);
	
//	LL_TIM_OC_SetCompareCH1(TIM2, 800);
//	LL_TIM_OC_SetCompareCH2(TIM2, 800);
	
//	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH1);
//	LL_TIM_CC_EnableChannel(TIM12,LL_TIM_CHANNEL_CH2);
//	LL_TIM_EnableCounter(TIM12);
//	LL_TIM_EnableAllOutputs(TIM12);
	
	
  /* Infinite loop */
  for(;;)
  {
		FrictionControlTask();
    osDelay(2);
  }
  /* USER CODE END FrictionTask */
}

/* USER CODE BEGIN Header_VisionTask */
/**
* @brief Function implementing the Vision_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VisionTask */
void VisionTask(void const * argument)
{
  /* USER CODE BEGIN VisionTask */
  /* Infinite loop */
  for(;;)
  {
		//VisionControlTask();
    osDelay(1);
  }
  /* USER CODE END VisionTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void DMAUsart1DataFinishedHandle(void)
{
	osSignalSet(Remote_TaskHandle,REMOTE_UART_RX_SIGNAL);
}





/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
