/****************************************************
*			Title:		���еײ����ýӿ�
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.09.14
*												LD.
*****************************************************/
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H
#include "sys.h"

/****************************************************
*											CAN1�ӿ�											*
****************************************************/

//CAN����������16λidlistģʽ
//0x000�رոù�����
#define CAN1_FILTER_ID_FMI0	(0x206)
#define CAN1_FILTER_ID_FMI1	(0x207)
#define CAN1_FILTER_ID_FMI2	(0x208)
#define CAN1_FILTER_ID_FMI3	(0x000)

//�����ڶ���
#define FILTER_NUMBER_2_OPEN	(1)
	#define CAN1_FILTER_ID_FMI4	(0x0206)
	#define CAN1_FILTER_ID_FMI5	(0x0207)
	#define CAN1_FILTER_ID_FMI6	(0x0208)
	#define CAN1_FILTER_ID_FMI7	(0x000)



/******************************************6**********
*											CAN2�ӿ�											*
****************************************************/

//CAN����������16λidlistģʽ
//0x000�رոù�����
//#define CAN2_FILTER_ID_FMI0	(0x101)
//#define CAN2_FILTER_ID_FMI1	(0x250)
//#define CAN2_FILTER_ID_FMI2	(0x201)
//#define CAN2_FILTER_ID_FMI3	(0x202)
#define CAN2_FILTER_ID_FMI0	(0x0203)
//#define CAN2_FILTER_ID_FMI1	(0x0205)
#define CAN2_FILTER_ID_FMI2	(0x0208)
#define CAN2_FILTER_ID_FMI3	(0x0230)
#define CAN2_FILTER_ID_FMI1	(0x0252)

/****************************************************
*											WatchDog�ӿ�										*
****************************************************/
#define CONFIG_USE_IWDG 1
#define CONFIG_USE_WWDG 0


//DRIVER
/****************************************************
*											Chassis�ӿ�										*
****************************************************/
#define	CONFIG_USE_CHASSIS		1

//DRIVER
/****************************************************
*											FeedMotor�ӿ�										*
****************************************************/
#define	CONFIG_USE_FEEDMOTOR		1


//DRIVER
/****************************************************
*											MPU9250�ӿ�										*
****************************************************/
#define	CONFIG_USE_MPU9250		0

#if	CONFIG_USE_MPU9250
		#define CONFIG_USE_MPU9250_DMP	0
#endif


/****************************************************
*											GIMBAL�ӿ�										*
****************************************************/
#define PITCH_INIT_VALUE_SET	(0.509f)
#define YAW_INIT_VALUE_SET	(0.25f)


/****************************************************
*											LostCounter�ӿ�										*
****************************************************/
#define	CONFIG_USE_LOSTCOUNTER		1

#if	CONFIG_USE_LOSTCOUNTER
#define NUMBERS_OF_COUNT		10
	
#define	GIMBAL_MOTOR_PITCH				0
#define	GIMBAL_MOTOR_YAW					1
#define FEEDMOTOR_LOST_COUNT 			2
#define	REMOTE_LOST_COUNT					3
#define	VISION_LOST_COUNT					4
#define	JUDGEMENT_LOST_COUNT			5
#define SINGLE_GYRO_LOST_COUNT		6
#define USB_RECV_LOST_COUNT				7
#define FRICTION_0_LOST_COUNT			8
#define FRICTION_1_LOST_COUNT			9
#define FEEDMOTORUP_LOST_COUNT    10
	
#define GIMBAL_LOST_TOLERANCE_MS				200
#define REMOTE_LOST_TOLERANCE_MS				200
#define FEEDMOTOR_LOST_TOLERANCE_MS			200
#define VISION_LOST_TOLERANCE_MS				500
#define JUDGEMENT_LOST_TOLERANCE_MS			200
#define SINGLE_GYRO_LOST_TOLERANCE_MS		200
#define USB_RECV_LOST_TOLERANCE_MS			500
#define FRICTION_LOST_TOLERANCE_MS			200
	
	
#endif
/****************************************************
*										����ģʽ�ӿ�										*
****************************************************/
#define	DEBUG_USE_GIMBALMOTOR_CANSEND		1
#define DEBUG_USE_CHASSISMOTOR_CANSEND  0


/****************************************************
*											�ڴ���Խӿ�									*
****************************************************/
//���������ջʹ��������ȰѺ� INCLUDE_uxTaskGetStackHighWaterMark ��1 λ��FreeRTOS.h
//������رմκ꣨�Ϸ�ʱ�䣩
//����ͨ��һ���궨������ȥ
#define CHECKMEMORYTASK	0


/****************************************************
*										�ź����ӿ�										  *
****************************************************/
#define REMOTE_UART_RX_SIGNAL  ( 1 << 0 )



#endif
