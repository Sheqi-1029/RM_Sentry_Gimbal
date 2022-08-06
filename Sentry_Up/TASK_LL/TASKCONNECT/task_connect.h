#ifndef TASK_CONNECT_H
#define TASK_CONNECT_H
#include "sys.h"


//���ػ����սṹ��
typedef struct
{
	float Pitch_location;
	float Yaw_location;
	short Command_PC;
}PC_Receive;

//PC_Receive PC_receive;

//���ػ����ͽṹ��
typedef struct
{
	struct 
	{
		short Pitch_loca;
		short Yaw_loca;
	}Gimbal_location;
	struct 
	{
		short Vx;
		short Vy;
		short Vz;
	}Gimbal_speed;
	char Command_PC;
}PC_Send;

//PC_Send PC_send;

void ConnectTask(void);

#endif
