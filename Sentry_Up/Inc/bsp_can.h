#ifndef _CAN_BSP_H
#define _CAN_BSP_H
#include "sys.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void LL_CAN_Receive(CAN_HandleTypeDef* hcan);

void CanInit(void);
unsigned char UserCan1FilterConfig(void);
unsigned char UserCan2FilterConfig(void);
u8 CAN1_Send_Msg(u8* msg,u8 len,int Id);
u8 CAN2_Send_Msg(u8* msg,u8 len,int Id);


#endif
