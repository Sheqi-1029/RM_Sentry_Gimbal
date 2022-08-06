#ifndef CAN_SEND_H
#define CAN_SEND_H
#include "sys.h"
#include "BSPconfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct
{
    uint8_t     CANx;               //CAN±àºÅ     1 CAN1      2 CAN2
    CanTxMsg    SendCanTxMsg;       //·¢ËÍÊý¾Ý
}CanSendMessegeStruct;

void CanSendProcessed(CanSendMessegeStruct CANSendData);

#endif
