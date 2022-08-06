#ifndef __WIFI__
#define __WIFI__
#include <stdlib.h>
#include "interface_base.h"
#include <string.h>
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "freertostask.h"
#include "cmsis_os.h"
#include "bsp_can.h"

#define SEND_LENTH 100
#define ERECEIVE_LENTH 100

extern USART_TypeDef* WIFI_USART;

extern SemaphoreHandle_t xSemaphore;
extern int PostFlag;
 //针对传输方式
typedef enum    
{
	TIMING_WAY = 0,//定时数据传输
	CURRENT_WAY = 1,//突发数据传输
	INSERT_WAY = 2,//插入数据。通过一次通信插入一条或多条需要显示的数据。
}ReceiveWay;//接收方式


//针对数据格式
typedef enum 
{
	INT = 0,
	FLOAT = 1,
}ReceiveForm;//收到数据类型()

//针对显示需求
typedef enum 
{
	CONSTITUTE = 0,//需要持续输出并显示图像的数据
	CONDITION = 1,//固定显示，表示状态的数据
	CHANGEABLE = 2,//可以通过上位机修改的数据
}ReceiveType;//收到的数据是哪种类型



typedef struct
{
	int number;//编号
	char name[20];//保存名字
	
	
	ReceiveForm rf;//int或者float
	ReceiveType rt;//类型
	char* address;//存放数据
}SendData;//发送的数据



void InsertData(char* address,char* name,ReceiveForm rf,ReceiveType rt);//插入需要看的变量
	//for example:
	//InsertData((char*)(&texta),"int  texta",INT,CONSTITUTE);
	//InsertData((char*)(&textb),"int textb",INT,CONDITION);
	//InsertData((char*)(&textc),"float textc",FLOAT,CONDITION);

void WifiInit(char* ip,int port,char* wifiName,char* password,USART_TypeDef* pUSART);

void SendAll(void);

void WifiPost(void);

void GetOrder(void);//从上位机得到指令



#endif


