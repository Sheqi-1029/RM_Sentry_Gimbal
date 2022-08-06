#include "task_wifi.h"
#include "math.h"


short Frequency=-1;
char sends[SEND_LENTH];
char receives[ERECEIVE_LENTH];
int Port;
char wifiName[50];
char passWord[50];

USART_TypeDef* WIFI_USART;
struct
{
	SendData sendData[200];//假设最多有200个值
	int number;
}Alldatas;
int number=0;//初始化用
int PostFlag=0;
float textc=334.55;
u32 time;
char* c=(char*)(&time);
void WifiInit(char* ip,int port,char* wifiname,char* password,USART_TypeDef* pUSART)
{
	strcpy(wifiName,wifiname);
	strcpy(passWord,password);
	xSemaphore= xSemaphoreCreateBinary();
	WIFI_USART=pUSART;
	WIFI_ConfigUsartDMA((u32)receives,100);
	Port=port;
	data_transmit_init(port,ip);  //端口 IP
	sends[0]=sends[1]=0x5A;
	sends[2]=sends[3]=0;
	
	
}


void InsertData(char* address,char* name,ReceiveForm rf,ReceiveType rt)
{
	if(Alldatas.number>=200)
		return;
	Alldatas.sendData[Alldatas.number].number=Alldatas.number+1;
	Alldatas.sendData[Alldatas.number].address=address;
	strcpy(Alldatas.sendData[Alldatas.number].name,name);
	Alldatas.sendData[Alldatas.number].rf=rf;
	Alldatas.sendData[Alldatas.number].rt=rt;
	Alldatas.number++;
}



void SendAll(void)
{
	if(Alldatas.number>number)
		PostFlag=0;
	u16 lenth;
	time=HAL_GetTick();
	
	
	sends[4]=c[0];
	sends[5]=c[1];
	sends[6]=c[2];
	sends[7]=c[3];
	
	sends[8]=(char)TIMING_WAY;//定时发送所有数据
	sends[9]=Alldatas.number;
	for(int i=0;i<Alldatas.number;i++)
	{
		sends[10+i*5]=Alldatas.sendData[i].number;
		sends[10+i*5+1]=Alldatas.sendData[i].address[0];
		sends[10+i*5+2]=Alldatas.sendData[i].address[1];
		sends[10+i*5+3]=Alldatas.sendData[i].address[2];
		sends[10+i*5+4]=Alldatas.sendData[i].address[3];
	}
	
	sends[10+5*Alldatas.number]=~(sends[8]+1);
	sends[11+5*Alldatas.number]='\0';
	lenth=11+5*Alldatas.number;
	sends[2]=lenth>>8;
	sends[3]=lenth&0xff;
	for(int i=0;i<11+5*Alldatas.number;i++)
	{
		while((WIFI_USART->SR&0X40)==0);
		WIFI_USART->DR = (uint8_t) sends[i];
	}
	

}

void WifiPost(void)
{
	if(Alldatas.number==number)
	{
		PostFlag=1;
		return ;
	}else
	{
		u16 lenth;
		time=HAL_GetTick();
		u16 count;//名字长度
		const char* name=Alldatas.sendData[number].name;
	
		sends[4]=c[0];
		sends[5]=c[1];
		sends[6]=c[2];
		sends[7]=c[3];
		sends[8]=(char)INSERT_WAY;//增加数据指令
		sends[9]=Alldatas.sendData[number].number;
		sends[10]=Alldatas.sendData[number].rf;
		sends[11]=Alldatas.sendData[number].rt;
		count=strlen(name)+1;
		sends[12]=count;
		for(int i=0;i<count-1;i++)
		{
			sends[13+i]=name[i];
		}
		sends[13+count-1]='\0';
		sends[13+count]=~(sends[8] + 1);
		sends[14+count]='\0';
		lenth=14+count;
		sends[2]=lenth>>8;
		sends[3]=lenth&0xff;
		
		for(int i=0;i<14+count;i++)
		{
			while((WIFI_USART->SR&0X40)==0);
			WIFI_USART->DR = (uint8_t) sends[i];
		}	
		number++;
	}
	
}


void GetOrder()
{
	if(receives[0]!=0x5A||receives[0]!=0x5A)//头校验
	{
		return ;
	}
	
	if(receives[7]!=0xA5)return;
	int number=receives[2];
	for(int i=0;i<Alldatas.number;i++)
	{
		if(Alldatas.sendData[i].number==number)
		{
			if(Alldatas.sendData[i].rt!=CONDITION)//不是condition模式
				return;
			Alldatas.sendData[i].address[0]=receives[3];
			Alldatas.sendData[i].address[1]=receives[4];
			Alldatas.sendData[i].address[2]=receives[5];
			Alldatas.sendData[i].address[3]=receives[6];
		}
	}
	
}
