/**
 * WIFI发送与接收逻辑处理，对数据进行封装，持有发送数据缓存，通过带参函数向上层提供WIFI发送能力。
 */
#include "delay.h"
#include "data_channel_wifi.h"
#include "usart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "data_analysis_custom.h"
#include "data_analysis_judge_system.h"
#include "data_analysis_tool_crc.h"
#include "data_task_main.h"
#include "data_channel_judge_system.h"
#include "interface_base.h"
#include "task_wifi.h"

extern int Port;
extern char wifiName[50];
extern char passWord[50];

#define INTERRUPT_INTERVAL 10



char debug_environment=DEBUG_ENVIRONMENT_RM;


u8 init_Wifi_flag=100;
char* server_ip;
//发送的车的状态数据总长度
u8 data_length=DATA_PART_1_LENGTH+DATA_PART_2_LENGTH+DATA_PART_3_LENGTH+DATA_PART_4_LENGTH+DATA_PART_5_LENGTH;
u8 wifi_send_flag;
//已封装好的直接传给WIFI芯片的数据
char send_vehicle_condition_flag=0;
u16 wifi_data_vehicle_condition[DATA_PART_1_LENGTH+DATA_PART_2_LENGTH+DATA_PART_3_LENGTH+DATA_PART_4_LENGTH+DATA_PART_5_LENGTH];
//身份标识数据
char send_connect_id_flag=0;
u16 wifi_data_connect_id[DATA_PART_1_LENGTH];
//测试数据
char send_test_data_flag=0;
u16 wifi_data_test_data[DATA_PART_9_LENGTH];
//A
char send_self_check_flag=0;
u16 wifi_data_self_check[DATA_PART_14_LENGTH];



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************************************************
//用户自定义区域1开始：
//用法：在此处定义若干个u32型的send_wifi_count_n;并在update_wifi_send_flag()中使其自增，则其值每100ms自增一次
//用途：在wifi_task()中检测其值数值可用于控制不同发送数据的发送频率
//****************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//测试数据发送频率控制
u32 send_wifi_count_1=0;
//身份验证信息频率控制
u32 send_wifi_count_2=0;
//保留未用
u32 send_wifi_count_3=0;


volatile u16 wifi_init_delay_count=0;

//更新wifi发送标志位，INTERRUPT_INTERVAL调用一次
void clock_wifi(){
	if(init_Wifi_flag==9){
		update_wifi_send_flag();
	}else{
	wifi_init_delay_count++;
		check_init_wifi_flag();
	}
}

void update_wifi_send_flag(){
	send_wifi_count_1++;
	send_wifi_count_2++;
	send_wifi_count_3++;
}

void check_init_wifi_flag(){
	switch (init_Wifi_flag)
	{
			case 100:
				wifi_init_delay_count=0;
			break;
			case 0:
				if(wifi_init_delay_count>=800/INTERRUPT_INTERVAL){   //200ms
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 1:
				if (wifi_init_delay_count>=400/INTERRUPT_INTERVAL)   //100ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 2:
				if (wifi_init_delay_count>=5000/INTERRUPT_INTERVAL)   //1250ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 3:	
				if (wifi_init_delay_count>=24000/INTERRUPT_INTERVAL)  //6000ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 4:
				if (wifi_init_delay_count>=12000/INTERRUPT_INTERVAL) //3000ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 5:
				if (wifi_init_delay_count>=10000/INTERRUPT_INTERVAL)  //2500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;
			case 6:
				if (wifi_init_delay_count>=2000/INTERRUPT_INTERVAL)  //500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 7:
				if (wifi_init_delay_count>=2000/INTERRUPT_INTERVAL)  //500ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 8:
				if (wifi_init_delay_count>=1200/INTERRUPT_INTERVAL)  //300ms
				{
					wifi_init_delay_count=0;
					init_wifi_device(Port,server_ip);
				}
				break;	
			case 9:
				wifi_init_delay_count=0;
				update_wifi_send_flag();
				break;
			default:
				wifi_init_delay_count=0;
				break;		
	}
}
extern unsigned char buffer[BUFFER_SIZE];
//wifi模块初始化对外接口
void init_wifi(int port,char* server_ip_arg)
{	

	init_wifi_device(port,server_ip_arg);
}
//向WIFI芯片发送指令，真正地初始化WIFI芯片
u8 init_wifi_device(int port,char* server_ip_arg)  //配置esp8266为 STA TCP客户端模式
{
	server_ip=server_ip_arg;
	if(init_Wifi_flag==100){
		printf("+++");
		init_Wifi_flag=0;
		return init_Wifi_flag;
	}
	else if (init_Wifi_flag==0)
	{
		int i;
		for(i=0;i<data_length;i++)
		{
			wifi_data_vehicle_condition[i]=0;
		}
		printf("AT+CWMODE=1");       //设置wifi模式为STA模式
		printNewLine();
		init_Wifi_flag=1;
		return init_Wifi_flag;
	}
	else if (init_Wifi_flag==1)
	{
		printf("AT+RST");                //重启生效
		printNewLine();
		init_Wifi_flag=2;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==2)
	{
		char c[50]="AT+CWJAP=\"";
		char tmp[10]="\",\"";
		char end[10]="\"";
		strcat(c,wifiName);
		strcat(c,tmp);
		strcat(c,passWord);
		strcat(c,end);
		printf("%s",c);
		
		printNewLine();
		init_Wifi_flag=3;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==3)
	{
		printf("AT+CIPMUX=0");    //开启单连接
		printNewLine();
		init_Wifi_flag=4;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==4)
	{
		//char c[100];
		printf("AT+CIPSTART=\"TCP\",\"%s\",%d",server_ip,port);
		
		//printf("%s",c);//建立TCP连接
		printNewLine();
		init_Wifi_flag=5;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==5)
	{
		printf("AT+CIPMODE=1");     //开启透传模式
		printNewLine();
		init_Wifi_flag=6;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==6)
	{
		printf("AT+CIPSEND");       //开始传输
		printNewLine();
		init_Wifi_flag=7;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==7)
	{
		printf("succeed!");
		printNewLine();
		init_Wifi_flag=8;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==8)
	{
		send_wifi_count_1=0;
		wifi_send_flag=0;
		init_Wifi_flag=9;
		return init_Wifi_flag;
	}
	else if(init_Wifi_flag==9)
	{
		return init_Wifi_flag;
	}
	else
	{
		return init_Wifi_flag;
	}
}


void printNewLine()//发送新行
{
	printf("\r\n");
}
