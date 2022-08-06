#include "interface_base.h"
#include "data_task_main.h"
#include "data_channel_wifi.h"
#include "data_analysis_custom.h"
#include "data_analysis_judge_system.h"
#include "data_device_usart.h"
/**
 *数据传输系统对外接口
 *总体上分为数据入口与数据出口两个部分
 *注意：带*的接口为必须调用或实现的接口
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *数据入口，传输系统实现，需用户调用
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//*接口1*：数据传输系统初始化
void data_transmit_init(int port,char* server_ip)
{
	init_wifi(port,server_ip);
}
//*接口2*：数据传输系统时钟
void data_transmit_clock()
	{
		clock_main();
}
u8 last_byte=0;
extern u8 init_Wifi_flag;
extern u16 wifi_init_delay_count;
//接口5：WIFI串口单字节数据输入

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *数据出口，传输系统调用，需用户实现
 *用户选择自己需要的数据进行处理即可
 */

 //接口2：向WIFI芯片所在串口写入一个字节
void send_one_byte_to_wifi(u8 data){
	LL_USART_TransmitData8(USART3,data);
	 while( LL_USART_IsActiveFlag_TC(USART3)!=SET); 
}

//接口4：WIFI串口DMA发送一次
void dma_send_wifi_data_once(){
	 LL_USART_EnableDMAReq_TX(USART3);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}
