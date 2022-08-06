//本文件貌似没用

#include "data_device_dma.h"

//内存到串口1的DMA
//void buffer_usart1_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//外设到内存
//	DMA_InitTypeDef DMA_InitStructure;
//	//DMA2时钟使能
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
//	//对DMA2数据流5进行默认初始化
//	DMA_DeInit(DMA2_Stream5);
//	//通道选择
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	//外设地址
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//	//DMA存储器0地址
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	//外设到存储器模式
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//数据传输量
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	//外设非增量模式
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//存储器增量模式
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//外设数据长度8位
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//存储器数据长度8位
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//使用循环模式
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	//非常高的优先级
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	//外设突发单次传输
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//初始化DMA Stream
//	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
//	//中断使能
//	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
//	
//	//使能串口1的DMA接收
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
//	//使能DMA2_Stream5，启动传输
//	DMA_Cmd(DMA2_Stream5,ENABLE);
//}

////串口1到内存的DMA
//void usart1_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//外设到内存
//	DMA_InitTypeDef DMA_InitStructure;
//	//DMA2时钟使能
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
//	//对DMA2数据流5进行默认初始化
//	DMA_DeInit(DMA2_Stream5);
//	//通道选择
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	//外设地址
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
//	//DMA存储器0地址
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	//外设到存储器模式
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	//数据传输量
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	//外设非增量模式
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//存储器增量模式
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//外设数据长度8位
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//存储器数据长度8位
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//使用循环模式
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	//非常高的优先级
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	//外设突发单次传输
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//初始化DMA Stream
//	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
//	//中断使能
//	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
//	
//	//使能串口1的DMA接收
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
//	//使能DMA2_Stream5，启动传输
//	DMA_Cmd(DMA2_Stream5,ENABLE);
//}
////串口4到内存的DMA
//void myuart4_dma_init(uint32_t memory0BaseAddr,uint32_t bufferSize){
//	//外设到内存
//	DMA_InitTypeDef DMA_InitStructure;
//	DMA_DeInit(DMA1_Stream2);
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
//	DMA_InitStructure.DMA_Memory0BaseAddr = memory0BaseAddr;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA1_Stream2,&DMA_InitStructure);
//	//中断使能
//	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
//	//使能串口4的DMA接收
//	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
//	//使DMA1_Stream2，启动传输
//	DMA_Cmd(DMA1_Stream2,ENABLE);
//}
//extern unsigned char buffer[];
//void usart4_init()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef  DMA_InitStructure; 

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟

//	DMA_DeInit(DMA1_Stream2);

////	while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}
///* 配置 DMA Stream */
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&UART4->DR;//DMA外设地址
//  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&buffer;//DMA 存储器0地址
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式
//  DMA_InitStructure.DMA_BufferSize =100;//数据传输量 
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环模式
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//高等优先级
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//  DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOThreshold_Full;
////  DMA_InitStructure.DMA_MemoryBurst =DMA_MemoryBurst_Single;	
////	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;	
//  DMA_Init(DMA1_Stream2, &DMA_InitStructure);//初始化DMA Stream

//	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);	
//	DMA_Cmd(DMA1_Stream2, ENABLE); 

//	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Stream2_IRQn; //
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel=UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOPA1复用为USART2
//  
//	//UART4端口配置
//  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1; //GPIO2/3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化

//   //UART4 初始化设置
//	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收发模式
//  USART_Init(UART4, &USART_InitStructure); //初始化

//  USART_Cmd(UART4, ENABLE);  //使能串口4
//	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);			//空闲中断
//	
//  USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
//	
//}
