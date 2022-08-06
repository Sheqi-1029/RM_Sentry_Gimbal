#include "bsp_usart.h"
#include "sys.h"
#include "task_wifi.h"

u32 Usart1DMAMemoryBaseAddress,Usart1DMABufferSize;
void ConfigUsart1DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)
{
	Usart1DMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	Usart1DMABufferSize=DMA_BufferSize;
	USART1ConfigEnable();
}

u32 Usart6DMAMemoryBaseAddress,Usart6DMABufferSize;
void ConfigUsart6DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)
{
	Usart6DMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	Usart6DMABufferSize=DMA_BufferSize;
	USART6ConfigEnable();
}

void USART1ConfigEnable(void)
{
	LL_DMA_SetMemoryAddress(DMA2,LL_DMA_STREAM_2,(u32)Usart1DMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_2,(u32)&USART1->DR);
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_2,Usart1DMABufferSize);	
	
	LL_USART_EnableIT_IDLE(USART1);
  LL_USART_EnableDMAReq_RX(USART1);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
//	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
}

void USART6ConfigEnable(void)
{ 
	LL_DMA_SetMemoryAddress(DMA2,LL_DMA_STREAM_1,(u32)Usart6DMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA2,LL_DMA_STREAM_1,(u32)&USART6->DR);
	LL_DMA_SetDataLength(DMA2,LL_DMA_STREAM_1,Usart6DMABufferSize);	
	
	LL_USART_EnableIT_IDLE(USART6);
  LL_USART_EnableDMAReq_RX(USART6);
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);
}

//WifiChange Begin   注：如果需要更改wifi的串口号，只需在cube下重新生成相应配置，在此处更改dma和dma通道信息，并在main函数中wifiinit()第三个参数写相应串口
//注：wifi串口波特率115200
/*u32 WIFI_UsartDMAMemoryBaseAddress,WIFI_UsartDMABufferSize;
void WIFI_ConfigUsartDMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)//wifi串口初始化必须在WiFi初始化之后,否则会出现WIFI_USART为空的情况
{
	WIFI_UsartDMAMemoryBaseAddress=DMA_Memory0BaseAddr;
	WIFI_UsartDMABufferSize=DMA_BufferSize;
	
	WIFI_USARTConfigEnable();
}

void WIFI_USARTConfigEnable(void)//
{
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_STREAM_1,(u32)WIFI_UsartDMAMemoryBaseAddress);
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_STREAM_1,(u32)&WIFI_USART->DR);
	LL_DMA_SetDataLength(DMA1,LL_DMA_STREAM_1,WIFI_UsartDMABufferSize);	
	
	LL_USART_EnableIT_IDLE(WIFI_USART);//空闲中断
  LL_USART_EnableDMAReq_RX(WIFI_USART);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
}
*/

//WifiChange End




void usart_communicate_config(usart_param_struct usart_param)
{
	LL_DMA_EnableDoubleBufferMode(usart_param.rx_DMAx,usart_param.rx_Stream);
	LL_DMA_SetMemoryAddress(usart_param.rx_DMAx,usart_param.rx_Stream,usart_param.rx_usart_dma_memory_base_address_0);
	LL_DMA_SetPeriphAddress(usart_param.rx_DMAx,usart_param.rx_Stream,(u32)&usart_param.USARTx->DR);
	LL_DMA_SetDataLength(usart_param.rx_DMAx,usart_param.rx_Stream,usart_param.rx_usart_dma_buffer_size);
	//****************************这里是在配置DMA的双缓冲区模式
	LL_DMA_SetMemory1Address(usart_param.rx_DMAx,usart_param.rx_Stream,usart_param.rx_usart_dma_memory_base_address_1);//**************
	
	LL_DMA_SetPeriphAddress(usart_param.tx_DMAx,usart_param.tx_Stream,(u32)&usart_param.USARTx->DR);
	
	LL_USART_EnableIT_IDLE(usart_param.USARTx);
  LL_USART_EnableDMAReq_RX(usart_param.USARTx);
	LL_USART_EnableDMAReq_TX(usart_param.USARTx);
	
	LL_DMA_EnableIT_TC(usart_param.rx_DMAx, usart_param.rx_Stream);
	LL_DMA_EnableIT_TC(usart_param.tx_DMAx, usart_param.tx_Stream);

	LL_DMA_EnableStream(usart_param.rx_DMAx, usart_param.rx_Stream);
	
}

void usart_inturrupt_processed(usart_param_struct usart_param)
{
		if(((usart_param.USARTx->SR)&(1<<4))!=0)//SR寄存器的IDLE位，如果置1表示检测到空闲线路
		{
			__IO uint32_t tmpreg = 0x00U;
			tmpreg = usart_param.USARTx->SR;
			tmpreg = usart_param.USARTx->DR;
			UNUSED(tmpreg);//用来屏蔽无用参数警告
			osSignalSet(*(usart_param.task), UART_IDLE_SIGNAL);
		}
}
void usart_dma_send(usart_param_struct *usart_param,u32 dma_addr,u32 dma_buffer_size)
{
		
		if(usart_param->tx_finish_flag)
		{
			LL_DMA_SetMemoryAddress(usart_param->tx_DMAx,usart_param->tx_Stream,dma_addr);
	
			LL_DMA_SetDataLength(usart_param->tx_DMAx,usart_param->tx_Stream,dma_buffer_size);
		
			LL_DMA_EnableStream(usart_param->tx_DMAx, usart_param->tx_Stream); //开启传输
			
			usart_param->tx_finish_flag =0;
		}
}
//DMA使用了双缓冲区模式，在每个缓冲区满了之后会产生传输完成中断，之后硬件会自动切换缓冲区
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)//根据CT标志查看DMA正在访问哪个存储区
{
  uint8_t tmp = 0;

  /* Get the current memory target */
  if ((dma_stream->CR & DMA_SxCR_CT) != 0)
  {
    /* Current memory buffer used is Memory 1 */
    tmp = 1;
  }
  else
  {
    /* Current memory buffer used is Memory 0 */
    tmp = 0;
  }
  return tmp;
}
uint16_t remain_data_counter_temp=0;
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));//此寄存器用于存储要传输的数据项数目，每次DMA传输后此寄存器将递减，设置循环模式后，会将之前的设定值重装载
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt =  dma_current_data_counter(dma_stream);
}
