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

//WifiChange Begin   ע�������Ҫ����wifi�Ĵ��ںţ�ֻ����cube������������Ӧ���ã��ڴ˴�����dma��dmaͨ����Ϣ������main������wifiinit()����������д��Ӧ����
//ע��wifi���ڲ�����115200
/*u32 WIFI_UsartDMAMemoryBaseAddress,WIFI_UsartDMABufferSize;
void WIFI_ConfigUsartDMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize)//wifi���ڳ�ʼ��������WiFi��ʼ��֮��,��������WIFI_USARTΪ�յ����
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
	
	LL_USART_EnableIT_IDLE(WIFI_USART);//�����ж�
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
	//****************************������������DMA��˫������ģʽ
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
		if(((usart_param.USARTx->SR)&(1<<4))!=0)//SR�Ĵ�����IDLEλ�������1��ʾ��⵽������·
		{
			__IO uint32_t tmpreg = 0x00U;
			tmpreg = usart_param.USARTx->SR;
			tmpreg = usart_param.USARTx->DR;
			UNUSED(tmpreg);//�����������ò�������
			osSignalSet(*(usart_param.task), UART_IDLE_SIGNAL);
		}
}
void usart_dma_send(usart_param_struct *usart_param,u32 dma_addr,u32 dma_buffer_size)
{
		
		if(usart_param->tx_finish_flag)
		{
			LL_DMA_SetMemoryAddress(usart_param->tx_DMAx,usart_param->tx_Stream,dma_addr);
	
			LL_DMA_SetDataLength(usart_param->tx_DMAx,usart_param->tx_Stream,dma_buffer_size);
		
			LL_DMA_EnableStream(usart_param->tx_DMAx, usart_param->tx_Stream); //��������
			
			usart_param->tx_finish_flag =0;
		}
}
//DMAʹ����˫������ģʽ����ÿ������������֮��������������жϣ�֮��Ӳ�����Զ��л�������
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)//����CT��־�鿴DMA���ڷ����ĸ��洢��
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
  return ((uint16_t)(dma_stream->NDTR));//�˼Ĵ������ڴ洢Ҫ�������������Ŀ��ÿ��DMA�����˼Ĵ������ݼ�������ѭ��ģʽ�󣬻Ὣ֮ǰ���趨ֵ��װ��
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt =  dma_current_data_counter(dma_stream);
}
