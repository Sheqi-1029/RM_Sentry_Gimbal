#ifndef _USART_BSP_H
#define _USART_BSP_H
#include "sys.h"
#include "task_communicate.h"

void USART1ConfigEnable(void);
void USART6ConfigEnable(void);

void ConfigUsart1DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
void ConfigUsart6DMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);

//WifiChange Begin
void WIFI_USARTConfigEnable(void);
void WIFI_ConfigUsartDMA(u32 DMA_Memory0BaseAddr,u32 DMA_BufferSize);
//WifiChange End

uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream);
void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);

void usart_dma_send(usart_param_struct *usart_param,u32 dma_addr,u32 dma_buffer_size);
void usart_inturrupt_processed(usart_param_struct usart_param);
void usart_communicate_config(usart_param_struct usart_param);
#endif
