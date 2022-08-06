#ifndef __INTERFACE_BASE_H
#define __INTERFACE_BASE_H	 
#include "sys.h"


void data_transmit_init(int port,char* server_ip);
void data_transmit_clock(void);
void data_transmit_task(void);

void receive_one_byte_from_wifi(u8 data);

void send_one_byte_to_wifi(u8 data);
void dma_send_wifi_data_once(void);

void SendMessageToWifi(void);

#endif
