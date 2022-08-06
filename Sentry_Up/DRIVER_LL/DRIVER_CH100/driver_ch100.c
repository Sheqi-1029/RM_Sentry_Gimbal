#include "driver_ch100.h"
#include "bsp_usart.h"
#include <string.h>

//#define GYRO_FRAME_LENGTH (82u)
receive_imusol_packet_t receive_imusol;

extern uint8_t GyroData[100];


void CH100_Gyroscope_Init()
{
	//ConfigUsart6DMA((uint32_t)GyroData,GYRO_FRAME_LENGTH);
	
}


void get_CH100GyroData()
{   
	  static uint8_t*p = GyroData;
		memcpy((void *) receive_imusol.acc, p + 18 , sizeof(float) * 16);

}