#ifndef __DRIVER_CH100_H
#define __DRIVER_CH100_H

#include "main.h"

__packed typedef  struct  receive_imusol_packet_t {
	uint8_t tag;
	uint8_t id;
	float acc[3];
	float gyr[3];
	float mag[3];
	float eul[3];
	float quat[4];

} receive_imusol_packet_t;


void CH100_Gyroscope_Init(void);
void get_CH100GyroData(void);

#endif
