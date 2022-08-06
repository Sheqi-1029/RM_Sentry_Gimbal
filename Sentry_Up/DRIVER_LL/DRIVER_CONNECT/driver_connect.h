
#ifndef DRIVER_CONNECT_H
#define DRIVER_CONNECT_H
#include "sys.h"


void driver_connect_receive(float *x_receive,
                            float *y_receive,
                            float *z_receive,
                            u8 *command_receive);
int driver_connect_send(
	                       uint16_t blood_judge,
	                       uint16_t bullet_judge,
												 uint16_t radar_judge);
#endif