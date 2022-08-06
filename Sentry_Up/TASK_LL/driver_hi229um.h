#ifndef DRIVER_HI229UM_H
#define DRIVER_HI229UM_H
#include "sys.h"

void HI229_Gyroscope_Init(void);

void get_HI229UMGyroData(void);

u8 HI229_Get_Gyroscope_Location(float *yaw, float *pitch, float *roll);

u8 HI229_Get_Gyroscope_Speed(float *gx, float *gy, float *gz);

u8 HI229_Get_Gyroscope_Acceleration(float *ax, float *ay, float *az);

typedef struct
{
	float yaw;
	float	pitch;
	float	roll;
	float gx;
	float	gy;
	float	gz;
	float ax;
	float	ay;
	float	az;
	float mx;
	float	my;
	float	mz;
}GyroDataPacketStruct;

#endif
