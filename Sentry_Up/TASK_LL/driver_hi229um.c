#include "driver_hi229um.h"
#include "bsp_usart.h"
#include "math.h"

#define GYRO_FRAME_LENGTH (40u)

u8 GyroData[100];

float l_yaw,l_pitch, l_roll;          //繁琐而复杂
int watch_gx,watch,watch_gy,watch_gz; //繁琐而复杂
float gx, gy, gz;                     //繁琐而复杂

GyroDataPacketStruct GyroDataPacket;  //高效而简介

void HI229_Gyroscope_Init()
{
	ConfigUsart6DMA((u32)GyroData,GYRO_FRAME_LENGTH);
	
}


void get_HI229UMGyroData()
{   
	//-----------------------高效而简介的新代码和新结构体--------------------//
		GyroDataPacket.gx = (s16)((GyroData[17]<<8)+ GyroData[16]) / 10; 
		GyroDataPacket.gy = (s16)((GyroData[19]<<8)+ GyroData[18]) / 10;
		GyroDataPacket.gz = (s16)((GyroData[21]<<8)+ GyroData[20]) / 10;

		GyroDataPacket.roll   = (int16_t)((GyroData[31]<<8)+ GyroData[30]) / 100 ;
		GyroDataPacket.pitch    = (int16_t)((GyroData[33]<<8)+ GyroData[32]) / 100;
		GyroDataPacket.yaw     = (int16_t)((GyroData[35]<<8)+ GyroData[34]) / 10;
	
		GyroDataPacket.ax = (int16_t)((GyroData[10]<<8)+ GyroData[9]) / 10.0f; 
		GyroDataPacket.ay = (int16_t)((GyroData[12]<<8)+ GyroData[11]) / 10.0f;
		GyroDataPacket.az = (int16_t)((GyroData[14]<<8)+ GyroData[13]) / 10.0f;


		GyroDataPacket.mx = (int16_t)((GyroData[24]<<8)+ GyroData[23]) / 1000.0f; 
		GyroDataPacket.my = (int16_t)((GyroData[26]<<8)+ GyroData[25]) / 1000.0f;
		GyroDataPacket.mz = (int16_t)((GyroData[28]<<8)+ GyroData[27]) / 1000.0f;
	
	//-----------------------繁琐而复杂的旧代码和旧变量设定--------------------//
		gx = (s16)((GyroData[17]<<8)+ GyroData[16]) / 10.0; 
		gy = (s16)((GyroData[19]<<8)+ GyroData[18]) / 10.0;
		gz = (s16)((GyroData[21]<<8)+ GyroData[20]) / 10.0;
	
    watch_gx=gx*100;	
	  watch_gy=gy*100;
	  watch_gz=gz*100;

		l_pitch   = (int16_t)((GyroData[31]<<8)+ GyroData[30]) / 100.0 ;
		l_roll    = (int16_t)((GyroData[33]<<8)+ GyroData[32]) / 100.0;
		l_yaw     = (int16_t)((GyroData[35]<<8)+ GyroData[34]) / 10.0;

}

u8 HI229_Get_Gyroscope_Location(float *yaw, float *pitch, float *roll)
{
	*roll  = (int16_t)((GyroData[31]<<8)+ GyroData[30]) / 100.0f;
	*pitch = (int16_t)((GyroData[33]<<8)+ GyroData[32]) / 100.0f;
	*yaw   = (int16_t)((GyroData[35]<<8)+ GyroData[34]) / 10.0f ;
	return 0;
}


u8 HI229_Get_Gyroscope_Speed(float *gx, float *gy, float *gz)
{
	*gx = (int16_t)((GyroData[17]<<8)+ GyroData[16]) / 10.0f; 
	*gy = (int16_t)((GyroData[19]<<8)+ GyroData[18]) / 10.0f;
	*gz = (int16_t)((GyroData[21]<<8)+ GyroData[20]) / 10.0f;
	return 0;
}


u8 HI229_Get_Gyroscope_Acceleration(float *ax, float *ay, float *az)
{
	*ax = (int16_t)((GyroData[10]<<8)+ GyroData[9]) / 10.0f; 
	*ay = (int16_t)((GyroData[12]<<8)+ GyroData[11]) / 10.0f;
	*az = (int16_t)((GyroData[14]<<8)+ GyroData[13]) / 10.0f;
	return 0;
}