#include "task_imu.h"
#include "driver_mpu9250.h"
#include "kalman.h"
#include "bsp_tim.h"
#include "math.h"

#define RAD_TO_DEG 57.295779513082320876798154814105  // 弧度转角度
#define DEG_TO_RAD 0.01745329251994329576923690768489 // 角度转弧度

#define USE_FIRST_WAY_CALC_PITCH_ROLL 1

typedef struct{
	KALMAN x;
	KALMAN y;
	KALMAN z;
}IMUKalmanStruct;

typedef struct
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值

    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	
		short Mag_X;    //寄存器原生X轴磁力计表示值
		short Mag_Y;    //寄存器原生Y轴磁力计表示值
		short Mag_Z;    //寄存器原生Z轴磁力计表示值
	
//	  short Temp;     //寄存器原生温度表示值
}MPU9250_RAW_DATA;

typedef struct
{
		double Row;
		double Pitch;
		double Yaw;
		double AccelRow;
	  double AccelPitch;
		double MagYaw;
}MPU9250_STATE;

typedef struct
{
		IMUKalmanStruct     IMUKalman;
		MPU9250_RAW_DATA    MPU9250_Raw_Data;    //原始数据
		MPU9250_STATE       MPU9250_State;       //结算结果
}IMULogic;

IMULogic IMU;

void IMUKalmanInit(void)
{
    KalmanInit(&IMU.IMUKalman.x);
	  KalmanInit(&IMU.IMUKalman.y);
	  KalmanInit(&IMU.IMUKalman.z);
}

void GetMPU9250AccGyroRawData(void)
{
	
		MPU_Get_Accelerometer(&IMU.MPU9250_Raw_Data.Accel_X , &IMU.MPU9250_Raw_Data.Accel_Y , &IMU.MPU9250_Raw_Data.Accel_Z);
		MPU_Get_Gyroscope(&IMU.MPU9250_Raw_Data.Gyro_X , &IMU.MPU9250_Raw_Data.Gyro_Y , &IMU.MPU9250_Raw_Data.Gyro_Z);
}
void GetMPU9250MagRawData(void)
{
		MPU_Get_Magnetometer(&IMU.MPU9250_Raw_Data.Mag_X , &IMU.MPU9250_Raw_Data.Mag_Y , &IMU.MPU9250_Raw_Data.Mag_Z);
}

void AccelUpdatePitchRoll(void) //根据加速度计刷新PITCH,ROLL，两种方法
{
#if USE_FIRST_WAY_CALC_PITCH_ROLL
    IMU.MPU9250_State.AccelRow   = atan2(IMU.MPU9250_Raw_Data.Accel_Y,IMU.MPU9250_Raw_Data.Accel_Z) * RAD_TO_DEG;
    IMU.MPU9250_State.AccelPitch = atan(-IMU.MPU9250_Raw_Data.Accel_X / sqrt(IMU.MPU9250_Raw_Data.Accel_Y * IMU.MPU9250_Raw_Data.Accel_Y + IMU.MPU9250_Raw_Data.Accel_Z * IMU.MPU9250_Raw_Data.Accel_Y)) * RAD_TO_DEG;
#else
		IMU.MPU9250_State.AccelRow   = atan(IMU.MPU9250_Raw_Data.Accel_Y / sqrt(IMU.MPU9250_Raw_Data.Accel_X * IMU.MPU9250_Raw_Data.Accel_X + IMU.MPU9250_Raw_Data.Accel_Z * IMU.MPU9250_Raw_Data.Accel_Z)) * RAD_TO_DEG;
    IMU.MPU9250_State.AccelPitch = atan2(-IMU.MPU9250_Raw_Data.Accel_X, IMU.MPU9250_Raw_Data.Accel_Z) * RAD_TO_DEG;
#endif
}
 
void MagUpdateYaw() //根据磁力计刷新YAW
{ 
    double RollAngle,PitchAngle,Bfy,Bfx;
		short MagX,MagY,MagZ;
  
			MagX = -IMU.MPU9250_Raw_Data.Mag_X; 
			MagY = IMU.MPU9250_Raw_Data.Mag_Y;
			MagZ = -IMU.MPU9250_Raw_Data.Mag_Z;
   
//    magX *= magGain[0];
//    magY *= magGain[1];
//    magZ *= magGain[2];
//   
//    magX -= magOffset[0];
//    magY -= magOffset[1];
//    magZ -= magOffset[2];   
   
    RollAngle  = IMU.IMUKalman.x.angle * DEG_TO_RAD;
    PitchAngle = IMU.IMUKalman.y.angle * DEG_TO_RAD;
   
    Bfy = MagZ * sin(RollAngle) - MagY * cos(RollAngle);
    Bfx = MagX * cos(PitchAngle) + MagY * sin(PitchAngle) * sin(RollAngle) + MagZ * sin(PitchAngle) * cos(RollAngle);
    IMU.MPU9250_State.MagYaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
   
//    yaw *= -1;
}

volatile uint32_t LastUpdate, Now;
double dt,dtAdd = 0;
void IMU_CalcYawPitchRollValue(void)
{		
		double GyroXval,GyroYval,GyroZval;
	
		GetMPU9250AccGyroRawData();
		AccelUpdatePitchRoll();
	
		if(dtAdd>0.01)//每10ms读取一次磁力计
		{
				GetMPU9250MagRawData();
				MagUpdateYaw();
				dtAdd = 0;
		}
		
		GyroXval = (double)IMU.MPU9250_Raw_Data.Gyro_X / 32.8;
		GyroYval = (double)IMU.MPU9250_Raw_Data.Gyro_Y / 32.8;
		GyroZval = (double)IMU.MPU9250_Raw_Data.Gyro_Z / 32.8;
	
		Now = GetTimeMicros();//读取时间 单位是us
    if(Now<LastUpdate)
				dt =  (double)((double)(Now + (0xffffffff- LastUpdate)) / 1000000.0f); 
    else
        dt =  (double)((double)(Now - LastUpdate) / 1000000.0f);
    LastUpdate = Now;	//更新时间
		dtAdd += dt;

 // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees		
		if((IMU.MPU9250_State.AccelRow<-90&&IMU.IMUKalman.x.angle>90)||(IMU.MPU9250_State.AccelRow>90&&IMU.IMUKalman.x.angle<-90))
				SetAngle(&IMU.IMUKalman.x,IMU.MPU9250_State.AccelRow);
		else	
				KalmanCalc(&IMU.IMUKalman.x , IMU.MPU9250_State.AccelRow , GyroXval ,dt); //Row    以陀螺仪为预测，以加速度计计算值为测量

		if(fabs(IMU.IMUKalman.x.angle)>90)
				GyroYval = -GyroYval;
		KalmanCalc(&IMU.IMUKalman.y ,IMU.MPU9250_State.AccelPitch , GyroYval , dt);//Pitch
		
		if((IMU.MPU9250_State.MagYaw < -90 && IMU.IMUKalman.z.angle > 90) || (IMU.MPU9250_State.MagYaw > 90 && IMU.IMUKalman.z.angle < -90))
				SetAngle(&IMU.IMUKalman.z,IMU.MPU9250_State.MagYaw);
		else
				KalmanCalc(&IMU.IMUKalman.z , IMU.MPU9250_State.MagYaw , GyroZval , dt);//Yaw
				
		IMU.MPU9250_State.Row = IMU.IMUKalman.x.angle;
		IMU.MPU9250_State.Pitch = IMU.IMUKalman.y.angle;
		IMU.MPU9250_State.Yaw = IMU.IMUKalman.z.angle;
}



