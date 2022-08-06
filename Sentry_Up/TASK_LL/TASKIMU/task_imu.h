#ifndef _TASK_IMU_
#define _TASK_IMU_

void IMUKalmanInit(void);
void IMU_CalcYawPitchRollValue(void);
void IMU_UpdateMagValue(void);

#endif

