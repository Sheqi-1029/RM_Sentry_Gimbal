#include <stdio.h>
#include <math.h>

#define GRAVITY (9.7944)
#define RAD2DEG  (57.29578)             //180/PI
#define DEFAULT_K_42  (0.035544 / 4)       //42mm弹丸的初始k值 ///0.017772f;        
#define DEFAULT_K_17 (0.08334 / 4)       //17MM弹丸的初始k值

typedef struct{
    float offset_x;
    float offset_y;
    float offset_z;
    float init_k;
    float init_v;
    float offset_yaw;
    float offset_pitch;
}GimbalCtrl;

typedef struct{
    float x;
    float y;
    float z;
}Point3fStruct;
void Transform(Point3fStruct position,float inPitch,float inYaw, float *pitch, float *yaw,
    float *delay, float trans_vel, float forward_vel);//FOWARD是子弹速度+前进分量
void VisionInit(void);

