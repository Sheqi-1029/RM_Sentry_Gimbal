#include "driver_vision.h"
#include "pid.h"
/***********
			该程序从视觉处拷来，具体源代码参照视觉组
			*********************************////
			                    //      
#define OFFSET_X 			(0)
#define OFFSET_Y 			(100.0)
#define OFFSET_Z 			(-38.5)
#define VISION_K 			(1.0)//     干燥0.5   潮湿1.0(?)
#define VISION_V 			(19.7)//   23.5  21    23.3打下一个装甲板    22.9打上一个装甲板
#define VISION_YAW 		(0.3)//     0.0
#define VISION_PITCH 	(-0.6)	//       0      0

#define VISION_PITCH_KP (0.0)
#define VISION_PITCH_KI (0.0)
#define VISION_PITCH_KD (0.0)//0
#define VISION_PITCH_AP (-1.0)//
#define VISION_PITCH_BP (-2)//
#define VISION_PITCH_CP (-0.1)//

#define VISION_YAW_KP (0.0)
#define VISION_YAW_KI (0)
#define VISION_YAW_KD (0.0)//0
#define VISION_YAW_AP (-1.3)//
#define VISION_YAW_BP (-2)//
#define VISION_YAW_CP (-0.1)//


GimbalCtrl VisionGimbalCtrl;
PID VisionPitchIncreasement = {0}, VisionYawIncreasement = {0};

/**
 * @brief Init the Transformation matrix from camera to ballistic //TODO: write in ros tf
 * @param x Translate x, 单位mm
 * @param y Translate y, 单位mm
 * @param z Translate z, 单位mm
 * @param pitch 摄像头与枪管的pitch角度差, 单位角度（deg）
 * @param yaw 摄像头与枪管的yaw角度差, 单位角度（deg）
 * @param init_v 初速度，单位m/s
 * @param init_k 空气摩擦因数，默认为0.1
 */
void VisionInit(void)
{
    VisionGimbalCtrl.offset_x = OFFSET_X;
    VisionGimbalCtrl.offset_y = OFFSET_Y;
    VisionGimbalCtrl.offset_z = OFFSET_Z;
    VisionGimbalCtrl.init_k = VISION_K;
    VisionGimbalCtrl.init_v = VISION_V;
    VisionGimbalCtrl.offset_yaw = VISION_YAW;
    VisionGimbalCtrl.offset_pitch = VISION_PITCH;
	
	  VisionPitchIncreasement.Kp = VISION_PITCH_KP;
    VisionPitchIncreasement.Ki = VISION_PITCH_KI;
    VisionPitchIncreasement.Kd = VISION_PITCH_KD;
    VisionPitchIncreasement.Ap = VISION_PITCH_AP;
    VisionPitchIncreasement.Bp = VISION_PITCH_BP;
    VisionPitchIncreasement.Cp = VISION_PITCH_CP;
    VisionPitchIncreasement.OutMax = 3;
    VisionPitchIncreasement.OutMin = -3;
    VisionPitchIncreasement.calc = &PidCalc;
    VisionPitchIncreasement.clear = &PidClear;
    VisionPitchIncreasement.clear(&VisionPitchIncreasement);

    VisionYawIncreasement.Kp = VISION_YAW_KP;
    VisionYawIncreasement.Ki = VISION_YAW_KI;
    VisionYawIncreasement.Kd = VISION_YAW_KD;
    VisionYawIncreasement.Ap = VISION_YAW_AP;
    VisionYawIncreasement.Bp = VISION_YAW_BP;
    VisionYawIncreasement.Cp = VISION_YAW_CP;
    VisionYawIncreasement.OutMax = 0.0005;
    VisionYawIncreasement.OutMin = -0.0005;
    VisionYawIncreasement.calc = &PidCalc;
    VisionYawIncreasement.clear = &PidClear;
    VisionYawIncreasement.clear(&VisionPitchIncreasement);
}

/**
 * @fn BulletModel:定义了SIGNGLE_DIRECTION是，定义是单向空气阻力模型，否则为双向空气阻力模型
 * @brief 完全的弹道模型（考虑x与y方向上的空气阻力）
 * @param x x方向上的位移（敌方装甲板到枪口的距离，tVec的[0]位置）
 * @param v 子弹出射初速度
 * @param angle 迭代当前pitch角度
 * @return 本次迭代所计算的pitch实际值（位置）
 */
float BulletModel(float x, float v, float angle) 
{
    return x * GRAVITY/(VisionGimbalCtrl.init_k * v * cos(angle)) + tan(angle) * x +
        1/(VisionGimbalCtrl.init_k * VisionGimbalCtrl.init_k) * GRAVITY * log(1-VisionGimbalCtrl.init_k * x/ (v * cos(angle)));
}

/** @brief 以上弹道模型应该是有错误的 @ref GetPitch, 由于PnP求出的(x, y, z)定义引起
 * @param pit 云台当前pitch， 水平时为0, 向上为正
 * @param xyz 最佳装甲板PnP解算结果（按照相机坐标系的定义）
 * @param v 子弹速度（摩擦轮 + 底盘平移运动在平行于枪管上的分量）
 * @param t 输出 子弹击中目标延迟
 * @return pitch增量
 */
float pnpGetPit(float pit, float x, float y, float z, float v, float *t){
    /// 注意t为pit转弧度
    *t = pit / RAD2DEG;
    double d = sqrt(pow((double)x, 2) + pow((double)z, 2)),          // 过渡量
        dist = fabs(y * sin(*t) + d * cos(*t));
    double y_temp, y_actual, dy, ypos, ang = *t;
    y_temp = dist * tan(*t);
    ypos = d * sin(*t) - y * cos(*t);
    for (int i = 0; i < 25; ++i){
        ang = atan2(y_temp, dist);
        y_actual = BulletModel(dist, v, ang);
        dy = ypos - y_actual;
        y_temp += dy;
        if (fabs(dy) < 0.001) {
            break;
        }
    }
    *t =(-1/(VisionGimbalCtrl.init_k) * log(1-VisionGimbalCtrl.init_k * dist/(v*cos(ang))));
    return ang * RAD2DEG - pit;
}

/**
 * @brief Get the ballistic control info.
 * @param postion Enemy position(actually it should be the target armor).
 * @param pitch Input and output actual pitch angle
 * @param yaw Input and output actual yaw angle
 * @param delay shoot delay while the ball is in the air towards the target
 * 所有的输入参数中，position的单位是mm,pitch，yaw的单位是角度（deg）
 */
void Transform(Point3fStruct position,float inPitch,float inYaw, float *pitch, float *yaw,
    float *delay, float trans_vel, float forward_vel)//FOWARD是子弹速度+前进分量
	{
    //y取负数是因为，图像中tVec为正时，装甲板在摄像头正对处下方
    //如果车在z轴方向存在运动，则加上运动速度forward_vel
    *pitch = pnpGetPit(inPitch, position.x / 1000, (position.y - VisionGimbalCtrl.offset_y) / 1000, (position.z + VisionGimbalCtrl.offset_z) / 1000, 
        forward_vel + VisionGimbalCtrl.init_v, delay) + VisionGimbalCtrl.offset_pitch;
    //yaw轴是逆时针为正
    //使用横向速度trans_vel时注意，向右为正（与相机坐标系一致）,则在角度解算时，子弹飞行时间车本身会位移，x需要- 速度 × 时间
    *yaw = -atan2(position.x + VisionGimbalCtrl.offset_x - trans_vel * (*delay), position.z + VisionGimbalCtrl.offset_z) * RAD2DEG
            + VisionGimbalCtrl.offset_yaw;
		*delay=*delay/0.375;//同系统时钟保持一致
		if (fabs(*yaw)>50)
		{
			*yaw=0;
		}
}
	


