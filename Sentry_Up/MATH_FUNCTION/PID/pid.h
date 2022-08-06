#ifndef __PID_H__
#define __PID_H__
#include "sys.h"
typedef struct 
{
	float  Ref;   			// Input: Reference input
	float  Fdb;   			// Input: Feedback input
	float  Err;				// Variable: Error
	
	float  Kp;				// Parameter: Proportional gain
	float  Ki;			    // Parameter: Integral gain
	float  Kd; 		        // Parameter: Derivative gain
	
	float  Up;				// Variable: Proportional output
	float  Ui;				// Variable: Integral output
	float  Ud;				// Variable: Derivative output
	float  OutPreSat; 		// Variable: Pre-saturated output
	float  OutMax;		    // Parameter: Maximum output
	float  OutMin;	    	// Parameter: Minimum output
	float  Out;   			// Output: PID output
	float  SatErr;			// Variable: Saturated difference
	float  Kc;		     	// Parameter: Integral correction gain
	float  Up1;		   	    // History: Previous proportional output
	void  (*calc)();	  	// Pointer to calculation function
	void  (*clear)();
	/*float Kp0;//非线性pid使用的参数
	float Ki0;
	float a;
	float b;
	float c;
	float K1;
	float K0;
	float e0;*/
	
	///////////////////////以下是变结构PID  的参数
	float Ap;           //电机静止时的kp值（防止kp过大而抖动）
	float Bp;						//电机以最大Kp运行时的Kp值（在Err过大时保证快速响应）
	float Cp;						//Kp增大的速率

} PID;

typedef struct
{
	float AP,BP,CP; /*AP,BP,CP为正实常数。当误差较大时，Kp取最大值为AP+BP；当e=0时，Kp取最小值为AP；
                                        *BP是Kp的变化区间；
                                        *调整CP的大小可调整KP变化的速率 
                                        */

	float AI,CI,K0,K1,Ki0;/*AI,CI为正实数，Ki的取值范围为[0,AI]，CI 决定了Ki的变化速率。
	                                       *Ki0是速度误差e的函数，当|e|≥e0时，Ki0恒为1，不影响控制器的积分作用；
	                                       *而当|e|<e0。时，Ki0将积分增益放大Ki0倍，加强了积分作用。Ki0= K1*exp(-K0*e1); 
                                         *K0,K1和e0之间的约束关系为：K1*exp(-K0*e0)=1,为了保证Ki0是连续平滑变化的，
                                        */

	
  float AD,BD,CD;
	float e0,e1;
}PidParameter;

typedef struct
{
    float Expect;
	  float Real;	 
	  float Out;
	  float e1, e2, e3, lastout;	 
	  float Kp,Ki,Kd;
	  float GRMExpect, GRMReal, GRMOut;

		float clear;//初始化
}Pid;
/*------------------------------------------------------------------------------
Prototypes for the functions in PID.C
------------------------------------------------------------------------------*/
void PidCalc(PID *);
void PidClear(PID *v);
void PIDControl1(Pid* motor,PidParameter* Pmotor);
float GetMiddleValue( float *pusBuffer, uint8_t ucLength );
void BubbleSort_int16(float *a,int len);
float LowPassFilter_RemoveExtremumAverage(float data[],int16_t length);
#endif
