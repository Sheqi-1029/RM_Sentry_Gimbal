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
	/*float Kp0;//������pidʹ�õĲ���
	float Ki0;
	float a;
	float b;
	float c;
	float K1;
	float K0;
	float e0;*/
	
	///////////////////////�����Ǳ�ṹPID  �Ĳ���
	float Ap;           //�����ֹʱ��kpֵ����ֹkp�����������
	float Bp;						//��������Kp����ʱ��Kpֵ����Err����ʱ��֤������Ӧ��
	float Cp;						//Kp���������

} PID;

typedef struct
{
	float AP,BP,CP; /*AP,BP,CPΪ��ʵ�����������ϴ�ʱ��Kpȡ���ֵΪAP+BP����e=0ʱ��Kpȡ��СֵΪAP��
                                        *BP��Kp�ı仯���䣻
                                        *����CP�Ĵ�С�ɵ���KP�仯������ 
                                        */

	float AI,CI,K0,K1,Ki0;/*AI,CIΪ��ʵ����Ki��ȡֵ��ΧΪ[0,AI]��CI ������Ki�ı仯���ʡ�
	                                       *Ki0���ٶ����e�ĺ�������|e|��e0ʱ��Ki0��Ϊ1����Ӱ��������Ļ������ã�
	                                       *����|e|<e0��ʱ��Ki0����������Ŵ�Ki0������ǿ�˻������á�Ki0= K1*exp(-K0*e1); 
                                         *K0,K1��e0֮���Լ����ϵΪ��K1*exp(-K0*e0)=1,Ϊ�˱�֤Ki0������ƽ���仯�ģ�
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

		float clear;//��ʼ��
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
