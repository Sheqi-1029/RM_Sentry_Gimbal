#include "pid.h"
#include "math.h"
#include "sys.h"

//将PID函数放在RAM中
#pragma arm section code = "RAMCODE" 

void PidCalc(PID *v)
{	
		//变结构//
		v->Kp = v->Ap + v->Bp*(1 - exp(-v->Cp*fabs(v->Err))); 
    // Compute the error
    v->Err = v->Ref - v->Fdb;
    // Compute the proportional output
    v->Up = v->Kp*v->Err;
    // Compute the integral output
    v->Ui = v->Ui + v->Ki*v->Up + v->Kc*v->SatErr;
	  if(v->Ui>0.5 && v->Err<0)//转向之后积分项清零
	  {
		  v->Ui=0;
	  }
	  if(v->Ui<-0.5 && v->Err>0)
		{
			v->Ui=0;
		}
    // Compute the derivative output
    v->Ud = v->Kd*(v->Up - v->Up1);
    // Compute the pre-saturated output
    v->OutPreSat = v->Up + v->Ui + v->Ud;     
    
    // Saturate the output
    if (v->OutPreSat > v->OutMax)                   
      v->Out =  v->OutMax;
    else if (v->OutPreSat < v->OutMin)
      v->Out =  v->OutMin;  
    else
      v->Out = v->OutPreSat;
		
    // Compute the saturate difference
    v->SatErr = v->Out - v->OutPreSat;
    // Update the previous proportional output
    v->Up1 = v->Up;
}


void PidClear(PID *v)
{
		v->Ui = 0;
		v->Ud = 0;
}

#pragma arm section


void PIDControl1(Pid* motor,PidParameter* Pmotor)
 {
	 
 }
 
 float GetMiddleValue( float *pusBuffer, u8 ucLength )
{
	u8 i;
	uint16_t usCount = 0,usPoint;
	float usMiddle,usMax;
	while( usCount <= (ucLength/2) )
		{
			usPoint = 0;
			usMax = pusBuffer[0];
			for( i=1;i<ucLength;i++ )
			{
				if( pusBuffer[i] > usMax )
					{
						usMax = pusBuffer[i];
						usPoint = i;
					}
			}
			usMiddle = usMax;
			//pusBuffer[usPoint] = 0;
			usCount++;
		}
		return usMiddle;
}


void BubbleSort_int16(float *a,int len)
   {
        int i;
        int j;
        float mid;
        for(i=0;i<len;i++)
        {
            for(j=0;j<len-i-1;j++)
            {
                if(*(a+j)>*(a+j+1))
                {
                    mid=*(a+j);
                    *(a+j)=*(a+j+1);
									  *(a+j+1)=mid;
								}
						}
				}
	 }
	 
	 
float LowPassFilter_RemoveExtremumAverage(float data[],int16_t length)
{
  #define EXTERMUM_NUMBER 1 //number of extermum
  int32_t sum=0;
  int16_t i;

  if(length<=2*EXTERMUM_NUMBER)
  {
    return 0;//length shorter than 2*EXTERMUM_NUMBER, return err
  }

  //sort
  BubbleSort_int16(data,length);

  //average filter
  for(i=EXTERMUM_NUMBER;i<length-EXTERMUM_NUMBER;i++)
  {
    sum+=data[i];
  }

  return sum/(length-2*EXTERMUM_NUMBER);
}


