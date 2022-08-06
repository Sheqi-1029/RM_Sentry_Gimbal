#include "kalman.h"

/***************************卡尔曼滤波基本公式******************************
1.预测阶段：
			a.先验估计  ： X(k|k-1) = AX(k-1|k-1) + BU(k)
									X(k|k-1)	 ————————   根据上一次状态的预测结果
									X(k-1|k-1) ————————   上一次的最优结果
									U(k)       ————————   现在状态的控制量
									A,B        ————————   矩阵系数
									
			b.误差协方差： P(k|k-1)	= A P(k-1|k-1) AT + Q
									P(k|k-1)	 ————————   X(k|k-1)的协方差
									X(k-1|k-1) ————————   X(k-1|k-1)的协方差
									Q          ————————   估计过程，误差协方差
									
2.校正阶段：
			a.计算卡尔曼增益： Kg(k) = P(k|k-1) HT/(H P(k-1|k-1))
									H          ————————   系数矩阵
									R          ————————   测量值的噪声协方差
			b.修正估计      ： X(k|k) = X(k|k-1) + Kg(k)(Z(K)-HX(k|k-1))
									Z(K)       ————————   测量值
			c.更新误差协方差： P(k|k) = (I-Kg(k)H)P(k|k-1)
									I          ————————   单位矩阵

******************************************************************************/ 

void KalmanInit(KALMAN* klm)
{
    /* We will set the variables like so, these can also be tuned by the user */
    klm->Q_angle = 0.001;
    klm->Q_bias = 0.003;
    klm->R_measure = 0.03;

    klm->angle = 0; // Reset the angle
    klm->bias = 0; // Reset bias

    klm->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en->wikipedia->org/wiki/Kalman_filter#Example_application->2C_technical
    klm->P[0][1] = 0;
    klm->P[1][0] = 0;
    klm->P[1][1] = 0;
}

void KalmanCalc(KALMAN* klm, double NewAngle, double NewRate, double dt)
{
		/* Step 1 */
    klm->rate = NewRate - klm->bias;
    klm->angle += dt * klm->rate;
	
	  /* Step 2 */
    klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
    klm->P[0][1] -= dt * klm->P[1][1];
    klm->P[1][0] -= dt * klm->P[1][1];
    klm->P[1][1] += klm->Q_bias * dt;
	
	  /* Step 3 */
    klm->S = klm->P[0][0] + klm->R_measure;
	
    /* Step 4 */
    klm->K[0] = klm->P[0][0] / klm->S;
    klm->K[1] = klm->P[1][0] / klm->S;
	
	  /* Step 5 */
    klm->y = NewAngle - klm->angle;
	
    /* Step 6 */
    klm->angle += klm->K[0] * klm->y;
    klm->bias += klm->K[1] * klm->y;
		
		/* Step 7 */
    klm->P[0][0] -= klm->K[0] * klm->P[0][0];
    klm->P[0][1] -= klm->K[0] * klm->P[0][1];
    klm->P[1][0] -= klm->K[1] * klm->P[0][0];
    klm->P[1][1] -= klm->K[1] * klm->P[0][1];
}
	
void SetAngle(KALMAN* klm,double newAngle) // Used to set angle, this should be set as the starting angle
{ 
		klm->angle = newAngle; 
} 

double GetRate(KALMAN* klm)// Return the unbiased rate 
{ 
		return klm->rate; 
} 

void SetQangle(KALMAN* klm , double newQ_angle) 
{ 
		klm->Q_angle = newQ_angle; 
}

void SetQbias(KALMAN* klm , double newQ_bias) 
{ 
		klm->Q_bias = newQ_bias; 
}
void SetRmeasure(KALMAN* klm , double newR_measure) 
{ 
		klm->R_measure = newR_measure; 
}

double GetQangle(KALMAN* klm) 
{ 
		return klm->Q_angle; 

}
double GetQbias(KALMAN* klm) 
{ 
		return klm->Q_bias; 
}

double GetRmeasure(KALMAN* klm) 
{ 	
		return klm->R_measure;
} 

