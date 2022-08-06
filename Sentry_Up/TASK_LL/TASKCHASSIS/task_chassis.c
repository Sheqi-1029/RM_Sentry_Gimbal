#include "task_chassis.h"
#include "driver_chassis.h"


ChassisSpeedMessegePort ChassisSpeed;

void ChassisSetSpeed(float SpeedX,float SpeedY,float Spin)
{
	ChassisSpeed.SetSpeedX	=	 SpeedX;
	ChassisSpeed.SetSpeedY	=	 SpeedY;
	ChassisSpeed.Spin	=	 Spin;
}

void ChassisControlTask()
{
//	ChassisFollowCalculate(&ChassisSpeed);

//	ChassisShakeCalculate(&ChassisSpeed);
//	ChassisChangeFollow(&ChassisSpeed);
	
	//ChassisControl(ChassisSpeed);
}
