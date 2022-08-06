#include "driver_laser.h"

void LaserControl(unsigned char judge)
{
	PCout(7)=judge;
}

