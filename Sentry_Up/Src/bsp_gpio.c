#include "bsp_gpio.h"
#include "sys.h"


void GPIO_Config(void)
{
	PBout(13)=1;
	PBout(14)=1;
}

