#ifndef __INTERFACE_SUPPLY_H
#define __INTERFACE_SUPPLY_H
#include "sys.h"

//补给站状态信息
typedef __packed struct
{
	//左边可用弹仓数量
	u8 leftMagazines;
	//右边可用弹仓数量
	u8 rightMagazines;
}Supply_Condition_TypeDef;


#endif
