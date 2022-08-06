/****************************************************
*			Title:		ø¥√≈π∑
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.10.18
*												HH.
*****************************************************/
#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "sys.h"
#include "BSPconfig.h"

void WatchDogInit(void);
void FeedIndependentWatchDog(void);
void FeedWindowWatchDog(void);

#endif
