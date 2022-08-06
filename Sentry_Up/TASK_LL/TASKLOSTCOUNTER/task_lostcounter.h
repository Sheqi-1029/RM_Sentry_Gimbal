/****************************************************
*			Title:		LOSTCounter
*			ChipType:	STM32F405RGT6
*			Version:	1.2.0
*			Date:			2017.09.20
*												LD.
*****************************************************/

#ifndef LOSTCOUNTER_H
#define LOSTCOUNTER_H
#include "sys.h"
#include "BSPconfig.h"

#if	CONFIG_USE_LOSTCOUNTER
	void LostCounterInit(void);
	u32* GetLostCounterData(void);
	void LostCounterFeed(u8 i);
	u16 LostCounterCount(void);
	void LostCounterControl(u16 SystemErrorStatus);

typedef struct
{
	u16 system_error_status;
	u32 count_number[NUMBERS_OF_COUNT];
}LostCounterStruct;

#endif



#endif
