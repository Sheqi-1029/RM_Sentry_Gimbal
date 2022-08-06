/****************************************************
*			Title:		Ä¦²ÁÂÖ
*			Version:	1.0.0
*			Data:			2016.12.06
*												LD.
*****************************************************/
#ifndef __DRIVERFRICTION_H__
#define __DRIVERFRICTION_H__
#include "sys.h"


void FrictionControl(u8 Judge);
void FrictionStop(void);
void FrictionStart(void);
void FrictionDriver(int FRAR,int FRAL);
uint8_t Getfriction_Full(void);
#endif
