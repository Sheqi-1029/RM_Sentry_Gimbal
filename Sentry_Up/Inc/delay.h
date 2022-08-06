#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>
/****************************************************
*			Title:		delay.c
*			ChipType:	STM32F405RGT6
*			Lib:		HAL_LL
*			Version:	1.0.2 
*			Data:		2018.01.13
*												LD.
*****************************************************/
void DelayInit(u8 SYSCLK);
void Delayus(u32 nus);
void DelayXms(u32 nms);
#endif





























