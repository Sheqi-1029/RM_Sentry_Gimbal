#ifndef __DATA_ANALYSIS_TOOL_CRC_H
#define __DATA_ANALYSIS_TOOL_CRC_H
#include "sys.h"

unsigned  char  Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int
	dwLength,unsigned char ucCRC8);
u16 Get_CRC16_Check_Sum(u8 *pchMessage,u32 dwLength,u16
	wCRC);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC16_Check_Sum(u8 *pchMessage, u32 dwLength);




#endif
