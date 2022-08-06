#ifndef __DATA_CHANNEL_JUDGE_SYSTEM_H
#define __DATA_CHANNEL_JUDGE_SYSTEM_H	 
#include "sys.h"
#include "interface_supply.h"

//血量变化信息
typedef __packed struct
{
	u8 weakId:4;
	u8 way:4;
	u16 value;
}tRealBloodChangedData;

//实时射击信息
typedef __packed struct
{
	float realBulletShootSpeed;
	float realBulletShootFreq;
	float realGolfShootSpeed;
	float realGolfShootFreq;
}tRealShootData;


typedef __packed struct  
{  
	//机器人阵型标志位
	//0x0：红方
	//0x1：蓝方
	uint8_t RobotColor :2;  
	//红方基地状态标志位
	//0x0：基地普通状态
	//0x1：基地无敌
	uint8_t RedBaseSta :2;
	//蓝方基地状态标志位
	//0x0：基地普通状态
	//0x1：基地无敌
	uint8_t BlueBaseSta:2;
	//资源岛登岛标志位
	//0x0：无机器人登岛
	//0x1：红方英雄登岛
	//0x2：蓝方英雄登岛
	//0x3：双方英雄均登岛
	uint8_t IslandLanding :2;  

	//恢复立柱状态
	//红方停机坪状态
	//0x0：恢复立柱无效
	//0x1：可被激活
	//0x2：正在被激活
	//0x3：已激活
	//0x4：冷却中
	uint8_t RedAirPortSta :4;  
	//蓝方停机坪状态
	uint8_t BlueAirPortSta:4;  

	//机关立柱状态
	//0x0：立柱无效
	//0x1：立柱可被占领
	//0x2：红方正在占领立柱
	//0x3：蓝方正在占领该立柱
	//0x4：红方已占领
	//0x5：蓝方已占领
	//1号
	uint8_t No1PillarSta:4;  
	//2号
	uint8_t No2PillarSta:4;
	//3号
	uint8_t No3PillarSta:4;  
	//4号
	uint8_t No4PillarSta:4;  
	//5号
	uint8_t No5PillarSta:4;  
	//6号
	uint8_t No6PillarSta:4;  

	//加子弹机构状态
	//红方加弹机构状态
	//0x00：停止加弹
	//0x01：加弹中
	uint8_t RedBulletBoxSta :4;  
	//蓝方加弹机构状态
	uint8_t BlueBulletBoxSta:4;  
	//红方加弹数量
	uint16_t RedBulletAmount;  
	//蓝方加弹数量
	uint16_t BlueBulletAmount;  

	//大符状态
	//0x0：大符无效
	//0x1：可被激活
	//0x2：正在被红方激活
	//0x3：正在被蓝方激活
	//0x4：已被红方激活
	//0x5：已被蓝方激活
	//0号大符状态
	uint8_t No0BigRuneSta :4;  
	//1号大符状态
	uint8_t No1BigRuneSta :4;  

	//防御加成百分比
	uint8_t AddDefendPrecent;  
}tStudentPropInfo;

typedef __packed struct{
	//数据1
	float data1;
	//数据2
	float data2;
	//数据3
	float data3;
}tRealCustomMessage;

typedef union{
	char byte_data[12];
	tRealCustomMessage packed_data;
}Custom_Message_TypeDef;
typedef union{
	unsigned char byte_data[16];
	tRealShootData packed_data;
}Shoot_Data_TypeDef;
typedef union{
	char byte_data[12];
	tStudentPropInfo packed_data;
}Field_Data_TypeDef;


#endif
