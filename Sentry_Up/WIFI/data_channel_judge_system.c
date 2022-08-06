/**
 * 裁判系统发送与接收逻辑处理，对数据进行封装，持有发送数据缓存，通过带参函数向上层提供裁判系统发送能力。
 */
#include "data_channel_judge_system.h"
#include "data_task_main.h"
#include "data_analysis_tool_crc.h"
#include "data_analysis_judge_system.h"
#include "interface_base.h"
#include "data_channel_wifi.h"


//part16：发给裁判系统的自定义信息
#define DATA_PART_16_LENGTH 21
//发给裁判系统的自定义信息
char send_custom_message_flag=0;
u8 wifi_data_custom_message[DATA_PART_16_LENGTH];

//自定义裁判系统数据-步兵
Custom_Message_TypeDef custom_data_pacer;
//自定义裁判系统数据-工程车
Custom_Message_TypeDef custom_data_engineer;
//自定义裁判系统数据-英雄车
Custom_Message_TypeDef custom_data_hero;

//场地机关状态数据
Field_Data_TypeDef field_data;

extern u8 role_id;
extern Energy Referee_Earn_Energy;
extern Refereevoltage Referee_Voltage;
extern Refereecurrent Referee_Current;
extern  float Refereepower;




