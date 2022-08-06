#include "data_task_supply.h"
#include "data_channel_wifi.h"
#include "interface_base.h"

extern u8 role_id;
//补给站状态信息频率控制
u32 supply_clock_count_1=0;

//用户发送战车离开补给站信息的flag
u8 user_send_leave_supply_flag=0;



