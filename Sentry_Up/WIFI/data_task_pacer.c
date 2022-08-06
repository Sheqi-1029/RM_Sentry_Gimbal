#include "data_task_pacer.h"
#include "data_channel_wifi.h"
#include "interface_base.h"
#include "data_channel_pc.h"
#include "data_channel_judge_system.h"
#include "task_chassis.h"

Self_Check_TypeDef self_check;

extern u8 role_id;
extern u8 wifi_send_flag;
extern u8 init_Wifi_flag;
extern float Power_Judge;


//用户发送基地伤害信息的flag
u8 user_send_matrix_hurt_flag=0;

//整车状态信息频率控制
u32 pacer_clock_count_1=0;
//裁判系统信息频率控制
u32 send_wifi_count_5=0;
//前方敌人信息频率控制
u32 send_wifi_count_7=0;

//调试车信息频率控制
u32 debug_vehicle_data_count_1=0;


extern u8 race_progress_data[35];

extern Shoot_Data_TypeDef shoot_data;
//剩余小子弹数量
extern u16 remain_small_bullet_num;
//剩余大子弹数量
extern u16 remain_big_bullet_num;

void clock_pacer_task(){
	pacer_clock_count_1++;
	send_wifi_count_5++;
	send_wifi_count_7++;
	debug_vehicle_data_count_1++;
}

void pacer_task(){

}


//发送调试车数据
Debug_Vehicle_Data debug_vehicle_data;
Debug_Vehicle_Data_Union debug_vehicle_data_union;
void send_debug_vehicle_data_task(){

}
extern float Current_Save;
extern float Energy_Integral;
extern float SuperC_Voltage;
//extern int J_60;
//extern float Current_Limit;
//发送整车状态信息任务
Pacer_Vehicle_Condition_TypeDef pacer_vehicle_condition;

int test0_sun=0;
void send_pacer_vihecle_condition_task(){

}
