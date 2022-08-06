#ifndef __DATA_CHANNEL_WIFI_H
#define __DATA_CHANNEL_WIFI_H	 
#include "sys.h"
#include "interface_pacer.h"
#include "interface_supply.h"
#include "data_channel_judge_system.h"


typedef struct
{
	float Kp[10];
	float Ki[10];
}PID_value;

//简单的PID结构体
typedef struct{
	float kp;
	float ki;
	float kd;
}Simple_PID_TypeDef;

//电机状态信息结构体
typedef struct{
	float i;//电流
	float v;//速度
	float s;//位置
	float p;//功率
}Motor_Condition_TypeDef;

//步兵车状态信息结构体
typedef struct 
{
	//底盘电机1状态信息
	Motor_Condition_TypeDef chassis_motor1_condition;
	//底盘电机2状态信息
	Motor_Condition_TypeDef chassis_motor2_condition;
	//底盘电机3状态信息
	Motor_Condition_TypeDef chassis_motor3_condition;
	//底盘电机4状态信息
	Motor_Condition_TypeDef chassis_motor4_condition;
	//云台电机yaw轴状态信息
	Motor_Condition_TypeDef cradle_yaw_motor_condition;
	//云台电机pitch轴状态信息
	Motor_Condition_TypeDef cradle_pitch_motor_condition;
	//拨弹电机状态信息
	Motor_Condition_TypeDef rammer_motor_condition;
	//摩擦轮电机1状态信息
	Motor_Condition_TypeDef shooter1_motor_condition;
	//摩擦轮电机2状态信息
	Motor_Condition_TypeDef shooter2_motor_condition;
}Pacer_Vehicle_Condition_TypeDef;

//符文状态
typedef enum{
BUFF_TYPE_NONE, //无效
BUFF_TYPE_ARMOR = 0x01, //防御符
BUFF_TYPE_SUPPLY = 0x04, //加血符
BUFF_TYPE_BULLFTS= 0x08, //加弹符
}eBuffType;

//GPS数据
typedef __packed struct
{
u8 flag; //0 无效，1 有效
u32 x;
u32 y;
u32 z;
u32 compass;
}tGpsData;





//IP地址
//char* IP_address="192.168.1.101";
//char* IP_address;
//调试环境：机器人队
#define DEBUG_ENVIRONMENT_RM 1
//调试环境：宿舍
#define DEBUG_ENVIRONMENT_DORMITORY 2
//调试环境：比赛
#define DEBUG_ENVIRONMENT_GAME 3
//调试环境：宿舍，智哥路由
#define DEBUG_ENVIRONMENT_DORMITORY_ZHI 4
//步兵车1AP
#define DEBUG_ENVIRONMENT_GAME_PACER1 5
//步兵车2AP
#define DEBUG_ENVIRONMENT_GAME_PACER2 6
//步兵车3AP
#define DEBUG_ENVIRONMENT_GAME_PACER3 7
//英雄车AP
#define DEBUG_ENVIRONMENT_GAME_HERO 8
//工程车AP
#define DEBUG_ENVIRONMENT_GAME_ENGINEER 9
//基地
#define DEBUG_ENVIRONMENT_GAME_MATRIX 10
//补给站
#define DEBUG_ENVIRONMENT_GAME_SUPPLY 11
//裁判系统网络
#define DEBUG_ENVIRONMENT_JUDGE_NET 12


//WIFI数据种类
//车的状态信息
#define WIFI_MESSAGE_TYPE_VEHICLE_CONDITION 1
//比赛进程信息
#define WIFI_MESSAGE_TYPE_RACE_PROGRESS 2
//基地伤害信息
#define WIFI_MESSAGE_TYPE_MATRIX_HURT 3
//身份确认信息
#define WIFI_MESSAGE_TYPE_CONNECT_ID 4
//召唤工程车信息
#define WIFI_MESSAGE_TYPE_CALL_ENGINEER 5
//测试数据
#define WIFI_MESSAGE_TYPE_TEST_DATA 6

//WIFI数据发送触发模式
//主函数中根据状态信息按节拍触发发送
#define WIFI_SEND_TRIGGER_MAIN 1
//中断函数中直接触发发送
#define WIFI_SEND_TRIGGER_INTERRUPT 2
//主函数中按节拍发送同时中断函数中也可直接触发发送
#define WIFI_SEND_TRIGGER_BOTH 3

//各部分数据长度
//车的connect_id
#define DATA_PART_1_LENGTH 8
//底盘电机12的电流、速度、功率信息
#define DATA_PART_2_LENGTH 11
//底盘电机34的电流、速度、功率信息
#define DATA_PART_3_LENGTH 11
//云台yaw轴和pitch电机的电流、速度、位置信息
#define DATA_PART_4_LENGTH 11
//拨弹电机的电流、速度、位置信息+摩擦轮电机的电流、速度信息
#define DATA_PART_5_LENGTH 12
//part6:比赛进程信息数据
#define DATA_PART_6_LENGTH 42
//part7:基地伤害信息
#define DATA_PART_7_LENGTH 8
//part8:召唤工程车信息
#define DATA_PART_8_LENGTH 8
//part9:测试数据
#define DATA_PART_9_LENGTH 11
//part10:补给站状态信息数据
#define DATA_PART_10_LENGTH 9
//part11:射击信息数据=7帧头帧尾+16裁判系统数据+2小子弹数量+2大子弹数量
#define DATA_PART_11_LENGTH 27
//part12:战车离开补给站信息 
#define DATA_PART_12_LENGTH 8
//part13:摩擦轮是否开启信息
#define DATA_PART_13_LENGTH 9
//part14:整车自检信息
#define DATA_PART_14_LENGTH 9
//part15：前方敌人信息
#define DATA_PART_15_LENGTH 8

////////////////////////////////////////////////////////////////////////////////// 
//wifi模块初始化对外接口
void init_wifi(int port,char* server_ip_arg);
//wifi模块初始化硬件实现
u8 init_wifi_device(int port,char* server_ip_arg);
void check_init_wifi_flag(void);//检测初始化标识
void printNewLine(void);
//更新wifi发送标识，在中断中每1ms调用一次
void clock_wifi(void);

void channel_task_wifi(void);

//获取默认的电机状态信息结构体
Motor_Condition_TypeDef get_default_motor_conditon(void);
//获取默认的步兵车信息结构体
Pacer_Vehicle_Condition_TypeDef get_default_pacer_vehicle_conditon(void);


//发送整车所有电机状态信息至上位机
void send_motor_conditions(Motor_Condition_TypeDef* chassis_motor1_condition
	,Motor_Condition_TypeDef* chassis_motor2_condition
	,Motor_Condition_TypeDef* chassis_motor3_condition
	,Motor_Condition_TypeDef* chassis_motor4_condition
	,Motor_Condition_TypeDef* cradle_yaw_motor_condition
	,Motor_Condition_TypeDef* cradle_pitch_motor_condition
	,Motor_Condition_TypeDef* rammer_motor_condition
	,Motor_Condition_TypeDef* shooter1_motor_condition
	,Motor_Condition_TypeDef* shooter2_motor_condition
	);
//发送步兵车状态信息至上位机
void send_pacer_vihecle_condition(Pacer_Vehicle_Condition_TypeDef* pacer_vehicle_condition);

//发送身份确认信息
void send_connect_id(void);
//发送步兵车状态信息至上位机
void send_pacer_vihecle_condition(Pacer_Vehicle_Condition_TypeDef* pacer_vehicle_condition);
//发送整车自检信息
void send_self_check(Self_Check_TypeDef* self_check_data);
//更新wifi发送标志位
void update_wifi_send_flag(void);


//立即把数据发送出去，主要用于在中断函数中就立即发送的情况
//在main函数中根据发送状态发送数据，不阻塞中断
void send_in_main(void);
//处理收到的connect_id
void handle_connect_id(u8 id);

void handle_subsection_pid_values(PID_value *pid_value);

//中断任务，100ms调用一次
void interrupt_task(void);
//用户WIFI任务
void user_wifi_task(void);
//通过main循环发送的WIFI任务
void wifi_task_main(void);
//WIFI硬件发送
void send_to_wifi(void);

#endif
