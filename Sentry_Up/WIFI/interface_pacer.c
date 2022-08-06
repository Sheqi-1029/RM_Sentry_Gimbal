#include "interface_pacer.h"
#include "sys.h"
#include "data_channel_wifi.h"
#include "data_channel_pc.h"



//接口6：调试功率环
extern Pacer_Vehicle_Condition_TypeDef pacer_vehicle_condition;
void set_pacer_vihecle_condition(Pacer_Vehicle_Condition_TypeDef* pacer_vehicle_condition_args){
	pacer_vehicle_condition.chassis_motor1_condition=pacer_vehicle_condition_args->chassis_motor1_condition;
	pacer_vehicle_condition.chassis_motor2_condition=pacer_vehicle_condition_args->chassis_motor2_condition;
	pacer_vehicle_condition.chassis_motor3_condition=pacer_vehicle_condition_args->chassis_motor3_condition;
	pacer_vehicle_condition.chassis_motor4_condition=pacer_vehicle_condition_args->chassis_motor4_condition;
	pacer_vehicle_condition.cradle_yaw_motor_condition=pacer_vehicle_condition_args->cradle_yaw_motor_condition;
	pacer_vehicle_condition.cradle_pitch_motor_condition=pacer_vehicle_condition_args->cradle_pitch_motor_condition;
	pacer_vehicle_condition.rammer_motor_condition=pacer_vehicle_condition_args->cradle_pitch_motor_condition;
	pacer_vehicle_condition.shooter1_motor_condition=pacer_vehicle_condition_args->shooter1_motor_condition;
	pacer_vehicle_condition.shooter2_motor_condition=pacer_vehicle_condition_args->shooter2_motor_condition;
}

//接口7：调试车数据
extern Debug_Vehicle_Data debug_vehicle_data;
void set_debug_vehicle_data(Debug_Vehicle_Data* debug_vehicle_data_args){
	//填充自己的数据////////////////////////
		debug_vehicle_data.power=debug_vehicle_data_args->power;
		debug_vehicle_data.gyroscope_yaw=debug_vehicle_data_args->gyroscope_yaw;
		debug_vehicle_data.gyroscope_pitch=debug_vehicle_data_args->gyroscope_pitch;
		debug_vehicle_data.lost_counter_controller=debug_vehicle_data_args->lost_counter_controller;
		debug_vehicle_data.lost_counter_chassis_1=debug_vehicle_data_args->lost_counter_chassis_1;
		debug_vehicle_data.lost_counter_chassis_2=debug_vehicle_data_args->lost_counter_chassis_2;
		debug_vehicle_data.lost_counter_chassis_3=debug_vehicle_data_args->lost_counter_chassis_3;
		debug_vehicle_data.lost_counter_chassis_4=debug_vehicle_data_args->lost_counter_chassis_4;
		debug_vehicle_data.lost_counter_cloud_deck_yaw=debug_vehicle_data_args->lost_counter_cloud_deck_yaw;
		debug_vehicle_data.lost_counter_cloud_deck_pitch=debug_vehicle_data_args->lost_counter_cloud_deck_pitch;
		debug_vehicle_data.lost_counter_gyroscope=debug_vehicle_data_args->lost_counter_gyroscope;
		debug_vehicle_data.encoding_disk_yaw=debug_vehicle_data_args->encoding_disk_yaw;
		debug_vehicle_data.encoding_disk_pitch=debug_vehicle_data_args->encoding_disk_pitch;
}

//处理收到的分段pid参数
float current_kp[10];
void handle_subsection_pid_values(PID_value *pid_value){
		current_kp[0]=pid_value->Ki[0];
		current_kp[1]=pid_value->Ki[1];
		current_kp[2]=pid_value->Ki[2];
		current_kp[3]=pid_value->Ki[3];
		current_kp[4]=pid_value->Ki[4];
		current_kp[5]=pid_value->Ki[5];
		current_kp[6]=pid_value->Ki[6];
		current_kp[7]=pid_value->Ki[7];
		current_kp[8]=pid_value->Ki[8];

}
