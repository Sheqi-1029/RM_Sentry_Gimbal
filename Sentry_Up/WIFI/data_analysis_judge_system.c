#include "data_analysis_judge_system.h"
#include "data_analysis_tool_crc.h"
#include "interface_base.h"
#include "data_channel_judge_system.h"
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
unsigned char judge_system_buffer[BUFFER_SIZE];


