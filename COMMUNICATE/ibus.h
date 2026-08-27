#ifndef __IBUS_H
#define __IBUS_H

#include "sys.h"

#define IBUS_RANGE_MIN 1000.0f            //油门最小值      
#define IBUS_RANGE_MAX 2000.0f            //油门最大值 

#define IBUS_T200C_MIN 1100.0f            //推进器T200C最小值 
#define IBUS_T200C_MAX 1900.0f            //推进器T200C最大值 

#define IBUS_T80_MIN 1000.0f            //推进器T80最小值 
#define IBUS_T80_MAX 2000.0f            //推进器T80最大值

#define IBUS_RS300_MIN 1040.0f            //舵机DS300最小值 
#define IBUS_RS300_MAX 1750.0f            //舵机DS300最大值
#define IBUS_CAM_MIN 1000.0f              //云台舵机最小值 
#define IBUS_CAM_MAX 2000.0f              //云台舵机最大值

#define DEAD_RANGE_MIN 1460               //油门死区
#define DEAD_RANGE_MAX 1540
#define IBUS_RANGE_MIDDLE 1500.0f         //油门中值
#define IBUS_CONNECT_FLAG 0x00



//控制命令，对应通道1-10
#define SURGE         0
#define SWAY          1
#define HEAVE         2
#define YAW           3
#define MODE          4
#define LIGHT         5
#define EXCHANGE_MODE 6
#define GRAB          7
#define CAM_PAN1      8
#define CAM_PAN2      9
#define SURGE_DS      0
#define SWAY_DS       1

 //定义手动模式和深度保持模式
#define HAND_MODE     0
#define DEPTH_MODE    1

 //定义总线舵机模式和推进器模式
#define RS485_MODE    1
#define TH_MODE       2
 
 //定义手爪模式
#define CLOSE         0
#define OPEN          1

#define IBUS_T200C_FACTOR ((IBUS_T200C_MAX - IBUS_T200C_MIN) / (IBUS_RANGE_MAX - IBUS_RANGE_MIN))
#define IBUS_T80_FACTOR ((IBUS_T80_MAX - IBUS_T80_MIN) / (IBUS_RANGE_MAX - IBUS_RANGE_MIN))
#define IBUS_DS300_FACTOR ((IBUS_RS300_MAX - IBUS_RS300_MIN) / (IBUS_RANGE_MAX - IBUS_RANGE_MIN))
#define IBUS_CAM_FACTOR ((IBUS_CAM_MAX - IBUS_CAM_MIN) / (IBUS_RANGE_MAX - IBUS_RANGE_MIN))

u16 ibus_to_T200pwm(u16 ibus_value);
u16 ibus_to_DS300(u16 ibus_value);
u16 ibus_to_CAMpwm(u16 ibus_value);
u16 ibus_to_T80pwm(u16 ibus_value);

#endif
