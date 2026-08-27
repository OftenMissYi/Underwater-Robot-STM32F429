#include "auto_need_state.h"
#include "FreeRTOS.h"
#include "task.h"
#include "auto_control.h"
#include "attitude_pid.h"
#include "delay.h"
#include "math.h"

extern state_t need_state; //期望姿态
extern state_t state; //当前姿态
state_t nstate; //期望目的地
int if_busy[3] = {0,0,0};

/*
if_busy[0] 控制循环
if_busy[1] 0表示在循环中，1表示循环完，等待下一个目标
if_busy[2] 1再循环中收到下一个目标，用于跳出循环
*/

void auto_receive_state_task(void)//nstate赋值，还需要一个接收视觉数据的函数
{
	while(1)
	{
		if(1 /*&& 是目前的目标*/) //与前一个发送的目标是同一个目标，无论if_busy[1]是什么都可以赋值
		{
			/*这里写接收视觉的目标物位置到nstate*/
			if_busy[0] = 0; 
		}
		if(if_busy[1] == 1 /*&& 是下一个目标*/) //可以发下一个目标（需要每个目标有一个标识，以确定是不同的目标）
		{
			/*这里写接收视觉的目标物位置到nstate*/
			if_busy[0] = 0; 
		}
		if(if_busy[1] == 0 /*&& 是下一个目标*/) //可以发下一个目标（需要每个目标有一个标识，以确定是不同的目标）
		{
			if_busy[2] = 1; 
		}
		delay_ms(100);
	}
}

void auto_need_state_task(void) //到达期望目的地
{
	while(1)
	{
		if(if_busy[0] == 0) //目的防止一直循环，每发一次目标最多只循环一次
		{
			if_busy[0] = 1; //控制循环
			if_busy[1] = 0; //正在循环，不能发下个目标
			
			need_state.realDepth = nstate.realDepth; //先定深
			while(!(-5 < need_state.realDepth - state.realDepth && need_state.realDepth - state.realDepth< 5))
			{
				if(if_busy[2] == 1)
					break;				
				need_state.realDepth = nstate.realDepth;
				delay_ms(10);
			}
			
			need_state.realAngle.yaw = nstate.realAngle.yaw; //再定角
			while(!(-1 < need_state.realAngle.yaw - state.realAngle.yaw < 1))
			{
				if(if_busy[2] == 1)
					break;				
				need_state.realDepth = nstate.realDepth;
				need_state.realAngle.yaw = nstate.realAngle.yaw;
				delay_ms(10);
			}
			
			need_state.position.x = nstate.position.x; //最后平移
			need_state.position.y = nstate.position.y;
			while(!(-5 < pow(need_state.position.x,2) + pow(need_state.position.y,2) && pow(need_state.position.x,2) + pow(need_state.position.y,2)< 5)) 
			{
				if(if_busy[2] == 1)
					break;				
				need_state.realDepth = nstate.realDepth;
				need_state.realAngle.yaw = nstate.realAngle.yaw;
				need_state.position.x = nstate.position.x;
				need_state.position.y = nstate.position.y;
				delay_ms(10);	
			}
			
			if(if_busy[2] == 1) //如果跳出循环则赋当前值停止
			{
				need_state.realDepth = state.realDepth;
				need_state.realAngle.yaw = state.realAngle.yaw;
				need_state.position.x = 0;
				need_state.position.y = 0;
			}
			
			if_busy[2] = 0; //已跳出循环
			if_busy[1] = 1; //已循环完，可以发下个目标
		}
		delay_ms(50);	
	}
}



