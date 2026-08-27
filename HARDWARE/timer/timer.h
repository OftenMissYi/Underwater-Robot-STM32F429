#ifndef __TIMER_H
#define __TIMER_H
	
#include "sys.h"

/**
  ******************************************************************************
  * @file    			timer.h
  * @author  			魏灵洁
  * @version 			V1.0
  * @date    			2021-06-02
  * @brief   			双舵机控制函数
	* @ownership		重庆大学水下机器人团队
  ******************************************************************************
  * @attention			 
	*	函数:	    void TIM3_Init(u16 arr,u16 psc);
					void TIM3_PWM_Init(u16 arr,u16 psc);
					void TIM_SetTIM3Compare(u32 compare1,u32 compare
  ******************************************************************************
  */
	
extern TIM_HandleTypeDef TIM3_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM3_CH1Handler;  //定时器3通道1句柄
extern TIM_OC_InitTypeDef TIM3_CH2Handler;  //定时器3通道2句柄


void TIM3_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr1,u16 psc1);
void TIM_SetTIM3Compare(u32 compare1,u32 compare2);


#endif
