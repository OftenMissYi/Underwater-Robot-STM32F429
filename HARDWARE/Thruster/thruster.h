#ifndef __THRUSTER_H
#define __THRUSTER_H
	
#include "sys.h"

/**
  ******************************************************************************
  * @file    			thruster.h
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2021-07-12
  * @brief   			照明灯控制函数
	* @ownership		重庆大学水下机器人团队
  ******************************************************************************
  ******************************************************************************
  */

extern TIM_HandleTypeDef TIM4_Handler;         //定时器4PWM句柄 
extern TIM_OC_InitTypeDef TIM4_CH1Handler;	    //定时器4通道1句柄
extern TIM_OC_InitTypeDef TIM4_CH2Handler;	    //定时器4通道2句柄
extern TIM_OC_InitTypeDef TIM4_CH3Handler;	    //定时器4通道3句柄
extern TIM_OC_InitTypeDef TIM4_CH4Handler;	    //定时器4通道4句柄

extern TIM_HandleTypeDef TIM5_Handler;         //定时器5PWM句柄 
extern TIM_OC_InitTypeDef TIM5_CH1Handler;	    //定时器5通道1句柄
extern TIM_OC_InitTypeDef TIM5_CH2Handler;	    //定时器5通道2句柄
extern TIM_OC_InitTypeDef TIM5_CH3Handler;	    //定时器5通道3句柄
extern TIM_OC_InitTypeDef TIM5_CH4Handler;	    //定时器5通道4句柄


void HThurster_Init(u16 arr,u16 psc);
void VThurster_Init(u16 arr,u16 psc);
void TIM_SetTIM4Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4);
void TIM_SetTIM5Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4);
void HAL_TIM4_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM5_PWM_MspInit(TIM_HandleTypeDef *htim);
void thruster_reset(void);

#endif
