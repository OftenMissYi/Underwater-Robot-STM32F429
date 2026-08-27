#ifndef __LIGHT_H
#define __LIGHT_H
	
#include "sys.h"

/**
  ******************************************************************************
  * @file    			timer2.h
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2021-07-12
  * @brief   			照明灯控制函数
	* @ownership		重庆大学水下机器人团队
  ******************************************************************************
  * @attention
  *
  * 平台: STM32F429IGT6 
	* 占用引脚:	PA15     	------> TIM2_CH1
							PA1     	------> TIM2_CH2
						 
	*	函数:	void TIM2_Init(u16 arr,u16 psc);
					void TIM2_PWM_Init(u16 arr,u16 psc);
					void TIM_SetTIM2Compare(u32 compare1,u32 compare
  ******************************************************************************
  */
	
	
	
//Light端口定义
#define Light1 PAout(1)
#define Light2 PAout(15)

extern TIM_HandleTypeDef TIM2_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM2_CH1Handler;  //定时器3通道1句柄
extern TIM_OC_InitTypeDef TIM2_CH2Handler;  //定时器3通道2句柄

void Light_Init(void);
void TIM2_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM_SetTIM2Compare(u32 compare1,u32 compare2);
void HAL_TIM2_PWM_MspInit(TIM_HandleTypeDef *htim);
void light_task(void *pvParameters);

#endif
