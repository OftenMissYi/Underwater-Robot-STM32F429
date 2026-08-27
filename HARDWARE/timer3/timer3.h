#ifndef __TIMER3_H
#define __TIMER3_H
	
#include "sys.h"
	
extern TIM_HandleTypeDef TIM3_Handler;      //定时器3PWM句柄 
extern TIM_OC_InitTypeDef TIM3_CH1Handler;  //定时器3通道1句柄
extern TIM_OC_InitTypeDef TIM3_CH2Handler;  //定时器3通道2句柄


void TIM3_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr1,u16 psc1);
void TIM_SetTIM3Compare(u32 compare1,u32 compare2);


#endif
