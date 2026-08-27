#ifndef __TIMER9_H
#define __TIMER9_H
#include "sys.h"


extern TIM_HandleTypeDef TIM9_Handler;      //定时器9PWM句柄 
extern TIM_OC_InitTypeDef TIM9_CH1Handler;  //定时器9通道1句柄
extern TIM_OC_InitTypeDef TIM9_CH2Handler;  //定时器9通道2句柄

void TIM9_Init(u16 arr,u16 psc);
void TIM9_PWM_Init(u16 arr,u16 psc);
void TIM_SetTIM9Compare(u32 compare1,u32 compare2);


#endif
