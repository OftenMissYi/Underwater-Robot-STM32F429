#ifndef __TIMER12_H
#define __TIMER12_H
#include "sys.h"

/**
  ******************************************************************************		 
	*	函数:	    void TIM12_Init(u16 arr,u16 psc);
					void TIM12_PWM_Init(u16 arr,u16 psc);
					void TIM_SetTIMH3Compare(u32 compare1,u32 compare
  ******************************************************************************
  */
extern TIM_HandleTypeDef  TIM12_Handler;      //定时器12PWM句柄 
extern TIM_OC_InitTypeDef TIM12_CH1Handler;  //定时器12通道1句柄
extern TIM_OC_InitTypeDef TIM12_CH2Handler;  //定时器12通道2句柄

void TIM12_Init(u16 arr,u16 psc);
void TIM12_PWM_Init(u16 arr,u16 psc);
void TIM_SetTIM12Compare(u32 compare1,u32 compare2);

#endif

