#include "timer.h"

/**
	* 占用引脚:	机械臂舵机D30：
				PB4     	------> TIM3_CH1
				PB5     	------> TIM3_CH2（未使用）
				
			***************************************************************	
						 
	*	函数:	void TIM3_Init(u16 arr,u16 psc);
				void TIM3_PWM_Init(u16 arr,u16 psc);
				void TIM_SetTIMB3Compare(u32 compare1,u32 compare);
				void TIM_SetTIMC3Compare(u32 compare1,u32 compare2);
  ******************************************************************************
  */
	
TIM_HandleTypeDef TIM3_Handler;         //定时器3PWM句柄 
TIM_OC_InitTypeDef TIM3_CH1Handler;	    //定时器3通道1句柄
TIM_OC_InitTypeDef TIM3_CH2Handler;	    //定时器3通道2句柄


//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr1,u16 psc1) //同时初始化定时器3和9
{ 
	__HAL_RCC_TIM3_CLK_ENABLE();			//使能定时器3
	__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
	
    TIM3_Handler.Instance=TIM3;            //定时器3
    TIM3_Handler.Init.Prescaler=psc1;       //定时器分频
    TIM3_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM3_Handler.Init.Period=arr1;          //自动重装载值
    TIM3_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	
    HAL_TIM_PWM_Init(&TIM3_Handler);       //初始化PWM
    
    TIM3_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM3_CH1Handler.Pulse=arr1/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM3_CH1Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH1Handler,TIM_CHANNEL_1);//配置TIM3通道1
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_1);//开启PWM通道1

	TIM3_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM3_CH2Handler.Pulse=arr1/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM3_CH2Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler,&TIM3_CH2Handler,TIM_CHANNEL_2);//配置TIM3通道2
    HAL_TIM_PWM_Start(&TIM3_Handler,TIM_CHANNEL_2);//开启PWM通道2
}


//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIOB_Initure;
	GPIO_InitTypeDef GPIOE_Initure;
	
    GPIOB_Initure.Pin=GPIO_PIN_4|GPIO_PIN_5;   //PB4、5
    GPIOB_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIOB_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIOB_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIOB_Initure.Alternate= GPIO_AF2_TIM3;	//PB4/5复用为TIM3_CH1\2
    HAL_GPIO_Init(GPIOB,&GPIOB_Initure);	
}
 

//设置TIM3机械臂舵机占空比
//compare:比较值
void TIM_SetTIM3Compare(u32 compare1,u32 compare2)
{
	TIM3->CCR1=compare1;
	TIM3->CCR2=compare2; 
}

