#include "timer12.h"
/**
	* 占用引脚:	
				PH6     	------> TIM12_CH1
				PH9     	------> TIM12_CH2
				暂时没有硬件使用TIM12
						 
	*	函数:	void TIM12_Init(u16 arr,u16 psc);
				void TIM12_PWM_Init(u16 arr,u16 psc);
				void TIM_SetTIMH3Compare(u32 compare1,u32 compare);			
  ******************************************************************************
  */
	TIM_HandleTypeDef TIM12_Handler;            //定时器12PWM句柄 
	TIM_OC_InitTypeDef TIM12_CH1Handler;	    //定时器12通道1句柄
	TIM_OC_InitTypeDef TIM12_CH2Handler;	    //定时器12通道2句柄
	GPIO_InitTypeDef GPIOH_Initure;
	
//TIM12 PWM部分初始化 
//arr：自动重装值
//psc：时钟预分频数
void TIM12_PWM_Init(u16 arr,u16 psc)
{ 
	__HAL_RCC_TIM12_CLK_ENABLE();			//使能定时器12
	__HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟
	
	TIM12_Handler.Instance=TIM12;            //定时器12
    TIM12_Handler.Init.Prescaler=psc;       //定时器分频
    TIM12_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM12_Handler.Init.Period=arr;          //自动重装载值
    TIM12_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM12_Handler);       //初始化PWM
	
	GPIOH_Initure.Pin=GPIO_PIN_6|GPIO_PIN_9;    //PH6、9
    GPIOH_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIOH_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIOH_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIOH_Initure.Alternate= GPIO_AF9_TIM12;	//PH6/9复用为TIM12_CH1/2
    HAL_GPIO_Init(GPIOH,&GPIOH_Initure);
	
    TIM12_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM12_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM12_CH1Handler.OCPolarity=TIM_OCNPOLARITY_LOW; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM12_Handler,&TIM12_CH1Handler,TIM_CHANNEL_1);//配置TIM12通道1
    HAL_TIM_PWM_Start(&TIM12_Handler,TIM_CHANNEL_1);//开启PWM通道1
	
	TIM12_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM12_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM12_CH2Handler.OCPolarity=TIM_OCNPOLARITY_LOW; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM12_Handler,&TIM12_CH2Handler,TIM_CHANNEL_2);//配置TIM12通道2
    HAL_TIM_PWM_Start(&TIM12_Handler,TIM_CHANNEL_2);//开启PWM通道2
}

//设置TIM12占空比
void TIM_SetTIM12Compare(u32 compare1,u32 compare2)
{
	TIM12->CCR2=compare2; 	
	TIM12->CCR1=compare1;
}

