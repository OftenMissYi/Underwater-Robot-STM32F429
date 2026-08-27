#include "timer9.h"
/**
  ******************************************************************************
  * @file    			timer9.c
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2022-07-04
  * @brief   		  云台舵机初始化
  * @ownership		重庆大学水下机器人团队
  * @attention      
  ******************************************************************************
	* 占用引脚:	
			
			云台舵机：
				PE5     	------> TIM9_CH1
				PE6     	------> TIM9_CH2
				相机云台的控制频率为：50Hz
				PWM范围为：1ms-2ms，当满占空比为20000时，对应PWM值为1000-2000
						 		
  ******************************************************************************
  */
	TIM_HandleTypeDef TIM9_Handler;         //定时器9PWM句柄 
    TIM_OC_InitTypeDef TIM9_CH1Handler;	    //定时器9通道1句柄
    TIM_OC_InitTypeDef TIM9_CH2Handler;	    //定时器8通道2句柄
	GPIO_InitTypeDef GPIOE_Initure;
	
//TIM9 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM9_PWM_Init(u16 arr,u16 psc)
{ 
	__HAL_RCC_TIM9_CLK_ENABLE();			//使能定时器9
	__HAL_RCC_GPIOE_CLK_ENABLE();			//开启GPIOH时钟
	
	TIM9_Handler.Instance=TIM9;            //定时器9
    TIM9_Handler.Init.Prescaler=psc;       //定时器分频
    TIM9_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM9_Handler.Init.Period=arr;          //自动重装载值
    TIM9_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM9_Handler);       //初始化PWM
	
	GPIOE_Initure.Pin=GPIO_PIN_5|GPIO_PIN_6;    //PE5、6
    GPIOE_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIOE_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIOE_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIOE_Initure.Alternate= GPIO_AF3_TIM9;	//PE5/6复用为TIM9_CH1/2
    HAL_GPIO_Init(GPIOE,&GPIOE_Initure);
	
    TIM9_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM9_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM9_CH1Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM9_Handler,&TIM9_CH1Handler,TIM_CHANNEL_1);//配置TIM9通道1
    HAL_TIM_PWM_Start(&TIM9_Handler,TIM_CHANNEL_1);//开启PWM通道1
	
	TIM9_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM9_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM9_CH2Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM9_Handler,&TIM9_CH2Handler,TIM_CHANNEL_2);//配置TIM9通道2
    HAL_TIM_PWM_Start(&TIM9_Handler,TIM_CHANNEL_2);//开启PWM通道2
}

//设置TIM9云台舵机占空比
void TIM_SetTIM9Compare(u32 compare1,u32 compare2)
{
	TIM9->CCR2=compare2; 	
	TIM9->CCR1=compare1;
}

