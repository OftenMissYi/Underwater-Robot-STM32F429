#include "light.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/**
  ******************************************************************************
  * @file    			timer2.c
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2021-07-12
  * @brief   			照明灯控制函数
	* @ownership		重庆大学水下机器人团队
  ******************************************************************************
  * @attention
  *
  * 平台: STM32F429IGT6 
	* 占用引脚:	    PA15   ------> TIM2_CH1
					PB3    	------> TIM2_CH2
						 
	*	函数:	    void TIM2_Init(u16 arr,u16 psc);
					void TIM2_PWM_Init(u16 arr,u16 psc);
					void TIM_SetTIM2Compare(u32 compare1,u32 compare
  ******************************************************************************
  */

extern u8 LIGHT_flag;                               //照明灯打开标志（置1打开，置0关闭）

TIM_HandleTypeDef  TIM2_Handler;         //定时器2PWM句柄 
TIM_OC_InitTypeDef TIM2_CH1Handler;	    //定时器2通道1句柄
TIM_OC_InitTypeDef TIM2_CH2Handler;	    //定时器2通道1句柄

void Light_Init(void)
{
	TIM2_PWM_Init(20000-1,90-1);    //频率为50Hz
	TIM_SetTIM2Compare(1100,1100);   //初始状态关闭照明灯 
	delay_us(10);
}

//TIM2 PWM部分初始化 
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u16 arr,u16 psc)
{ 
    TIM2_Handler.Instance=TIM2;            //定时器2
	__HAL_RCC_TIM2_CLK_ENABLE();			//使能定时器2
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
	 __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOB时钟
	
    TIM2_Handler.Init.Prescaler=psc;       //定时器分频
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM2_Handler.Init.Period=arr;          //自动重装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM2_Handler);       //初始化PWM
    
    TIM2_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH1Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH1Handler,TIM_CHANNEL_1);//配置TIM2通道1
	
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_1);//开启PWM通道1

	TIM2_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM2_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM2_CH2Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM2_Handler,&TIM2_CH2Handler,TIM_CHANNEL_2);//配置TIM2通道2
	
    HAL_TIM_PWM_Start(&TIM2_Handler,TIM_CHANNEL_2);//开启PWM通道2
	HAL_TIM2_PWM_MspInit(&TIM2_Handler);
}
   

//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM2_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	
    GPIO_Initure.Pin=GPIO_PIN_15;           	//PA15
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF1_TIM2;	//PA15复用为TIM2_CH1
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_3;           	//PB3
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF1_TIM2;	//PB3复用为TIM2_CH2
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

//设置TIM通道4的占空比
//compare:比较值
void TIM_SetTIM2Compare(u32 compare1,u32 compare2)
{
	TIM2->CCR1=compare1;
	TIM2->CCR2=compare2; 	
}

//照明灯控制任务函数—调光模式
/*信号范围：1100us-1900us*/
void light_task(void *pvParameters)
{
	Light_Init();
	while(1)
	{
		if(LIGHT_flag==1)              //打开照明灯
		{
			TIM_SetTIM2Compare(1900,1900);
			delay_us(1);
		}
		else               //照明灯关闭
		{
			TIM_SetTIM2Compare(1100,1100);
			delay_us(1);
		}
	 delay_ms(30);
	}
}
