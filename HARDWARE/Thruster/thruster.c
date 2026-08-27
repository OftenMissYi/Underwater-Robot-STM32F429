#include "thruster.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
/**
  ******************************************************************************
  * @file    			thruster.c
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2021-07-12
  * @brief   			推进器控制函数
	* @ownership		重庆大学水下机器人团队
  ******************************************************************************
  * @attention
  *
  * 平台: STM32F429IGT6 
  *4个水平推进器
	* 占用引脚:	PD12     	------> TIM4_CH1
				PD13    	------> TIM4_CH2
				PD14    	------> TIM4_CH3
				PD15    	------> TIM4_CH4
 
 *4个垂直推进器
	* 占用引脚:	PH10     	------> TIM5_CH1
				PH11    	------> TIM5_CH2
				PH12    	------> TIM5_CH3
				PI0      	------> TIM5_CH4
						 
	*	函数:	void Thurster_Init(u16 arr,u16 psc);
				void HAL_TIM4_PWM_MspInit(TIM_HandleTypeDef *htim);
				void HAL_TIM5_PWM_MspInit(TIM_HandleTypeDef *htim);
				void TIM_SetTIM4Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4);
				void TIM_SetTIM4Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4);
				void thruster_reset(void);
  ******************************************************************************
  */


TIM_HandleTypeDef TIM4_Handler;         //定时器4PWM句柄 
TIM_OC_InitTypeDef TIM4_CH1Handler;	    //定时器4通道1句柄
TIM_OC_InitTypeDef TIM4_CH2Handler;	    //定时器4通道2句柄
TIM_OC_InitTypeDef TIM4_CH3Handler;	    //定时器4通道3句柄
TIM_OC_InitTypeDef TIM4_CH4Handler;	    //定时器4通道4句柄

TIM_HandleTypeDef TIM5_Handler;         //定时器5PWM句柄 
TIM_OC_InitTypeDef TIM5_CH1Handler;	    //定时器5通道1句柄
TIM_OC_InitTypeDef TIM5_CH2Handler;	    //定时器5通道2句柄
TIM_OC_InitTypeDef TIM5_CH3Handler;	    //定时器5通道3句柄
TIM_OC_InitTypeDef TIM5_CH4Handler;	    //定时器5通道4句柄

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void HThurster_Init(u16 arr,u16 psc)
{
	// 初始化定时器4
	TIM4_Handler.Instance=TIM4;            //定时器4
	HAL_TIM4_PWM_MspInit(&TIM4_Handler);
	
    TIM4_Handler.Init.Prescaler=psc;       //定时器分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM4_Handler.Init.Period=arr;          //自动重装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM4_Handler);       //初始化PWM
	
    
    TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH1Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);//配置TIM4通道1
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);//开启PWM通道1

	TIM4_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH2Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH2Handler,TIM_CHANNEL_2);//配置TIM4通道2
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_2);//开启PWM通道2
	
	TIM4_CH3Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH3Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH3Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH3Handler,TIM_CHANNEL_3);//配置TIM4通道3
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_3);//开启PWM通道3
	
	TIM4_CH4Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM4_CH4Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH4Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH4Handler,TIM_CHANNEL_4);//配置TIM4通道4
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_4);//开启PWM通道4
}

void VThurster_Init(u16 arr,u16 psc)
{
	//初始化定时器5
	TIM5_Handler.Instance=TIM5;            //定时器5
	HAL_TIM5_PWM_MspInit(&TIM5_Handler);
	
    TIM5_Handler.Init.Prescaler=psc;       //定时器分频
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
    TIM5_Handler.Init.Period=arr;          //自动重装载值
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&TIM5_Handler);       //初始化PWM
    
    TIM5_CH1Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM5_CH1Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM5_CH1Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高 
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler,&TIM5_CH1Handler,TIM_CHANNEL_1);//配置TIM5通道1
    HAL_TIM_PWM_Start(&TIM5_Handler,TIM_CHANNEL_1);//开启PWM通道1

	TIM5_CH2Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM5_CH2Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM5_CH2Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler,&TIM5_CH2Handler,TIM_CHANNEL_2);//配置TIM5通道2
    HAL_TIM_PWM_Start(&TIM5_Handler,TIM_CHANNEL_2);//开启PWM通道2
	
	TIM5_CH3Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM5_CH3Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM5_CH3Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler,&TIM5_CH3Handler,TIM_CHANNEL_3);//配置TIM5通道3
    HAL_TIM_PWM_Start(&TIM5_Handler,TIM_CHANNEL_3);//开启PWM通道2
	
	TIM5_CH4Handler.OCMode=TIM_OCMODE_PWM1; //模式选择PWM1
    TIM5_CH4Handler.Pulse=arr/2;            //设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM5_CH4Handler.OCPolarity=TIM_OCNPOLARITY_HIGH; //输出比较极性为高
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler,&TIM5_CH4Handler,TIM_CHANNEL_4);//配置TIM5通道4
    HAL_TIM_PWM_Start(&TIM5_Handler,TIM_CHANNEL_4);//开启PWM通道2
	
}


//定时器底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_PWM_Init()调用
//htim:定时器句柄
void HAL_TIM4_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();			//使能定时器4
    __HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOD时钟
	
    GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;           	//PD12、13、14、15
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF2_TIM4;	//PD12、13、14、15复用为TIM4_CH1、TIM4_CH2、TIM4_CH3、TIM4_CH4
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
}

void HAL_TIM5_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_TIM5_CLK_ENABLE();			//使能定时器5
    __HAL_RCC_GPIOH_CLK_ENABLE();			//开启GPIOH时钟
	__HAL_RCC_GPIOI_CLK_ENABLE();			//开启GPIOI时钟
	
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;           	//PH10\11\12
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF2_TIM5;	//PH10\11\12复用为TIM5_CH1\TIM5_CH2\TIM5_CH3
    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_0;           	//PI0
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
	GPIO_Initure.Alternate= GPIO_AF2_TIM5;	//PI0复用为TIM5_CH4
    HAL_GPIO_Init(GPIOI,&GPIO_Initure);
}



//设置TIM4通道1234的占空比(水平推进器)
//compare:比较值
void TIM_SetTIM4Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4)
{
	TIM4->CCR1=compare1;
	TIM4->CCR2=compare2; 
	TIM4->CCR3=compare3;
	TIM4->CCR4=compare4; 
}



//设置TIM5通道1234的占空比（垂直推进器）
//compare:比较值
void TIM_SetTIM5Compare(u32 compare1,u32 compare2,u32 compare3,u32 compare4)
{
	TIM5->CCR1=compare1;
	TIM5->CCR2=compare2; 
	TIM5->CCR3=compare3;
	TIM5->CCR4=compare4; 
}

 
//推进器复位函数
void thruster_reset(void)
{
	TIM_SetTIM4Compare(1500,1500,1500,1500);
	TIM_SetTIM5Compare(1500,1500,1500,1500);
	delay_us(100);
}
