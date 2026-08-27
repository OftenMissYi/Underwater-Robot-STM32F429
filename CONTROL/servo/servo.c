#include "servo.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ibus.h"
#include "ds300.h"
#include "rs485.h"
#include "timer3.h"
#include "link232.h"
#include "timer9.h"
#include "ibus.h"
/*
  ******************************************************************************
  * @file    			Servo.c
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2022-07-04
  * @brief   			
  * @ownership		重庆大学水下机器人团队
  * @attention      PB4控制手爪舵机D30，RS485控制关节舵机DS300，其余未使用
                    RS485和D30的初始化在main中
  
	* 占用引脚:	机械臂舵机D30：
    *			PB4     	------> TIM3_CH1
				PB5     	------> TIM3_CH2（设置好的，未使用）
				频率：50Hz，信号：500-2500usec
				PWM = 2500 时 转角270°
				
				关节舵机DS300：
				角度范围：0-360°
				信号范围：0-4095，0-2047表示正方向，2048-4095表示负方向
  ******************************************************************************
 */

extern u16 command[11];
extern int16_t rx232buf[12];

pos_t pos;

void Servo_Init(void)
{
	//暂未使用云台舵机
//	TIM9_PWM_Init(20000-1,90-1);                           /*云台舵机初始化，频率为50Hz*/
//	TIM_SetTIM9Compare(Servo_Mid_PWM,Servo_Mid_PWM);     //云台舵机先给中值，手爪舵机启动准备
	
	//手爪舵机刚开始给1150，转到初始位置
	TIM_SetTIM3Compare(1150,1150);   //第二个比较值没用                
	delay_us(2000);
}

//舵机控制任务函数
void arm_task(void *pvParameters)
{
	u8 len=0;
	Servo_Init();
	while(1)
	{
		/*通道1和2控制总线舵机PAN1和PAN2*/
		if(command[EXCHANGE_MODE]==RS485_MODE)              /*RS485舵机控制模式*/ 
		{
			if((rx232buf[2]>1460)&&(rx232buf[2]<1540))      /*摇杆1处于中位死区，输出中值*/   
			{
				command[SURGE]=1395;                        /*该值应该为限制最大角度和最小角度的中间值*/
			}				  
			if((rx232buf[1]>1460)&&(rx232buf[1]<1540))	    /*摇杆2处于中位死区，输出中值*/ 
			{
				command[SWAY]=1395; 
			}		
			pos.position1=DS300_Limit(command[SURGE]);                   /*死区之外直接输出角度位置*/  
			pos.position2=DS300_Limit(command[SWAY]);
			
			//总线舵机01
			len = DS300_Write_Control_Instruction(0x01,pos.position1, 0x0000, 0xff);
			//len = DS300_Write_Control_Instruction(0x01,pos.position1, 0x0000, 0);       //最大速度运动
			RS485_Send_Data(DS300_Instruction_Data, len);
			//while(__HAL_UART_GET_FLAG(&USART2_RS485Handler,USART_FLAG_TC)==RESET); 	//等待发送结束	
			delay_us(10);
			
			//总线舵机02
			len = DS300_Write_Control_Instruction(0x02,pos.position2, 0x0000, 0x03e8);
			RS485_Send_Data(DS300_Instruction_Data, len);
			//while(__HAL_UART_GET_FLAG(&USART2_RS485Handler,USART_FLAG_TC)==RESET); 	//等待发送结束	
		}
		else       /*本体运动控制模式下，DS300舵机在指定位置1150*/ 
		{
			len = DS300_Write_Control_Instruction(0x01,1150, 0x0000, 0xff);
			RS485_Send_Data(DS300_Instruction_Data, len);
		}
		
		/*通道8控制手爪的张开与闭合，角度范围45°*/
		if(command[GRAB]==CLOSE)         //手爪闭合
		{
			Servo_control(1530,1530);
			delay_us(10);
		}
		else if(command[GRAB]==OPEN)                        //手爪打开
		{   
			Servo_control(1900,1900);
			delay_us(10);
		}	
		delay_ms(1); 
	}
}

 /* D30舵机安全限位函数*/ 
u16 Servo_Limit(u16 value)
{
	if(value > 2000)
		return value=2000;
	else if(value< 1000)
		return value=1000;
	return value;
}
 /* DS300舵机安全限位函数*/ 
u16 DS300_Limit(u16 value)    
{
	if(value > IBUS_RS300_MAX)
		return value=IBUS_RS300_MAX;
	else if(value< IBUS_RS300_MIN)
		return value=IBUS_RS300_MIN;
	return value;
}


 /*手爪舵机D30输出函数*/
void Servo_control(u32 PWM1,u32 PWM2)
{
	u16 pwm1,pwm2;
	pwm1=Servo_Limit(PWM1);
	pwm2=Servo_Limit(PWM2);
	TIM_SetTIM3Compare(pwm1,pwm2);
}
