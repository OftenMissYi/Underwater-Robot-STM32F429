#include "link232.h"
#include "rs232.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "delay.h"
#include "pcf8574.h"
#include "led.h"
#include "sensors.h"
#include "ibus.h"
#include "pid_flash_init.h"
#include "semphr.h"

//点灯LED1：通信正常，则LED1状态反转

extern u8 RS232_flag;
extern u8 remoteliink_flag;                      // 遥控器连接标志
extern u8 PID_flag;                            // PID参数接收标志
extern QueueHandle_t link485DataDelivery; 
extern UART_HandleTypeDef USART2_RS485Handler;  //USART2句柄(用于RS485)
extern TaskHandle_t lightTask_Handler; 
extern QueueHandle_t Message_Queue;	//信息队列句柄

static u16 time_count;

//定义状态标志位
u8 LIGHT_flag;                               //照明灯标志
u8 ARMSERVO_flag;  
   
//定义数据储存数
int16_t rx232buf[12];
u8 tx232buf_data[64];
u8 len;
u16 command[11];

extern SemaphoreHandle_t BinarySemaphore;	//二值信号量句柄
extern rawData_t rawData;

extern double Temperature;
extern sensorData_t sensorData;
u8 data_count=0;     //数据个数统计变量

/*******************************************************************************
* 函 数 名         : link232Rx_task
* 函数功能		   :232数据接收后的处理函数
* 输    入         :232数据接收函数的命令存储数组
* 输    出         : 无
通讯结构: 上位机发送16位命令
第 0位：下行帧头“D”——0x44
第 1位：功能字位，决定执行什么功能：照明灯、舵机、推进器
第 2位:具体执行命令位
第 3位：帧尾“E” ——0x45

*******************************************************************************/ 

//232接收任务函数
void link232Rx_task(void *pvParameters)
{
	u8 i;
	BaseType_t err=pdFALSE;
	while(1)
		{
			if(BinarySemaphore!=NULL)
		{
			err=xSemaphoreTake(BinarySemaphore,portMAX_DELAY);	//获取信号量
			if(err==pdTRUE)	//获取信号量成功
     { 				
		//接收遥控器命令
		   if(RS232_RXBuffer[0]==0x20 && RS232_RXBuffer[1]==0x40)      //校验数据接收正确
		   {
			ch[1] = (int16_t)(((u16)(RS232_RXBuffer[3]&0xFF)<<8) | (u16)RS232_RXBuffer[2]);               //通道1
			ch[2] = (int16_t)(((u16)(RS232_RXBuffer[5]&0xFF)<<8) | (u16)RS232_RXBuffer[4]);               //通道2
			ch[3] = (int16_t)(((u16)(RS232_RXBuffer[7]&0xFF)<<8) | (u16)RS232_RXBuffer[6]);               //通道3
			ch[4] = (int16_t)(((u16)(RS232_RXBuffer[9]&0xFF)<<8) | (u16)RS232_RXBuffer[8]);               //通道4
			ch[5] = (int16_t)(((u16)(RS232_RXBuffer[11]&0xFF)<<8) | (u16)RS232_RXBuffer[10]);             //通道5
			ch[6] = (int16_t)(((u16)(RS232_RXBuffer[13]&0xFF)<<8) | (u16)RS232_RXBuffer[12]);             //通道6
			ch[7] = (int16_t)(((u16)(RS232_RXBuffer[15]&0xFF)<<8) | (u16)RS232_RXBuffer[14]);             //通道7
			ch[8] = (int16_t)(((u16)(RS232_RXBuffer[17]&0xFF)<<8) | (u16)RS232_RXBuffer[16]);             //通道8
			ch[9] = (int16_t)(((u16)(RS232_RXBuffer[19]&0xFF)<<8) | (u16)RS232_RXBuffer[18]);             //通道9
			ch[10] = (int16_t)(((u16)(RS232_RXBuffer[21]&0xFF)<<8) | (u16)RS232_RXBuffer[20]);            //通道10
			LED0=!LED0;
			RS232_flag=1;            //数据正常接收，标志位为1
			remoteliink_flag=1;
			for(i=1;i<=10;i++)
			{  
				rx232buf[i]=ch[i];	
			}	          
			delay_xms(5);
		  }                              //接收PID调试参数         
		else if(RS232_RXBuffer[0]==0x01 && RS232_RXBuffer[1]==0x02)
		{
			PID_flag=1;
			Rs232_Receive_PID((uint8_t*)RS232_RXBuffer);//接收发来的pid参数并写入flash
			LED0=!LED0;           //两灯快速闪烁，表示在PID参数调整阶段
			LED1=!LED1;
		}
   }
  }
		
	    //LED1=!LED1;    /*提示程序正在运行*/
			if(RS232_flag==1)	
			{   
				//先判断通道7，是否切换485总线控制模式
				if(rx232buf[7]<1540)
				{
					command[EXCHANGE_MODE]=TH_MODE ;  //推进器模式
					//左手油门,右手油门
					command[SURGE]=ibus_to_T200pwm(rx232buf[2]);
					command[SWAY]=ibus_to_T200pwm(rx232buf[1]);
					command[HEAVE]=ibus_to_T80pwm(rx232buf[3]);
					command[YAW]=ibus_to_T200pwm(rx232buf[4]);
				}
				else if(rx232buf[7]>=1540)        //总线舵机模式
				{
					command[EXCHANGE_MODE]=RS485_MODE ;  //总线舵机模式
					//左手油门,右手角度
					command[SURGE]=ibus_to_DS300(rx232buf[2]);
					command[SWAY]=ibus_to_DS300(rx232buf[1]);
					command[HEAVE]=ibus_to_T80pwm(rx232buf[3]);
					command[YAW]=ibus_to_T200pwm(rx232buf[4]);
				}
				if(rx232buf[5]<1500)              //通道5，选择手动模式和定深度模式
					command[MODE]=HAND_MODE;
				else
					command[MODE]=DEPTH_MODE; 
				    
				if(rx232buf[6]>1500)               //通道6，照明灯打开和关闭
				{
					LIGHT_flag=1;
				}
				else
					LIGHT_flag=0;
					
				if(rx232buf[8]<1500)         //通道8，手爪开和关闭
					command[GRAB]=CLOSE;
				else if(rx232buf[8]>1500)
					command[GRAB]=OPEN;
				
				//通道9和10，云台舵机指令
				command[CAM_PAN1]=ibus_to_CAMpwm(rx232buf[9]);
				command[CAM_PAN2]=ibus_to_CAMpwm(rx232buf[10]);
				RS232_flag=0;  
		}	
			delay_ms(20); 
	}
}	
//*******************************************************************************
//* 函 数 名         : link232Tx_task
//* 函数功能		   :232发送单片机上采集到的数据,30ms发送一次
//           16位的数据需要拆分为8bit上传
//* 输    入         :无
//* 输    出         : 无
//*******************************************************************************
void link232Tx_task(void *pvParameters)
{
	time_count=0;
	while(1)
	{
		u8 check_sum,i;
		//tx232buf_data[0]=0x44;     //帧头 
		
		//测试
//		rawData.depth=-512;
//		rawData.water_temp=-203;
        //第1-6位为加速度计数据
//		tx232buf_data[1]=rawData.acc.x & 0xff;        //低位先出
//		tx232buf_data[2]=(rawData.acc.x>>8) & 0xff;   //高位后出
//		tx232buf_data[3]=rawData.acc.y & 0xff;
//		tx232buf_data[4]=(rawData.acc.y>>8) & 0xff; 
//		tx232buf_data[5]=rawData.acc.z & 0xff;
//		tx232buf_data[6]=(rawData.acc.z>>8) & 0xff; 
//		
//		//第7-12位为角度数据
//		tx232buf_data[7]=rawData.angle.roll & 0xff;
//		tx232buf_data[8]=(rawData.angle.roll>>8) & 0xff;
//		tx232buf_data[9]=rawData.angle.pitch & 0xff;
//		tx232buf_data[10]=(rawData.angle.pitch>>8) & 0xff;
//		tx232buf_data[11]=rawData.angle.yaw & 0xff;
//		tx232buf_data[12]=(rawData.angle.yaw>>8) & 0xff;
//		
//		//第13-18位为角速度数据
//		tx232buf_data[13]=rawData.gyro.roll & 0xff;
//		tx232buf_data[14]=(rawData.gyro.roll>>8) & 0xff;
//		tx232buf_data[15]=rawData.gyro.pitch & 0xff;
//		tx232buf_data[16]=(rawData.gyro.pitch>>8) & 0xff;
//		tx232buf_data[17]=rawData.gyro.yaw & 0xff;
//		tx232buf_data[18]=(rawData.gyro.yaw>>8) & 0xff;

//		//第19-20位为深度数据，第21-22位为水温数据
//		tx232buf_data[19]=rawData.depth & 0xff;
//		tx232buf_data[20]=(rawData.depth>>8) & 0xff;
//		tx232buf_data[21]=rawData.water_temp & 0xff;
//		tx232buf_data[22]=(rawData.water_temp>>8) & 0xff;
//		
//		//第23-24位为电子舱温度，第25-26位为湿度，第27-30位为压力数据
//		tx232buf_data[23]=rawData.cabin_temp & 0xff;
//		tx232buf_data[24]=(rawData.cabin_temp>>8) & 0xff;
//		tx232buf_data[25]=rawData.cabin_humi & 0xff;
//		tx232buf_data[26]=(rawData.cabin_humi>>8) & 0xff;
//		tx232buf_data[27]=rawData.cabin_press & 0xff;
//		tx232buf_data[28]=(rawData.cabin_press >>8) & 0xff;
//		tx232buf_data[29]=(rawData.cabin_press >>16) & 0xff;
//		tx232buf_data[30]=(rawData.cabin_press >>24) & 0xff;
//		
//		//第31-46位为陀螺仪零偏
//		tx232buf_data[31]=rawData.HWT905_bais.acc_x & 0xff;
//		tx232buf_data[32]=(rawData.HWT905_bais.acc_x>>8) & 0xff;
//		tx232buf_data[33]=rawData.HWT905_bais.acc_y & 0xff;
//		tx232buf_data[34]=(rawData.HWT905_bais.acc_y>>8) & 0xff;
//		tx232buf_data[35]=rawData.HWT905_bais.acc_z & 0xff;
//		tx232buf_data[36]=(rawData.HWT905_bais.acc_z>>8) & 0xff;
//		tx232buf_data[37]=rawData.HWT905_bais.ro11 & 0xff;
//		tx232buf_data[38]=(rawData.HWT905_bais.ro11>>8) & 0xff;
//		tx232buf_data[39]=rawData.HWT905_bais.pitch & 0xff;
//		tx232buf_data[40]=(rawData.HWT905_bais.pitch>>8) & 0xff;
//		tx232buf_data[41]=rawData.HWT905_bais.ro11_w & 0xff;
//		tx232buf_data[42]=(rawData.HWT905_bais.ro11_w>>8) & 0xff;
//		tx232buf_data[43]=rawData.HWT905_bais.pitch_w & 0xff;
//		tx232buf_data[44]=(rawData.HWT905_bais.pitch_w>>8) & 0xff;
//		tx232buf_data[45]=rawData.HWT905_bais.yaw_w & 0xff;
//		tx232buf_data[46]=(rawData.HWT905_bais.yaw_w>>8) & 0xff;
   if(Message_Queue!=NULL)
      {
        if(xQueueReceive(Message_Queue,tx232buf_data,portMAX_DELAY)==pdPASS)//请求消息Message_Queue)
		  {
		   check_sum=0;                  //校验位先清零
		   for(i=1;i<47;i++)
		  {
		   	check_sum+=tx232buf_data[i];    //计算校验位
		  }
		   tx232buf_data[47]=check_sum;
		   tx232buf_data[48]=0x45;     //帧尾
		   data_count=49;
		
		//每隔2s上传一次数据（湿温度模块的采样时间需要2s）
		    delayms();               //调用延时1ms函数
	  	if(time_count>500)        //调用500次延时函数，每次延时1ms,剩余时间由其他地方的延时提供
	  	{
			  while(__HAL_UART_GET_FLAG(&UART3_Handler, UART_FLAG_TXE) == RESET)
				;	        //判断接收完成或者其他，保证没有处于接收过程
			  RS232_Send_Data(tx232buf_data,data_count);
			  while(__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_TC)!=SET)    //等待发送完成
			  time_count=0;
		  	for(i=0;i<64;i++)
			 {
			  	tx232buf_data[i]=0;
			 }
	  	}
	  	else time_count++;
			
		delay_ms(10);
//		taskYIELD(); 	
	}
  }
  }
	}
void delayms(void)
{
	delay_xms(1);
}
//将16位数据拆分为2个8位，16位数据位有符号数据类型（short），范围：-32767—+32768，反码表示,拆分为两个8位。
//组合时相反，先将高位扩充为short类型，在左移8为后与低字节相或,最后组合得到的数据类型强制转化为short，才能表示出正负数。
//short a = 0xffff;
//unsigned char b = (a & 0xFF00)>>8;
//unsigned char c = a & 0x00FF;
//short d = ((short)b<<8) | c;

