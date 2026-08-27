/**
  ******************************************************************************
  * @file    			main.c
  * @author  			马祖兴
  * @version 			V1.0
  * @date    			2022-07-04
  * @brief   		  初始化、创建任务函数
  * @ownership		重庆大学水下机器人团队
  * @attention      
  ******************************************************************************
 */
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "depth_iic.h"
#include "rs485.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "light.h"
#include "link232.h"
#include "rs232.h"
#include "usart6.h"
#include "thruster.h"
#include "ds300.h"
#include "semphr.h"


//开始任务1
#define START_TASK_PRIO		2              //任务优先级
#define START_STK_SIZE 		300            //任务堆栈大小
TaskHandle_t StartTask_Handler;            //任务句柄
void start_task(void *pvParameters);       //任务函数

//RS232接收任务2
#define link232Rx_TASK_PRIO		3          //任务优先级
#define link232Rx_STK_SIZE 		400        //任务堆栈大小
TaskHandle_t link232RxTask_Handler;        //任务句柄
void link232Rx_task(void *pvParameters);   //任务函数

//RS232发送任务3
#define link232Tx_TASK_PRIO		4          //任务优先级
#define link232Tx_STK_SIZE 		300        //任务堆栈大小
TaskHandle_t link232TxTask_Handler;        //任务句柄
void link232Tx_task(void *pvParameters);   //任务函数

//传感器处理任务4
#define sensors_TASK_PRIO		4          //任务优先级
#define sensors_STK_SIZE 		600        //任务堆栈大小
TaskHandle_t sensorsTask_Handler;          //任务句柄
void sensors_task(void *pvParameters);     //任务函数	

////姿态控制任务5
//#define  stabilizer_TASK_PRIO		4       //任务优先级
//#define stabilizer_STK_SIZE 		700     //任务堆栈大小
//TaskHandle_t stabilizerTask_Handler;      //任务句柄
//void stabilizer_task(void *pvParameters); //任务函数	

//姿态控制任务5
#define  autocontrol_TASK_PRIO		3       //任务优先级
#define autocontrol_STK_SIZE 		700     //任务堆栈大小
TaskHandle_t autocontrolTask_Handler;       //任务句柄
void auto_control_task(void *pvParameters); //任务函数	

//机械臂控制任务6
#define  arm_TASK_PRIO		4               //任务优先级
#define arm_STK_SIZE 		300             //任务堆栈大小
TaskHandle_t armTask_Handler;               //任务句柄
void arm_task(void *pvParameters);          //任务函数

//照明灯控制任务7
#define light_TASK_PRIO	     5              //任务优先级
#define light_STK_SIZE 		300             //任务堆栈大小
TaskHandle_t lightTask_Handler;             //任务句柄
void light_task(void *pvParameters);        //任务函数

//修改期望值任务8
#define autoremote_TASK_PRIO	     4      //任务优先级
#define autoremote_STK_SIZE 		300     //任务堆栈大小
TaskHandle_t autoremoteTask_Handler;        //任务句柄
void auto_remote_task(void *pvParameters);  //任务函数

//二值信号量句柄
SemaphoreHandle_t BinarySemaphore;	//二值信号量句柄

//定义队列存传感器数据
#define MESSAGE_Q_NUM   1   	//发送数据的消息队列的数量 
QueueHandle_t Message_Queue;	//信息队列句柄

extern void RS232_Init(u32 bound);
						 
int main(void)
{
	u8 len=0;
  HAL_Init();                     //初始化HAL库   
  Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
	delay_init(180);                //初始化延时函数
	uart_init(115200);              //初始化串口1（串口助手）
  LED_Init();                     //初始化LED 
	RS485_Init(115200);             //RS485初始化（DS300）
	len = DS300_Write_Control_Instruction(0x01,1494, 0x0000, 0xff);    //关节舵机复位，运动速度：255（0xff）*0.087=22.185（度/秒）
	RS485_Send_Data(DS300_Instruction_Data, len);
	delay_xms(500);//延时以供关节复位时间
	TIM3_PWM_Init(20000-1,90-1);    //初始化定时器3（D30）
	TIM_SetTIM3Compare(1150,1150);//手爪舵机复位
	uart3_Init(9600);	            //初始化串口3(上位机下位机通讯串口232)
	IIC_Init();//深度传感器 	
	uart6_init(9600);        //初始化串口6，接收陀螺仪数据
	printf("初始化完成！\r\n");
    //创建开始任务
	xTaskCreate((TaskFunction_t )start_task,            //任务函数
								(const char*    )"start_task",          //任务名称
								(uint16_t       )START_STK_SIZE,        //任务堆栈大小
								(void*          )NULL,                  //传递给任务函数的参数
								(UBaseType_t    )START_TASK_PRIO,       //任务优先级
								(TaskHandle_t*  )&StartTask_Handler);   //任务句柄                
	vTaskStartScheduler();          //开启任务调度
}

//开始任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	  //创建二值信号量
	  BinarySemaphore=xSemaphoreCreateBinary();
	
	  //创建消息队列
	  Message_Queue=xQueueCreate(MESSAGE_Q_NUM,64); //创建消息Message_Queue
	
    //创建232接收任务2
    xTaskCreate((TaskFunction_t )link232Rx_task,             
                (const char*    )"link232Rx_task",           
                (uint16_t       )link232Rx_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )link232Rx_TASK_PRIO,        
                (TaskHandle_t*  )&link232RxTask_Handler);   
    //创建232发送任务3
    xTaskCreate((TaskFunction_t )link232Tx_task,     
                (const char*    )"link232Tx_task",   
                (uint16_t       )link232Tx_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )link232Tx_TASK_PRIO,
                (TaskHandle_t*  )&link232TxTask_Handler); 
	//创建传感器处理任务4
    xTaskCreate((TaskFunction_t )sensors_task,             
                (const char*    )"sensors_task",           
                (uint16_t       )sensors_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )sensors_TASK_PRIO,        
                (TaskHandle_t*  )&sensorsTask_Handler);
//	//创建姿态和运动控制任务5
//  xTaskCreate((TaskFunction_t )stabilizer_task,             
//                (const char*    )"stabilizer_task",           
//                (uint16_t       )stabilizer_STK_SIZE,        
//                (void*          )NULL,                  
//                (UBaseType_t    )stabilizer_TASK_PRIO,        
//                (TaskHandle_t*  )&stabilizerTask_Handler);
	//创建舵机控制任务6
    xTaskCreate((TaskFunction_t ) arm_task,             
                (const char*    )"arm_task",           
                (uint16_t       )arm_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )arm_TASK_PRIO,        
                (TaskHandle_t*  )&armTask_Handler);
	//创建照明灯控制任务7
    xTaskCreate((TaskFunction_t ) light_task,             
                (const char*    )"light_task",           
                (uint16_t       )light_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )light_TASK_PRIO,        
                (TaskHandle_t*  )&lightTask_Handler);	
	//创建姿态和运动控制任务5
    xTaskCreate((TaskFunction_t )auto_control_task,             
                (const char*    )"auto_control_task",           
                (uint16_t       )autocontrol_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )autocontrol_TASK_PRIO,        
                (TaskHandle_t*  )&autocontrolTask_Handler);		
	//创建修改期望值任务8
    xTaskCreate((TaskFunction_t ) auto_remote_task,             
                (const char*    )"auto_remote_task",           
                (uint16_t       )autoremote_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )autoremote_TASK_PRIO,        
                (TaskHandle_t*  )&autoremoteTask_Handler);
								
	printf("Free heap: %d bytes\n", xPortGetFreeHeapSize());			/*打印剩余堆栈大小*/
	printf("\r\n\r\n");
   
	vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}

