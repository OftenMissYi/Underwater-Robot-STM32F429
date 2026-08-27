#include "usart6.h"
#include "usart.h"
#include "delay.h"
#include "string.h"
#include "HWT905.h"
#include "led.h"



//USART6->TX---------PC6
//USART6->RX---------PC7

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//os 使用	  
#endif

////////////////////////////////////////////////////////////////////////////////// 	  

#if EN_USART6_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF6[USART_REC_LEN6];     //接收缓冲,最大USART_REC_LEN6个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA6=0;       //接收状态标记	

u8 aRxBuffer6[RXBUFFERSIZE6];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART6_Handler; //UART句柄

//初始化IO 串口1 
//bound:波特率
void uart6_init(u32 bound)
{	
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();			//使能GPIOC时钟
	__HAL_RCC_USART6_CLK_ENABLE();			//使能USART6时钟
	
	GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7; //PC6,7
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;		//高速
	GPIO_Initure.Alternate=GPIO_AF8_USART6;	//复用为USART6
	HAL_GPIO_Init(GPIOC,&GPIO_Initure);	   	//初始化PC6,7
	
	//UART 初始化设置
	UART6_Handler.Instance=USART6;					    //USART6
	UART6_Handler.Init.BaudRate=bound;				    //波特率
	UART6_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART6_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART6_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART6_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART6_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART6_Handler);					    //HAL_UART_Init()会使能UART6
	
//	HAL_UART_Receive_IT(&UART6_Handler, (u8 *)aRxBuffer6, RXBUFFERSIZE6);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
 #if EN_USART6_RX
		HAL_NVIC_EnableIRQ(USART6_IRQn);				//使能USART6中断通道
		HAL_NVIC_SetPriority(USART6_IRQn,1,0);			//抢占优先级1，子优先级0
#endif
}


/*下面代码我们直接把中断控制逻辑写在中断服务函数内部.
 说明：采用HAL库处理逻辑，效率不高。*/
void USART6_IRQHandler(void)                	
{ 
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART6_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		HAL_UART_Receive(&UART6_Handler,&Res,1,1000);
		USART_RX_BUF6[0] = Res;
		
#if !TEST_MODE			
		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF6,1,1);	//发送接收到的数据
#endif	
		
#if TEST_MODE
		CopeSerial2Data((unsigned char)Res);
#endif
		
		USART_RX_STA6=0;		
	}
	HAL_UART_IRQHandler(&UART6_Handler);	
} 
/*
//串口6中断服务程序
void USART6_IRQHandler(void)                	
{ 
	u8 Res;
	if((__HAL_UART_GET_FLAG(&UART6_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		HAL_UART_Receive(&UART6_Handler,&Res,1,1000);
		USART_RX_BUF6[0] = Res;
		HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF6,1,1);	//发送接收到的数据
//		CopeSerial2Data((unsigned char)Res);
		USART_RX_STA6=0;		
	}
	HAL_UART_IRQHandler(&UART6_Handler);	
} 
*/
#endif	
/*
void USART6_IRQHandler(void)                	
{ 
	u8 Res;
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	if((__HAL_UART_GET_FLAG(&UART6_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
        HAL_UART_Receive(&UART6_Handler,&Res,1,1000); 
		if((USART_RX_STA6&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA6&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA6=0;//接收错误,重新开始
				else 
				{
					USART_RX_STA6|=0x8000;	//接收完成了
					printf("串口6接收完成了\r\n");  //操作函数
					printf("您发送的消息为:\r\n");
					HAL_UART_Transmit(&UART1_Handler,(uint8_t*)USART_RX_BUF6,USART_RX_STA6&0x6fff,1000);	//发送接收到的数据
					printf("\r\n");
					USART_RX_STA6=0;
				}
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA6|=0x4000;
				else
				{
					USART_RX_BUF6[USART_RX_STA6&0X3FFF]=Res ;
					USART_RX_STA6++;
					if(USART_RX_STA6>(USART_REC_LEN6-1))USART_RX_STA6=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&UART6_Handler);	
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
} 
#endif
*/
 




