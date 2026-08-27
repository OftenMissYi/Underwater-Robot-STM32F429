#ifndef _USART6_H
#define _USART6_H
#include "sys.h"
#include "stdio.h"	

#define USART_REC_LEN6  		128  	//定义最大接收字节数 64
#define EN_USART6_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF6[USART_REC_LEN6]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA6;         		//接收状态标记	
extern UART_HandleTypeDef UART6_Handler; //UART句柄

#define RXBUFFERSIZE6   1 //缓存大小
extern u8 aRxBuffer6[RXBUFFERSIZE6];//HAL库USART接收Buffer

//如果想串口中断接收，请不要注释以下宏定义
void uart6_init(u32 bound);


#endif


