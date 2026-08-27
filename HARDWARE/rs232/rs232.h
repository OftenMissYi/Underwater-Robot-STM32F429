#ifndef __RS232_H
#define __RS232_H
#include "sys.h"
#include "stdio.h"	

#define RECEIVE_SIZE   128
extern UART_HandleTypeDef UART3_Handler;     //USART3句柄(用于RS232)
extern uint8_t RS232_RXBuffer[RECEIVE_SIZE]; 		  //接收缓冲,最大64个字节
extern int16_t ch[12];  

void uart3_Init(u32 bound);
void RS232_Send_Data(u8 *buf,u8 len);

///*下行指令ID*/
//typedef enum 
//{
//	DOWN_LIGHT    	 =0x01,        //照明灯的打开和关闭
//	DOWM_ARMSERVO    =0x02,        //控制机械臂舵机
//	DOWM_CAMERASERVO =0x03,        //控制相机云台舵机 
//	DOWN_THRUSTER    =0x04,        //推进器复位
//	DOWN_RESET       =0x05,        //参数复位（舵机，照明灯和推进器）
//	DOWN_M	         =0x06,        //手动模式
//	DOWN_A	         =0x07,        //自动模式

//}downmsgID_e;
#endif

