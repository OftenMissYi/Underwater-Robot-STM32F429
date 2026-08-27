#ifndef __PID_FLASH_INIT_H__
#define __PID_FLASH_INIT_H__

#include "sys.h"
#include "string.h"
#include "attitude_pid.h"

/*
调用pid_flash_init，flash，先在 HALLIB 文件夹中加入 stm32f4xx_hal_flash，stm32f4xx_hal_flash_ex，stm32f4xx_hal_flash_ramfunc
上位机由 RS232 发送数据到机器人，以此来实时调节水下机器人的姿态 pid 参数，其中 Rs232_Receive_PID 在 rs232.c 中的接收中断中调用，
！！！注意 RECEIVE_SIZE 接收缓冲最大字节数需要修改， sizeof(datatemp[19]) 为 76！！！
*/
#define	head	(-0.0)		/*帧头*/
#define TEXT_LENTH sizeof(pid_rs232_para)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

extern float pid_rs232_para[30];
extern float datatemp[30];

void Rs232_Receive_PID(uint8_t* receive_byte);
void PID_Write_Flash(void);
void Flash_Read_PID(void);

#endif

