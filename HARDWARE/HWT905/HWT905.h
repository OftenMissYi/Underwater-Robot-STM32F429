#ifndef __HWT905_H
#define __HWT905_H
#include "sensors.h"


//启用对匿名结构和联合的支持
#if defined ( __CC_ARM)
#pragma anon_unions
#endif

#include "JY901.h"
#include "usart6.h"
#include "string.h"
#include "usart.h"

//零偏结构体
typedef struct
{
	float accx_bias;
	float accy_bias;
	float accz_bias;
	float anglex_bias;
	float angley_bias;
	float angle_ratex_bias;
	float angle_ratey_bias;
	float angle_ratez_bias;
}DIAS_t;

//TEST_MODE为 1 模块到串口助手，为 0 到上位机
#define TEST_MODE 1 

//uint8_t ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
//uint8_t GYRO[5]= {0XFF,0XAA,0x63,0X01,0X00};//选择陀螺仪自动校准,0X01。去掉陀螺仪自动校准,0X00。
//uint8_t DIRECTION[5]= {0XFF,0XAA,0x23,0X01,0X00};//设置安装方向,设置为垂直安装,0X01。设置为水平安装,0X00。
//uint8_t ALG[5]= {0XFF,0XAA,0x24,0X00,0X00};//设置成 9 轴算法
//uint8_t RATE[5]= {0XFF,0XAA,0x03,0x06,0X00};//设置回传速率,10Hz（默认）,设置完成以后需要点保存配置按钮， 再给模块重新上电后生效
//uint8_t BAUD_[5]= {0XFF,0XAA,0x04,0x02,0X00};//设置串口波特率,9600（默认）
//uint8_t SAVACALSW[5]= {0XFF,0XAA,0x06,0X00,0X00};//保存当前配置


void HWT_Init(void);
void sendcmd(char cmd[]);
void Cal_HWT_Data(sensorData_t* sensorData);
void CopeSerial2Data(unsigned char ucData);
void HWT_CALIBRATION(sensorData_t* sensorData,DIAS_t* zero_bias);

#endif
