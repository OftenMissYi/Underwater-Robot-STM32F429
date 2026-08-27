#ifndef __DHT22_H
#define __DHT22_H
#include "sys.h"


//IO方向设置
#define DHT22_IO_IN()  {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=0<<(2*2);}	//PD2输入模式
#define DHT22_IO_OUT() {GPIOD->MODER&=~(3<<(2*2));GPIOD->MODER|=1<<(2*2);} 	//PD2输出模式
 
////IO操作函数											   
#define	DHT22_DQ_OUT    PDout(2)//数据端口	PD2
#define	DHT22_DQ_IN     PDin(2) //数据端口	PD2

 	
u8 DHT22_Init(void);//初始化DHT22
u8 DHT22_Read_Data(float *temp,float *humi);//读取温湿度
u8 DHT22_Read_Byte(void);//读出一个字节
u8 DHT22_Read_Bit(void);//读出一个位
u8 DHT22_Check(void);//检测是否存在DHT22
void DHT22_Rst(void);//复位DHT22  


#endif
