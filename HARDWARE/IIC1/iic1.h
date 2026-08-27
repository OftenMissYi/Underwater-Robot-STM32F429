#ifndef __IIC1_H
#define __IIC1_H
#include "sys.h"

//IO操作函数	 
#define IIC_BMP_SCL		PBout(14) //SCL
#define IIC_BMP_SDA     PBout(15) //SDA
#define BMP_READ_SDA	PBin(15)  //输入SDA 
//IO方向设置
#define BMP_SDA_IN()  {GPIOB->MODER&=~((u32)3<<(15*2));GPIOB->MODER|=(u32)0<<15*2;}	//PB15输入模式
#define BMP_SDA_OUT() {GPIOB->MODER&=~((u32)3<<(15*2));GPIOB->MODER|=(u32)1<<15*2;} //PB15输出模式
void IIC_BMP_Init(void);
void IIC_BMP_Start(void);
void IIC_BMP_Stop(void);
u8 IIC_BMP_Wait_Ack(void);
void IIC_BMP_Ack(void);
void IIC_BMP_NAck(void);
void IIC_BMP_Send_Byte(u8 txd);
u8 IIC_BMP_Read_Byte(unsigned char ack);
void IIC_Slave_List(void);
u8 BMP280_Write_Byte(u8 bmp_reg,u8 bmp_data);
u8 BMP280_Read_Byte(u8 bmp_reg);
u8 BMP280_Read_Len(u8 bmp_reg,u8 len,u8 *buf);


#endif
