#ifndef __MS5837_H_
#define __MS5837_H_

#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "depth_iic.h"


/* 函数申明 -----------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////		 
// 线序             定义 
//   PH3 ----------- SDA(白)
//	 PH2 ----------- SCL(绿)
//	 Vcc  ----------- 5V(红)
//	 Gnd  ----------- 地（黑）
/////////////////////////////////////////////////////////////////////////////////////////	




#define MS5837_30BA_WriteCommand     0xEC
#define MS5837_30BA_ReadCommand      0xED

#define MS5837_30BA_ResetCommand     0x1E                //复位
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM读取,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC读取              

#define MS5837_30BA_OSR256					 0x40
#define MS5837_30BA_OSR512					 0x42
#define MS5837_30BA_OSR1024					 0x44
#define MS5837_30BA_OSR2048					 0x46
#define MS5837_30BA_OSR4096					 0x48
#define	MS5837_30BA_D2_OSR_8192   	 0x58                 //16.44ms转换时间
#define	MS5837_30BA_D1_OSR_8192   	 0x48                 //16.44ms转换时间

u8   MS5837_30BA_init(void);
void MS5837_30BA_GetData(void);
void MS5837_30BA_ReSet(void);
unsigned char MS5837_30BA_Crc4(void);
unsigned long MS5837_30BA_GetConversion(u8 command);
extern double Temperature;
extern int32_t Pressure;
extern uint32_t Cal_C[7];


#endif


