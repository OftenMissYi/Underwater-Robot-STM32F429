#include "iic1.h"
#include "sys.h"

/********************************
* 引脚：
	* PB14----------SCL
	* PB15----------SDA
	* 5V----------VCC
	* GND----------GND
***************************************/
//IIC初始化
void IIC_BMP_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOB时钟
    
    //PB14,15初始化设置
    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    IIC_BMP_SDA=1;
    IIC_BMP_SCL=1;  
}
//IIC延时函数
static void IIC_Delay_us(u16 us)
{    
   u16 i=0;  
   while(us--)
   {
      i=5;  
      while(i--) ;    
   }
}
//产生IIC起始信号
void IIC_BMP_Start(void)
{
	BMP_SDA_OUT();     //sda线输出
	IIC_BMP_SDA=1;	  	  
	IIC_BMP_SCL=1;
	IIC_Delay_us(4);
 	IIC_BMP_SDA=0;//START:when CLK is high,DATA change form high to low 
	IIC_Delay_us(4);
	IIC_BMP_SCL=0;//钳住I2C总线，准备发送或接收数据 
}
//产生IIC停止信号
void IIC_BMP_Stop(void)
{
	BMP_SDA_OUT();//sda线输出
	IIC_BMP_SCL=0;
	IIC_BMP_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_Delay_us(4);
	IIC_BMP_SCL=1;  
	IIC_BMP_SDA=1;//发送I2C总线结束信号
	IIC_Delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_BMP_Wait_Ack(void)
{
	u8 ucErrTime=0;
	BMP_SDA_OUT();      //SDA设置为输入  
	IIC_BMP_SDA=1;IIC_Delay_us(1);   
	IIC_BMP_SCL=1;IIC_Delay_us(1);	 
	while(BMP_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_BMP_Stop();
			return 1;
		}
	}
	IIC_BMP_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_BMP_Ack(void)
{
	IIC_BMP_SCL=0;
	BMP_SDA_OUT();
	IIC_BMP_SDA=0;
	IIC_Delay_us(2);
	IIC_BMP_SCL=1;
	IIC_Delay_us(2);
	IIC_BMP_SCL=0;
}
//不产生ACK应答		    
void IIC_BMP_NAck(void)
{
	IIC_BMP_SCL=0;
	BMP_SDA_OUT();
	IIC_BMP_SDA=1;
	IIC_Delay_us(2);
	IIC_BMP_SCL=1;
	IIC_Delay_us(2);
	IIC_BMP_SCL=0;
}	
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_BMP_Send_Byte(u8 txd)
{                        
    u8 t;   
	BMP_SDA_OUT(); 	    
    IIC_BMP_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		IIC_BMP_SDA=(txd&0x80)>>7;
		txd<<=1; 	  
		IIC_Delay_us(2);
		IIC_BMP_SCL=1;
		IIC_Delay_us(2); 
		IIC_BMP_SCL=0;	
		IIC_Delay_us(2);
    }	 
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_BMP_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	BMP_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_BMP_SCL=0; 
        IIC_Delay_us(2); 
		IIC_BMP_SCL=1;
        receive<<=1;
        if(BMP_READ_SDA) receive++;   
		IIC_Delay_us(1); 
    }					 
    if (!ack)
        IIC_BMP_NAck();//发送nACK
    else
        IIC_BMP_Ack(); //发送ACK   
    return receive;
}
//列出IIC总线上所有从机地址
void IIC_Slave_List(void)
{
	u8 i=0,res = 0;
	for(i=0;i<255;i++)
	{
		IIC_BMP_Start();
		IIC_BMP_Send_Byte((i<<1)|0);
		res = IIC_BMP_Wait_Ack();	//等待应答
		if(res == 0)
			printf("IIC_ADDR = %#x\r\n",i);
		IIC_BMP_Stop();
	}
	printf("\r\n");
}
//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
#define BMP280_SlaveAddr 0x76    //BMP280的器件地址
u8 BMP280_Write_Byte(u8 bmp_reg,u8 bmp_data)
{
	IIC_BMP_Start();
    IIC_BMP_Send_Byte((BMP280_SlaveAddr<<1)|0); //发送器件地址+写命令
    if(IIC_BMP_Wait_Ack())          //等待应答
    {
        IIC_BMP_Stop();
        return 1;
    }
    IIC_BMP_Send_Byte(bmp_reg);         //写寄存器地址
    IIC_BMP_Wait_Ack();             //等待应答
    IIC_BMP_Send_Byte(bmp_data);        //发送数据
    if(IIC_BMP_Wait_Ack())          //等待ACK
    {
        IIC_BMP_Stop();
        return 1;
    }
    IIC_BMP_Stop();
    return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 BMP280_Read_Byte(u8 bmp_reg)
{
	 u8 res;
    IIC_BMP_Start();
    IIC_BMP_Send_Byte((BMP280_SlaveAddr<<1)|0); //发送器件地址+写命令
    IIC_BMP_Wait_Ack();             //等待应答
    IIC_BMP_Send_Byte(bmp_reg);         //写寄存器地址
    IIC_BMP_Wait_Ack();             //等待应答
	IIC_BMP_Start();                
    IIC_BMP_Send_Byte((BMP280_SlaveAddr<<1)|1); //发送器件地址+读命令
    IIC_BMP_Wait_Ack();             //等待应答
    res=IIC_BMP_Read_Byte(0);		//读数据,发送nACK  
    IIC_BMP_Stop();                 //产生一个停止条件
	return res; 
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 BMP280_Read_Len(u8 bmp_reg,u8 len,u8 *buf)
{
	IIC_BMP_Start();
    IIC_BMP_Send_Byte((BMP280_SlaveAddr<<1)|0); //发送器件地址+写命令
    if(IIC_BMP_Wait_Ack())          //等待应答
    {
        IIC_BMP_Stop();
        return 1;
    }
    IIC_BMP_Send_Byte(bmp_reg);         //写寄存器地址
    IIC_BMP_Wait_Ack();             //等待应答
	IIC_BMP_Start();                
    IIC_BMP_Send_Byte((BMP280_SlaveAddr<<1)|1); //发送器件地址+读命令
    IIC_BMP_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_BMP_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_BMP_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_BMP_Stop();                 //产生一个停止条件
    return 0;  
}



