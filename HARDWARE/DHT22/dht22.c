#include "dht22.h"
#include "delay.h"
#include "sensors.h"

/**************************************************************************************/
//湿温度传感器型号：DHT22
//通信端口：单总线—>PD2，具体时序见开发资料
//采样周期不少于2s，一次通信时间5ms左右
//数据格式：40bit数据=16bit湿度数据+16bit温度数据+8bit校验和，高位先出
//湿度高8位+湿度低8位+温度高8位+温度低8位=的末8位=校验和
//电压：3.3-5.5V
/***************************************************************************************/

//初始化DHT22的IO口 DQ 同时检测DHT22的存在
//返回1:不存在
//返回0:存在     	 
u8 DHT22_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOD_CLK_ENABLE();			//开启GPIOD时钟
	
    GPIO_Initure.Pin=GPIO_PIN_2;           //PD2
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;     //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //初始化
	  delay_xms(2000);           //传感器上电后要等待2s，越过不稳定期
  	DHT22_Rst();             //主机发送一次开始信号，DHT22从低功耗模式转化为高速模式
	  return DHT22_Check();
}

//复位DHT22（主机产生开始信号）
void DHT22_Rst(void)	   
{   
    /*拉低数据线600us*/	
	DHT22_IO_OUT(); 	//设置为输出
	DHT22_DQ_OUT=0; 	//主机拉低DQ
	delay_xms(1);    	//拉低至少800us
	
	 /*拉高数据线20-40us*/	
	DHT22_DQ_OUT=1; 	//DQ=1 
	delay_us(30);     	//主机拉高20~40us
}

//等待DHT22的回应
//返回1:DHT22响应超时（包括DHT22不存在）
//返回0:存在
u8 DHT22_Check(void) 	   
{   
	u8 retry=0;
	DHT22_IO_IN();      //设置为输入	 
	while (DHT22_DQ_IN && retry < 100)//DHT22会拉低至少80us
	{
		retry++;
		delay_us(1);
	} 
	if(retry>=100)
	{
		return 1;        //响应超时
	}
	retry=0;
	while (!DHT22_DQ_IN && retry < 100)//DHT22拉低后会再次拉高80us
	{
		retry++;
		delay_us(1);
	}
	if(retry>=100)
	{
		return 1;	      //响应超时    
	}  
	return 0;
}

//从DHT22读取一个位
//返回值：1/0
u8 DHT22_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(DHT22_DQ_IN&&retry<100)//等待变为低电平
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!DHT22_DQ_IN&&retry<200)//等待变高电平
	{
		retry++;
		delay_us(1);
	}	
	delay_us(40);//等待40us，通过40us高电平来判断是信号0还是1，信号0只有22-28us高电平，而信号1有70us高电平
	if(DHT22_DQ_IN)  
		return 1;
	else 
		return 0;		   
}

//从DHT22读取一个字节
//返回值：读到的数据
u8 DHT22_Read_Byte(void)    
{        
	u8 i,dat;
	dat=0;
	for (i=0;i<8;i++) 
	{
		dat=dat<<1; 
		dat|=DHT22_Read_Bit();
	}		
    return dat;
}

//从DHT22读取一次数据
//temp:温度值(范围:-40~80°)
//humi:湿度值(范围:0%~100%)
//返回值：0,正常;1,读取失败
u8 DHT22_Read_Data(float *temp,float *humi)    
{        
 	u8 buf[5],check_sum;
	DHT22_Rst();              //主机发送开始信号
	if(DHT22_Check()==0)      //从机正确响应
	{
		buf[0]=DHT22_Read_Byte();       //读湿度高8位
		buf[1]=DHT22_Read_Byte();       //读湿度高8位
		buf[2]=DHT22_Read_Byte();       //读温度高8位
		buf[3]=DHT22_Read_Byte();       //读温度低8位
		buf[4]=DHT22_Read_Byte();       //读校验位
		
	    //释放总线
		DHT22_IO_OUT();
		DHT22_DQ_OUT=1;
		
		check_sum=buf[0]+buf[1]+buf[2]+buf[3];
		if(check_sum==buf[4])
		{
			*temp=(float)((buf[2]<<8)+buf[3])/10;
			*humi=(float)((buf[0]<<8)+buf[1])/10;
			
			rawData.cabin_humi=0; //电子舱温度拷贝上传
			rawData.cabin_temp=0; //电子舱湿度拷贝上传
			rawData.cabin_humi=(u16)((buf[0]<<8)+buf[1]); //电子舱温度拷贝上传
			rawData.cabin_temp=(u16)((buf[2]<<8)+buf[3]); //电子舱湿度拷贝上传
			
			//输出测试(湿度、温度)	
			printf("*humi= %f\r\n",*humi);
			printf("*temp= %f\r\n",*temp);
			printf("\r\n");
		}
		return 0;
	}
	else 
		return 1;
}



