#include "link232.h"
#include "rs485.h"
#include "pcf8574.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>
#include <delay.h>
#include <stdint.h>
#include <ds300.h>



UART_HandleTypeDef USART2_RS485Handler;  //USART2句柄(用于RS485)
DMA_HandleTypeDef DMA1_InitStructure;     //DMA句柄
uint8_t RS485_RXBuffer[RECEIVE485_SIZE];                       //接收缓冲,最大128个字节
u8 RS485_flag;        // RS485接收标志
  
////接收缓存区 	
//u8 RS485_RX_BUF[64];  	//接收缓冲,最大64个字节.
////接收到的数据长度
//u8 RS485_RX_CNT=0;  
//extern u8 rx485buf[64];


void USART2_IRQHandler(void)
{
	unsigned char num;
    u8 i;	  
	 if(__HAL_UART_GET_FLAG(&USART2_RS485Handler,UART_FLAG_IDLE)==SET)  //空闲中断
	 {
		HAL_UART_DMAStop(&USART2_RS485Handler);//关闭DMA接收
		num = USART2->SR;
		num = USART2->DR; 				//先读SR再读DR,清USART_IT_IDLE标志
		num = RECEIVE485_SIZE - (__HAL_DMA_GET_COUNTER(&DMA1_InitStructure));	//获得数据长度
		__HAL_UART_CLEAR_IDLEFLAG(&USART2_RS485Handler);//清除空闲中断接受标志位
	 
		//******处理函数开始******//
		for(i=0;i<num;i++)
		{
			printf("%x ", RS485_RXBuffer[i]);
		}
		printf("\r\n\r\n");
		
		if(num==21)	//舵机所有反馈参数的应答包总计21字节
		{
			DS300_Read_Feedback_Response(RS485_RXBuffer, &DS300_Chassis);
		}
		//******处理函数开始******//
		RS485_flag=1;
	}
	HAL_UART_Receive_DMA(&USART2_RS485Handler,(uint8_t *)RS485_RXBuffer,RECEIVE485_SIZE);//使能DMA接收		
}    




/*******************************************************************************
* 函 数 名         : RS485_Init
* 函数功能	  	   : 485初始化函数(urat2)
* 输    入         : bound:波特率
* 输    出         : 无
* DMA通道          : DMA1 4通道 数据流5（USART2_RX）
*******************************************************************************/  

void RS485_Init(u32 bound)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_LINKDMA(&USART2_RS485Handler,hdmarx,DMA1_InitStructure);//连接DMA1与串口2
	
	PCF8574_Init();                         //初始化PCF8574，用于控制RE脚
//	PCF8574_WriteBit(BEEP_IO,0);                             //关闭PCF8574的P0引脚，关闭蜂鸣器

	__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
	__HAL_RCC_USART2_CLK_ENABLE();			//使能USART2时钟
	__HAL_RCC_DMA1_CLK_ENABLE();		    //DMA1时钟使能
	 
	GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3; //PA2,3
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;		//高速
	GPIO_Initure.Alternate=GPIO_AF7_USART2;	//复用为USART2
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA2,3
    
    //USART 初始化设置
	USART2_RS485Handler.Instance=USART2;			        //USART2
	USART2_RS485Handler.Init.BaudRate=bound;		        //波特率
	USART2_RS485Handler.Init.WordLength=UART_WORDLENGTH_8B;	//字长为8位数据格式
	USART2_RS485Handler.Init.StopBits=UART_STOPBITS_1;		//一个停止位
	USART2_RS485Handler.Init.Parity=UART_PARITY_NONE;		//无奇偶校验位
	USART2_RS485Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;	//无硬件流控
	USART2_RS485Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&USART2_RS485Handler);			        //HAL_UART_Init()会使能USART2
	
	//DMA1通道4，数据流5
	DMA1_InitStructure.Instance=DMA1_Stream5;                                //数据流选择5
	DMA1_InitStructure.Init.Channel = DMA_CHANNEL_4;  						//通道选择4
	DMA1_InitStructure.Init.Direction =DMA_PERIPH_TO_MEMORY;					//存储器到外设模式
	DMA1_InitStructure.Init.PeriphInc = DMA_PINC_DISABLE;		            //外设非增量模式
	DMA1_InitStructure.Init.MemInc =DMA_MINC_ENABLE;		          			//存储器增量模式
	DMA1_InitStructure.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    	//外设数据长度:8位
	DMA1_InitStructure.Init.MemDataAlignment= DMA_MDATAALIGN_BYTE;		    //存储器数据长度:8位
	DMA1_InitStructure.Init.Mode = DMA_CIRCULAR ;							//使用循环模式 
	DMA1_InitStructure.Init.Priority = DMA_PRIORITY_HIGH;					//高优先级
	DMA1_InitStructure.Init.FIFOMode= DMA_FIFOMODE_DISABLE;         
	DMA1_InitStructure.Init.FIFOThreshold= DMA_FIFO_THRESHOLD_FULL;
	DMA1_InitStructure.Init.MemBurst= DMA_MBURST_SINGLE;				//存储器突发单次传输
	DMA1_InitStructure.Init.PeriphBurst = DMA_PBURST_SINGLE;		//外设突发单次传输					
	HAL_DMA_DeInit(&DMA1_InitStructure);              	//初始化DMA Stream
	HAL_DMA_Init(&DMA1_InitStructure);
	
	HAL_NVIC_SetPriority(USART2_IRQn,3,0);			        //抢占优先级3，子优先级0
	RS485_TX_Set(0);                                        //设置为接收模式	
    
	__HAL_UART_ENABLE_IT(&USART2_RS485Handler,UART_IT_IDLE);     //开启空闲接收中断
	HAL_UART_Receive_DMA(&USART2_RS485Handler,(uint8_t *)RS485_RXBuffer,RECEIVE485_SIZE);//开启DMA接收
 
 	HAL_NVIC_EnableIRQ(USART2_IRQn);				              //使能USART3中断
	HAL_NVIC_SetPriority(USART2_IRQn,3,0);			        //抢占优先级3，子优先级0
	RS485_TX_Set(0);                                        //设置为接收模式	
	RS485_flag=0;

}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(u8 *buf,u8 len)
{
	RS485_TX_Set(1);			//设置为发送模式
	HAL_UART_Transmit(&USART2_RS485Handler,buf,len,1000);//串口2发送数据
	while(__HAL_UART_GET_FLAG(&USART2_RS485Handler,USART_FLAG_TC)==RESET); 	//等待发送结束	
	RS485_TX_Set(0);			//设置为接收模式	
}

//RS485模式控制.
//en:0,接收;1,发送.
void RS485_TX_Set(u8 en)
{
	PCF8574_WriteBit(RS485_RE_IO,en);
}
