#include "rs232.h"
#include "delay.h"
#include "led.h"
#include "sys.h"
#include "string.h"
#include "link232.h"
#include "pid_flash_init.h"
#include "FreeRTOS.h"
#include "semphr.h"

//点灯LED0：DMA每正确接收一次，则状态就反转
 
//DMA_HandleTypeDef dma1_Handler;//DMA1句柄

UART_HandleTypeDef UART3_Handler;     //USART3句柄(用于RS232)
DMA_HandleTypeDef DMA_InitStructure;//DMA1句柄

uint8_t RS232_RXBuffer[RECEIVE_SIZE];                       //接收缓冲,最大128个字节
int16_t ch[12]={0};                      //遥控器数据转换

extern  int16_t rx232buf[12];
u8 RS232_flag;                            // RS232接收标志
u8 remoteliink_flag=0;                      // 遥控器连接标志
u8 PID_flag=0;                            // PID参数接收标志
 
/*******************************************************************************
* 函 数 名         : RS232_Init
* 函数功能	  	   :232初始化函数(urat2)
* 输    入         : bound:波特率
* 输    出         : 无
*******************************************************************************/  

void uart3_Init(u32 bound)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_DMA1_CLK_ENABLE();		    //DMA1时钟使能
	__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOB时钟
	__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
	
	__HAL_LINKDMA(&UART3_Handler,hdmarx,DMA_InitStructure);//连接DMA1与串口3
	
	GPIO_Initure.Pin=GPIO_PIN_10; //PB10,11
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_FAST;		//快速
	
	GPIO_Initure.Alternate=GPIO_AF7_USART3;	//复用为USART3
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10,11
	
	GPIO_Initure.Pin=GPIO_PIN_11;
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);	
	
	//DMA1通道4，数据流1
	DMA_InitStructure.Instance=DMA1_Stream1;                                //数据流选择1
	DMA_InitStructure.Init.Channel = DMA_CHANNEL_4;  						//通道选择4
	DMA_InitStructure.Init.Direction =DMA_PERIPH_TO_MEMORY;					//存储器到外设模式    ？外设到存储器
	DMA_InitStructure.Init.PeriphInc = DMA_PINC_DISABLE;		            //外设非增量模式
	DMA_InitStructure.Init.MemInc =DMA_MINC_ENABLE;		          			//存储器增量模式
	DMA_InitStructure.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;    	//外设数据长度:8位
	DMA_InitStructure.Init.MemDataAlignment= DMA_MDATAALIGN_BYTE;		    //存储器数据长度:8位
	DMA_InitStructure.Init.Mode = DMA_CIRCULAR ;							//使用循环模式 
	DMA_InitStructure.Init.Priority = DMA_PRIORITY_HIGH;					//高优先级
	DMA_InitStructure.Init.FIFOMode= DMA_FIFOMODE_DISABLE;         
	DMA_InitStructure.Init.FIFOThreshold= DMA_FIFO_THRESHOLD_FULL;
	DMA_InitStructure.Init.MemBurst= DMA_MBURST_SINGLE;				//存储器突发单次传输
	DMA_InitStructure.Init.PeriphBurst = DMA_PBURST_SINGLE;		//外设突发单次传输					
	HAL_DMA_DeInit(&DMA_InitStructure);              	//初始化DMA Stream
	HAL_DMA_Init(&DMA_InitStructure);
    
    //USART 初始化设置
	UART3_Handler.Instance=USART3;			        //USART3
	UART3_Handler.Init.BaudRate=bound;		        //波特率
	UART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;	//字长为8位数据格式
	UART3_Handler.Init.StopBits=UART_STOPBITS_1;		//一个停止位
	UART3_Handler.Init.Parity=UART_PARITY_NONE;		//无奇偶校验位
	UART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;	//无硬件流控
	UART3_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART3_Handler);			        //HAL_UART_Init()会使能USART3
  
	HAL_NVIC_EnableIRQ(USART3_IRQn);				              //使能USART3中断
	HAL_NVIC_SetPriority(USART3_IRQn,5,0);			              //抢占优先级5，子优先级0
		
	__HAL_UART_ENABLE_IT(&UART3_Handler,UART_IT_IDLE);     //开启空闲接收中断
	HAL_UART_Receive_DMA(&UART3_Handler,(uint8_t *)RS232_RXBuffer,RECEIVE_SIZE);//开启DMA接收
	
   RS232_flag=0;

}

extern SemaphoreHandle_t BinarySemaphore;	//二值信号量句柄

void USART3_IRQHandler(void)
{	
	BaseType_t xHigherPriorityTaskWoken;
    if(__HAL_UART_GET_FLAG(&UART3_Handler,UART_FLAG_IDLE)==SET)  //空闲中断
	{	 	
		__HAL_UART_CLEAR_IDLEFLAG(&UART3_Handler);//清除空闲中断接受标志位
		HAL_UART_DMAStop(&UART3_Handler);//关闭DMA接收
		
	//释放二值信号量
	if(BinarySemaphore!=NULL)//接收到数据，并且二值信号量有效
	{
		xSemaphoreGiveFromISR(BinarySemaphore,&xHigherPriorityTaskWoken);	//释放二值信号量，将后续逻辑处理放入任务中，减少中断处理函数执行时间，提高系统实时性
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
	}
//		LED0=!LED0;
//		//接收遥控器命令
//		if(RS232_RXBuffer[0]==0x20 && RS232_RXBuffer[1]==0x40)      //校验数据接收正确
//		{
//			ch[1] = (int16_t)(((u16)(RS232_RXBuffer[3]&0xFF)<<8) | (u16)RS232_RXBuffer[2]);               //通道1
//			ch[2] = (int16_t)(((u16)(RS232_RXBuffer[5]&0xFF)<<8) | (u16)RS232_RXBuffer[4]);               //通道2
//			ch[3] = (int16_t)(((u16)(RS232_RXBuffer[7]&0xFF)<<8) | (u16)RS232_RXBuffer[6]);               //通道3
//			ch[4] = (int16_t)(((u16)(RS232_RXBuffer[9]&0xFF)<<8) | (u16)RS232_RXBuffer[8]);               //通道4
//			ch[5] = (int16_t)(((u16)(RS232_RXBuffer[11]&0xFF)<<8) | (u16)RS232_RXBuffer[10]);             //通道5
//			ch[6] = (int16_t)(((u16)(RS232_RXBuffer[13]&0xFF)<<8) | (u16)RS232_RXBuffer[12]);             //通道6
//			ch[7] = (int16_t)(((u16)(RS232_RXBuffer[15]&0xFF)<<8) | (u16)RS232_RXBuffer[14]);             //通道7
//			ch[8] = (int16_t)(((u16)(RS232_RXBuffer[17]&0xFF)<<8) | (u16)RS232_RXBuffer[16]);             //通道8
//			ch[9] = (int16_t)(((u16)(RS232_RXBuffer[19]&0xFF)<<8) | (u16)RS232_RXBuffer[18]);             //通道9
//			ch[10] = (int16_t)(((u16)(RS232_RXBuffer[21]&0xFF)<<8) | (u16)RS232_RXBuffer[20]);            //通道10
//			LED0=!LED0;
//			RS232_flag=1;            //数据正常接收，标志位为1
//			remoteliink_flag=1;
//			for(i=1;i<=10;i++)
//			{  
//				rx232buf[i]=ch[i];	
//			}	          
//			delay_xms(5);
//		}                              //接收PID调试参数         
//		else if(RS232_RXBuffer[0]==0x01 && RS232_RXBuffer[1]==0x02)
//		{
//			PID_flag=1;
//			Rs232_Receive_PID((uint8_t*)RS232_RXBuffer);//接收发来的pid参数并写入flash
//			LED0=!LED0;           //两灯快速闪烁，表示在PID参数调整阶段
//			LED1=!LED1;
//		}
	}
		HAL_UART_Receive_DMA(&UART3_Handler,(uint8_t *)RS232_RXBuffer,RECEIVE_SIZE);//使能DMA接收
}   

//RS232发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS232_Send_Data(u8 *buf,u8 len)
{
	HAL_UART_Transmit(&UART3_Handler,buf,len,1000);//串口3发送数据
}


