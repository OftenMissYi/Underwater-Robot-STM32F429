/*
************************************************************
*	文件：	ds300舵机驱动文件
*	作者：	赵斌
*	日期：	2021.08.12
*	说明：	目前编写的指令包的生成，可根据需求进行修改，详情请查阅DS300通信协议和内存表
*			目前仅编写了反馈数据的应答包处理，可根据需求进行增删
*			应答包采用rs485+dma中断接收，详情请查阅rs485.c
*	修改记录：
************************************************************
*/
#include "ds300.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "usart.h"



unsigned char DS300_Instruction_Data[24] = {0};		//指令包，参数数量超范围请扩大数组
DS300_Measure DS300_Chassis = {0};					//舵机反馈参数


/*
************************************************************
*	函数名称：	DS300_Instruction_Data_Fun
*	函数功能：	生成舵机指令包(通用)
*	入口参数：	ID：舵机ID
*				instruction：指令类型
*				length：参数长度，不定长参数列表
*	ID编号	：	非广播ID 0x00-0xFD
*				广播ID 0xFE
*	指令类型：	PING			工作状态查询	无参数列表								总线存在多个舵机时禁止使用广播ID
*				READ_DATA		读数据			2个参数 = 首地址 + 读取长度				总线存在多个舵机时禁止使用广播ID
*				WRITE_DATA 		写数据			N+1个参数 = 首地址 + N个写入数据
*				REG_WRITE_DATA	异步写数据		N+1个参数 = 首地址 + N个写入数据
*				ACTION			触发异步		无参数列表								使用广播ID可同时触发所有异步写的舵机
*				SYCN_WRITE_DATA 同步写数据		(L+1)*N+2个参数 = 首地址 + 写入数据长度L + N*(舵机ID+L个写入数据)	指令包ID使用广播ID，参数列表ID使用非广播ID
*				DS300_RESET		恢复出厂数据	无参数列表
*	返回参数：	指令包发送数据长度
*	注意事项：	参数列表详情请参阅DS300通信协议
*				u16参数请按高低位输入参数列表，低位在前高位在后
************************************************************
*/
u8 DS300_Instruction_Fun(u8 ID, u8 instruction, u8 length, ...)
{
	u8 check_sum = 0;
	u8 len = 0;
	u8 i = 0;
	
	len = length + 2;
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;			//帧头
	DS300_Instruction_Data[2] = ID;				//ID
	DS300_Instruction_Data[3] = len;			//数据长度=参数长度+2
	DS300_Instruction_Data[4] = instruction;	//指令类型
	
	if(length > 0)								//如果具有参数
	{
		va_list vap;							//创建参数列表vap
		va_start(vap, length);					//初始化参数列表vap
		for(i=0; i<length; i++)
		{
			DS300_Instruction_Data[i+5] = va_arg(vap, int);
		}
		va_end(vap);							//关闭参数列表
	}

	len = len + 3;
	check_sum = DS300_Check_Sum(len);
	DS300_Instruction_Data[len] = check_sum;
	
	return len+1;
}



/*
************************************************************
*	函数名称：	DS300_Check_Sum
*	函数功能：	计算指令包校验和
*	入口参数：	len：数组长度
*	返回参数：	指令包校验和
*	注意事项：	check_sum定义为u8，无需关心通信协议中大于255取低位，故if可以省略
************************************************************
*/
u8 DS300_Check_Sum(u8 len)
{
	u8 i = 0;
	u8 check_sum = 0;
	for(i=2; i<len; i++)
	{
		check_sum += DS300_Instruction_Data[i];
	}
//	if(check_sum > 255)
//	{
//		check_sum = (u8)check_sum;
//	}
	check_sum = ~check_sum;
	
	return check_sum;
}



/*
************************************************************
*	函数名称：	DS300_Set_ID
*	函数功能：	设置舵机ID的指令
*	入口参数：	ID_old：原始ID
*				ID_new：设置ID
*	返回参数：	指令包发送数据长度
*	注意事项：
************************************************************
*/
u8 DS300_Set_ID(u8 ID_old, u8 ID_new)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID_old;
	DS300_Instruction_Data[3] = 0x04;		//数据长度为4
	DS300_Instruction_Data[4] = WRITE_DATA;	//写指令
	DS300_Instruction_Data[5] = ID_t;		//地址为ID
	DS300_Instruction_Data[6] = ID_new;		//设置ID

	check_sum = DS300_Check_Sum(7);
	DS300_Instruction_Data[7] = check_sum;
	
	return 8;      //返回指令长度
}



/*
************************************************************
*	函数名称：	DS300_Set_BaudRate
*	函数功能：	设置舵机波特率的指令
*	入口参数：	ID：舵机ID
*				baud_rate：波特率
*				0x00-1M, 0x01-500K, 0x02-250K, 0x03-128K,
*				0x04-115200, 0x05-76800, 0x06-57600, 0x07-38400
*	返回参数：	指令包发送数据长度
*	注意事项：
************************************************************
*/
u8 DS300_Set_BaudRate(u8 ID, u8 baud_rate)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x04;		//数据长度为4
	DS300_Instruction_Data[4] = WRITE_DATA;	//写指令
	DS300_Instruction_Data[5] = BaudRate;	//地址为波特率
	DS300_Instruction_Data[6] = baud_rate;	//设置波特率
	
	check_sum = DS300_Check_Sum(7);
	DS300_Instruction_Data[7] = check_sum;
	
	return 8;
}



/*
************************************************************
*	函数名称：	DS300_Ping_Data
*	函数功能：	读取舵机工作状态的指令
*	入口参数：	ID：舵机ID
*	返回参数：	指令包发送数据长度
*	注意事项：	总线存在多个舵机时禁止使用广播ID
************************************************************
*/
u8 DS300_Ping_Instruction(u8 ID)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x02;		//数据长度为2
	DS300_Instruction_Data[4] = PING;

	check_sum = DS300_Check_Sum(5);
	DS300_Instruction_Data[5] = check_sum;
	
	return 6;
}



/*
************************************************************
*	函数名称：	DS300_Read_Feedback_Data
*	函数功能：	读取舵机所有反馈参数的指令
*	入口参数：	ID：舵机ID
*	返回参数：	指令包发送数据长度
*	注意事项：	总线存在多个舵机时禁止使用广播ID
************************************************************
*/
u8 DS300_Read_Feedback_Instruction(u8 ID)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x04;		//数据长度为4
	DS300_Instruction_Data[4] = READ_DATA;
	DS300_Instruction_Data[5] = RealAngle;	//首地址为当前角度
	DS300_Instruction_Data[6] = 0x0F;		//读取15个字节

	check_sum = DS300_Check_Sum(7);
	DS300_Instruction_Data[7] = check_sum;
	
	return 8;
}



/*
************************************************************
*	函数名称：	DS300_Write_Control_Instruction
*	函数功能：	写入舵机控制参数的指令
*	入口参数：	ID：舵机ID
*				target_angle	目标角度	[0x0000, 0x0FFF](0-4095)
*				running_time	运行时间	[0x0000, 0x0FFF]			1ms	0为最大速度
*				running_speed	运行速度	[0x0000, 0x0FFF]			0.087°/s 0为最大速度		
*	返回参数：	指令包发送数据长度
*	注意事项：	速度优先级>时间优先级，同时写入将执行速度参数
************************************************************
*/
u8 DS300_Write_Control_Instruction(u8 ID, u16 target_angle, u16 running_time, u16 running_speed)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x09;				//数据长度为9
	DS300_Instruction_Data[4] = WRITE_DATA;
	DS300_Instruction_Data[5] = TargetAngle;		//首地址为目标角度
	DS300_Instruction_Data[6] = (u8)target_angle;	//低位在前高位在后
	DS300_Instruction_Data[7] = (u8)(target_angle>>8);
	DS300_Instruction_Data[8] = (u8)running_time;
	DS300_Instruction_Data[9] = (u8)(running_time>>8);
	DS300_Instruction_Data[10] = (u8)running_speed;
	DS300_Instruction_Data[11] = (u8)(running_speed>>8);

	check_sum = DS300_Check_Sum(12);
	DS300_Instruction_Data[12] = check_sum;
	
	return 13;    //返回指令长度
}



/*
************************************************************
*	函数名称：	DS300_Reg_Write_Control_Instruction
*	函数功能：	异步写入舵机控制参数的指令，配合Action指令同时控制多个舵机
*	入口参数：	ID：舵机ID
*				target_angle	目标角度	[0x0000, 0x0FFF](0-4095)
*				running_time	运行时间	[0x0000, 0x0FFF]			1ms	0为最大速度
*				running_speed	运行速度	[0x0000, 0x0FFF]			0.087°/s 0为最大速度
*	返回参数：	指令包发送数据长度
*	注意事项：
************************************************************
*/
u8 DS300_Reg_Write_Control_Instruction(u8 ID, u16 target_angle, u16 running_time, u16 running_speed)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x09;				//数据长度为9
	DS300_Instruction_Data[4] = REG_WRITE_DATA;
	DS300_Instruction_Data[5] = TargetAngle;		//首地址为目标角度
	DS300_Instruction_Data[6] = (u8)target_angle;	//低位在前高位在后
	DS300_Instruction_Data[7] = (u8)(target_angle>>8);
	DS300_Instruction_Data[8] = (u8)running_time;
	DS300_Instruction_Data[9] = (u8)(running_time>>8);
	DS300_Instruction_Data[10] = (u8)running_speed;
	DS300_Instruction_Data[11] = (u8)(running_speed>>8);

	check_sum = DS300_Check_Sum(12);
	DS300_Instruction_Data[12] = check_sum;
	
	return 13;
}



/*
************************************************************
*	函数名称：	DS300_Action
*	函数功能：	执行异步写入的指令，配合Reg_Write指令同时控制多个舵机
*	入口参数：	ID：舵机ID
*	返回参数：	指令包发送数据长度
*	注意事项：	同时控制多电机使用广播ID
************************************************************
*/
u8 DS300_Action_Instruction(u8 ID)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x02;		//数据长度为2
	DS300_Instruction_Data[4] = ACTION;

	check_sum = DS300_Check_Sum(5);
	DS300_Instruction_Data[5] = check_sum;
	
	return 6;
}



/*
************************************************************
*	函数名称：	DS300_Sync_Write_Control_Instruction
*	函数功能：	同步写入舵机控制参数的指令，同时控制多个舵机，相当于Reg_Write+Action
*	入口参数：	ID1：舵机ID1
*				ID2：舵机ID2
*				target_angle	目标角度	[0x0000, 0x0FFF](0-4095)
*				running_time	运行时间	[0x0000, 0x0FFF]			1ms	0为最大速度
*				running_speed	运行速度	[0x0000, 0x0FFF]			0.087°/s 0为最大速度
*	返回参数：	指令包发送数据长度
*	注意事项：	目前是两个舵机同步运行，可根据需求进行修改
************************************************************
*/
u8 DS300_Sync_Write_Control_Instruction(u8 ID1, u8 ID2, u16 target_angle, u16 running_time, u16 running_speed)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = 0xFE;				//广播ID
	DS300_Instruction_Data[3] = 0x12;				//2个舵机数据长度为18
	DS300_Instruction_Data[4] = SYNC_WRITE_DATA;
	DS300_Instruction_Data[5] = TargetAngle;		//首地址为目标角度
	DS300_Instruction_Data[6] = 0x06;				//每个舵机写入6个字节
	DS300_Instruction_Data[7] = ID1;				//舵机ID1
	DS300_Instruction_Data[8] = (u8)target_angle;	//低位在前高位在后
	DS300_Instruction_Data[9] = (u8)(target_angle>>8);
	DS300_Instruction_Data[10] = (u8)running_time;
	DS300_Instruction_Data[11] = (u8)(running_time>>8);
	DS300_Instruction_Data[12] = (u8)running_speed;
	DS300_Instruction_Data[13] = (u8)(running_speed>>8);
	DS300_Instruction_Data[14] = ID2;				//舵机ID2
	DS300_Instruction_Data[15] = (u8)target_angle;
	DS300_Instruction_Data[16] = (u8)(target_angle>>8);
	DS300_Instruction_Data[17] = (u8)running_time;
	DS300_Instruction_Data[18] = (u8)(running_time>>8);
	DS300_Instruction_Data[19] = (u8)running_speed;
	DS300_Instruction_Data[20] = (u8)(running_speed>>8);

	check_sum = DS300_Check_Sum(21);
	DS300_Instruction_Data[21] = check_sum;
	
	return 22;
}



/*
************************************************************
*	函数名称：	DS300_Reset_Data
*	函数功能：	恢复出厂数据的指令
*	入口参数：	ID：舵机ID
*	返回参数：	指令包发送数据长度
*	注意事项：
************************************************************
*/
u8 DS300_Reset_Instruction(u8 ID)
{
	u8 check_sum = 0;
	
	DS300_Instruction_Data[0] = 0xFF;
	DS300_Instruction_Data[1] = 0xFF;
	DS300_Instruction_Data[2] = ID;
	DS300_Instruction_Data[3] = 0x02;		//数据长度为2
	DS300_Instruction_Data[4] = DS300_RESET;

	check_sum = DS300_Check_Sum(5);
	DS300_Instruction_Data[5] = check_sum;
	
	return 6;
}



/*
************************************************************
*	函数名称：	DS300_Read_Feedback_Response
*	函数功能：	处理读取的舵机所有反馈参数
*	入口参数：	u8 *buf：反馈参数数组
*				DS300_Measure *ptr：反馈参数结构体
*	返回参数：	无
*	注意事项：	根据需求进行删减
************************************************************
*/
void DS300_Read_Feedback_Response(u8 *buf, DS300_Measure *ptr)
{
	u8 i = 0;
	u8 check_sum = 0;
	
	if(buf[0]==0xFF && buf[1]==0xFF)
	{
		for(i=2; i<20; i++)		//舵机所有反馈参数的应答包总计21字节
		{
			check_sum += buf[i];
		}
		check_sum = ~check_sum;
		if(check_sum==buf[20])
		{
			ptr->id_real = buf[2];
			ptr->angle_real = (uint16_t)(buf[5] | buf[6]<<8);
			ptr->speed_real = (uint16_t)(buf[7] | buf[8]<<8);
			ptr->load_real = (uint16_t)(buf[9] | buf[10]<<8);
			ptr->voltage_real = buf[11];
			ptr->temp_real = buf[12];
			ptr->regwrite_flag = buf[13];
			ptr->error_flag = buf[14];
			ptr->running_flag = buf[15];
			ptr->tarang_real = (uint16_t)(buf[16] | buf[17]<<8);
			ptr->current_real = (uint16_t)(buf[18] | buf[19]<<8);

//测试使用
//			printf("id = %d\r\n", ptr->id_real);
//			printf("angle = %d\r\n", ptr->angle_real);
//			printf("speed = %d\r\n", ptr->speed_real);
//			printf("load = %d\r\n", ptr->load_real);
//			printf("voltage = %d\r\n", ptr->voltage_real);
//			printf("temp = %d\r\n", ptr->temp_real);
//			printf("regwrite = %d\r\n", ptr->regwrite_flag);
//			printf("error = %d\r\n", ptr->error_flag);
//			printf("running = %d\r\n", ptr->running_flag);
//			printf("tarang = %d\r\n", ptr->tarang_real);
//			printf("current = %d\r\n\r\n", ptr->current_real);
		}
	}	
}
