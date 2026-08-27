#ifndef __DS300_H
#define __DS300_H
#include "sys.h"


//非广播ID 0x00-0xFD
//广播ID 0xFE
typedef enum
{	
	ID_1 = 0x01,
	ID_2 = 0x02,
	ID_ALL = 0xFE,
}DS300_ID;


//指令类型
typedef enum
{	
	PING = 0x01,			//工作状态查询
	READ_DATA = 0x02,		//读数据
	WRITE_DATA = 0x03,		//写数据
	REG_WRITE_DATA = 0x04,	//异步写数据
	ACTION = 0x05,			//触发异步
	SYNC_WRITE_DATA = 0x83,	//同步写数据
	DS300_RESET = 0x06,		//恢复出厂数据
}DS300_INSTRUCTION;


//内存表设置参数(读/写)
//存储区域为EEPROM
//低位在前高位在后
//2字节参数范围为0x0000-0x0FFF，1字节参数范围为0x00-0xFE
typedef enum
{
	ID_t = 0x05,			//舵机ID
	BaudRate = 0x06,		//波特率
//	DelayTime = 0x07,		//应答延时
//	ResponseLevel = 0x08,	//应答级别
//	MinAngleLimit = 0x09,	//最小角度限制，2字节
//	MaxAngleLimit = 0x0B,	//最大角度限制，2字节
//	MaxTemperature = 0x0D,	//最高工作温度
//	MaxVoltage = 0x0E,		//最高工作电压
//	MinVoltage = 0x0F,		//最低工作电压
//	MaxTorque = 0x10,		//最大输出力矩，2字节
//	Unloading = 0x13,		//卸载条件
//	Warning = 0x14,			//报警条件
//	DS300_Kp = 0x15,		//比例系数
//	DS300_Kd = 0x16,		//微分系数
//	DS300_Ki = 0x17,		//积分系数
//	MinPwm = 0x18,			//最小PWM，2字节
//	SNoArea = 0x1A,			//顺时针死区
//	NNoArea = 0x1B,			//逆时针死区
//	IntegralLimit = 0x1C,	//积分限制，2字节
//	PositionCor = 0x21,		//0点位置校正，2字节
//	RunningMode = 0x23,		//运行模式
//	MaxCurrent = 0x24,		//保护电流，2字节
}DS300_WR_SET_DATA;


//内存表控制参数(读/写)
//存储区域为RAM
//低位在前高位在后
//2字节参数范围为0x0000-0x0FFF，1字节参数范围为0x00-0xFE
typedef enum
{
	TargetAngle = 0x2A,		//目标角度，2字节
	RunningTime = 0x2C,		//运行时间，2字节
	RunningSpeed = 0x2E,	//运行速度，2字节
	
//	TorqueSwitch = 0x28,	//力矩输出开关
//	Lock = 0x30,			//锁功能位
}DS300_WR_CONTROL_DATA;


//内存表反馈参数(只读)
//存储区域为RAM
//低位在前高位在后
//2字节参数范围为0x0000-0x0FFF，1字节参数范围为0x00-0xFE
typedef enum
{
	RealAngle = 0x38,		//当前角度，2字节
	RealSpeed = 0x3A,		//当前速度，2字节
	RealLoad = 0x3C,		//当前负载，2字节
	RealVoltage = 0x3E,		//当前电压
	RealTemp = 0x3F,		//当前温度
	RegWriteFlag = 0x40,	//异步写执行标志
	ErrorFlag = 0x41,		//舵机工作状态
	RunningFlag = 0x42,		//舵机运行标志
	RealTarAng = 0x43,		//当前目标角度，2字节
	RealCurrent = 0x45,		//当前电流，2字节
}DS300_R_FEEDBACK_DATA;


//舵机反馈参数
typedef struct
{
	uint8_t		id_real;		//当前ID
	uint16_t 	angle_real;		//当前角度			[0,4095]
	uint16_t	speed_real;		//当前速度			[0,4095]
	uint16_t  	load_real;		//当前负载			[0,1000]
	uint8_t		voltage_real;	//当前电压			精度0.1V
	uint8_t		temp_real;		//当前温度			精度1摄氏度
	uint8_t		regwrite_flag;	//异步写执行标志	1等待执行，0执行完毕
	uint8_t		error_flag;		//舵机工作状态		0状态正常，其他查阅内存表
	uint8_t		running_flag;	//舵机运行状态		1正在运行，0停止运行
	uint16_t 	tarang_real;	//当前目标角度		[0,4095]
	int16_t		current_real;	//当前电流			最高位为方向位
}DS300_Measure;


extern unsigned char DS300_Instruction_Data[24];	//指令包
extern DS300_Measure DS300_Chassis;					//舵机反馈参数


u8 DS300_Instruction_Fun(u8 ID, u8 instruction, u8 length, ...);	//生成舵机指令包(通用)
u8 DS300_Check_Sum(u8 len);											//计算指令包校验和
u8 DS300_Set_ID(u8 ID_old, u8 ID_new);								//设置舵机ID
u8 DS300_Set_BaudRate(u8 ID, u8 baud_rate);							//设置舵机波特率
u8 DS300_Ping_Instruction(u8 ID);									//读取舵机工作状态
u8 DS300_Read_Feedback_Instruction(u8 ID);							//读取舵机所有反馈参数
u8 DS300_Write_Control_Instruction(u8 ID, u16 target_angle, u16 running_time, u16 running_speed);				//写入舵机控制参数
u8 DS300_Reg_Write_Control_Instruction(u8 ID, u16 target_angle, u16 running_time, u16 running_speed);			//异步写入舵机控制参数，配合Action指令同时控制多个舵机
u8 DS300_Action_Instruction(u8 ID);																				//执行异步写入的参数，配合Reg_Write指令同时控制多个舵机
u8 DS300_Sync_Write_Control_Instruction(u8 ID1, u8 ID2, u16 target_angle, u16 running_time, u16 running_speed);	//同步写入舵机控制参数，同时控制多个舵机，相当于Reg_Write+Action
u8 DS300_Reset_Data(u8 ID);											//恢复出厂数据
void DS300_Read_Feedback_Response(u8 *buf, DS300_Measure *ptr);		//处理读取的舵机所有反馈参数

#endif

