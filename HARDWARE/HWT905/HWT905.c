#include "HWT905.h"
#include "delay.h"
#include "JY901.h"
#include "sensors.h"
#include "usart6.h"
/**************************************************************************************/
//陀螺仪型号：HWT905
//通信端口：TTL(USART6)、波特率：9600
/*   红色---->9-36VCC  */
/*   黄色---->TX       */
/*   绿色---->RX       */
/*   黑色---->GND      */
//TX和RX需要交叉
//采样周期不少于0.1s（10Hz）
//数据内容：加速度+角度+角速度+磁场，重力加速度取9.8
/***************************************************************************************/

extern rawData_t rawData;
DIAS_t zero_bias={0.0f};

//陀螺仪数据存储结构体，定义见JY901.h
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	    stcAngle;
struct SMag 		stcMag;
struct SDStatus     stcDStatus;
struct SPress 	    stcPress;
struct SLonLat 	    stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ           stcQ;

void HWT_Init(void)
{
	char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
//  char MAG[5] = {0XFF,0XAA,0X01,0X02,0X00};//进入磁场校准模式
	char GYRO[5]= {0XFF,0XAA,0x63,0X01,0X00};//选择陀螺仪自动校准,0X01。去掉陀螺仪自动校准,0X00。（改变倒数第二指令）
//	char DIRECTION[5]= {0XFF,0XAA,0x23,0X01,0X00};//设置安装方向,设置为垂直安装,0X01。设置为水平安装,0X00。
//	char ALG[5]= {0XFF,0XAA,0x24,0X00,0X00};//设置成 9 轴算法（最后一位：0—九轴算法（默认），1—6轴算法）
//	char RATE[5]= {0XFF,0XAA,0x03,0x06,0X00};//设置回传速率,10Hz（默认）,设置完成以后需要点保存配置按钮， 再给模块重新上电后生效
//	char BAUD_[5]= {0XFF,0XAA,0x04,0x02,0X00};//设置串口波特率,9600（默认）（倒数第2位：0x02—默认9600，0x06-115200）
	char SAVACALSW[5]= {0XFF,0XAA,0x00,0X00,0X00};//保存当前配置
	
	sendcmd(ACCCALSW);   /*进入加速度校准*/
	sendcmd (GYRO);      /*选择陀螺仪自动校准角速度*/
	sendcmd(SAVACALSW);  /*保存当前设置*/
	delay_xms(200);      /*延时0.2s，等待当前设置完成*/
	HAL_UART_Receive_IT(&UART6_Handler, (u8 *)aRxBuffer6, RXBUFFERSIZE6);  /*开启接收中断*/
}

//陀螺仪校准（加速度计、XY角度和XYZ角速度校准,读取前10次均值为零偏）
void HWT_CALIBRATION(sensorData_t* sensorData,DIAS_t* zero_bias)
{
	int i;
	float sumaccx,sumaccy,sumaccz,sumanglex,sumangley,sumangleratex,sumangleratey,sumangleratez;
	for(i=0;i<10;i++)   /*获取10次测量值求和*/
	{
		Cal_HWT_Data(sensorData);
		if(i>4)               //实际取后面5次求零偏
		{
			sumaccx+=sensorData->acc.x;
			sumaccy+=sensorData->acc.y;
			sumaccz+=sensorData->acc.z;
			sumanglex+=sensorData->angle.roll;
			sumangley+=sensorData->angle.pitch;
			sumangleratex+=sensorData->gyro.roll;
			sumangleratey+=sensorData->gyro.pitch;
			sumangleratez+=sensorData->gyro.yaw;
		}
		delay_xms(110);  /*默认一次回传速率为10Hz，故延时110ms*/
	}
	//求平均计算三轴加速度计偏差
	zero_bias->accx_bias=(float)sumaccx*0.2f-0.0f;
	zero_bias->accy_bias=(float)sumaccy*0.2f-0.0f;
	zero_bias->accz_bias=(float)sumaccz*0.2f-9.8f;   /*标准状态下，Z轴的加速度为g*/
	//求平均计算XY轴角度计偏差
	zero_bias->anglex_bias=(float)sumanglex*0.2f-0.0f;
	zero_bias->angley_bias=(float)sumangley*0.2f-0.0f;
	//求平均计算三轴角速度计偏差
	zero_bias->angle_ratex_bias=(float)sumangleratex*0.2f-0.0f;
	zero_bias->angle_ratey_bias=(float)sumangleratey*0.2f-0.0f;
	zero_bias->angle_ratez_bias=(float)sumangleratez*0.2f-0.0f;
}

void sendcmd(char cmd[])
{
	int i;
	for(i=0;i<5;i++)
	{	
		HAL_UART_Transmit(&UART6_Handler,(uint8_t*) cmd[i],1,1000);
	}
	while(__HAL_UART_GET_FLAG(&UART6_Handler,UART_FLAG_TC)!=SET);
}

void CharToShort(unsigned char cTemp[],short sTemp[],short sShortNum)
{
	int i;
	for (i = 0;i<3;i++) 
		sTemp[i] = (cTemp[2*i+sShortNum]<<8)|(cTemp[2*i+sShortNum+1]&0xff);
}

//CopeSerialData为写在串口6的读中断中的调用函数，串口每收到一个数据，调用一次这个函数（串口单字节传输）。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	
#if TEST_MODE	
	if (ucRxCnt<11) return;//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//清空缓存区
	}
//	printf("acc.x= %.2f\r\n",(float)stcAcc.a[0]/32768*16);
#endif
}	
 
/*读取姿态传感器数据(减去零偏值的校准输出)*/
void Cal_HWT_Data(sensorData_t* sensorData)
{
	//加速度(单位：m/s2)
	float g=9.8;     /*取重力加速度为9.8m/s2*/
	sensorData->acc.x = (float)stcAcc.a[0]/32768*16*g;
	sensorData->acc.y = (float)stcAcc.a[1]/32768*16*g;
	sensorData->acc.z = (float)stcAcc.a[2]/32768*16*g;
	
	//角速度(单位：deg/s)
	sensorData->gyro.roll = (float)stcGyro.w[0]/32768*2000;
	sensorData->gyro.pitch = (float)stcGyro.w[1]/32768*2000;
	sensorData->gyro.yaw= (float)stcGyro.w[2]/32768*2000;
	
	//角度(单位：deg)
	sensorData->angle.roll = (float)stcAngle.Angle[0]/32768*180;
	sensorData->angle.pitch= (float)stcAngle.Angle[1]/32768*180;
	sensorData->angle.yaw= (float)stcAngle.Angle[2]/32768*180;
	
	//磁场
	sensorData->mag.x = stcMag.h[0];
	sensorData->mag.y = stcMag.h[1];
	sensorData->mag.z = stcMag.h[2];
	
	//原始数据拷贝上传
	rawData.acc.x=stcAcc.a[0];      //short为16为有符号数
	rawData.acc.y=stcAcc.a[1];
	rawData.acc.z=stcAcc.a[2];
	rawData.gyro.roll=stcGyro.w[0];
	rawData.gyro.pitch=stcGyro.w[1];
	rawData.gyro.yaw=stcGyro.w[2];
	rawData.angle.roll=stcAngle.Angle[0];
	rawData.angle.pitch=stcAngle.Angle[1];
	rawData.angle.yaw=stcAngle.Angle[2];
	rawData.mag.x=stcMag.h[0];
	rawData.mag.y=stcMag.h[1];
	rawData.mag.z=stcMag.h[2];
	
}
