#include "sensors.h"
#include "JY901.h"
#include "string.h"
#include <math.h>
#include "usart6.h"
#include "HWT905.h"
#include "control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "MS5837.h"
#include "pid.h"
#include "dht22.h"
#include "link232.h"
#include "bmp280.h"
#include "led.h"
#include "queue.h"	

/*陀螺仪和深度计的信息读取、滤波和数据融合*/
/*陀螺仪用来测量自身状态：角度、角速度、加速度，用于姿态调整和运动控制*/
/*深度计用来测量深度信息，用于定高模式*/
extern DIAS_t zero_bias;
extern state_t state;
extern sensorData_t sensorData;     /*定义传感器结构体*/
extern double Temperature;
extern int32_t Pressure;
extern setstate_t setstate;
extern vec3_s acc;

extern QueueHandle_t Message_Queue;	//信息队列句柄

float bmp280_press;             /*定义bmp280气压和温度变量*/
float bmp280_temp;

rawData_t rawData={0};
att_t attt;

u16 dht22_temp,dht22_humi;
u32 bmp280_pre;

//验证陀螺仪校准输出
SensorClibData_t SensorClibData={0};

////陀螺仪
//uint8_t ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
//uint8_t SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置

//定义存储传感器数据的buf
u8 sensors_buf[64]; 

 /*传感器初始化*/
void sensorsInit(void)
{
	u8 count=10;
	//uart6_init(9600);        //初始化串口6，接收陀螺仪数据
	HWT_Init();             /*陀螺仪传感器初始化*/
	delay_xms(10);
	DHT22_Init();            //初始化湿温传感器

	if(DHT22_Init()==1)
	{
		printf("\r\n湿温度传感器响应超时！！！\r\n");
	}
	while(count--)   //刚开始传感器数据不准，先读10次等数据稳定，可能需要1s
	{
		Cal_HWT_Data(&sensorData); 
	}
	HWT_CALIBRATION(&sensorData,&zero_bias); //再读10次，取后5次计算陀螺仪初始偏差  
  MS5837_30BA_init();      // 深度传感器初始化
	BMP280_Init();           //初始化BMP280
	delay_xms(100);
	printf("\r\n所有传感器初始化完成！\r\n");
}

/*****************************************************传感器处理任务函数*************************************************************************/
void sensors_task(void *pvParameters)
{
	/*初始获取传感器值,这部分应该只执行一次，下一个while循环里使用延时发生任务调度*/
	//time1_count=2000;
	sensorsInit ();          /*传感器初始化*/   
	Cal_HWT_Data(&sensorData);   /*获取陀螺仪数据(减去零偏之后的真实值)*/
	depthFIR(&sensorData);        /*获取深度*/
	
	/*初始化之后，所有期望值复制为实际值初值*/
	state.realAngle.roll = sensorData.angle.roll-zero_bias.anglex_bias;
	state.realAngle.pitch = sensorData.angle.pitch-zero_bias.angley_bias;
	state.realAngle.yaw = sensorData.angle.yaw;
	state.realDepth = sensorData.depth;
	
	//初始化之后将当前的姿态角作为期望姿态角初值
	setstate.expectedAngle.roll = state.realAngle.roll;
	setstate.expectedAngle.pitch = state.realAngle.pitch;
	setstate.expectedAngle.yaw = state.realAngle.yaw; 
	setstate.expectedDepth = state.realDepth;

	while(1)
	{ 
	
		/*获取机器人的真实状态*/
		Cal_HWT_Data(&sensorData);                 /*获取陀螺仪数据*/
		
		//实际状态（减零偏）
		sensorData.acc.x=sensorData.acc.x-zero_bias.accx_bias;
		sensorData.acc.y=sensorData.acc.y-zero_bias.accy_bias;
		sensorData.acc.z=sensorData.acc.z-zero_bias.accz_bias;
		sensorData.angle.roll=sensorData.angle.roll-zero_bias.anglex_bias;
		sensorData.angle.pitch=sensorData.angle.pitch-zero_bias.angley_bias;
		sensorData.gyro.roll=sensorData.gyro.roll-zero_bias.angle_ratex_bias;
		sensorData.gyro.pitch=sensorData.gyro.pitch-zero_bias.angle_ratey_bias;
		sensorData.gyro.yaw=sensorData.gyro.yaw-zero_bias.angle_ratez_bias;
		
		//将零偏值扩大10倍取整，上传
		rawData.HWT905_bais.acc_x=(short)(zero_bias.accx_bias*10); 
		rawData.HWT905_bais.acc_y=(short)(zero_bias.accy_bias*10);
		rawData.HWT905_bais.acc_z=(short)(zero_bias.accz_bias*10);
		rawData.HWT905_bais.ro11=(short)(zero_bias.anglex_bias*10);
		rawData.HWT905_bais.pitch=(short)(zero_bias.angley_bias*10);
		rawData.HWT905_bais.ro11_w=(short)(zero_bias.angle_ratex_bias*10);
		rawData.HWT905_bais.pitch_w=(short)(zero_bias.angle_ratey_bias*10);
		rawData.HWT905_bais.yaw_w=(short)(zero_bias.angle_ratez_bias*10);
		
//		sensorData.acc.x=sensorData.acc.x;
//		sensorData.acc.y=sensorData.acc.y;
//		sensorData.acc.z=sensorData.acc.z;
//		sensorData.angle.roll=sensorData.angle.roll;
//		sensorData.angle.pitch=sensorData.angle.pitch;
//		sensorData.gyro.roll=sensorData.gyro.roll;
//		sensorData.gyro.pitch=sensorData.gyro.pitch;
//		sensorData.gyro.yaw=sensorData.gyro.yaw;
		
		//实际状态（未减零偏）
		SensorClibData.acc.x=sensorData.acc.x;
		SensorClibData.acc.y=sensorData.acc.y;
		SensorClibData.acc.z=sensorData.acc.z;
		SensorClibData.angle.roll=sensorData.angle.roll;
		SensorClibData.angle.pitch=sensorData.angle.pitch;
		SensorClibData.angle.yaw=sensorData.angle.yaw;
		SensorClibData.gyro.roll=sensorData.gyro.roll;
		SensorClibData.gyro.pitch=sensorData.gyro.pitch;
		SensorClibData.gyro.yaw=sensorData.gyro.yaw;
	
		//输出测试(加速度、角度、角速度)
		printf("sensorData.acc.x= %.2f\r\n",(float)sensorData.acc.x);
		printf("sensorData.acc.y= %.2f\r\n",(float)sensorData.acc.y);
		printf("sensorData.acc.z= %.2f\r\n",(float)sensorData.acc.z);
		printf("sensorData.angle.roll= %.2f\r\n",(float)sensorData.angle.roll);
		printf("sensorData.angle.pitch= %.2f\r\n",(float)sensorData.angle.pitch);
		printf("sensorData.angle.yaw= %.2f\r\n",(float)sensorData.angle.yaw);
		printf("sensorData.gyro.roll= %.2f\r\n",(float)sensorData.gyro.roll);
		printf("sensorData.gyro.pitch= %.2f\r\n",(float)sensorData.gyro.pitch);
		printf("sensorData.gyro.yaw= %.2f\r\n",(float)sensorData.gyro.yaw);
		
	
		/*结构体类型转换*/
		acc.x=sensorData.acc.x;
		acc.y=sensorData.acc.y;
		acc.z=sensorData.acc.z;
		integralUpdate(&acc,&state.realvelocity);         /*加速度积分得到速度*/
		
		depthFIR(&sensorData);                        /*读取滑动滤波后的深度计数据*/
		
		/* 这种指定周期方式在这里不起作用，不知道原因	*/	
		/*原因是程序其他地方存在延时，导致这里每执行一次不是理想中的1ms，而这个程序需要执行2000次，才会更新一次湿温度参数，所以短时间看不到更新*/
//		//每隔2s更新一次湿温度数据（湿温度模块的采样时间需要2s）
//		delayms();               //调用延时1ms函数
//		if(time1_count>=2000)        //调用2000次延时函数，每次延时1ms
//		{
//			DHT22_Read_Data(&sensorData.cabin_temp,&sensorData.cabin_humi);       /*读取湿度和温度数据，读取周期2s*/
//			time1_count=0;
//		}
//		else time1_count++;

//		/*由于上面方法不行，这里还是采用硬延时，保证大于2s的采样周期*/
//		DHT22_Read_Data(&sensorData.cabin_temp,&sensorData.cabin_humi);       /*读取湿度和温度数据，读取周期2s*/
//		delay_xms(2000);
		/*不管传感器的采样时间为2s，直接读取，不过未到采样时间下读取的上一个采样值，但问题不大*/
		DHT22_Read_Data(&sensorData.cabin_temp,&sensorData.cabin_humi);       /*读取湿度和温度数据，读取周期2s*/
		if(BMP280_ReadPressureTemperature(&sensorData.cabin_press,&bmp280_temp)==0)                 /*读取BMP280气压数据*/ 
		{
			rawData.cabin_press=(short)sensorData.cabin_press*10;                      /*将BMP280气压拷贝上传(u32)*/  
		}                 		
	
		//输出测试(湿度、温度、气压)
		printf("sensorData.cabin_temp= %.2f\r\n",(float)sensorData.cabin_temp);
		printf("sensorData.cabin_humi= %.2f\r\n",(float)sensorData.cabin_humi);
		printf("sensorData.cabin_press= %.2f\r\n",(float)sensorData.cabin_press);
		
		/*获取真实角度*/ 
		state.realAngle.roll = sensorData.angle.roll;
		state.realAngle.pitch =sensorData.angle.pitch;
		state.realAngle.yaw = sensorData.angle.yaw;
		
		/*获取真实深度*/   
		state.realDepth =sensorData.depth;          
		
		/*获取真实角速度*/ 		
		state.realRate.roll = sensorData.gyro.roll;
		state.realRate.pitch = sensorData.gyro.pitch;
		state.realRate.yaw =sensorData.gyro.yaw;
		
		
		
		//将传感器数据存入队列
		sensors_buf[0]=0x44; 
		sensors_buf[1]=rawData.acc.x & 0xff;        //低位先出
		sensors_buf[2]=(rawData.acc.x>>8) & 0xff;   //高位后出
		sensors_buf[3]=rawData.acc.y & 0xff;
		sensors_buf[4]=(rawData.acc.y>>8) & 0xff; 
		sensors_buf[5]=rawData.acc.z & 0xff;
		sensors_buf[6]=(rawData.acc.z>>8) & 0xff; 
		
		//第7-12位为角度数据
		sensors_buf[7]=rawData.angle.roll & 0xff;
		sensors_buf[8]=(rawData.angle.roll>>8) & 0xff;
		sensors_buf[9]=rawData.angle.pitch & 0xff;
		sensors_buf[10]=(rawData.angle.pitch>>8) & 0xff;
		sensors_buf[11]=rawData.angle.yaw & 0xff;
		sensors_buf[12]=(rawData.angle.yaw>>8) & 0xff;
		
		//第13-18位为角速度数据
		sensors_buf[13]=rawData.gyro.roll & 0xff;
		sensors_buf[14]=(rawData.gyro.roll>>8) & 0xff;
		sensors_buf[15]=rawData.gyro.pitch & 0xff;
		sensors_buf[16]=(rawData.gyro.pitch>>8) & 0xff;
		sensors_buf[17]=rawData.gyro.yaw & 0xff;
		sensors_buf[18]=(rawData.gyro.yaw>>8) & 0xff;

		//第19-20位为深度数据，第21-22位为水温数据
		sensors_buf[19]=rawData.depth & 0xff;
		sensors_buf[20]=(rawData.depth>>8) & 0xff;
		sensors_buf[21]=rawData.water_temp & 0xff;
		sensors_buf[22]=(rawData.water_temp>>8) & 0xff;
		
		//第23-24位为电子舱温度，第25-26位为湿度，第27-30位为压力数据
		sensors_buf[23]=rawData.cabin_temp & 0xff;
		sensors_buf[24]=(rawData.cabin_temp>>8) & 0xff;
		sensors_buf[25]=rawData.cabin_humi & 0xff;
		sensors_buf[26]=(rawData.cabin_humi>>8) & 0xff;
		sensors_buf[27]=rawData.cabin_press & 0xff;
		sensors_buf[28]=(rawData.cabin_press >>8) & 0xff;
		sensors_buf[29]=(rawData.cabin_press >>16) & 0xff;
		sensors_buf[30]=(rawData.cabin_press >>24) & 0xff;
		
		//第31-46位为陀螺仪零偏
		sensors_buf[31]=rawData.HWT905_bais.acc_x & 0xff;
		sensors_buf[32]=(rawData.HWT905_bais.acc_x>>8) & 0xff;
		sensors_buf[33]=rawData.HWT905_bais.acc_y & 0xff;
		sensors_buf[34]=(rawData.HWT905_bais.acc_y>>8) & 0xff;
		sensors_buf[35]=rawData.HWT905_bais.acc_z & 0xff;
		sensors_buf[36]=(rawData.HWT905_bais.acc_z>>8) & 0xff;
		sensors_buf[37]=rawData.HWT905_bais.ro11 & 0xff;
		sensors_buf[38]=(rawData.HWT905_bais.ro11>>8) & 0xff;
		sensors_buf[39]=rawData.HWT905_bais.pitch & 0xff;
		sensors_buf[40]=(rawData.HWT905_bais.pitch>>8) & 0xff;
		sensors_buf[41]=rawData.HWT905_bais.ro11_w & 0xff;
		sensors_buf[42]=(rawData.HWT905_bais.ro11_w>>8) & 0xff;
		sensors_buf[43]=rawData.HWT905_bais.pitch_w & 0xff;
		sensors_buf[44]=(rawData.HWT905_bais.pitch_w>>8) & 0xff;
		sensors_buf[45]=rawData.HWT905_bais.yaw_w & 0xff;
		sensors_buf[46]=(rawData.HWT905_bais.yaw_w>>8) & 0xff;
		
		if(Message_Queue!=NULL)
		{
				if(xQueueSend(Message_Queue,sensors_buf,portMAX_DELAY)!=pdPASS)//向队列中发送数据
				{
						printf("队列Key_Queue已满，数据发送失败!\r\n");
				}
		else
		{
			 //memset(sensors_buf,0,64);//清除数据接收缓冲区USART_RX_BUF,用于下一次数据接收;
		}
	
		}
				delay_ms(20);	/*每次深度传感器读取时间近20ms，故这里延时小一点*/
		}
}
/******************************************************************************************************************************/
/*读取深度传感器数据*/
void sensorReadMS5837(float *d)
{
	static float g=9.8;
	static float air_pressure=980.0f;   //设置大气压力
	static float water_density=997.0f;   //设置水密度
//	static float factor=0.983615;               //设置重量加速度
	float press_factor=1/(water_density*g);
  MS5837_30BA_GetData();         //获取深度计数据温度和压力，拷贝到全局变量Temperature和Pressure中
	
	*d =(float)((Pressure-air_pressure)*press_factor);    // 把水压转化为深度值
	rawData.depth=(short)(*d*100);               //原始深度乘上100，变为十六位的有符号数，拷贝上传
	rawData.water_temp=(short)(Temperature*10); //原始温度乘上10，强行取整变为十六位的有符号数，拷贝上传 
	sensorData.water_temp=Temperature;
}  

//深度值滤波参数
float filterDepth[10];
float sumDepth;
float w_d;

/* 对深度数据进行滑动平均滤波——取2次求平均*/
void depthFIR(sensorData_t* depth1)
{
	float w_depth;
	float temp;
	u8 count_depth=0;
	u8 count=2;
	while(count--)              //取2次数据求平均
	{
		sensorReadMS5837(&w_depth);
		w_d=w_depth;
		temp = filterDepth[count_depth];
		filterDepth[count_depth] = w_depth;
		sumDepth += filterDepth[count_depth] - temp;
		depth1->depth = sumDepth/2.0f;
		count_depth++;
		delay_xms(20);      //深度传感器读取周期为20ms
	}
}







