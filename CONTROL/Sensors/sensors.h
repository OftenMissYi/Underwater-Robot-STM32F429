#ifndef __SENSORS_H
#define __SENSORS_H

#pragma anon_unions          //添加后匿名结构体才不会报错
//#include "control.h"
#include "sys.h"



typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

typedef union 
{
	struct 
	{
		short x;
		short y;
		short z;
	};
	float axis[3];
} Axis3s;

typedef struct  
{

	float roll;
	float pitch;
	float yaw;
}att_t;

typedef struct  
{

	short roll;
	short pitch;
	short yaw;
}att_ts;

typedef struct  
{
	u8 low;
	u8 high;
}dht_t;

typedef struct  
{
	short acc_x;
	short acc_y;
	short acc_z;
	short ro11;
	short pitch;
	short ro11_w;
	short pitch_w;
	short yaw_w;
}Bais_t;

typedef struct
{
	Axis3f acc;           //加速度计
	att_t angle;          //陀螺仪角度计
	att_t gyro;           //陀螺仪角速度计
	Axis3f mag;           //地磁计
	float depth;          //深度计深度值
	float water_temp;     //深度计水温值
	float cabin_press;    //电子舱压力值
	float cabin_temp;     //电子舱温度值
	float cabin_humi;     //电子舱湿度值
} sensorData_t;

//short类型结构体
typedef struct
{
	Axis3s acc;           //加速度计
	att_ts angle;         //陀螺仪角度计
	att_ts gyro;          //陀螺仪角速度计
	Axis3s mag;           //地磁计
	short  depth;         //深度计深度值
	short  water_temp;    //深度计水温值
	u32  cabin_press;   //电子舱压力值
	u16  cabin_temp;     //电子舱温度值
	u16  cabin_humi;    //电子舱湿度值
	Bais_t HWT905_bais;
} rawData_t;

typedef struct
{
	Axis3f acc;           //校准加速度计
	att_t angle;         //校准陀螺仪角度计
	att_t gyro;          //校准陀螺仪角速度计
}SensorClibData_t;

extern rawData_t rawData;

void sensorsInit(void);
void sensors_task(void *pvParameters);
void processBarometerMeasurements(const u8 *buffer);

/*读取深度传感器数据*/
void sensorReadMS5837(float *d);
void depthFIR(sensorData_t* depth1);

#endif
