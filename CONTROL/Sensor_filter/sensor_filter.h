#ifndef _SENSOR_FILTER_H
#define _SENSOR_FILTER_H

/*	陀螺仪滑动滤波数据存储结构体	*/
typedef struct
{
	float angle_filter_d[3];
	float gyro_filter_d[3];
	float acc_filter_d[3];
	float mag_filter_d[3];
	float angle_storage_array[3][10];
	float gyro_storage_array[3][10];
	float acc_storage_array[3][10];
	float mag_storage_array[3][10];
} sensor_filter_DATA;

/*	陀螺仪滑动滤波数据存储结构体，设置为全局变量	*/
extern sensor_filter_DATA sensor_filter_data;

void angle_filter(float data_x,float data_y,float data_z);
void gyro_filter(float data_x,float data_y,float data_z);
void acc_filter(float data_x,float data_y,float data_z);
void mag_filter(float data_x,float data_y,float data_z);
void sensorFIR(void);

#endif
