#include "sensor_filter.h"
#include "sys.h"

sensor_filter_DATA sensor_filter_data;

/*	最近十次数据求平均，启动前九次数据误差较大	*/
float sensor_filter(float data,int count,float storage_array[])
{
	int i = 0;
	float sum_data;
	storage_array[count] = data;
	while(i < 10)
	{
		sum_data += storage_array[i];
	}
	return (sum_data/10.0f);
}

/*	陀螺仪角度滤波器，启动前九次数据误差较大	*/
void angle_filter(float data_x,float data_y,float data_z)
{
	/*	陀螺仪角度滤波计数器（0~9），累加	*/
	static int angle_filter_count = 0;
	if(angle_filter_count == 9)	angle_filter_count = 0;
	
	sensor_filter_data.angle_filter_d[0] = sensor_filter(data_x,angle_filter_count,sensor_filter_data.angle_storage_array[0]);
	sensor_filter_data.angle_filter_d[1] = sensor_filter(data_y,angle_filter_count,sensor_filter_data.angle_storage_array[1]);
	sensor_filter_data.angle_filter_d[2] = sensor_filter(data_z,angle_filter_count,sensor_filter_data.angle_storage_array[2]);
	
	angle_filter_count++;
}

/*	陀螺仪角速度滤波器，启动前九次数据误差较大	*/
void gyro_filter(float data_x,float data_y,float data_z)
{
	/*	陀螺仪角速度滤波计数器（0~9），累加	*/
	static int gyro_filter_count = 0;
	if(gyro_filter_count == 9)	gyro_filter_count = 0;
	
	sensor_filter_data.gyro_filter_d[0] = sensor_filter(data_x,gyro_filter_count,sensor_filter_data.gyro_storage_array[0]);
	sensor_filter_data.gyro_filter_d[1] = sensor_filter(data_y,gyro_filter_count,sensor_filter_data.gyro_storage_array[1]);
	sensor_filter_data.gyro_filter_d[2] = sensor_filter(data_z,gyro_filter_count,sensor_filter_data.gyro_storage_array[2]);
	
	gyro_filter_count++;
}
	
/*	陀螺仪角加速度滤波器，启动前九次数据误差较大	*/
void acc_filter(float data_x,float data_y,float data_z)
{
	/*	陀螺仪角加速度滤波计数器（0~9），累加	*/
	static int acc_filter_count = 0;
	if(acc_filter_count == 9)	acc_filter_count = 0;
	
	sensor_filter_data.acc_filter_d[0] = sensor_filter(data_x,acc_filter_count,sensor_filter_data.acc_storage_array[0]);
	sensor_filter_data.acc_filter_d[1] = sensor_filter(data_y,acc_filter_count,sensor_filter_data.acc_storage_array[1]);
	sensor_filter_data.acc_filter_d[2] = sensor_filter(data_z,acc_filter_count,sensor_filter_data.acc_storage_array[2]);
	
	acc_filter_count++;
}

/*	陀螺仪磁场滤波器，启动前九次数据误差较大	*/
void mag_filter(float data_x,float data_y,float data_z)
{
	/*	陀螺仪角磁场滤波计数器（0~9），累加	*/
	static int mag_filter_count = 0;
	if(mag_filter_count == 9)	mag_filter_count = 0;
	
	sensor_filter_data.mag_filter_d[0] = sensor_filter(data_x,mag_filter_count,sensor_filter_data.mag_storage_array[0]);
	sensor_filter_data.mag_filter_d[1] = sensor_filter(data_y,mag_filter_count,sensor_filter_data.mag_storage_array[1]);
	sensor_filter_data.mag_filter_d[2] = sensor_filter(data_z,mag_filter_count,sensor_filter_data.mag_storage_array[2]);
	
	mag_filter_count++;
}

/*	陀螺仪滤波器，启动前九次数据误差较大，按需更改	*/
void sensorFIR(void)
{
//	angle_filter(float data_x,float data_y,float data_z);
//	gyro_filter(float data_x,float data_y,float data_z);
//	acc_filter(float data_x,float data_y,float data_z);
//	mag_filter(float data_x,float data_y,float data_z);
}

	


