#include <stdbool.h>
#include "attitude_pid.h"
#include "pid.h"
#include "control.h"

//定义结构体
PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

//定义初始化角度环PID参数（可修改）
 pid_Para pidrollAngle_para={0,0,0};
 pid_Para pidpitchAngle_para={0,0,0};
 pid_Para pidyawAngle_para={0,0,0};

//定义初始化角度环PID参数（可修改）
 pid_Para pidrollrateAngle_para={0,0,0};
 pid_Para pidpitchrateAngle_para={0,0,0};
 pid_Para pidyawrateAngle_para={0,0,0};
 
 void pidSetIntegralLimit(PidObject* pid, const float limit);      
 void pidSetOutLimit(PidObject* pid, const float maxoutput);
 float pidUpdate(PidObject* pid, const float error);

//设置PID输出限制，不超出16位表示范围
static int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

//姿态控制PID初始化，确定角度和角速度环的PID参数和积分及输出限制
void attitudeControlInit(void)
{
	pidInit(&pidAngleRoll, 0, &pidrollAngle_para, ATTITUDE_UPDATE_DT);      /*roll  角度PID初始化*/
	pidInit(&pidAnglePitch, 0, &pidpitchAngle_para, ATTITUDE_UPDATE_DT);    /*pitch 角度PID初始化*/
//	pidInit(&pidAngleYaw, 0, &pidyawAngle_para, ATTITUDE_UPDATE_DT);	    /*yaw   角度PID初始化*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);   /*roll  角度积分限幅设置*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT); /*pitch 角度积分限幅设置*/
//	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);	    /*yaw   角度积分限幅设置*/
	pidSetOutLimit(&pidAngleRoll, PID_ANGLE_ROLL_OUTPUT_LIMIT);        /*roll  角度环PID输出限幅设置*/  
	pidSetOutLimit(&pidAnglePitch, PID_ANGLE_PITCH_OUTPUT_LIMIT);      /*pitch 角度环PID输出限幅设置*/
//	pidSetOutLimit(&pidAngleYaw, PID_ANGLE_YAW_OUTPUT_LIMIT);          /*yaw   角度环PID输出限幅设置*/

	pidInit(&pidRateRoll, 0,&pidrollrateAngle_para, ATTITUDE_UPDATE_DT);	/*roll  角速度PID初始化*/
	pidInit(&pidRatePitch, 0, &pidpitchrateAngle_para, ATTITUDE_UPDATE_DT); /*pitch 角速度PID初始化*/
//	pidInit(&pidRateYaw, 0, &pidyawrateAngle_para, ATTITUDE_UPDATE_DT);	    /*yaw   角速度PID初始化*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);	    /*roll  角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);   /*pitch 角速度积分限幅设置*/
//	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);	    /*yaw   角速度积分限幅设置*/
	pidSetOutLimit(&pidRateRoll, PID_RATE_ROLL_OUTPUT_LIMIT);          /*roll  角速度环PID输出限幅设置*/ 
	pidSetOutLimit(&pidRatePitch, PID_RATE_PITCH_OUTPUT_LIMIT);        /*pitch  角速度环PID输出限幅设置*/ 
//	pidSetOutLimit(&pidRateYaw, PID_RATE_YAW_OUTPUT_LIMIT);            /*yaw  角速度环PID输出限幅设置*/ 
}

/************************************************************************************************/
//角度环PID函数
//输入：实际角度（来自陀螺仪测量值）和期望角度（来自遥控器转换值）
//输出：期望角速度
/************************************************************************************************/
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle, attitude_t *outDesiredRate)	
{
//	float yawError = desiredAngle->yaw - actualAngle->yaw; /*偏航角误差*/ 
//	if (yawError > 180.0f)                                 /*偏航角误差限制在±180°*/ 
//		yawError -= 360.0f;
//	else if (yawError < -180.0) 
//		yawError += 360.0f;
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->pitch);    /*roll角度环PID输出期望角速度*/ 
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->roll);/*pitch角度环PID输出期望角速度*/ 
//	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);                                    /*yaw角度环PID输出期望角速度*/ 
}

/************************************************************************************************/
//角速度环PID函数
//输入：实际角速度指向陀螺结构体变量（来自陀螺仪直接输出值）、期望角速度（来自角度环输出）
//输出：控制量
/************************************************************************************************/
void attitudeRatePID(attitude_t *actualRate,attitude_t *desiredRate,control_t *output)	/* 角速度环PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->pitch));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->roll));
//	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->yaw));
}





