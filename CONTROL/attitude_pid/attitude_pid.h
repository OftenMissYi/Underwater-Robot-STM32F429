#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H
#include <stdbool.h>
#include "sys.h"
#include "control.h"

#define ATTITUDE_UPDATE_RATE 	500
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)        //PID更新时间

/*角度环PID积分限幅(单位：deg)*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT   200.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT  200.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT    200.0

/*角速度环PID积分限幅(单位：deg/s)*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT	   200.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT   200.0
#define PID_RATE_YAW_INTEGRATION_LIMIT	   200.0

/*角度环PID输出限幅(单位：deg/s)*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT        500.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT       500.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT         500.0

/*角速度环PID输出限幅(单位：油门值)*/
#define PID_RATE_ROLL_OUTPUT_LIMIT		   500.0
#define PID_RATE_PITCH_OUTPUT_LIMIT	       500.0
#define PID_RATE_YAW_OUTPUT_LIMIT	       500.0

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

//定义初始化角度环PID参数（可修改）
extern pid_Para pidrollAngle_para;
extern pid_Para pidpitchAngle_para;
extern pid_Para pidyawAngle_para;

//定义初始化角度环PID参数（可修改）
extern pid_Para pidrollrateAngle_para;
extern pid_Para pidpitchrateAngle_para;
extern pid_Para pidyawrateAngle_para;
 
static int16_t pidOutLimit(float in);
void attitudeControlInit(void);
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);
void attitudeRatePID(attitude_t *actualRate,attitude_t *desiredRate,control_t *output);
#endif
