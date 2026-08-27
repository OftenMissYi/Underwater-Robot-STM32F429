#ifndef __DEPTH_PID_H
#define __DEPTH_PID_H
#include "sys.h"
#include "pid.h"
#include "control.h"

#define POS_UPDATE_RATE 		200
#define POS_UPDATE_DT 			(1.0f / POS_UPDATE_RATE)
#define PID_DEPTH_INTEGRATION_LIMIT 100.0
#define PID_DEPTH_OUTPUT_LIMIT 500.0

typedef struct  
{
	PidObject pidX;
	PidObject pidY;
	PidObject pidZ;
	float thrustBase; 			// 定深时的油门基准值，这个值可以让水下机器人悬停
	bool preMode;				// 前一次的模式，为true时对应定深模式
	bool isAltHoldMode;         // 为true时对应定深模式
}depthPid_t;


extern pid_Para depthpid_para;

float limitThrustBase(float input);
void depthControlInit(void);
void depthPID(float *actualDepth, float *desiredDepth, control_t *output);

#endif


