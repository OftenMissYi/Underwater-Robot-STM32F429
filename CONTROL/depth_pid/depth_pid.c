#include "depth_pid.h"
#include "pid.h"
#include "control.h"

#define THRUST_SCALE	(1.0f)             /*深度PID输出缩放因子，不知道具体用途，保留*/
#define THRUSTBASE_HIGH	(2000.f)          /* 定深模式下的基础油门最大值，可修改*/
#define THRUSTBASE_LOW	(1000.f)
#define DEPTH_THRUSTBASE	(1600.f)          /* 定深模式下的基础油门默认值，可修改*/


depthPid_t depthPid;
pid_Para depthpid_para={0,0,0};

/*基础油门值限制*/
float limitThrustBase(float input)
{
	if(input > THRUSTBASE_HIGH)
		return THRUSTBASE_HIGH;
	else if(input < THRUSTBASE_LOW)
		return THRUSTBASE_LOW;
	else 
		return input;
}

/*深度控制PID初始化*/
void depthControlInit(void)
{
	pidInit(&depthPid.pidZ, 0,&depthpid_para, POS_UPDATE_DT);           /*depthPID初始化*/
	pidSetIntegralLimit(&depthPid.pidZ, PID_DEPTH_INTEGRATION_LIMIT);	/*深度积分限幅设置*/
	pidSetOutLimit(&depthPid.pidZ, PID_DEPTH_OUTPUT_LIMIT);             /*深度环PID输出限幅设置*/
	depthPid.thrustBase = limitThrustBase(DEPTH_THRUSTBASE);		    /*每次初始化重新给定油门基值*/
}

/*深度控制PID*/
void depthPID(float *actualDepth, float *desiredDepth, control_t *output)
{
	float PIDoutThrust;
	float depthError = *desiredDepth - *actualDepth;
	PIDoutThrust = THRUST_SCALE * pidUpdate(&depthPid.pidZ,depthError);
	// thrustBase是正值
	output->thrust = limitVThrust(depthPid.thrustBase+PIDoutThrust);		// z轴向下为正，油门值向上为正
}
