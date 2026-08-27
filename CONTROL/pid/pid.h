#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "stdbool.h"
#include "control.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  		5000.0

//定义pid相关的结构体
typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float iLimitLow;    //< integral limit
	float maxOutput;
	float dt;           //< delta-time dt
}PidObject;

typedef struct
{
	float kp;
	float ki;
	float kd;
}pid_Para;

typedef struct 
{
	pid_Para pidrollrateAngle_para;
	pid_Para pidpitchrateAngle_para;
	pid_Para pidyawrateAngle_para;
	pid_Para VZpid_para;
	pid_Para depthpid_para;
}configParam_t;





float pidUpdate(PidObject* pid, const float error);
void pidInit(PidObject* pid, const float desired,  pid_Para* pidPara, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);
void pidSetIntegralLimitLow(PidObject* pid, const float limitLow);
void pidSetOutLimit(PidObject* pid, const float maxoutput);
void pidReset(PidObject* pid);
#endif


