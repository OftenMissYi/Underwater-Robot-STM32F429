#include "pid.h"
#include "control.h"

configParam_t configParam;

//PID初始化函数
void pidInit(PidObject* pid, const float desired, pid_Para* pidPara, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidPara->kp;
	pid->ki = pidPara->ki;
	pid->kd = pidPara->kd;
	pid->iLimit    = DEFAULT_PID_INTEGRATION_LIMIT;        //最大值调用设置积分限制函数可以更改
	pid->iLimitLow = -DEFAULT_PID_INTEGRATION_LIMIT;      //最小值一般默认
	pid->dt        = dt;
}
/*****************************************************************************/
 // PID迭代更新函数
 // 输入：PID结构体和误差
// 输出：output
/*****************************************************************************/
float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;  
	pid->iLimitLow=-pid->iLimit;  

	pid->integ += pid->error * pid->dt;
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < pid->iLimitLow)
	{
		pid->integ = pid->iLimitLow;
	}

	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	output = pid->outP + pid->outI + pid->outD;
	//输出限幅
	if(pid->maxOutput!=0)
	{
		if(output>pid->maxOutput)
			output=pid->maxOutput;
		else if(output<-pid->maxOutput)
			output=-pid->maxOutput;
	}
	pid->prevError = pid->error;
	return output;
}

/*设置PID积分最大值*/  
void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}

/*设置PID积分最小值*/
void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) 
{
    pid->iLimitLow = limitLow;
}

/*设置PID输出最大值*/
void pidSetOutLimit(PidObject* pid, const float maxoutput) 
{
    pid->maxOutput = maxoutput;
}

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}



