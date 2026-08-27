#include <stdbool.h>
#include "velocity_pid.h"
#include "pid.h"
#include "control.h"

PidObject VXpid;
PidObject VYpid;
PidObject VZpid;

//定义初始化PID参数（可修改）
 pid_Para VXpid_para={0,0,0};
 pid_Para VYpid_para={0,0,0};
 pid_Para VZpid_para={0,0,0};

 void pidSetIntegralLimit(PidObject* pid, const float limit);       //函数声明
 void pidSetOutLimit(PidObject* pid, const float maxoutput);
 float pidUpdate(PidObject* pid, const float error);

 /*水平运动PID初始化*/
void motionControlInit()
{
//	pidInit(&VXpid, 0, &VXpid_para, ATTITUDE_UPDATE_DT);            /*vxPID初始化*/
//	pidInit(&VYpid, 0, &VYpid_para, ATTITUDE_UPDATE_DT);            /*vyPID初始化*/
	pidInit(&VZpid, 0, &VZpid_para, ATTITUDE_UPDATE_DT);	        /*vzPID初始化*/
//	pidSetIntegralLimit(&VXpid, PID_VELOCITY_X_INTEGRATION_LIMIT);  /*vx积分限幅设置*/
//	pidSetIntegralLimit(&VYpid, PID_VELOCITY_Y_INTEGRATION_LIMIT);  /*vy积分限幅设置*/
	pidSetIntegralLimit(&VZpid, PID_VELOCITY_Z_INTEGRATION_LIMIT);	/*vz积分限幅设置*/
//	pidSetOutLimit(&VXpid, PID_VELOCITY_X_OUTPUT_LIMIT);       /*vx环PID输出限幅设置*/
//	pidSetOutLimit(&VYpid, PID_VELOCITY_Y_OUTPUT_LIMIT);       /*vy环PID输出限幅设置*/
	pidSetOutLimit(&VZpid, PID_VELOCITY_Z_OUTPUT_LIMIT);       /*vz环PID输出限幅设置*/
}

/************************************************************************************************/
/*X平移PID函数*/
void motionXVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control)	/* X移动PID */
{
	float velocityXError = desiredVelocity->x - actualVelocity->x;
	control->forward_thrust = pidUpdate(&VXpid, velocityXError);
}
/************************************************************************************************/
/*Y平移PID函数*/
void motionYVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control)	/* Y移动PID */
{
	float velocityYError = desiredVelocity->y - actualVelocity->y;
	control->right_thrust = pidUpdate(&VYpid, velocityYError);
}
/************************************************************************************************/
/*Z转动PID函数*/
void motionZVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control)	/* Z转动PID */
{
	float velocityZError = desiredVelocity->z - actualVelocity->z;
	control->turn_thrust = pidUpdate(&VZpid, velocityZError);
}





