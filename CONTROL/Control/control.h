#ifndef __CONTROL_H
#define __CONTROL_H
#include "pid.h"
#include "sys.h"
#include "sensors.h"
#include "ibus.h"



//垂直推进器编号
#define THRUSTER_V1  		1
#define THRUSTER_V2  		2
#define THRUSTER_V3  		3
#define THRUSTER_V4  		4


#define ft  (0.05f)

extern sensorData_t sensorData;

typedef struct 
{

	float x;
	float y;
	float z;
}vec3_s;


extern vec3_s positon ;
extern vec3_s velocity;
extern vec3_s acc;


/*姿态结构体变量*/
typedef struct  
{

	float roll;
	float pitch;
	float yaw;
}attitude_t;

/*状态结构体变量*/
typedef struct
	{
	attitude_t realAngle;//角度;
	attitude_t realRate;//角加速度
	vec3_s position;
	vec3_s realvelocity;//线速度
	vec3_s acc;//线加速度
	
	float realDepth;//深度
	float preDepth;
} state_t;

/*期望设置状态结构体变量*/
typedef struct
{
	attitude_t expectedAngle;
	vec3_s position;
	vec3_s velocity;
	attitude_t expectedRate;
	float expectedDepth;
	float thrust;
	
} setstate_t;

/*控制结构体变量*/
typedef struct
	{
		int roll;
		int pitch;
		int yaw;
		float thrust;  //深度控制油门值
		float forward_thrust;  //前后方向PID速度调节量
		float right_thrust;    //横移方向PID速度调节量
		float turn_thrust;     //偏航方向PID角速度调节量
		
		float forward_thrustbase;  //前后方向油门基值
		float right_thrustbase;    //横移方向油门基值
		float turn_thrustbase;     //偏航方向油门基值
		
		float hightOut;
		float depthOut;
		float ds1;           //添加舵机控制量,未使用
		float ds2;
		int premode;
		int nowmode;
	} control_t;


/*4个垂直推进器*/
typedef struct 
{
	u32 v1;
	u32 v2;
	u32 v3;
	u32 v4;
	u32 h1;
	u32 h2;
	u32 h3;
	u32 h4;
} thrusterPWM_t;

#define HTHRUST_MAX 1900            /*水平推进器油门最大值*/
#define HTHRUST_MIN 1100            /*水平推进器油门最小值*/
#define VTHRUST_MAX 2000            /*垂直推进器油门最大值*/
#define VTHRUST_MIN 1000            /*垂直推进器油门最小值*/

void stabilizer_task(void *pvParameters);
void VthrusterControl(control_t* control);
u16 limitHPWM(int input);
u16 limitVPWM(int input);
float limitVThrust(float value);
float limitHThrust(float value);
float invSqrt(float x);
void imuUpdate(Axis3f acc, att_t angle, attitude_t *attitudeClib, float dt);
u16 Vthrust2pwm(float thrust_value);
u16 Hthrust2pwm(float thrust_value);
void Attitude_Control(control_t *output);
void Motion_Control(control_t *output);
void HthrusterControl(control_t* control);
void integralUpdate(vec3_s* in,  vec3_s* out);
float pwm2Range(int pwm_value, float p_min, float p_max);

float limitHPThrust(float value);
float limitHFThrust(float value);
float ThuraterPWM_Calc_limit(float PWM_Calc);

#endif
