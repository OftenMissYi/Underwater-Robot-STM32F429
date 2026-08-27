#ifndef __AUTOCONTROL_H
#define __AUTOCONTROL_H

#include "sys.h" 

struct Remote_Speed
{
	int remote_speed_X;
	int remote_speed_Y;
	int remote_speed_Z;
	int remote_speed_YAW;
};

/*角速度环积分限幅(单位：m/s)*/
#define PID_AUTOROLLRATE_INTEGRATION_LIMIT    30.0
#define PID_AUTOPITCHRATE_INTEGRATION_LIMIT   30.0


/*角速度环输出限幅(单位：油门值)*/
#define PID_AUTOROLLRATE_OUTPUT_LIMIT         100.0
#define PID_AUTOPITCHRATE_OUTPUT_LIMIT        100.0

extern struct Remote_Speed remote_speed;

void Pid_Init(void);
int PWM_Calc_limit(int PWM_Calc);
int PWM_Down_limit(int PWM);
int PWM_Up_limit(int PWM);
float Angle_Plus(float first_angle, float second_angle);
float Angle_Reduce(float first_angle, float second_angle);
void auto_control_task(void);

#endif
