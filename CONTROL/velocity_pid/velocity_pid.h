#ifndef __VELOCITY_PID_H
#define __VELOCITY_PID_H
#include <stdbool.h>
#include "sys.h"
#include "control.h"
#include "pid.h"

#define ATTITUDE_UPDATE_RATE 	500
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)        //PID更新时间

/*速度环积分限幅(单位：m/s)*/
#define PID_VELOCITY_X_INTEGRATION_LIMIT    100.0
#define PID_VELOCITY_Y_INTEGRATION_LIMIT    100.0
#define PID_VELOCITY_Z_INTEGRATION_LIMIT    100.0


/*速度环输出限幅(单位：油门值)*/
#define PID_VELOCITY_X_OUTPUT_LIMIT         500.0
#define PID_VELOCITY_Y_OUTPUT_LIMIT         500.0
#define PID_VELOCITY_Z_OUTPUT_LIMIT         500.0

extern PidObject VXpid;
extern PidObject VYpid;
extern PidObject VZpid;

extern pid_Para VXpid_para;
extern pid_Para VYpid_para;
extern pid_Para VZpid_para;

//typedef struct 
//{

//	float x;
//}velx_t;

//typedef struct 
//{

//	float y;
//}vely_t;

typedef struct 
{

	float z;
}velz_t;


static int16_t pidOutLimit(float in);
void  motionControlInit(void);

void motionXVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control);
void motionYVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control);
void motionZVelocityPID(vec3_s *actualVelocity,vec3_s *desiredVelocity, control_t *control);



#endif
