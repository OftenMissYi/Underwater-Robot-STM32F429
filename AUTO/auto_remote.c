#include "auto_remote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "link232.h"
#include "control.h"
#include "delay.h"
#include "auto_control.h"

extern state_t need_state; //期望姿态
extern u16 command[11]; //遥控器的值
extern struct Remote_Speed remote_speed;
#if 0
void auto_remote_task(void) //遥控器改变期望值
{
	while(1)
	{
		need_state.position.x = command[SURGE] - 1500;
		need_state.position.y = command[SWAY] - 1500;
		need_state.realDepth = command[HEAVE] - 1500;
		need_state.realAngle.yaw = Angle_Plus(need_state.realAngle.yaw, ((float)(command[YAW] - 1500))/500); //100ms一度
		delay_ms(100);
	}
}
#endif
void auto_remote_task(void) //遥控器
{
	while(1)
	{
		remote_speed.remote_speed_X = command[SURGE] - 1500;		//遥控器控制X方向PWM-400-400
		remote_speed.remote_speed_Y = command[SWAY] - 1500;			//遥控器控制Y方向PWM-400-400
		remote_speed.remote_speed_Z = command[HEAVE] - 1500;		//遥控器控制竖直方向PWM-500-500
		remote_speed.remote_speed_YAW = command[YAW] - 1500;	    //遥控器控制YAW角度PWM-400-400
		delay_ms(20);
	}
}
