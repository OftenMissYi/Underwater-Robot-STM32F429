#include "auto_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "auto_pid.h"
#include "control.h"
#include "attitude_pid.h"
#include "pid.h"
#include "thruster.h"
#include "delay.h"

struct Remote_Speed remote_speed;
extern u8 remoteliink_flag; 
extern u16 command[11];

int Motor_PWM[8] = {1500,1500,1500,1500,1500,1500,1500,1500};  //8个电机的PWM输出
int Motor_PWM_Calc[4] = {0,0,0,0}; //ROW,PITCH平衡增量
float PID_Error[6];  //当前角度差
float PID_Max_Error[6]; //变量x最大
float PID_Error_Speed[6];  //角度对应的角速度
int PWM_Calc[6];  //PID增量
	
pid_Para roll_pid = {1,0,0};
pid_Para pitch_pid = {1,0,0};
pid_Para yaw_pid = {1,0,0};
pid_Para deep_pid = {1,0,0};
pid_Para x_pid = {1,0,0}; 
pid_Para y_pid = {1,0,0}; 
PidObject ROLL_,PITCH_,YAW_,DEEP_,X_,Y_;

state_t need_state; //期望姿态
state_t first_state; //初始姿态
extern state_t state; //当前姿态
extern sensorData_t sensorData;     

void Pid_Init(void)  //PID初始化
{
	pidInit(&ROLL_, 0, &roll_pid, ATTITUDE_UPDATE_DT);
	pidInit(&PITCH_, 0, &pitch_pid, ATTITUDE_UPDATE_DT);
	pidInit(&YAW_, 0, &yaw_pid, ATTITUDE_UPDATE_DT);
	pidInit(&DEEP_, 0, &deep_pid, ATTITUDE_UPDATE_DT);
	pidInit(&X_, 0, &x_pid, ATTITUDE_UPDATE_DT);
	pidInit(&Y_, 0, &y_pid, ATTITUDE_UPDATE_DT);
	pidSetIntegralLimit(&ROLL_,PID_AUTOROLLRATE_INTEGRATION_LIMIT);
	pidSetIntegralLimit(&PITCH_,PID_AUTOPITCHRATE_INTEGRATION_LIMIT);
	pidSetOutLimit(&ROLL_, PID_AUTOROLLRATE_INTEGRATION_LIMIT);
	pidSetOutLimit(&PITCH_, PID_AUTOPITCHRATE_INTEGRATION_LIMIT);
}

int PWM_Calc_limit(int PWM_Calc) //垂直推进器角度调节PID增量限幅（-100，100）
{
    if (PWM_Calc > 100)
    {
        PWM_Calc = 100;
    }
    else if (PWM_Calc < -100)
    {
        PWM_Calc = -100;
    }
    return PWM_Calc;
}

int PWM_Down_limit(int PWM) //垂直推进器PWM限幅
{
    if (	PWM > 2000)
    {
        PWM = 2000;
    }
    else if (PWM <1000)
    {
        PWM =1000;
    }
	else if (PWM < 1550 && PWM > 1450)
    {
        PWM =1500;
    }
    return PWM;
}

int PWM_Up_limit(int PWM) //水平推进器PWM限幅
{
    if (	PWM > 1900)
    {
        PWM = 1900;
    }
    else if (PWM < 1100)
    {
        PWM =1100;
    }
	else if (PWM < 1550 && PWM > 1450)
    {
        PWM =1500;
    }
    return PWM;
}

float Angle_Plus(float first_angle, float second_angle)
{
	float num = first_angle + second_angle;
	if(num > 180)
		return num - 360;
	else if(num < -180)
		return num + 360;
	else return num;
}
	
float Angle_Reduce(float first_angle, float second_angle)
{
	float num = first_angle - second_angle;
	if(num > 180)
		return num - 360;
	else if(num < -180)
		return num + 360;
	else return num;
}

#if 0
void auto_control_task(void) //自动调节任务
{
	int i=0;
	HThurster_Init(5000-1,89);               /*水平推进器初始化（定时器4），频率为200Hz*/       
	VThurster_Init(20000-1,89);              /*垂直推进器初始化（定时器5），频率为50Hz*/ 
	TIM_SetTIM4Compare(1500,1500,1500,1500); /*给中值激活水平推进器*/
	TIM_SetTIM5Compare(1500,1500,1500,1500); /*给中值激活垂直推进器*/
	Pid_Init();
	delay_us(200);
	
	while(1)
	{
		imuUpdate(sensorData.acc, sensorData.gyro,state.realAngle, ATTITUDE_UPDATE_DT);//将九轴的数据融合,校准三轴角度值
		
		//陀螺仪XY轴反了
		PID_Error[0] = 0 - state.realAngle.pitch; //计算偏移角度差，期望值pitch和roll角为0
		PID_Error[1] = 0 - state.realAngle.roll; 
		PID_Error_Speed[0] = angel_speed(PID_Error[0], 10, 0);
		PID_Error_Speed[1] = angel_speed(PID_Error[1], 10, 1); //算出对应角速度期望值
		
		PWM_Calc[0] = pidUpdate(&ROLL_,PID_Error_Speed[0] - state.realRate.pitch); //偏移角速度差算出PWM增量
		PWM_Calc[1] = pidUpdate(&PITCH_,PID_Error_Speed[1] - state.realRate.roll); 
		
//		PID_Error[2] = need_state.realAngle.yaw - state.realAngle.yaw;
//		PID_Max_Error[2] = 10*(int)(PID_Error[2]/10 + 1); //10为一个单位，0-10，MAX为10,10-20，MAX为20
//		PID_Error_Speed[2] = angel_speed(PID_Error[2], PID_Max_Error[2], 2);
//		PWM_Calc[2] = PI_Calc(&YAW_, PID_Error_Speed[2] - state.realRate.yaw);
		
//		PID_Error[3] = need_state.realDepth - state.realDepth;
//		if(PID_Error[3] > 300)
//			PID_Max_Error[3] = 100*(int)(PID_Error[3]/1000 + 10);
//		if(PID_Error[3] <= 300)
//			PID_Max_Error[3] = 10*(int)(PID_Error[3]/1000 + 10); 
//		PID_Error_Speed[3] = angel_speed(PID_Error[3], PID_Max_Error[3], 3);
//		PWM_Calc[3] = PI_Calc(&DEEP_, PID_Error_Speed[3] - state.realvelocity.z); 

//		//前后左右平移
//		PID_Error[4] = need_state.position.x;
//		PID_Error[5] = need_state.position.y;
//		PID_Max_Error[4] = 100*(int)(PID_Error[4]/1000 + 10); //10M为一个单位，0-10，MAX为10,10-20，MAX为20
//		PID_Max_Error[5] = 100*(int)(PID_Error[5]/1000 + 10);
//		PID_Error_Speed[4] = angel_speed(PID_Error[4], PID_Max_Error[4], 4);
//		PID_Error_Speed[5] = angel_speed(PID_Error[5], PID_Max_Error[5], 5);
//		PWM_Calc[4] = PI_Calc(&X_, PID_Error_Speed[4] - state.realvelocity.x);
//		PWM_Calc[5] = PI_Calc(&Y_, PID_Error_Speed[5] - state.realvelocity.y);
		
		if(remoteliink_flag!=1)   /*遥控器未开启或一直没接收到遥控器数据*/
		{
			TIM_SetTIM5Compare(1500,1500,1500,1500);
			TIM_SetTIM4Compare(1500,1500,1500,1500);
		}
		else
		{
			if(command[EXCHANGE_MODE]==TH_MODE)         //推进器模式
			{
				/*垂直推进器调节*/

				/*PID结果必须累加，直接遥控器竖直PWM加上PID结果的值并不定能保持平衡*/
				/*例如左边电机需要1800才能和右边电机1600输出平衡，现在遥控器输出1700，角度倾斜*/
				/*计算得出100，变成1800,1600，使其回正，此时又计算，为结果为0，输出遥控器数据，又产生倾斜*/

				/*PID增量累加，限幅100，防止全变成正1000之类的情况（也是平衡的）*/
				Motor_PWM_Calc[0] = PWM_Calc_limit(Motor_PWM_Calc[0] - PWM_Calc[0] - PWM_Calc[1]); 
				Motor_PWM_Calc[1] = PWM_Calc_limit(Motor_PWM_Calc[1] - PWM_Calc[0] + PWM_Calc[1]);
				Motor_PWM_Calc[2] = PWM_Calc_limit(Motor_PWM_Calc[2] + PWM_Calc[0] - PWM_Calc[1]);
				Motor_PWM_Calc[3] = PWM_Calc_limit(Motor_PWM_Calc[3] + PWM_Calc[0] + PWM_Calc[1]);
				
				if(-50<remote_speed.remote_speed_Z && remote_speed.remote_speed_Z< 50) //控制竖直遥控器摇杆输出为1450-1550，停机
				{
					for(i = 0; i < 4; i++)
					{
						Motor_PWM_Calc[i] = 0; //将PID增量累计清零
					}
				}
				
				/*遥控器竖直PWM加上PID增量保持平衡*/
				Motor_PWM[0] = PWM_Down_limit(1500 - remote_speed.remote_speed_Z + Motor_PWM_Calc[0]); 
				Motor_PWM[1] = PWM_Down_limit(1500 + remote_speed.remote_speed_Z + Motor_PWM_Calc[1]); 
				Motor_PWM[2] = PWM_Down_limit(1500 + remote_speed.remote_speed_Z + Motor_PWM_Calc[2]); 
				Motor_PWM[3] = PWM_Down_limit(1500 - remote_speed.remote_speed_Z + Motor_PWM_Calc[3]); 
				
				TIM_SetTIM5Compare(Motor_PWM[0],Motor_PWM[1],Motor_PWM[2],Motor_PWM[3]); //PWM输出
			
				/*水平推进器调节*/
				
				/*电机静止PWM1500，加上遥控器X,Y,YAW三个方向的控制PWM*/
				Motor_PWM[4] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y); 
				Motor_PWM[5] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW + remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
				Motor_PWM[6] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X + remote_speed.remote_speed_Y);
				Motor_PWM[7] = PWM_Up_limit(1500 + remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
				
				TIM_SetTIM4Compare(Motor_PWM[4],Motor_PWM[5],Motor_PWM[6],Motor_PWM[7]); //PWM输出
			}
			else //使用机械臂时，沉底不动，水平面无法移动
			{
				TIM_SetTIM4Compare(1500,1500,1500,1500);
			}
		}
				
		//深度调节
//		if(-5 < need_state.realDepth - state.realDepth < 5) //到达目的地（期望与目前深度差小于5cm），将起始深度改为当前深度，准备下次计算
//		{
//			first_state.realDepth = state.realDepth;
//			PID_Max_Error[3] = 5;
//		}
//		else PID_Max_Error[3] = need_state.realDepth - first_state.realDepth;
//		PID_Error[3] = need_state.realDepth - state.realDepth;
//		PID_Error_Speed[3] = angel_speed(PID_Error[3], PID_Max_Error[3], 3);
//		PWM_Calc[3] = PI_Calc(&DEEP_, PID_Error_Speed[3] - state.realvelocity.z); 
	
		//角度调节
//		if(-1 < Angle_Reduce(need_state.realAngle.yaw, state.realAngle.yaw) < 1) //到达目的地（期望与目前角度差小于1度），将起始角度改为当前角度，准备下次计算
//		{
//			first_state.realAngle.yaw = state.realAngle.yaw;
//			PID_Max_Error[3] = 5;
//		}
//		else PID_Max_Error[3] = Angle_Reduce(need_state.realAngle.yaw, first_state.realAngle.yaw);
//		PID_Error[2] = Angle_Reduce(need_state.realAngle.yaw, state.realAngle.yaw);
//		PID_Error_Speed[2] = angel_speed(PID_Error[2], PID_Max_Error[2], 2);
//		PWM_Calc[2] = PI_Calc(&YAW_, PID_Error_Speed[2] - state.realRate.yaw);

		delay_ms(20);
	}
}
#endif

void auto_control_task(void) //调节任务
{
	int i = 0;
	HThurster_Init(5000-1,89);               /*水平推进器初始化（定时器4），频率为200Hz*/        
	VThurster_Init(20000-1,89);               /*垂直推进器初始化（定时器5），频率为50Hz*/ 
	TIM_SetTIM4Compare(1500,1500,1500,1500); /*给中值激活水平推进器*/
	TIM_SetTIM5Compare(1500,1500,1500,1500); /*给中值激活垂直推进器*/
	Pid_Init();
	delay_us(200);
	
	while(1)
	{
		//利用三轴加速度对三轴角度进行校准
		imuUpdate(sensorData.acc, sensorData.gyro, &state.realAngle, ATTITUDE_UPDATE_DT);
		
		//陀螺仪XY轴反了
		PID_Error[0] = 0 - state.realAngle.pitch; //计算偏移角度差，期望值pitch和roll角为0
		PID_Error[1] = 0 - state.realAngle.roll; 
		PWM_Calc[0] = PI_Calc(&ROLL_,PID_Error[0]); //偏移角度差算出PWM增量
		PWM_Calc[1] = PI_Calc(&PITCH_, PID_Error[1]);  
		if(remoteliink_flag!=1)   /*遥控器未开启或一直没接收到遥控器数据*/
		{
			TIM_SetTIM5Compare(1500,1500,1500,1500);
			TIM_SetTIM4Compare(1500,1500,1500,1500);
		}
		else
		{
			if(command[EXCHANGE_MODE]==TH_MODE)         //推进器模式
			{
				/*垂直推进器调节*/

				/*PID结果必须累加，直接遥控器竖直PWM加上PID结果的值并不定能保持平衡*/
				/*例如左边电机需要1800才能和右边电机1600输出平衡，现在遥控器输出1700，角度倾斜*/
				/*计算得出100，变成1800,1600，使其回正，此时又计算，为结果为0，输出遥控器数据，又产生倾斜*/

				/*PID增量累加，限幅100，防止全变成正1000之类的情况（也是平衡的）*/
				Motor_PWM_Calc[0] = PWM_Calc_limit(Motor_PWM_Calc[0] - PWM_Calc[0] - PWM_Calc[1]); 
				Motor_PWM_Calc[1] = PWM_Calc_limit(Motor_PWM_Calc[1] - PWM_Calc[0] + PWM_Calc[1]);
				Motor_PWM_Calc[2] = PWM_Calc_limit(Motor_PWM_Calc[2] + PWM_Calc[0] - PWM_Calc[1]);
				Motor_PWM_Calc[3] = PWM_Calc_limit(Motor_PWM_Calc[3] + PWM_Calc[0] + PWM_Calc[1]);
				
				if(-50<remote_speed.remote_speed_Z && remote_speed.remote_speed_Z< 50) //控制竖直遥控器摇杆输出为1450-1550，停机
				{
					for(i = 0; i < 4; i++)
					{
						Motor_PWM_Calc[i] = 0; //将PID增量累计清零
					}
				}
				
				/*遥控器竖直PWM加上PID增量保持平衡*/
				Motor_PWM[0] = PWM_Down_limit(1500 - remote_speed.remote_speed_Z + Motor_PWM_Calc[0]); 
				Motor_PWM[1] = PWM_Down_limit(1500 + remote_speed.remote_speed_Z + Motor_PWM_Calc[1]); 
				Motor_PWM[2] = PWM_Down_limit(1500 + remote_speed.remote_speed_Z + Motor_PWM_Calc[2]); 
				Motor_PWM[3] = PWM_Down_limit(1500 - remote_speed.remote_speed_Z + Motor_PWM_Calc[3]); 
				
				TIM_SetTIM5Compare(Motor_PWM[0],Motor_PWM[1],Motor_PWM[2],Motor_PWM[3]); //PWM输出
			
				/*水平推进器调节*/
				
				/*电机静止PWM1500，加上遥控器X,Y,YAW三个方向的控制PWM*/
				Motor_PWM[4] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y); 
				Motor_PWM[5] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW + remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
				Motor_PWM[6] = PWM_Up_limit(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X + remote_speed.remote_speed_Y);
				Motor_PWM[7] = PWM_Up_limit(1500 + remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
				
				TIM_SetTIM4Compare(Motor_PWM[4],Motor_PWM[5],Motor_PWM[6],Motor_PWM[7]); //PWM输出
			}
			else   //使用机械臂时，沉底不动,水平面不移动
			{
				TIM_SetTIM4Compare(1500,1500,1500,1500); //水平推进器停转
			}
		}
		delay_ms(20);
	}
}




