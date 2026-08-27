#include "control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "thruster.h"
#include "stdbool.h"
#include "math.h"
#include "attitude_pid.h"
#include "velocity_pid.h"
#include "depth_pid.h"
#include "pid_flash_init.h"
#include "depth_pid.h"
#include "pid.h"
#include "auto_remote.h"
#include "auto_control.h"

#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */

float Kp = 0.4f;		/*比例增益*/
float Ki = 0.001f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float qq0 = 1.0f;	/*四元数*/
static float qq1 = 0.0f;
static float qq2 = 0.0f;
static float qq3 = 0.0f;	

extern u16 command[11];

/*定义加速度积分变量*/
float integX=0.0f;
float integY=0.0f;
float integZ=0.0f;

vec3_s positon ;
vec3_s velocity;
vec3_s acc;
attitude_t a;  //消除warning

control_t control={0};	/*控制参数结构体*/
setstate_t setstate = {0};
state_t state={0};
sensorData_t sensorData = {0};
extern att_t attt;
extern depthPid_t depthPid;
extern configParam_t configParam;

sensorData_t sensorData;	/*传感器原始数据结构体,作指针引用（sensorData->acc\gyro\mag\position）*/ 
thrusterPWM_t PWMcontrol;

//定义8个全局变量查看推进器值变化
u16 THH1,THH2,THH3,THH4;
u16 THV1,THV2,THV3,THV4;
extern u8 remoteliink_flag; 
extern u8 PID_flag;                            // PID参数接收标志

float Thurater_PWM_Calc[4]={0,0,0,0};

/*姿态控制任务函数*/
/*******************************************************************************************************************************************************************/
void stabilizer_task(void *pvParameters)
{
	HThurster_Init(5000-1,89);               /*水平推进器初始化，频率为200Hz*/       
	VThurster_Init(20000-1,89);              /*垂直推进器初始化，频率为50Hz*/ 
	TIM_SetTIM4Compare(1500,1500,1500,1500); /*给中值激活水平推进器*/
	TIM_SetTIM5Compare(1500,1500,1500,1500); /*给中值激活垂直推进器*/
	
	Flash_Read_PID();  //读取闪存中PID参数
	
	//更新角速度环PID参数
	pidrollrateAngle_para.kp=pid_rs232_para[0];pidrollrateAngle_para.ki=pid_rs232_para[1];pidrollrateAngle_para.kd =pid_rs232_para[2];
	pidpitchrateAngle_para.kp=pid_rs232_para[3];pidpitchrateAngle_para.ki=pid_rs232_para[4];pidpitchrateAngle_para.kd =pid_rs232_para[5];
//	pidyawrateAngle_para.kp=pid_rs232_para[6];pidyawrateAngle_para.ki=pid_rs232_para[7];pidyawrateAngle_para.kd=pid_rs232_para[8];
				
	//更新Z轴运动环PID参数(Z轴角速度控制)
	VZpid_para.kp=pid_rs232_para[9];VZpid_para.ki =pid_rs232_para[10];VZpid_para.kd=pid_rs232_para[11];

	//更新Z轴定深PID参数
	depthpid_para.kp=pid_rs232_para[12];depthpid_para.ki=pid_rs232_para[13];depthpid_para.kd=pid_rs232_para[14];
	
	attitudeControlInit();  /*配置默认姿态PID参数(roll和pitch)*/ 
	motionControlInit();    /*配置默认平面运动PID参数(x,y,yaw)*/
	depthControlInit();     /*配置默认深度PID参数(z)*/
	delay_us(200);
	
	while(1)
	{   
		if(remoteliink_flag!=1)   /*遥控器未开启或一直没接收到遥控器数据*/
		{
			TIM_SetTIM5Compare(1500,1500,1500,1500);
			TIM_SetTIM4Compare(1500,1500,1500,1500);
		}
		else
		{
			/*垂直面运动控制*/ 		
			Attitude_Control(&control);    /*调用姿态和深度控制函数*/ 
			VthrusterControl(&control);    /*将控制量转化为PWM输出*/ 
		
			/*水平面运动控制*/ 
			Motion_Control(&control);      /*调用平面运动控制函数*/ 
			HthrusterControl(&control);    /*将控制量转化为PWM输出*/ 
		}
		delay_ms(5); 
	}
}

void Attitude_Control(control_t *output)       
{
    float z_speed;         /*定义Z轴上下速度*/
//	imuUpdate(sensorData.acc, sensorData.gyro,state.realAngle, ATTITUDE_UPDATE_DT); /*调用数据融合函数，处理加速度计和陀螺仪数据，处理后的角度存在结构体指针state中*/
	  z_speed = pwm2Range(command[HEAVE], -100.0f, 100.0f);                         /*将摇杆3油门转化为Z轴速度,保留给定深模式使用*/
    if (z_speed < 10.0f && z_speed > -10.0f)  /*定义Z轴速度死区 */   
        z_speed = 0.0f; 
/**************************************** 手动模式下控制 **********************************************/
  	if (command[MODE] == HAND_MODE)
    {  
	    	control.nowmode=HAND_MODE;                          /*当前控制模式为手动模式*/
		// 手动模式下油门直接作用垂直4推进器
        control.thrust = (float)command[HEAVE];             /*保存油门值*/
	    	if (control.thrust < 1540 && control.thrust > 1460) /*油门死区,防止摇杆信号干扰，该区间内推进器停转*/
	    	{ 
		     	control.thrust = 1500;    
	    	}
	    	if((int)control.thrust == 1500)           /*油门摇杆居中，停止竖直方向运动*/        
		   {
			    pidReset(&depthPid.pidZ);          //PID误差参数清零
		   }			
		   else  //油门输出
		   {
		     	setstate.expectedDepth = state.realDepth;        /*手动模式下期望深度始终等于当前深度*/
			    setstate.expectedAngle.roll =0.0f;               /*期望横滚角为0*/  
			    setstate.expectedAngle.pitch =0.0f;              /*期望纵倾角为0*/  
			    setstate.expectedAngle.yaw = state.realAngle.yaw;/*期望偏航角等于当前偏航角*/
			 
			//升沉运动+姿态调节PID控制
          attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* 角度环PID（roll和pitch） */
          attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* 角速度环PID */
        }
	     	control.premode=control.nowmode;
    }
/******************************************* 定深模式下控制 ********************************************/
	else if(command[MODE] == DEPTH_MODE)
	{
		  control.nowmode=DEPTH_MODE;                      /*当前控制模式为定深模式*/
		  depthPid.thrustBase=1600;                        /*定深油门基值,该值能保证水下机器人悬浮*/
		  setstate.expectedAngle.pitch =0.0f;              /*期望纵倾角为0*/  
      setstate.expectedAngle.roll =0.0f;               /*期望横滚角为0*/ 
      setstate.expectedAngle.yaw = state.realAngle.yaw;/*期望偏航角等于当前偏航角*/
		if(control.premode==HAND_MODE && control.nowmode==DEPTH_MODE) /*从手动模式切换为定深模式*/
		{
			pidReset(&depthPid.pidZ);                                 /*PID误差参数清零*/
			setstate.expectedDepth=state.realDepth;                   /*定深模式切换时刻深度作为期望深度*/
		}    
		//PID控制
		depthPID(&state.realDepth,&setstate.expectedDepth,&control);                         /*深度环PID */
	  attitudeAnglePID(&state.realAngle, &setstate.expectedAngle, &setstate.expectedRate); /* 角度环PID */
    attitudeRatePID(&state.realRate, &setstate.expectedRate, &control);                  /* 角速度环PID */
		control.premode=control.nowmode;
	}
}

void VthrusterControl(control_t* control)    //姿态调整PWM波输出函数
{
	float  r=control->roll*0.5f;                          /*增益系数可以修改，默认0.5*/
	float p=control->pitch*0.5f;   
	float Vthruster1, Vthruster2, Vthruster3, Vthruster4; /*定义4个推进器油门值*/
	
	Thurater_PWM_Calc[0]=ThuraterPWM_Calc_limit(Thurater_PWM_Calc[0]-r-p);
	Thurater_PWM_Calc[1]=ThuraterPWM_Calc_limit(Thurater_PWM_Calc[1]-r+p);
	Thurater_PWM_Calc[2]=ThuraterPWM_Calc_limit(Thurater_PWM_Calc[2]+r-p);
	Thurater_PWM_Calc[3]=ThuraterPWM_Calc_limit(Thurater_PWM_Calc[3]+r+p);
	
	Vthruster1=limitVThrust(control->thrust+Thurater_PWM_Calc[0]);         /*在摇杆油门的基础上增加或减小对应推进器推力，实现姿态调平*/
	Vthruster2=limitVThrust(control->thrust+Thurater_PWM_Calc[1]);
	Vthruster3=limitVThrust(control->thrust+Thurater_PWM_Calc[2]);
	Vthruster4=limitVThrust(control->thrust+Thurater_PWM_Calc[3]);
	
	//将油门值转化为PWM值
	PWMcontrol.v1=Vthrust2pwm(Vthruster1);
	PWMcontrol.v2=Vthrust2pwm(Vthruster2);
	PWMcontrol.v3=Vthrust2pwm(Vthruster3);
	PWMcontrol.v4=Vthrust2pwm(Vthruster4);
	
	THV1=PWMcontrol.v1;
	THV2=PWMcontrol.v2;
	THV3=PWMcontrol.v3;
	THV4=PWMcontrol.v4;
	
	//将PWM值给相应定时器通道
	TIM_SetTIM5Compare(PWMcontrol.v1,PWMcontrol.v2,PWMcontrol.v3,PWMcontrol.v4);
}
/*********************************************************************************************************************************************************/
//水平推进控制函数
//控制X、Y轴的移动和Z轴的转动，移动过程中将油门值转化为期望速度，实际速度由加速度计积分得到，作PID控制，使机器人保持较为固定的速度运动
//如果直接将油门值作为PWM的输出，推动油门的过程中，运动过程差不多，但油门为0时，对应推进器无输出，则洋流会导致机器人移动，且在推杆回到最低时，存在惯性，不能保证停留在想停下的位置。
//而油门转化为速度，油门为0时，表示期望速度为0，如果存在洋流，此时推进器仍然可能转动，保证悬浮时能稳定，但可能在沉底时，推进器发生转动，导致机器人微小运动。
/***************************************************************************************************************************************************************************************/
void Motion_Control(control_t* output)
{
	 float zturn_speed;                /*绕Z轴的转速*/
	 float x_speed;                    /*X轴速度*/
     float y_speed;                    /*Y轴速度*/
	state_t motionstate;               /*XY平面真实运动状态 */    
	setstate_t motionsetstate;         /*XY平面期望运动状态 */ 
	
	//保存平面运动油门基础值
	output->forward_thrustbase=(float)command[SURGE];
	output->right_thrustbase=(float)command[SWAY];
	output->turn_thrustbase=(float)command[YAW];
	
	//摇杆4输出转化为Z轴转向速度(偏航运动只控制角速度环，即遥控命令转化为角速度)
	zturn_speed = (float)pwm2Range(command[YAW], -100.0f, 100.0f);
	if (zturn_speed < 5.0f && zturn_speed > -5.0f)    /*转向速度死区*/
		zturn_speed = 0.f;     
	if((int)zturn_speed==0)
	{
		pidReset(&VZpid);           /*yaw速度环误差参数清0*/     
	}
	
//后面使用遥控器摇杆输出期望角度使用（期望角度等于摇杆输出叠加到真实角度上的角度）
//	setstate.expectedAngle.yaw -= zturn_speed * ft;       
//    if (setstate.expectedAngle.yaw > 180.0f)            /*偏航角限幅*/
//		setstate.expectedAngle.yaw -= 360.0f;
//    if (setstate.expectedAngle.yaw < -180.0f)
//        setstate.expectedAngle.yaw += 360.0f;
/**********************************************************推进器控制模式*****************************************/
	if(command[EXCHANGE_MODE]==TH_MODE)     
	{
		//油门转化为对应速度
		x_speed=(float)pwm2Range(command[SURGE], -100.0f, 100.0f);       
		if (x_speed < 5.0f && x_speed > -5.0f)  /*X移动速度死区*/
			x_speed = 0.f;       
					
		y_speed=(float)pwm2Range(command[SWAY], -100.0f, 100.0f);
		if (y_speed < 5.0f && y_speed > -5.0f)  /*Y移动速度死区*/
			y_speed = 0.f;          
	
		//将油门速度设置为期望速度
//		motionsetstate.velocity.x= x_speed;         
//		motionsetstate.velocity.y= y_speed;  
		motionsetstate.velocity.z= zturn_speed; 
	
		//实际速度为加速度积分,z轴为偏航角速度
//		motionstate.realvelocity.x=state.realvelocity.x;
//		motionstate.realvelocity.y=state.realvelocity.y;
		motionstate.realvelocity.z=state.realRate.yaw;

		//X,Y速度PID（速度积分漂移严重，不使用）
//		motionXVelocityPID(&motionstate.realvelocity,&motionsetstate.velocity,output);
//		motionYVelocityPID(&motionstate.realvelocity,&motionsetstate.velocity,output);
		
		//Z轴转向角速度PID
		motionZVelocityPID(&motionstate.realvelocity,&motionsetstate.velocity,output);
	}	
	else  //总线舵机控制模式                   
	{
		//使用机械臂时，沉底不动，但保持转向可调
		motionsetstate.velocity.z= zturn_speed;
		motionstate.realvelocity.z=state.realRate.yaw;
		output->forward_thrustbase=1500;
		output->right_thrustbase=1500;
		output->forward_thrust=0.0f;            
		output->right_thrust=0.0f;
		
		/*角速度PID*/
		motionZVelocityPID(&motionstate.realvelocity,&motionsetstate.velocity,output);
	}
}

   /* 水平运动PWM输出函数*/
void HthrusterControl(control_t* control) 
{
	//float thruster5, thruster6, thruster7, thruster8;                                   /*定义4个推进器油门值*/
	if (control->forward_thrustbase< 1540.0f && control->forward_thrustbase > 1460.0f)
			control->forward_thrustbase= 1500.f;                                        /*前后运动推进器中值死区*/
	if (control->right_thrustbase< 1540.0f && control->right_thrustbase > 1460.0f)
			control->right_thrustbase= 1500.f;                                          /*左右运动推进器中值死区*/
	if (control->turn_thrustbase< 1540.0f && control->turn_thrustbase > 1460.0f)
			control->turn_thrustbase= 1500.f;                                           /*偏航运动推进器中值死区*/

	  /*推进器静止PWM1500，加上遥控器X,Y,YAW三个方向的控制PWM*/
	PWMcontrol.h1=Hthrust2pwm(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
	PWMcontrol.h2=Hthrust2pwm(1500 - remote_speed.remote_speed_YAW + remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
	PWMcontrol.h3=Hthrust2pwm(1500 - remote_speed.remote_speed_YAW - remote_speed.remote_speed_X + remote_speed.remote_speed_Y);
	PWMcontrol.h4=Hthrust2pwm(1500 + remote_speed.remote_speed_YAW - remote_speed.remote_speed_X - remote_speed.remote_speed_Y);
	
	 /*将PWM值给相应定时器通道*/
	TIM_SetTIM4Compare(PWMcontrol.h1,PWMcontrol.h2,PWMcontrol.h3,PWMcontrol.h4);
	
	THH1=PWMcontrol.h1;
	THH2=PWMcontrol.h2;
	THH3=PWMcontrol.h3;
	THH4=PWMcontrol.h4;
}

float ThuraterPWM_Calc_limit(float PWM_Calc) //垂直推进器角度调节PID增量限幅（-100，100）
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
 
/*垂直推进器PWM值限幅，默认为1000—2000*/
u16 limitVPWM(int input)           
{
	if(input>2000)
	{
		return 2000;
	}
	else if(input<1000)
	{
		return 1000;
	}
	else
	return input;
}

/*水平推进器PWM值限幅，默认为1100—1900*/
u16 limitHPWM(int input)           
{
	if(input>1900)
	{
		return 1900;
	}
	else if(input<1100)
	{
		return 1100;
	}
	else
		return input;
}

/* 垂直推进器油门值限幅*/
float limitVThrust(float value)
{
    if (value > 2000)
    {
        value = 2000;
    }
    else if (value <1000)
    {
        value =1000;
    }
    return value;
}

/* 水平推进器油门值限幅*/
float limitHThrust(float value)
{
    if (value > 1900)
    {
        value = 1900;
    }
    else if (value <1000)
    {
        value =1000;
    }
    return value;
}

//将垂直推进器油门值转为PWM值
u16 Vthrust2pwm(float thrust_value)
{
    float pwm_value;
    //油门值转化为 PWM 1000-2000, [VTHRUST_MIN, VTHRUST_MAX] 到 [1000,2000]的映射
    pwm_value = 1000.0f + (thrust_value - VTHRUST_MIN) * 1000.0f / (VTHRUST_MAX - VTHRUST_MIN);
    return limitVPWM((int)pwm_value);
}

//将水平油门值转为PWM值
u16 Hthrust2pwm(float thrust_value)
{
    float pwm_value;
    //油门值转化为 PWM 1100-1900, [VTHRUST_MIN, VTHRUST_MAX] 到 [1100,1900]的映射
    pwm_value = 1100.0f + (thrust_value - HTHRUST_MIN) * 1100.0f / (HTHRUST_MAX - HTHRUST_MIN);
    return limitHPWM((int)pwm_value);
}
/*************************************备用函数*********************************************/

float pwm2Range(int pwm_value, float p_min, float p_max)
{
    float p;
    p = p_min + ((float)(pwm_value)-1000.0f) * (p_max - p_min) / 1000.0f;
    if (p > p_max)
        p = p_max;
    if (p < p_min)
        p = p_min;
    return p;
}

///* 水平推进器油门值正向运动限幅*/
//float limitHPThrust(float value)
//{
//    if (value > 1900)
//    {
//        value = 1900;
//    }
//    else if (value <1500)
//    {
//        value =1500;
//    }
//    return value;
//}

///* 水平推进器油门值反向运动限幅*/
//float limitHFThrust(float value)
//{
//    if (value > 1500)
//    {
//        value = 1500;
//    }
//    else if (value <1100)
//    {
//        value =1100;
//    }
//    return value;
//}
/**************************************************************************************************************************/
/*函数功能：将九轴的数据融合，用加速度计读数校准陀螺仪的角度输出，欧拉角转四元数再转欧拉角*/
//输入：     
/*  acc.x,acc.y,acc.z----->三轴加速计原始输出数据   */
/*  gyro.x,gyro.y, gyro.z------>三轴陀螺仪原始角度输出 */
/*   dt------>PI积分时间微元dt   */
//输出：
/*   state->attitude.pitch\roll\yaw----->校准后的三轴角度值   */
/**************************************************************************************************************************/
void imuUpdate(Axis3f acc, att_t gyro, attitude_t *attitudeClib, float dt)//修改函数参数列表时注意修改.h文件中的函数声明
{
	float normalise;
	float ex, ey, ez;
	float q0s, q1s, q2s, q3s;	       /*四元数的平方*/
	static float R11,R21;		       /*矩阵(1,1),(2,1)项*/
	static float vecxZ, vecyZ, veczZ;  /*机体坐标系下的Z方向向量*/
	float halfT =0.5f * dt;
	//Axis3f tempacc =acc;
	float q0Last = qq0;
	float q1Last = qq1;
	float q2Last = qq2;
	float q3Last = qq3;
	
	//度转弧度
	gyro.roll =gyro.roll * DEG2RAD;	
	gyro.pitch=gyro.pitch * DEG2RAD;
	gyro.yaw= gyro.yaw* DEG2RAD;

	//数据融合处理误差, 某一个方向加速度不为0      
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		//单位化加速计测量值
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		//加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
		ex = (acc.y * veczZ - acc.z * vecyZ);
		ey = (acc.z * vecxZ - acc.x * veczZ);
		ez = (acc.x * vecyZ - acc.y * vecxZ);
		
		//误差累计，与积分常数相乘
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		//用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量
		gyro.roll += Kp * ex + exInt;
		gyro.pitch += Kp * ey + eyInt;
		gyro.yaw += Kp * ez + ezInt;
	}
	//一阶近似算法，四元数运动学方程的离散化形式和积分
	qq0 += (-q1Last * gyro.roll  - q2Last * gyro.pitch - q3Last * gyro.yaw) * halfT;
	qq1 += ( q0Last * gyro.roll + q2Last * gyro.yaw  - q3Last * gyro.pitch) * halfT;
	qq2 += ( q0Last * gyro.pitch - q1Last * gyro.yaw  + q3Last * gyro.roll) * halfT;
	qq3 += ( q0Last * gyro.yaw  + q1Last * gyro.pitch - q2Last * gyro.roll) * halfT;
	
	//单位化四元数
	normalise = invSqrt(qq0 * qq0 + qq1 * qq1 + qq2 * qq2 + qq3 * qq3);
	qq0 *= normalise;
	qq1 *= normalise;
	qq2 *= normalise;
	qq3 *= normalise;
	//四元数的平方
	q0s = qq0 * qq0;
	q1s = qq1 * qq1;
	q2s = qq2 * qq2;
	q3s = qq3 * qq3;
	
	R11 = q0s + q1s - q2s - q3s;	    /*矩阵(1,1)项*/
	R21 = 2 * (qq1 * qq2 + qq0 * qq3);	/*矩阵(2,1)项*/

	//机体坐标系下的Z方向向量
	vecxZ = 2 * (qq1 * qq3 - qq0 * qq2);/*矩阵(3,1)项*/
	vecyZ = 2 * (qq0 * qq1 + qq2 * qq3);/*矩阵(3,2)项*/
	veczZ = q0s - q1s - q2s + q3s;   	/*矩阵(3,3)项*/
	
	if (vecxZ>1) vecxZ=1;
	if (vecxZ<-1) vecxZ=-1;
	
	/*计算roll pitch yaw 欧拉角*/
	attitudeClib->pitch = -asinf(vecxZ) * RAD2DEG; 
	attitudeClib->roll = atan2f(vecyZ, veczZ) * RAD2DEG;
	attitudeClib->yaw = atan2f(R21, R11) * RAD2DEG;
//	a.pitch=attitudeClib->pitch;
//	a.roll=attitudeClib->roll;
//	a.yaw=attitudeClib->yaw;
//	attitudeClib.pitch = -asinf(vecxZ) * RAD2DEG; 
//	attitudeClib.roll = atan2f(vecyZ, veczZ) * RAD2DEG;
//	attitudeClib.yaw = atan2f(R21, R11) * RAD2DEG;
//	a.pitch=attitudeClib.pitch;
//	a.roll=attitudeClib.roll;
//	a.yaw=attitudeClib.yaw;
}

//开平方并求导函数
float invSqrt(float x)	
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*****************************************************************
  *说明:一次积分函数
  * 参数 : in, 输入加速度和角加速度数据指针结构体
  *        out, 输出速度和角速度数据指针结构体
***********************************************/
void integralUpdate(vec3_s* in,  vec3_s* out)
{
	integX+=in->x/1024.0f;  /*除以陀螺仪采样频率*/  
	integY+=in->y/1024.0f;
	integZ+=in->z/1024.0f; 

	out->x=integX;                   
	out->y=integY;  
	out->z=integZ;  	
}
 




