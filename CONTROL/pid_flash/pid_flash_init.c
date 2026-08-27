#include "pid_flash_init.h"
#include "flash.h"
#include "rs232.h"
#include "velocity_pid.h"
#include "depth_pid.h"

extern u8 PID_flag;   

float datatemp[30]={0};
float pid_rs232_para[SIZE]={0};//SIZE等于19
//float test[19]={-0.0,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};



////四字节转float
//float byte_2_float(uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3)
//{
//    char Char[4];
//    Char[0]=m0;
//		Char[1]=m1;
//		Char[2]=m2;
//		Char[3]=m3;
//    return *(float*)(Char);
//}

//将Rs232接收字节数组转换为 float 类型，并且判断帧头（0x01 0x02）
void Rs232_Receive_PID(uint8_t* receive_byte)
{
	int i;
	if(receive_byte[0]==0x01 && receive_byte[1]==0x02 && PID_flag==1)	/*判断帧头*/
	{
		/*转换为 float 类型*/
		for(i=0;i<12;i++)          //12个PID参数
		{
			datatemp[i] = (float)(((u16)receive_byte[2*i+3]&0x0f<<8) | receive_byte[2*i+2])/10.0f;
		}
		//更新角速度环PID参数
		pidrollrateAngle_para.kp=pid_rs232_para[0];pidrollrateAngle_para.ki=pid_rs232_para[1];pidrollrateAngle_para.kd =pid_rs232_para[2];
		pidpitchrateAngle_para.kp=pid_rs232_para[3];pidpitchrateAngle_para.ki=pid_rs232_para[4];pidpitchrateAngle_para.kd =pid_rs232_para[5];
		pidyawrateAngle_para.kp=pid_rs232_para[6];pidyawrateAngle_para.ki=pid_rs232_para[7];pidyawrateAngle_para.kd=pid_rs232_para[8];
				
		//更新Z轴运动(移动)环PID参数
		VZpid_para.kp=pid_rs232_para[9];VZpid_para.ki =pid_rs232_para[10];VZpid_para.kd=pid_rs232_para[11];

		//更新Z轴定深PID参数
		depthpid_para.kp=pid_rs232_para[12];depthpid_para.ki=pid_rs232_para[13];depthpid_para.kd=pid_rs232_para[14];	
	}
	/*写入 flash 中 */
	PID_Write_Flash();
	PID_flag=0;     //更新一次标志位置0
}

void PID_Write_Flash(void)
{
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)datatemp,SIZE);
}

/*从 flash 中读出 */
void Flash_Read_PID(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)pid_rs232_para,SIZE);
	/*从flash中读出数据到 pid_rs232_para 中*/
	
	/*在别处调用给 pid 参数赋值 */
	//定义初始化角度环PID参数（可修改）
//	pid_Para pidrollAngle_para={pid_rs232_para[1],pid_rs232_para[2],pid_rs232_para[3]};
//  pid_Para pidpitchAngle_para={pid_rs232_para[4],pid_rs232_para[5],pid_rs232_para[6]};
//  pid_Para pidyawAngle_para={pid_rs232_para[7],pid_rs232_para[8],pid_rs232_para[9]};

//  //定义初始化角速度环PID参数（可修改）
//  pid_Para pidrollrateAngle_para={pid_rs232_para[10],pid_rs232_para[11],pid_rs232_para[12]};
//  pid_Para pidpitchrateAngle_para={pid_rs232_para[13],pid_rs232_para[14],pid_rs232_para[15]};
//  pid_Para pidyawrateAngle_para={pid_rs232_para[16],pid_rs232_para[17],pid_rs232_para[18]};
}


