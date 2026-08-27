#ifndef __SERVO_H
#define __SERVO_H

#include "sys.h"
#include "timer.h"
#include <stdlib.h>

#define		Servo_Mid_PWM				1500

typedef struct 
{
	u16 position1;
	u16 position2;
}pos_t;

extern pos_t pos;

void Servo_Init(void);
u16 Servo_Limit(u16 value);
u16 DS300_Limit(u16 value);
void Servo_out(u32 PWM1,u32 PWM2);
void Servo_control(u32 PWM1,u32 PWM2);
void arm_task(void *pvParameters);
u16 Dec2Hex(u16 temp);



#endif
