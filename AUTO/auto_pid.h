#ifndef __AUTOPID_H
#define __AUTOPID_H

#include "sys.h" 
#include "pid.h"

float angel_speed(const float error_angel, const float max_error_angel, int i);
int PI_Calc(PidObject* pid, const float error_angel_speed);


#endif
