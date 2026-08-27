#include "ibus.h"

/*油门值转 T200C PWM*/
u16 ibus_to_T200pwm(u16 ibus_value)
{
    float pwm;
    pwm = (float)IBUS_T200C_MIN + (float)(ibus_value - IBUS_RANGE_MIN) * IBUS_T200C_FACTOR;
    if (pwm > 1900) pwm = 1900;
    if (pwm < 1100) pwm = 1100;
    return (u16)pwm;
}

/*油门值转 T80C PWM*/
u16 ibus_to_T80pwm(u16 ibus_value)
{
    float pwm;
    pwm = (float)IBUS_T80_MIN + (float)(ibus_value - IBUS_RANGE_MIN) * IBUS_T80_FACTOR;
    if (pwm > 2000) pwm = 2000;
    if (pwm < 1000) pwm = 1000;
    return (u16)pwm;
}

/*油门值转DS300角度量*/
u16 ibus_to_DS300(u16 ibus_value)
{
    float pwm;
    pwm = (float)IBUS_RS300_MIN + (float)(ibus_value - IBUS_RANGE_MIN) * IBUS_DS300_FACTOR;
    if (pwm > 1750) pwm = 1750;
    if (pwm < 1040) pwm =1040;
    return (u16)pwm;
}

/*油门值转相机云台PWM（暂时没用云台舵机）*/
u16 ibus_to_CAMpwm(u16 ibus_value)
{
    float pwm;
    pwm = (float)IBUS_CAM_MIN + (float)(ibus_value - IBUS_RANGE_MIN) * IBUS_CAM_FACTOR;
    if (pwm > 2000) pwm = 2000;
    if (pwm < 1000) pwm =1000;
    return (u16)pwm;
}

