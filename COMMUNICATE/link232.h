#ifndef __LINK485_H
#define __LINK485_H
#include <stdbool.h>
#include "sys.h"

extern int16_t rx232buf[12];
extern u16 command[11];
void link485Rx_task(void *pvParameters);
void link485Tx_task(void *pvParameters);
void delayms(void);

#endif
