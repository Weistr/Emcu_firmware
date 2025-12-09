#ifndef _CMSIS_DELAY_H
#define _CMSIS_DELAY_H

#include "main.h"

void cmsis_blocked_delay_init(void);
void cmsis_blocked_delay_us(uint16_t nus);
void cmsis_blocked_delay_ms(uint16_t nms);
#endif //