#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void Delay_Init(void);
void Delay_ms(uint32_t ms);
uint32_t millis(void);
void Delay_IncTick(void);

#endif
