#ifndef __START_BUTTON_H
#define __START_BUTTON_H

#include "stm32f10x.h"

/* PB1 uses an internal pull-down. The external button drives it to 3 V. */
void StartButton_Init(void);
void StartButton_Reset(void);
uint8_t StartButton_IsPressed(void);
uint8_t StartButton_WasPressed(void);

#endif
