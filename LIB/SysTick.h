#ifndef	__SysTick_H
#define __SysTick_H

#include "stm32f10x.h"


void Init_SysTick(void);
void delay_ms(u32 time);
void delay_us(u32 time);


#endif
