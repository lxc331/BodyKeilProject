#ifndef _LED_H
#define _LED_H

#include "stm32f10x.h"

#define 	LED_Port	GPIOF

#define		LED_CLK		RCC_APB2Periph_GPIOF

#define   LED1_Pin	GPIO_Pin_6
#define   LED2_Pin	GPIO_Pin_7
#define   LED3_Pin	GPIO_Pin_8
#define   LED4_Pin	GPIO_Pin_9

void LED_GPIO_Config(void);
void LED_On(u16 gpio_pin);
void LED_Off(u16 gpio_pin);

#endif
