#include "LED.h"

void LED_GPIO_Config(void)
{
	GPIO_InitTypeDef	GPIO_Initstructure;
	
	RCC_APB2PeriphClockCmd(LED_CLK,ENABLE);
	
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_Pin = LED1_Pin |LED2_Pin |LED3_Pin |LED4_Pin ;
	
	GPIO_Init(LED_Port,&GPIO_Initstructure);
	
	LED_Off(LED1_Pin);
	LED_Off(LED2_Pin);
	LED_Off(LED3_Pin);
	LED_Off(LED4_Pin);
}

void LED_On(u16 gpio_pin)
{
	GPIO_ResetBits(LED_Port,gpio_pin);
}

void LED_Off(u16 gpio_pin)
{
	GPIO_SetBits(LED_Port,gpio_pin);
}
	