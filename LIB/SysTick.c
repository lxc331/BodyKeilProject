/*
文件名：SysTick.c
功能：使用系统节拍定时器实现精确延时
注意：延时时间若超过1000 则要多次调用函数 如delay_ms（2000）错误 正确：delay_ms(1000);delay_ms(1000);
*/

#include "SysTick.h"

u16 	fms=0;
u8 	fus=0;

/**************************
函数名：Init_SysTick
功能：初始化定时器
输入：无
输出：无
*****************************/

void Init_SysTick(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//系统时钟8分频
	fus=72/8;//计数9次为1us
	fms=fus*1000;
}
/**************************
函数名：delay_us
功能：延时us
输入：延时时间time ，单位us
输出：无
*****************************/
void delay_us(u32 time)
{
	u32 temp;
	SysTick->LOAD=time*fus;//时间加载
	SysTick->VAL=0x00;//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 
}
/**************************
函数名：delay_ms
功能：延时ms
输入：延时时间time,单位ms  
输出：无
*****************************/
void delay_ms(u32 time)
{
	u32 temp;
	SysTick->LOAD=time*fms;//时间加载
	SysTick->VAL=0x00;//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 
}
