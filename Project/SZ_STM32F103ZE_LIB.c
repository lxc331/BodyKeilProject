/********************   (C) COPYRIGHT 2013 www.armjishu.com   ********************
 * 文件名  ：SZ_STM32F103ZE_LIB.c
 * 描述    ：提供STM32F103ZE神舟III号开发板的库函数
 * 实验平台：STM32神舟开发板
 * 作者    ：www.armjishu.com
**********************************************************************************/

///* Includes ------------------------------------------------------------------*/
//
//#include "SZ_STM32F103ZE_LIB.h"
//
//__IO uint32_t TimingDelay;

//GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT, LED4_GPIO_PORT};
//const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
//const uint32_t GPIO_CLK[LEDn] = {LED1_GPIO_CLK, LED2_GPIO_CLK, LED3_GPIO_CLK, LED4_GPIO_CLK};

/**-------------------------------------------------------
  * @函数名 delay
  * @功能   简单的delay延时函数.
  * @参数   延迟周期数 0--0xFFFFFFFF
  * @返回值 无
***------------------------------------------------------*/
//void delay(__IO uint32_t nCount)
//{
//  for (; nCount != 0; nCount--);
////}
////
/////**-------------------------------------------------------
//  * @函数名 SZ_STM32_LEDInit
//  * @功能   初始化LED的GPIO管脚，配置为推挽输出
//  * @参数   LED1  对应开发板上第一个指示灯
//  *         LED2  对应开发板上第二个指示灯
//  *         LED3  对应开发板上第三个指示灯
//  *         LED4  对应开发板上第四个指示灯
//  * @返回值 无
//***------------------------------------------------------*/
//void SZ_STM32_LEDInit(Led_TypeDef Led)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//
//    /* Enable the GPIO_LED Clock */
//    /* 使能LED对应GPIO的Clock时钟 */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
//
//    /* Configure the GPIO_LED pin */
//    /* 初始化LED的GPIO管脚，配置为推挽输出 */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//
//    GPIO_Init(GPIOD, &GPIO_InitStructure);
//}

/**-------------------------------------------------------
  * @函数名 SZ_STM32_LEDOn
  * @功能   点亮对应的LED指示灯
  * @参数   LED1  对应开发板上第一个指示灯
  *         LED2  对应开发板上第二个指示灯
  *         LED3  对应开发板上第三个指示灯
  *         LED4  对应开发板上第四个指示灯
  * @返回值 无
//***------------------------------------------------------*/
//void SZ_STM32_LEDOn(Led_TypeDef Led)
//{
//    /* 指定管脚输出低电平，点亮对应的LED指示灯 */
//    LED1_GPIO_PORT->BRR = GPIO_Pin_2;
//}

/**-------------------------------------------------------
  * @函数名 SZ_STM32_LEDOn
  * @功能   熄灭对应的LED指示灯
  * @参数   LED1  对应开发板上第一个指示灯
  *         LED2  对应开发板上第二个指示灯
  *         LED3  对应开发板上第三个指示灯
  *         LED4  对应开发板上第四个指示灯
  * @返回值 无
//***------------------------------------------------------*/
//void SZ_STM32_LEDOff(Led_TypeDef Led)
//{
//    /* 指定管脚输出高电平，熄灭对应的LED指示灯 */
//    GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];
//}

/**-------------------------------------------------------
  * @函数名 SZ_STM32_LEDToggle
  * @功能   将对应的LED指示灯状态取反
  * @参数   LED1  对应开发板上第一个指示灯
  *         LED2  对应开发板上第二个指示灯
  *         LED3  对应开发板上第三个指示灯
  *         LED4  对应开发板上第四个指示灯
//  * @返回值 无
//***------------------------------------------------------*/
//void SZ_STM32_LEDToggle(Led_TypeDef Led)
//{
//    /* 指定管脚输出异或 1，实现对应的LED指示灯状态取反目的 */
//    GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
//}

/******************* (C) COPYRIGHT 2013 www.armjishu.com *****END OF FILE****/
