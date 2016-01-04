#ifndef __SYSTICKDELAY_H
#define __SYSTICKDELAY_H

#include "stm32f10x.h"
//定时延时程序
/*
在main.c的NVIC_Configuration函数中
设置SysTick_Config(720)，每10us产生一次SysTick
在stm32f10x_it.c中的SysTick_Handler处理SysTick中断，
执行TimingDelay_Decrement();
并且每100个10us使TimeStamp_ms加1
*/
static __IO uint32_t TimingDelay; //SysTick计数变量
extern __IO uint32_t TimeStamp_ms;

void TimingDelay_Decrement(void);//获取节拍程序


void Delay_10us(__IO uint32_t nTime);//定时延时程序 10us为单位
void Delay_ms(__IO uint32_t nTime);//定时延时程序 1ms为单位

//定时延时程序结束

#endif
