#ifndef __SYSTICKDELAY_H
#define __SYSTICKDELAY_H

#include "stm32f10x.h"
//��ʱ��ʱ����
/*
��main.c��NVIC_Configuration������
����SysTick_Config(720)��ÿ10us����һ��SysTick
��stm32f10x_it.c�е�SysTick_Handler����SysTick�жϣ�
ִ��TimingDelay_Decrement();
����ÿ100��10usʹTimeStamp_ms��1
*/
static __IO uint32_t TimingDelay; //SysTick��������
extern __IO uint32_t TimeStamp_ms;

void TimingDelay_Decrement(void);//��ȡ���ĳ���


void Delay_10us(__IO uint32_t nTime);//��ʱ��ʱ���� 10usΪ��λ
void Delay_ms(__IO uint32_t nTime);//��ʱ��ʱ���� 1msΪ��λ

//��ʱ��ʱ�������

#endif
