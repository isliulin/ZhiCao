
#include "SysTickDelay.h"
__IO uint32_t TimeStamp_ms=0; //�������ֵ
/****************************************************************************
* ��    �ƣ�void TimingDelay_Decrement(void)
* ��    �ܣ���ȡ���ĳ���
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/  
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
/****************************************************************************
* ��    �ƣ�void Delay_us(__IO uint32_t nTime)
* ��    �ܣ���ʱ��ʱ���� 10usΪ��λ
* ��ڲ�������
* ���ڲ�������
* ˵    ����
* ���÷������� 
****************************************************************************/  
void Delay_10us(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  while(TimingDelay != 0);
}

void Delay_ms(__IO uint32_t nTime)
{ 
	uint32_t i;
	i=nTime;
	while(i--)
		Delay_10us(100);
}
