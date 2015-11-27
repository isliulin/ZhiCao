
#include "SysTickDelay.h"
__IO uint32_t TimeStamp_ms=0; //毫秒绝对值
/****************************************************************************
* 名    称：void TimingDelay_Decrement(void)
* 功    能：获取节拍程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/  
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
/****************************************************************************
* 名    称：void Delay_us(__IO uint32_t nTime)
* 功    能：定时延时程序 10us为单位
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
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
