#include "key.h"
#include "SysTickDelay.h"

void Key_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					    //输入上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_8 |GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					    //输入上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					    //输入上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

bool Key_Press(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)==0){		  //K3  
	  Delay_10us(2000);										  
		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)==0){		  //按键消抖动
			return TRUE;
		}                                                                                                                                                                                      
	}
	return FALSE;
}

bool Key_Release(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)==0){		  //K3  
	  Delay_10us(2000);										  
		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)==0){		  //按键消抖动
			while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin)==0);//是否松开按键	 
			return TRUE;
		}                                                                                                                                                                                      
	}
	return FALSE;
}
