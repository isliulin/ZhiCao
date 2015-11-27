#include "controller.h"
#include "TSL1401CCD.h"
#include "Encoder.h"


u8 CCD_Pixel[128];  //保存CCD数据
int32_t nPauseMeter[4];  //暂停米数
int32_t nStopMeter; //结束米数
int32_t nCurrentMeter; //当前米数
bool bLineDetected; 
void Controller_Init(u8 nInterval)
{
	u8 i;
 TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  //配置变频器X1~X6控制输出端口

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
	
	//PA4=X2,PA5=X3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PC4=X1, PC5=X6, PC0=LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_0;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	//PB0=X4,PB1=X5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	//配置运行周期定时器
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

 TIM_DeInit(TIM2);

 TIM_TimeBaseStructure.TIM_Period=nInterval*1000;		 //ARR的值
 TIM_TimeBaseStructure.TIM_Prescaler=0;
 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //采样分频
 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_PrescalerConfig(TIM2,SYS_CLK/1000000,TIM_PSCReloadMode_Immediate);//时钟分频系数SYS_CLK/1000 - 1，所以定时器时钟为1K
 TIM_ARRPreloadConfig(TIM2, DISABLE);//禁止ARR预装载缓冲器
 TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //启用定时中断，须在中断初始化程序中设定中断优先级

	nStopMeter=600;
	nCurrentMeter = 0;
	for(i=0;i<4;i++)nPauseMeter[i]=(i+1)*100;
	
	bLineDetected = FALSE;
	
 TIM_Cmd(TIM2, ENABLE);	//开启时钟
}

void Controller_Update(void)
{
// 	GPIO_WriteBit(GPIOC,GPIO_Pin_0,(BitAction)(1-GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_0)));
// 	GPIO_WriteBit(GPIOC,GPIO_Pin_0,Bit_SET);
// 	X4_SET;
// 	X5_SET;
	nCurrentMeter=ENC_ReadCNT();
	CCD_ImageCapture(CCD_Pixel);
	bLineDetected = CCD_DetectLine(CCD_Pixel,8);
// 	X4_CLR;
// 	X5_CLR;
// 	GPIO_WriteBit(GPIOC,GPIO_Pin_0,Bit_RESET);
}

bool Controller_is_Line_Detected(void)
{
	return bLineDetected;
}
void Controller_Stop(void)
{
	X1_CLR;
	X2_CLR;
	X3_SET;
	X4_CLR;
	X5_CLR;
	X6_CLR;
}
void Controller_SetSpeed(u8 nSpeedSel)
{
	X1_SET;
	X2_CLR;
	X3_CLR;
	switch(nSpeedSel)
	{
	case 0:
		X4_CLR;
		X5_CLR;
		X6_CLR;
		break;
	case 1:
		X4_SET;
		X5_CLR;
		X6_CLR;
		break;
	case 2:
		X4_CLR;
		X5_SET;
		X6_CLR;
		break;
	case 3:
		X4_CLR;
		X5_CLR;
		X6_SET;
		break;
	default:
		X4_CLR;
		X5_CLR;
		X6_CLR;
		break;	
	}
}

//void TIM2_IRQHandler(void){;}    //定时中断
