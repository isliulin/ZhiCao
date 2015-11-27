#include "controller.h"
#include "TSL1401CCD.h"
#include "Encoder.h"


u8 CCD_Pixel[128];  //����CCD����
int32_t nPauseMeter[4];  //��ͣ����
int32_t nStopMeter; //��������
int32_t nCurrentMeter; //��ǰ����
bool bLineDetected; 
void Controller_Init(u8 nInterval)
{
	u8 i;
 TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  //���ñ�Ƶ��X1~X6��������˿�

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
	
	//PA4=X2,PA5=X3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//PC4=X1, PC5=X6, PC0=LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_0;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	//PB0=X4,PB1=X5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	//�����������ڶ�ʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

 TIM_DeInit(TIM2);

 TIM_TimeBaseStructure.TIM_Period=nInterval*1000;		 //ARR��ֵ
 TIM_TimeBaseStructure.TIM_Prescaler=0;
 TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //������Ƶ
 TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_PrescalerConfig(TIM2,SYS_CLK/1000000,TIM_PSCReloadMode_Immediate);//ʱ�ӷ�Ƶϵ��SYS_CLK/1000 - 1�����Զ�ʱ��ʱ��Ϊ1K
 TIM_ARRPreloadConfig(TIM2, DISABLE);//��ֹARRԤװ�ػ�����
 TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //���ö�ʱ�жϣ������жϳ�ʼ���������趨�ж����ȼ�

	nStopMeter=600;
	nCurrentMeter = 0;
	for(i=0;i<4;i++)nPauseMeter[i]=(i+1)*100;
	
	bLineDetected = FALSE;
	
 TIM_Cmd(TIM2, ENABLE);	//����ʱ��
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

//void TIM2_IRQHandler(void){;}    //��ʱ�ж�
