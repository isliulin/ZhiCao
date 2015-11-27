#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#define SYS_CLK 64000000   //ϵͳʱ��64MHz

#define X1_CLR GPIOC->BRR = GPIO_Pin_4
#define X1_SET GPIOC->BSRR = GPIO_Pin_4

#define X2_CLR GPIOA->BRR = GPIO_Pin_4
#define X2_SET GPIOA->BSRR = GPIO_Pin_4

#define X3_CLR GPIOA->BRR = GPIO_Pin_5
#define X3_SET GPIOA->BSRR = GPIO_Pin_5

#define X4_CLR GPIOB->BRR = GPIO_Pin_0
#define X4_SET GPIOB->BSRR = GPIO_Pin_0

#define X5_CLR GPIOB->BRR = GPIO_Pin_1
#define X5_SET GPIOB->BSRR = GPIO_Pin_1

#define X6_CLR GPIOC->BRR = GPIO_Pin_5
#define X6_SET GPIOC->BSRR = GPIO_Pin_5

#define LED_OFF GPIOC->BRR = GPIO_Pin_0
#define LED_ON GPIOC->BSRR = GPIO_Pin_0


extern u8 CCD_Pixel[128];  //����CCD����
extern int32_t nPauseMeter[4];  //��ͣ����
extern int32_t nStopMeter; //��������
extern int32_t nCurrentMeter; //��ǰ����

void Controller_Init(u8 nInterval);  //��������ΪnInterval����
void Controller_Update(void);
bool Controller_is_Line_Detected(void);
void Controller_Stop(void);
void Controller_SetSpeed(u8 nSpeedSel);

#endif
