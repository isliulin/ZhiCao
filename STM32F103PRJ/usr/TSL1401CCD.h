#ifndef __TSL1401CCD_H
#define __TSL1401CCD_H

#define SI_SetVal()   GPIOA->BSRR = GPIO_Pin_0
#define SI_ClrVal()   GPIOA->BRR = GPIO_Pin_0
#define CLK_ClrVal()  GPIOA->BRR = GPIO_Pin_1
#define CLK_SetVal()  GPIOA->BSRR = GPIO_Pin_1

#include "stm32f10x.h"
void CCD_Init(void);
void CCD_StartIntegration(void);
void CCD_ImageCapture(unsigned char * ImageData);
void CCD_CalculateIntegrationTime(u8 *Pixel);
u8 CCD_PixelAverage(u8 len, u8 *data);
void CCD_SamplingDelay(void);
bool CCD_DetectLine(u8 *Pixel, u8 sen);
#endif
