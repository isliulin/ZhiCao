#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#define ENCODER_OVERFLOW 5120    //计数器溢出次数

extern __IO int32_t nEncOverFlowCount;

void ENC_Init(void);
int32_t ENC_ReadCNT(void);
#endif
