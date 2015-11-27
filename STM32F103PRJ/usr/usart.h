#ifndef __USART1_H
#define __USART1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdio.h"
#include "stdarg.h"

void USART1_Config(void);
void USART2_Config(void);
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
char *itoa(int value, char *string, int radix);
int fputc(int ch, FILE *f);

#endif /* __USART1_H */
