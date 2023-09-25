#ifndef  __RECEIVER_H
#define  __RECEIVER_H

#include "stm32f4xx.h"                  // Device header

void TIM1_init(void);

void TIM1_CC_IRQHandler(void);

#endif
