#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"                  // Device header

void Motor_Init(void);
void Motor_SetDutyCycle(uint32_t);

#endif
