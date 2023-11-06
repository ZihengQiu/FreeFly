#ifndef __MOTOR_H
#define __MOTOR_H

#include "os_cpu.h"
#include "stm32f4xx.h"                  // Device header

#include "ucos_ii.h"

#include "receiver.h"

extern uint32_t ppm_val[10];

void Motor_Init(void);
void MotorArmDetect(void);
void ESCUnlockDetect(void);
void SignalBlockDetect(void);
void MotorSetSpeed(void);

#endif
