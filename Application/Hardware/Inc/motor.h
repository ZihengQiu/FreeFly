#ifndef __MOTOR_H
#define __MOTOR_H

#include "os_cpu.h"
#include "stm32f4xx.h"                  // Device header

#include "ucos_ii.h"

#include "receiver.h"

#define MOTOR_COMPARE_MAX_VAL 2000
#define MOTOR_COMPARE_MIN_VAL 1000

extern uint32_t ppm_val[10];
extern uint32_t motor_compare[4];
extern BOOLEAN motor_armed , signal_blocked, ESC_unlock_need_execute, ESC_unlock_executed;

void Motor_Init(void);
void MotorArmDetect(void);
void ESCUnlockDetect(void);
void SignalBlockDetect(void);
void MotorSetSpeed(void);
BOOLEAN ESCUnlock(void);

#endif
