#ifndef  __RECEIVER_H
#define  __RECEIVER_H

#include "os_cpu.h"
#include "stm32f4xx.h"                  // Device header

#include "ucos_ii.h"

#define PPM_MAX_VAL 2000
#define PPM_MID_VAL 1500
#define PPM_MIN_VAL 1000

#define	AIL 1	// Aileron(roll)
#define ELE 2	// Elevator(pitch)
#define THR 3	// Throttle
#define RUD 4	// Rudder(yaw)

extern uint32_t ppm_val[10];
extern BOOLEAN ppm_error;

void TIM1_Init(void);
void Receiver_Init(void);
void TIM1_CC_IRQHandler(void);

#endif
