#ifndef  __RECEIVER_H
#define  __RECEIVER_H

#include "stm32f4xx.h"                  // Device header

#define	AIL 0	// Aileron(roll)
#define ELE 1	// Elevator(pitch)
#define THR 2	// Throttle
#define RUD 3	// Rudder(yaw)

extern uint32_t ppm_val[10];

void TIM1_init(void);

void TIM1_CC_IRQHandler(void);

#endif
