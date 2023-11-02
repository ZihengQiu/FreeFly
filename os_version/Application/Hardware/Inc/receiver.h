#ifndef  __RECEIVER_H
#define  __RECEIVER_H

#include "stm32f4xx.h"                  // Device header

#include "ucos_ii.h"

// #define PPM_MAX_VAL 2001
// #define PPM_MIN_VAL 1001
#define PPM_MAX_VAL 1990
#define PPM_MIN_VAL 1010

#define	AIL 4	// Aileron(roll)
#define ELE 2	// Elevator(pitch)
#define THR 3	// Throttle
#define RUD 1	// Rudder(yaw)

extern uint32_t ppm_val[10];

void TIM1_Init(void);

void Receiver_Init(void);

void TIM1_CC_IRQHandler(void);

#endif
