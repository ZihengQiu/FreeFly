#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"                  // Device header

#include "receiver.h"

extern uint32_t ppm_val[10];

typedef struct
{
	float kp, ki, kd;
	float err, err_last;
	float target, current;
	float p_out, i_out, d_out, out;
}pid_t;

#endif
