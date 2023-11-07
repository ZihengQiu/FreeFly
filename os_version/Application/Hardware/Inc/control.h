#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f4xx.h"                  // Device header

#include "receiver.h"
#include "mathkit.h"

extern uint32_t ppm_val[10], motor_compare[4];

typedef struct
{
	float kp, ki, kd;
	float err, err_last;
	float target, current;
	float p_out, i_out, d_out, out;
}pid_t;

void MotorControl(vec3d_t angle_cur, vec3d_t gyro);

#endif
