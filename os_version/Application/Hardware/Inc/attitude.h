#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"                   // Device header
#include "mathkit.h"

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler);

void GyroUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, double dt);

#endif