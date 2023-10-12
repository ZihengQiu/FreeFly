#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"                   // Device header

#include <math.h>

#include "includes.h"
#include "mathkit.h"
#include "gy86.h"

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler);

void AccMagUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, Vec3d_t *mag, Vec3d_t *mag0, double dt);
void GyroUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, double dt);
void MadgwickAHRS(Vec4d_t *q0, Vec3d_t *mag0, double dt);
#endif
