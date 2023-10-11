#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"                   // Device header
#include "mathkit.h"
#include <math.h>

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler);

void AccUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, double dt);
void GyroUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, double dt);
void MadgwickAHRS(Vec4d_t *q, Vec4d_t *q_acc, Vec4d_t *q_gyro, double dt);
#endif
