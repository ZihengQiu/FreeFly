#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f4xx.h"                   // Device header

#include <math.h>

#include "includes.h"
#include "mathkit.h"
#include "gy86.h"

extern vec3d_t acc, mag, gyro, euler;

void AccToEuler(vec3d_t *acc, vec3d_t *euler);

void AccUpdateQuat(vec4d_t *q0, vec4d_t *q1, vec3d_t *acc, vec3d_t *gyro, float dt);
void AccMagUpdateQuat(vec4d_t *q0, vec4d_t *q1, vec3d_t *acc, vec3d_t *gyro, vec3d_t *mag, float dt);
void GyroUpdateQuat(vec4d_t *q0, vec4d_t *q1, vec3d_t *gyro0, vec3d_t *gyro1, float dt);
void MadgwickAHRS(vec4d_t *q0, vec3d_t acc, vec3d_t gyro, vec3d_t mag);
#endif
