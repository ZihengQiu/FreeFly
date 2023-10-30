#ifndef __MATHKIT_H
#define __MATHKIT_H

#include "stm32f4xx.h"                   // Device header

#include <math.h>

#define PI 3.1415926535897932384626433832795

typedef struct
{
	float x, y, z;
}vec3d_t;

typedef struct
{
	float w, x, y, z;
}vec4d_t;

float Vec3Modulus(vec3d_t vec);
float Vec4Modulus(vec4d_t vec);
void Vec3Norm(vec3d_t *vec);
void Vec4Norm(vec4d_t *vec);
void RadToDeg(vec3d_t *rad);
void DegToRad(vec3d_t *deg);
void QuaterToEuler(vec4d_t *q, vec3d_t *euler);

void MatrixTranspose(float *MatrixB, float *MatrixA, uint8_t row, uint8_t col);
void MatrixInverse(float *MatrixB, float *MatrixA, uint8_t row);
void MatrixAdd(float *MatrixC, float *MatrixA, float *MatrixB, uint8_t row, uint8_t col);
void MatrixScale(float *MatrixB, float *MatrixA, float scale, uint8_t row, uint8_t col);
void MatrixesMultiply(float *MatrixC, float *MatrixA, float *MatrixB, uint8_t rowA, uint8_t colA, uint8_t colB);

void GaussNewton_LM(vec3d_t input[6], vec3d_t* offset, vec3d_t* scale);

#endif
