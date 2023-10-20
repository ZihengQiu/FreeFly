#ifndef __MATHKIT_H
#define __MATHKIT_H

#include "stm32f4xx.h"                   // Device header

#include <math.h>

#define PI 3.1415926535897932384626433832795

typedef struct
{
	float x, y, z;
}Vec3d_t;

typedef struct
{
	float w, x, y, z;
}Vec4d_t;

float Vec3Modulus(Vec3d_t vec);
float Vec4Modulus(Vec4d_t vec);
void Vec3Norm(Vec3d_t *vec);
void Vec4Norm(Vec4d_t *vec);
void RadToDeg(Vec3d_t *rad);
void DegToRad(Vec3d_t *deg);
void QuaterToEuler(Vec4d_t *q, Vec3d_t *euler);

void MatrixTranspose(float *MatrixB, float *MatrixA, uint8_t row, uint8_t col);
void MatrixInverse(float *MatrixB, float *MatrixA, uint8_t row);
void MatrixAdd(float *MatrixC, float *MatrixA, float *MatrixB, uint8_t row, uint8_t col);
void MatrixScale(float *MatrixB, float *MatrixA, float scale, uint8_t row, uint8_t col);
void MatrixesMultiply(float *MatrixC, float *MatrixA, float *MatrixB, uint8_t rowA, uint8_t colA, uint8_t colB);

void GaussNewton_LM(Vec3d_t input[6], Vec3d_t* offset, Vec3d_t* scale);

#endif
