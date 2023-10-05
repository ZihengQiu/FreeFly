#ifndef __MATH_H
#define __MATH_H

#include "stm32f4xx.h"                   // Device header

typedef struct
{
	double x, y, z;
}Vec3d_t;

typedef struct
{
	double w, x, y, z;
}Vec4d_t;

void MatrixAdd(double *MatrixC, double *MatrixA, double *MatrixB, uint8_t row, uint8_t col);
void MatrixScale(double *MatrixB, double *MatrixA, double scale, uint8_t row, uint8_t col);
void MatrixesMultiply(double *MatrixC, double *MatrixA, double *MatrixB, uint8_t rowA, uint8_t colA, uint8_t colB);

void GaussNewton_LM(Vec3d_t input[6], Vec3d_t* offset, Vec3d_t* scale);

#endif
