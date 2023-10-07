#include "attitude.h"
// #include <math.h>

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler)
{
	Vec3Norm(acc);
	euler->x = -1.0*asinf(acc->y);		// roll (rad)
	euler->y = atan2f(acc->x, acc->z);	// pitch
}

void GyroUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, double dt)
{
	// Runge-Kutta 4th order
	// paras : t0 : time at the beginning of the interval. (q0, gyro0)
	// 		   t1 : time at the end of the interval, t1 = t0+dt. (q1, gyro1)
	double M0[4][4] = {
		{0, -gyro0->x, -gyro0->y, -gyro0->z},
		{gyro0->x, 0, gyro0->z, -gyro0->y},
		{gyro0->y, -gyro0->z, 0, gyro0->x},
		{gyro0->z, gyro0->y, -gyro0->x, 0}
	};

	double M1[4][4] = {
		{0, -gyro1->x, -gyro1->y, -gyro1->z},
		{gyro1->x, 0, gyro1->z, -gyro1->y},
		{gyro1->y, -gyro1->z, 0, gyro1->x},
		{gyro1->z, gyro1->y, -gyro1->x, 0}
	};

	double M0_5[4][4];
	for(uint8_t i=0; i<4; i++) // M0_5 = (M0 + M1)/2
	{
		for(uint8_t j=0; j<4; j++)
		{
			M0_5[i][j] = (M0[i][j] + M1[i][j])/2;
		}
	}

	// transfer quaternion to matrix
	double  q0_matrix[4] = {q0->w, q0->x, q0->y, q0->z},
			q1_matrix[4] = {q1->w, q1->x, q1->y, q1->z};

	double k1[4], k2[4], k3[4], k4[4];
	double tmp1[4], tmp2[4], tmp3[4], tmp4[4];

	// k1 = (1/2)*M0*q0
	MatrixesMultiply(&tmp1, &M0, q0_matrix, 4, 4, 1);
	MatrixScale(&k1, &tmp1, 0.5, 4, 1);

	// k2 = (1/2)*M0_5*(q0+(1/2)*k1*dt)
	MatrixScale(&tmp1, &k1, dt/2, 4, 1);
	MatrixAdd(&tmp2, q0_matrix, &tmp1, 4, 1);
	MatrixesMultiply(&tmp1, &M0_5, &tmp2, 4, 4, 1);
	MatrixScale(&k2, &tmp1, 0.5, 4, 1);

	// k3 = (1/2)*M0_5*(q0+(1/2)*k2*dt)
	MatrixScale(&tmp1, &k2, dt/2, 4, 1);
	MatrixAdd(&tmp2, q0_matrix, &tmp1, 4, 1);
	MatrixesMultiply(&tmp1, &M0_5, &tmp2, 4, 4, 1);
	MatrixScale(&k3, &tmp1, 0.5, 4, 1);

	// k4 = (1/2)*M1*(q0+k3*dt)
	MatrixScale(&tmp1, &k3, dt, 4, 1);
	MatrixAdd(&tmp2, q0_matrix, &tmp1, 4, 1);
	MatrixesMultiply(&tmp1, &M1, &tmp2, 4, 4, 1);
	MatrixScale(&k4, &tmp1, 0.5, 4, 1);

	// q1 = q0 + (k1+2*k2+2*k3+k4)*dt/6
	MatrixScale(&tmp1, &k2, 2, 4, 1);
	MatrixScale(&tmp2, &k3, 2, 4, 1);
	MatrixAdd(&tmp3, &tmp1, &tmp2, 4, 1);
	MatrixAdd(&tmp4, &k1, &k4, 4, 1);
	MatrixAdd(&tmp1, &tmp3, &tmp4, 4, 1);
	MatrixScale(&tmp2, &tmp1, dt/6, 4, 1);
	MatrixAdd(q1_matrix, q0_matrix, &tmp2, 4, 1);

	// return q1
	q1->w = q1_matrix[0];
	q1->x = q1_matrix[1];
	q1->y = q1_matrix[2];
	q1->z = q1_matrix[3];

}