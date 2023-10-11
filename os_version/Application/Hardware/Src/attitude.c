#include "attitude.h"
#include "mathkit.h"
#include <math.h>

double mu = 0;

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler)
{
	Vec3Norm(acc);
	euler->x = -1.0*asinf(acc->y);		// roll (rad)
	euler->y = atan2f(acc->x, acc->z);	// pitch
}

void AccUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, double dt)
{
	// Gradient Decent Method

	// compute gradient of loss function
	Vec4d_t q_grad;
	// 8*q2*q2*q0+8*q1*q1*q0+4*ax*q2-4*ay*q1
	q_grad.w = 8*q0->y*q0->y + 8*q0->x*q0->x + 4*acc->x*q0->y - 4*acc->y*q0->x;
	// 8*q3*q3*q1+8*q0*q0*q1+16*q2*q2*q1+16*q3*q3-4*ax*q3-4*ay*q0+8*az*q1-8*q1
	q_grad.x = 8*q0->z*q0->z + 8*q0->w*q0->w*q0->x + 16*q0->y*q0->y*q0->x + 16*q0->z*q0->z - 4*acc->x*q0->z - 4*acc->y*q0->w + 8*acc->z*q0->x - 8*q0->x;
	// 8*q0*q0*q2+4*ax*q0+8*q3*q3*q2-4*ay*q3-8*q2+16*q1*q1*q2+16*q2*q2*q2+8*az*q2
	q_grad.y = 8*q0->w*q0->w*q0->y + 4*acc->x*q0->w + 8*q0->z*q0->z*q0->y - 4*acc->y*q0->z - 8*q0->y + 16*q0->x*q0->x*q0->y + 16*q0->y*q0->y*q0->y + 8*acc->z*q0->y;
	// 8*q1*q1*q3+8*q2*q2*q3-4*ax*q1-4*ay*q2
	q_grad.z = 8*q0->x*q0->x*q0->z + 8*q0->y*q0->y*q0->z - 4*acc->x*q0->x - 4*acc->y*q0->y;

	double q_grad_mod = Vec4Modulus(q_grad);
	double mu0=0.05, alpha=3000;
	mu = mu0+alpha*dt*Vec3Modulus(*gyro);
	q1->w = q0->w - mu*q_grad.w/q_grad_mod;
	q1->x = q0->x - mu*q_grad.x/q_grad_mod;
	q1->y = q0->y - mu*q_grad.y/q_grad_mod;
	q1->z = q0->z - mu*q_grad.z/q_grad_mod;

	Vec4Norm(q1);
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
	double  q0_matrix[4] = {q0->w, q0->x, q0->y, q0->z}, q1_matrix[4];

	double k1[4], k2[4], k3[4], k4[4];
	double tmp1[4], tmp2[4], tmp3[4], tmp4[4];

	// k1 = (1/2)*M0*q0
	MatrixesMultiply((double *)tmp1, (double *)M0, q0_matrix, 4, 4, 1);
	MatrixScale((double *)k1, (double *)tmp1, 0.5, 4, 1);

	// k2 = (1/2)*M0_5*(q0+(1/2)*k1*dt)
	MatrixScale((double *)tmp1, (double *)k1, dt/2, 4, 1);
	MatrixAdd((double *)tmp2, q0_matrix, (double *)tmp1, 4, 1);
	MatrixesMultiply((double *)tmp1, (double *)M0_5, (double *)tmp2, 4, 4, 1);
	MatrixScale((double *)k2, (double *)tmp1, 0.5, 4, 1);

	// k3 = (1/2)*M0_5*(q0+(1/2)*k2*dt)
	MatrixScale((double *)tmp1, (double *)k2, dt/2, 4, 1);
	MatrixAdd((double *)tmp2, q0_matrix, (double *)tmp1, 4, 1);
	MatrixesMultiply((double *)tmp1, (double *)M0_5, (double *)tmp2, 4, 4, 1);
	MatrixScale((double *)k3, (double *)tmp1, 0.5, 4, 1);

	// k4 = (1/2)*M1*(q0+k3*dt)
	MatrixScale((double *)tmp1, (double *)k3, dt, 4, 1);
	MatrixAdd((double *)tmp2, q0_matrix, (double *)tmp1, 4, 1);
	MatrixesMultiply((double *)tmp1, (double *)M1, (double *)tmp2, 4, 4, 1);
	MatrixScale((double *)k4, (double *)tmp1, 0.5, 4, 1);

	// q1 = q0 + (k1+2*k2+2*k3+k4)*dt/6
	MatrixScale((double *)tmp1, (double *)k2, 2, 4, 1);
	MatrixScale((double *)tmp2, (double *)k3, 2, 4, 1);
	MatrixAdd((double *)tmp3, (double *)tmp1, (double *)tmp2, 4, 1);
	MatrixAdd((double *)tmp4, (double *)k1, (double *)k4, 4, 1);
	MatrixAdd((double *)tmp1, (double *)tmp3, (double *)tmp4, 4, 1);
	MatrixScale((double *)tmp2, (double *)tmp1, dt/6, 4, 1);
	MatrixAdd(q1_matrix, q0_matrix, (double *)tmp2, 4, 1);

	// return q1
	q1->w = q1_matrix[0];
	q1->x = q1_matrix[1];
	q1->y = q1_matrix[2];
	q1->z = q1_matrix[3];
}

void MadgwickAHRS(Vec4d_t *q, Vec4d_t *q_acc, Vec4d_t *q_gyro, double dt)
{
	double beta = 5000;
	double error = beta*dt+mu;
	double alpha1 = 1-beta*dt/error,
		   alpha2 = 1-mu/error;
	q->w = alpha1*q_gyro->w + alpha2*q_acc->w;
	q->x = alpha1*q_gyro->x + alpha2*q_acc->x;
	q->y = alpha1*q_gyro->y + alpha2*q_acc->y;
	q->z = alpha1*q_gyro->z + alpha2*q_acc->z;
}