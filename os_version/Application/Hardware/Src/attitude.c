#include "attitude.h"
#include "gy86.h"
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

void AccMagUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, Vec3d_t *mag, Vec3d_t *mag0, double dt)
{
	// Gradient Decent Method

	// compute gradient of loss function
	// mag0->x = 0.0;
	// mag0->y = 0.0;
	// mag0->z = 0.0;
	// mag->x = 0;
	// mag->y = 0;
	// mag->z = 0;
	double f[6];
	// 2*(q1*q3-q0*q2)-ax
	f[0] = 2*(q0->x*q0->z-q0->w*q0->y)-acc->x;
	// 2*(q0*q1+q2*q3)-ay
	f[1] = 2*(q0->w*q0->x+q0->y*q0->z)-acc->y;
	// 2*(0.5-q1*q1-q2*q2)-az
	f[2] = 2*(0.5-q0->x*q0->x-q0->y*q0->y)-acc->z;
	// 2*bx*(0.5-q2*q2-q3*q3)+2*bz*(q1*q3-q0*q2)-mx
	f[3] = 2*mag0->x*(0.5-q0->y*q0->y-q0->z*q0->z)+2*mag0->z*(q0->x*q0->z-q0->w*q0->y)-mag->x;
	// 2*bx*(q1*q2-q0*q3)+2*bz*(q0*q1+q2*q3)-my
	f[4] = 2*mag0->x*(q0->x*q0->y-q0->w*q0->z)+2*mag0->z*(q0->w*q0->x+q0->y*q0->z)-mag->y;
	// 2*bx*(q0*q2+q1*q3)+2*bz*(0.5-q1*q1-q2*q2)-mz
	f[5] = 2*mag0->x*(q0->w*q0->y+q0->x*q0->z)+2*mag0->z*(0.5-q0->x*q0->x-q0->y*q0->y)-mag->z;

	Vec4d_t q_grad;
	// -2*q2	2*q1	0		-2*bz*q2			-2*bx*q3+2*bz*q1	2*bx*q2
	q_grad.w = -2*q0->y*f[0] + 2*q0->x*f[1] +0 -2*mag0->z*q0->y*f[3] + (-2*mag0->x*q0->z+2*mag0->z*q0->x)*f[4] + 2*mag0->x*q0->y*f[5];
	// 2*q3		2*q0	-4*q1	2*bz*q3				2*bx*q2+2*bz*q0		2*bx*q3-4*bz*q1
	q_grad.x = 2*q0->z*f[0] + 2*q0->w*f[1] -4*q0->x*f[2] + 2*mag0->z*q0->z*f[3] + (2*mag0->x*q0->y+2*mag0->z*q0->w)*f[4] + (2*mag0->x*q0->z-4*mag0->z*q0->x)*f[5];
	// -2*q0	2*q3	-4*q2	-4*bx*q2-2*bz*q0	2*bx*q1+2*bz*q3		2*bx*q0-4*bz*q2
	q_grad.y = -2*q0->w*f[0] + 2*q0->z*f[1] -4*q0->y*f[2] +(-4*mag0->x*q0->y-2*mag0->z*q0->w)*f[3] + (2*mag0->x*q0->x+2*mag0->z*q0->w)*f[4] + (2*mag0->x*q0->w-4*mag0->z*q0->y)*f[5];
	// 2*q1		2*q2	0		-4*bx*q3+2*bz*q1	-2*bx*q0+2*bz*q2	2*bx*q1
	q_grad.z = 2*q0->x*f[0] + 2*q0->y*f[1] +0 +(-4*mag0->x*q0->z+2*mag0->z*q0->x)*f[3] + (-2*mag0->x*q0->w+2*mag0->z*q0->y)*f[4] + 2*mag0->x*q0->x*f[5];
	double q_grad_mod = Vec4Modulus(q_grad);
	double mu0=0.05, alpha=3;
	mu = mu0+alpha*dt*Vec3Modulus(*gyro);
	q1->w = q0->w - mu*q_grad.w/q_grad_mod;
	q1->x = q0->x - mu*q_grad.x/q_grad_mod;
	q1->y = q0->y - mu*q_grad.y/q_grad_mod;
	q1->z = q0->z - mu*q_grad.z/q_grad_mod;

	Vec4Norm(q1);
}

void AccMagUpdateQuatDelta(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, Vec3d_t *mag, Vec3d_t *mag0, double dt)
{
	// Gradient Decent Method

	// compute gradient of loss function
	// mag0->x = 0.0;
	// mag0->y = 0.0;
	// mag0->z = 0.0;
	// mag->x = 0;
	// mag->y = 0;
	// mag->z = 0;
	double f[6];
	// 2*(q1*q3-q0*q2)-ax
	f[0] = 2*(q0->x*q0->z-q0->w*q0->y)-acc->x;
	// 2*(q0*q1+q2*q3)-ay
	f[1] = 2*(q0->w*q0->x+q0->y*q0->z)-acc->y;
	// 2*(0.5-q1*q1-q2*q2)-az
	f[2] = 2*(0.5-q0->x*q0->x-q0->y*q0->y)-acc->z;
	// 2*bx*(0.5-q2*q2-q3*q3)+2*bz*(q1*q3-q0*q2)-mx
	f[3] = 2*mag0->x*(0.5-q0->y*q0->y-q0->z*q0->z)+2*mag0->z*(q0->x*q0->z-q0->w*q0->y)-mag->x;
	// 2*bx*(q1*q2-q0*q3)+2*bz*(q0*q1+q2*q3)-my
	f[4] = 2*mag0->x*(q0->x*q0->y-q0->w*q0->z)+2*mag0->z*(q0->w*q0->x+q0->y*q0->z)-mag->y;
	// 2*bx*(q0*q2+q1*q3)+2*bz*(0.5-q1*q1-q2*q2)-mz
	f[5] = 2*mag0->x*(q0->w*q0->y+q0->x*q0->z)+2*mag0->z*(0.5-q0->x*q0->x-q0->y*q0->y)-mag->z;

	Vec4d_t q_grad;
	// -2*q2	2*q1	0		-2*bz*q2			-2*bx*q3+2*bz*q1	2*bx*q2
	q_grad.w = -2*q0->y*f[0] + 2*q0->x*f[1] +0 -2*mag0->z*q0->y*f[3] + (-2*mag0->x*q0->z+2*mag0->z*q0->x)*f[4] + 2*mag0->x*q0->y*f[5];
	// 2*q3		2*q0	-4*q1	2*bz*q3				2*bx*q2+2*bz*q0		2*bx*q3-4*bz*q1
	q_grad.x = 2*q0->z*f[0] + 2*q0->w*f[1] -4*q0->x*f[2] + 2*mag0->z*q0->z*f[3] + (2*mag0->x*q0->y+2*mag0->z*q0->w)*f[4] + (2*mag0->x*q0->z-4*mag0->z*q0->x)*f[5];
	// -2*q0	2*q3	-4*q2	-4*bx*q2-2*bz*q0	2*bx*q1+2*bz*q3		2*bx*q0-4*bz*q2
	q_grad.y = -2*q0->w*f[0] + 2*q0->z*f[1] -4*q0->y*f[2] +(-4*mag0->x*q0->y-2*mag0->z*q0->w)*f[3] + (2*mag0->x*q0->x+2*mag0->z*q0->w)*f[4] + (2*mag0->x*q0->w-4*mag0->z*q0->y)*f[5];
	// 2*q1		2*q2	0		-4*bx*q3+2*bz*q1	-2*bx*q0+2*bz*q2	2*bx*q1
	q_grad.z = 2*q0->x*f[0] + 2*q0->y*f[1] +0 +(-4*mag0->x*q0->z+2*mag0->z*q0->x)*f[3] + (-2*mag0->x*q0->w+2*mag0->z*q0->y)*f[4] + 2*mag0->x*q0->x*f[5];
	double q_grad_mod = Vec4Modulus(q_grad);
	double mu0=0.05, alpha=3;
	mu = mu0+alpha*dt*Vec3Modulus(*gyro);
	q1->w = q_grad.w/q_grad_mod;
	q1->x = q_grad.x/q_grad_mod;
	q1->y = q_grad.y/q_grad_mod;
	q1->z = q_grad.z/q_grad_mod;

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
void GyroUpdateQuatDelta(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, double dt)
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

	// return q1
	q1->w = q1_matrix[0];
	q1->x = q1_matrix[1];
	q1->y = q1_matrix[2];
	q1->z = q1_matrix[3];
}

void MadgwickAHRS(Vec4d_t *q0, Vec3d_t *mag0, double dt)
{
	Vec4d_t q1, delta_q_acc, delta_q_gyro;
	Vec3d_t gyro0 = {0, 0, 0}, gyro1, acc, mag, euler;
	uint32_t cnt = 0, t[10];
	t[0] = OSTimeGet();

	GetGyroData(&gyro1);
	GetAccData(&acc);
	GetMagData(&mag);
	t[1] = OSTimeGet();	// takes 2~3 ticks, while the reset part < 1 tick

	AccMagUpdateQuatDelta(&q0, &delta_q_acc, &acc, &gyro1, &mag, mag0, 0.001);
	t[3] = OSTimeGet();

	GyroUpdateQuatDelta(&q0, &delta_q_gyro, &gyro0, &gyro1, 0.001);
	Vec4Norm(&delta_q_gyro);
	gyro0 = gyro1;
	t[2] = OSTimeGet();
	
	double beta = 0.01;
	q0->w = q0->w + delta_q_gyro.w + beta*delta_q_acc.w;
	q0->x = q0->x + delta_q_gyro.x + beta*delta_q_acc.x;
	q0->y = q0->y + delta_q_gyro.y + beta*delta_q_acc.y;
	q0->z = q0->z + delta_q_gyro.z + beta*delta_q_acc.z;

	t[4] = OSTimeGet();
}