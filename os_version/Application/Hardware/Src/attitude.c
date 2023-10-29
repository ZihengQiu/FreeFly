#include "attitude.h"
#include "gy86.h"
#include "mathkit.h"
#include "ucos_ii.h"
#include <math.h>

float mu = 0;

void AccToEuler(Vec3d_t *acc, Vec3d_t *euler)
{
	Vec3Norm(acc);
	euler->x = -1.0*asinf(acc->y);		// roll (rad)
	euler->y = atan2f(acc->x, acc->z);	// pitch
}

void AccUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, float dt)
{
	// Gradient Decent Method

	// compute gradient of loss function
	Vec4d_t q_grad;
	// 4*q2*q2*q0+4*q1*q1*q0+2*ax*q2-2*ay*q1
	q_grad.w = 4*q0->y*q0->y + 4*q0->x*q0->x + 2*acc->x*q0->y - 2*acc->y*q0->x;
	// 4*q3*q3*q1+4*q0*q0*q1+8*q2*q2*q1+8*q1*q1*q1-2*ax*q3-2*ay*q0+4*az*q1-4*q1
	q_grad.x = 4*q0->z*q0->z + 4*q0->w*q0->w*q0->x + 8*q0->y*q0->y*q0->x + 8*q0->x*q0->x*q0->x - 2*acc->x*q0->z - 2*acc->y*q0->w + 4*acc->z*q0->x - 4*q0->x;
	// 4*q0*q0*q2+2*ax*q0+4*q3*q3*q2-2*ay*q3-4*q2+8*q1*q1*q2+8*q2*q2*q2+4*az*q2
	q_grad.y = 4*q0->w*q0->w*q0->y + 2*acc->x*q0->w + 4*q0->z*q0->z*q0->y - 2*acc->y*q0->z - 4*q0->y + 8*q0->x*q0->x*q0->y + 8*q0->y*q0->y*q0->y + 4*acc->z*q0->y;
	// 4*q1*q1*q3+4*q2*q2*q3-2*ax*q1-2*ay*q2
	q_grad.z = 4*q0->x*q0->x*q0->z + 4*q0->y*q0->y*q0->z - 2*acc->x*q0->x - 2*acc->y*q0->y;

	float q_grad_mod = Vec4Modulus(q_grad);
	float mu0=0.05, alpha=0.1;
	mu = mu0+alpha*dt*Vec3Modulus(*gyro);
	q1->w = q0->w - mu*q_grad.w/q_grad_mod;
	q1->x = q0->x - mu*q_grad.x/q_grad_mod;
	q1->y = q0->y - mu*q_grad.y/q_grad_mod;
	q1->z = q0->z - mu*q_grad.z/q_grad_mod;

	Vec4Norm(q1);
}

void AccMagUpdateQuatDelta(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, Vec3d_t *mag)
{	
	// Gradient Decent Method

#define FUSION_MAGNETIC 1

	// compute gradient of loss function

#if FUSION_MAGNETIC

	float f[6];
	Vec3d_t h, b;

	h.x = 2*(0.5-q0->y*q0->y-q0->z*q0->z)*mag->x \
		+ 2*(q0->x*q0->y-q0->w*q0->z)*mag->y \
		+ 2*(q0->w*q0->y+q0->x*q0->z)*mag->z;
	h.y = 2*(q0->x*q0->y+q0->w*q0->z)*mag->x \
		+ 2*(0.5-q0->x*q0->x-q0->z*q0->z)*mag->y \
		+ 2*(q0->y*q0->z-q0->w*q0->x)*mag->z;
	h.z = 2*(q0->x*q0->z-q0->w*q0->y)*mag->x \
		+ 2*(q0->w*q0->x+q0->y*q0->z)*mag->y \
		+ 2*(0.5-q0->x*q0->x-q0->y*q0->y)*mag->z;
	
	h.x = 2*(0.5-q0->y*q0->y-q0->z*q0->z)*mag->x \
		+ 2*(q0->x*q0->y-q0->w*q0->z)*mag->y \
		+ 2*(q0->w*q0->y+q0->x*q0->z)*mag->z;
	h.y = 2*(q0->x*q0->y+q0->w*q0->z)*mag->x \
		+ 2*(0.5-q0->x*q0->x-q0->z*q0->z)*mag->y \
		+ 2*(q0->y*q0->z-q0->w*q0->x)*mag->z;
	h.z = 2*(q0->x*q0->z-q0->w*q0->y)*mag->x \
		+ 2*(q0->w*q0->x+q0->y*q0->z)*mag->y \
		+ 2*(0.5-q0->x*q0->x-q0->y*q0->y)*mag->z;
	b.x = sqrt(h.x*h.x+h.y*h.y);
	b.y = 0;
	b.z = h.z;

	// printf("bx: %10f, %10f\n", b.x, b.z);

	// 2*(q1*q3-q0*q2)-ax
	f[0] = 2*(q0->x*q0->z-q0->w*q0->y)-acc->x;
	// 2*(q0*q1+q2*q3)-ay
	f[1] = 2*(q0->w*q0->x+q0->y*q0->z)-acc->y;
	// 2*(0.5-q1*q1-q2*q2)-az
	f[2] = 2*(0.5-q0->x*q0->x-q0->y*q0->y)-acc->z;
	// 2*bx*(0.5-q2*q2-q3*q3)+2*bz*(q1*q3-q0*q2)-mx
	f[3] = 2*b.x*(0.5-q0->y*q0->y-q0->z*q0->z) + 2*b.z*(q0->x*q0->z-q0->w*q0->y)-mag->x;
	// 2*bx*(q1*q2-q0*q3)+2*bz*(q0*q1+q2*q3)-my
	f[4] = 2*b.x*(q0->x*q0->y-q0->w*q0->z) + 2*b.z*(q0->w*q0->x+q0->y*q0->z)-mag->y;
	// 2*bx*(q0*q2+q1*q3)+2*bz*(0.5-q1*q1-q2*q2)-mz
	f[5] = 2*b.x*(q0->w*q0->y+q0->x*q0->z) + 2*b.z*(0.5-q0->x*q0->x-q0->y*q0->y)-mag->z;

	Vec4d_t q_grad;
	// -2*q2	2*q1	0		-2*bz*q2			-2*bx*q3+2*bz*q1	2*bx*q2
	q_grad.w = -2*q0->y*f[0] + 2*q0->x*f[1] +0 -2*b.z*q0->y*f[3] + (-2*b.x*q0->z+2*b.z*q0->x)*f[4] + 2*b.x*q0->y*f[5];
	// 2*q3		2*q0	-4*q1	2*bz*q3				2*bx*q2+2*bz*q0		2*bx*q3-4*bz*q1
	q_grad.x = 2*q0->z*f[0] + 2*q0->w*f[1] -4*q0->x*f[2] + 2*b.z*q0->z*f[3] + (2*b.x*q0->y+2*b.z*q0->w)*f[4] + (2*b.x*q0->z-4*b.z*q0->x)*f[5];
	// -2*q0	2*q3	-4*q2	-4*bx*q2-2*bz*q0	2*bx*q1+2*bz*q3		2*bx*q0-4*bz*q2
	q_grad.y = -2*q0->w*f[0] + 2*q0->z*f[1] -4*q0->y*f[2] +(-4*b.x*q0->y-2*b.z*q0->w)*f[3] + (2*b.x*q0->x+2*b.z*q0->z)*f[4] + (2*b.x*q0->w-4*b.z*q0->y)*f[5];
	// 2*q1		2*q2	0		-4*bx*q3+2*bz*q1	-2*bx*q0+2*bz*q2	2*bx*q1
	q_grad.z = 2*q0->x*f[0] + 2*q0->y*f[1] +0 +(-4*b.x*q0->z+2*b.z*q0->x)*f[3] + (-2*b.x*q0->w+2*b.z*q0->y)*f[4] + 2*b.x*q0->x*f[5];

#else

	Vec4d_t q_grad;
	// 8*q2*q2*q0+8*q1*q1*q0+4*ax*q2-4*ay*q1
	q_grad.w = 8*q0->y*q0->y + 8*q0->x*q0->x + 4*acc->x*q0->y - 4*acc->y*q0->x;
	// 8*q3*q3*q1+8*q0*q0*q1+16*q2*q2*q1+16*q3*q3-4*ax*q3-4*ay*q0+8*az*q1-8*q1
	q_grad.x = 8*q0->z*q0->z + 8*q0->w*q0->w*q0->x + 16*q0->y*q0->y*q0->x + 16*q0->z*q0->z - 4*acc->x*q0->z - 4*acc->y*q0->w + 8*acc->z*q0->x - 8*q0->x;
	// 8*q0*q0*q2+4*ax*q0+8*q3*q3*q2-4*ay*q3-8*q2+16*q1*q1*q2+16*q2*q2*q2+8*az*q2
	q_grad.y = 8*q0->w*q0->w*q0->y + 4*acc->x*q0->w + 8*q0->z*q0->z*q0->y - 4*acc->y*q0->z - 8*q0->y + 16*q0->x*q0->x*q0->y + 16*q0->y*q0->y*q0->y + 8*acc->z*q0->y;
	// 8*q1*q1*q3+8*q2*q2*q3-4*ax*q1-4*ay*q2
	q_grad.z = 8*q0->x*q0->x*q0->z + 8*q0->y*q0->y*q0->z - 4*acc->x*q0->x - 4*acc->y*q0->y;

#endif

	float q_grad_mod = Vec4Modulus(q_grad); 
	
	// q1->w = q_grad.w/q_grad_mod;
	// q1->x = q_grad.x/q_grad_mod;
	// q1->y = q_grad.y/q_grad_mod;
	// q1->z = q_grad.z/q_grad_mod;
	q1->w = q_grad.w;
	q1->x = q_grad.x;
	q1->y = q_grad.y;
	q1->z = q_grad.z;

	// Vec4Norm(q1);
 }

void AccMagUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *acc, Vec3d_t *gyro, Vec3d_t *mag, float dt)
{
	AccMagUpdateQuatDelta(q0, q1, acc, gyro, mag);

	float mu0=0.3, alpha=0.01;
	mu = mu0+alpha*dt*Vec3Modulus(*gyro);
	q1->w = q0->w - mu*q1->w;
	q1->x = q0->x - mu*q1->x;
	q1->y = q0->y - mu*q1->y;
	q1->z = q0->z - mu*q1->z;

	Vec4Norm(q1);
}

void GyroUpdateQuatDelta(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, float dt)
{
	// Runge-Kutta 4th order
	// paras : t0 : time at the beginning of the interval. (q0, gyro0)
	// 		   t1 : time at the end of the interval, t1 = t0+dt. (q1, gyro1)

	float M0[4][4] = {
		{0, -gyro0->x, -gyro0->y, -gyro0->z},
		{gyro0->x, 0, gyro0->z, -gyro0->y},
		{gyro0->y, -gyro0->z, 0, gyro0->x},
		{gyro0->z, gyro0->y, -gyro0->x, 0}
	};

	float M1[4][4] = {
		{0, -gyro1->x, -gyro1->y, -gyro1->z},
		{gyro1->x, 0, gyro1->z, -gyro1->y},
		{gyro1->y, -gyro1->z, 0, gyro1->x},
		{gyro1->z, gyro1->y, -gyro1->x, 0}
	};

	float M0_5[4][4];
	for(uint8_t i=0; i<4; i++) // M0_5 = (M0 + M1)/2
	{
		for(uint8_t j=0; j<4; j++)
		{
			M0_5[i][j] = (M0[i][j] + M1[i][j])/2;
		}
	}

	// transfer quaternion to matrix
	float  q0_matrix[4] = {q0->w, q0->x, q0->y, q0->z}, q1_matrix[4];

	float k1[4], k2[4], k3[4], k4[4];
	float tmp1[4], tmp2[4], tmp3[4], tmp4[4];

	// k1 = (1/2)*M0*q0
	MatrixesMultiply((float *)tmp1, (float *)M0, q0_matrix, 4, 4, 1);
	MatrixScale((float *)k1, (float *)tmp1, 0.5, 4, 1);

	// k2 = (1/2)*M0_5*(q0+(1/2)*k1*dt)
	MatrixScale((float *)tmp1, (float *)k1, dt/2, 4, 1);
	MatrixAdd((float *)tmp2, q0_matrix, (float *)tmp1, 4, 1);
	MatrixesMultiply((float *)tmp1, (float *)M0_5, (float *)tmp2, 4, 4, 1);
	MatrixScale((float *)k2, (float *)tmp1, 0.5, 4, 1);

	// k3 = (1/2)*M0_5*(q0+(1/2)*k2*dt)
	MatrixScale((float *)tmp1, (float *)k2, dt/2, 4, 1);
	MatrixAdd((float *)tmp2, q0_matrix, (float *)tmp1, 4, 1);
	MatrixesMultiply((float *)tmp1, (float *)M0_5, (float *)tmp2, 4, 4, 1);
	MatrixScale((float *)k3, (float *)tmp1, 0.5, 4, 1);

	// k4 = (1/2)*M1*(q0+k3*dt)
	MatrixScale((float *)tmp1, (float *)k3, dt, 4, 1);
	MatrixAdd((float *)tmp2, q0_matrix, (float *)tmp1, 4, 1);
	MatrixesMultiply((float *)tmp1, (float *)M1, (float *)tmp2, 4, 4, 1);
	MatrixScale((float *)k4, (float *)tmp1, 0.5, 4, 1);

	// q1 = q0 + (k1+2*k2+2*k3+k4)*dt/6
	MatrixScale((float *)tmp1, (float *)k2, 2, 4, 1);
	MatrixScale((float *)tmp2, (float *)k3, 2, 4, 1);
	MatrixAdd((float *)tmp3, (float *)tmp1, (float *)tmp2, 4, 1);
	MatrixAdd((float *)tmp4, (float *)k1, (float *)k4, 4, 1);
	MatrixAdd((float *)tmp1, (float *)tmp3, (float *)tmp4, 4, 1);
	MatrixScale((float *)q1_matrix, (float *)tmp1, dt/6, 4, 1);

	// return q1
	q1->w = q1_matrix[0];
	q1->x = q1_matrix[1];
	q1->y = q1_matrix[2];
	q1->z = q1_matrix[3];
}

void GyroUpdateQuat(Vec4d_t *q0, Vec4d_t *q1, Vec3d_t *gyro0, Vec3d_t *gyro1, float dt)
{
	GyroUpdateQuatDelta(q0, q1, gyro0, gyro1, dt);
	
	q1->w += q0->w;
	q1->x += q0->x;
	q1->y += q0->y;
	q1->z += q0->z;

	Vec4Norm(q1);
}

#define DT 0.0007

void MadgwickAHRS(Vec4d_t *q0, Vec3d_t acc, Vec3d_t gyro, Vec3d_t mag)
{
	Vec4d_t delta_q_acc, delta_q_gyro;

	AccMagUpdateQuatDelta(q0, &delta_q_acc, &acc, &gyro, &mag);

	delta_q_gyro.w = 0.5*(-q0->x*gyro.x-q0->y*gyro.y-q0->z*gyro.z)*DT;
	delta_q_gyro.x = 0.5*(q0->w*gyro.x+q0->y*gyro.z-q0->z*gyro.y)*DT;
	delta_q_gyro.y = 0.5*(q0->w*gyro.y-q0->x*gyro.z+q0->z*gyro.x)*DT;
	delta_q_gyro.z = 0.5*(q0->w*gyro.z+q0->x*gyro.y-q0->y*gyro.x)*DT;
	
	float beta = 0.033,
		  modulus_acc = Vec4Modulus(delta_q_acc),
		  lambda = 0.033/modulus_acc*200;

	if(modulus_acc > 0.1) // high variation of the drone
	{
		beta += 40;
	}

	lambda = (beta/modulus_acc+0.01)*0.5;

 	q0->w = q0->w + delta_q_gyro.w - lambda*delta_q_acc.w*DT;
	q0->x = q0->x + delta_q_gyro.x - lambda*delta_q_acc.x*DT;
	q0->y = q0->y + delta_q_gyro.y - lambda*delta_q_acc.y*DT;
	q0->z = q0->z + delta_q_gyro.z - lambda*delta_q_acc.z*DT;

	Vec4Norm(q0);
}