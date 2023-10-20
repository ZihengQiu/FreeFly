#include "stm32f4xx.h"                  // Device header
#include "mathkit.h"
#include "math.h"

double Vec3Modulus(Vec3d_t vec)
{
	return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

double Vec4Modulus(Vec4d_t vec)
{
	return sqrt(vec.w*vec.w + vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

void Vec3Norm(Vec3d_t *vec)
{
	double norm = Vec3Modulus(*vec);
	if(norm == 0)
	{
		return;
	}
	vec->x /= norm;
	vec->y /= norm;
	vec->z /= norm;
}


void Vec4Norm(Vec4d_t *vec)
{
	double norm = Vec4Modulus(*vec);
	if(norm == 0)
	{
		return;
	}
	vec->w /= norm;
	vec->x /= norm;
	vec->y /= norm;
	vec->z /= norm;
}

void RadToDeg(Vec3d_t *rad)
{
	rad->x *= 180.0/PI;
	rad->y *= 180.0/PI;
	rad->z *= 180.0/PI;
}

void DegToRad(Vec3d_t *deg)
{
	deg->x *= PI/180.0;
	deg->y *= PI/180.0;
	deg->z *= PI/180.0;
}

void QuaterToEuler(Vec4d_t *q, Vec3d_t *euler)
{
	euler->x = atan2f(2*(q->w*q->x+q->y*q->z), 1-2*(q->x*q->x+q->y*q->y));
	// euler->y = asinf(2*(q->w*q->y-q->z*q->x));
	euler->y = -PI/2+2*atan2f(sqrt(1+2*(q->w*q->y-q->z*q->x)), sqrt(1-2*(q->w*q->y-q->z*q->x)));
	euler->z = atan2f(2*(q->w*q->z+q->x*q->y), 1-2*(q->y*q->y+q->z*q->z));
	
    // roll  = -arcsin(2q1q3-2q0q2)
    // pitch = arctan((2q0q1+2q2q3)/(1-2q1q1-2q2q2))
    // yaw   = arctan((2q0q3-2q1q2)/(1-2q2q2-2q3q3))
	// euler->x = -1.0*asinf(2*q->x*q->z-2*q->w*q->y);
	// euler->y = atan2f(2*q->w*q->x+2*q->y*q->z, 1-2*q->x*q->x-2*q->y*q->y);
	// euler->z = atan2f(2*q->w*q->z+2*q->x*q->y, 1-2*q->y*q->y-2*q->z*q->z);
}

void MatrixTranspose(double *MatrixB, double *MatrixA, uint8_t row, uint8_t col)
{
	for(uint8_t i=0; i<row; i++)
	{
		for(uint8_t j=0; j<col; j++)
		{
			MatrixB[j*row+i] = MatrixA[i*col+j];
		}
	}
}

void MatrixInverse(double *MatrixB, double *MatrixA, uint8_t row)
{
	// use LU decomposition to get inverse matrix
	// MatrixA = L * U
	// MatrixB = U^-1 * L^-1
	double MatrixL[row][row], MatrixU[row][row];
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<row; j++)
		{
			MatrixL[i][j] = 0;
			MatrixU[i][j] = 0;
		}
	}
	// get matrix_l 
	for(int i=0; i<row; i++)
	{
		MatrixL[i][i] = 1;
	}

	for(int i=0; i<row; i++)
	{
		for(int j=i; j<row; j++)
		{
			double sum = 0;
			for(int k=0; k<i; k++)
			{
				sum += MatrixL[i][k]*MatrixU[k][j];
			}
			MatrixU[i][j] = MatrixA[i*row+j] - sum;
		}
		for(int j=i+1; j<row; j++)
		{
			double sum = 0;
			for(int k=0; k<i; k++)
			{
				sum += MatrixL[j][k]*MatrixU[k][i];
			}
			MatrixL[j][i] = (MatrixA[j*row+i] - sum)/MatrixU[i][i];
		}
	}

	// get matrix_l^-1
	double MatrixLInverse[row][row];
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<row; j++)
		{
			MatrixLInverse[i][j] = 0;
		}
	}

	for(int i=0; i<row; i++)
	{
		MatrixLInverse[i][i] = 1;
	}

	for(int i=1; i<row; i++)
	{
		for(int j=0; j<i; j++)
		{
			double sum = 0;
			for(int k=0; k<i; k++)
			{
				sum += MatrixL[i][k]*MatrixLInverse[k][j];
			}
			MatrixLInverse[i][j] = -1*sum;
		}
	}

	// get matrix_u^-1
	double MatrixUInverse[row][row];
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<row; j++)
		{
			MatrixUInverse[i][j] = 0;
		}
	}

	for(int i=0; i<row; i++)
	{
		MatrixUInverse[i][i] = 1/MatrixU[i][i];
	}

	for(int i=1; i<row; i++)
	{
		for(int j=i-1; j>=0; j--)
		{
			double sum = 0;
			for(int k=j+1; k<=i; k++)
			{
				sum += MatrixU[j][k]*MatrixUInverse[k][i];
			}
			MatrixUInverse[j][i] = -1*sum/MatrixU[j][j];
		}
	}

	// get matrix_b
	for(int i=0; i<row; i++)
	{
		for(int j=0; j<row; j++)
		{
			MatrixB[i*row+j] = 0;
			for(int k=0; k<row; k++)
			{
				MatrixB[i*row+j] += MatrixUInverse[i][k]*MatrixLInverse[k][j];
			}
		}
	}
}

void MatrixAdd(double *MatrixC, double *MatrixA, double *MatrixB, uint8_t row, uint8_t col)
{
	for(uint8_t i=0; i<row; i++)
	{
		for(uint8_t j=0; j<col; j++)
		{
			MatrixC[i*col+j] = MatrixA[i*col+j] + MatrixB[i*col+j];
		}
	}
}

void MatrixScale(double *MatrixB, double *MatrixA, double scale, uint8_t row, uint8_t col)
{
	for(uint8_t i=0; i<row; i++)
	{
		for(uint8_t j=0; j<col; j++)
		{
			MatrixB[i*col+j] = MatrixA[i*col+j] * scale;
		}
	}
}

void MatrixesMultiply(double *MatrixC, double *MatrixA, double *MatrixB, uint8_t rowA, uint8_t colA, uint8_t colB)
{
	for(uint8_t i=0; i<rowA; i++)
	{
		for(uint8_t j=0; j<colB; j++)
		{
			MatrixC[i*colB+j] = 0;
			for(uint8_t k=0; k<colA; k++)
			{
				// MatrixC[i][j] += MatrixA[i][k] * MatrixB[k][j];
				MatrixC[i*colB+j] += MatrixA[i*colA+k] * MatrixB[k*colB+j];
			}
		}
	}
}

void GaussNewton_LM(Vec3d_t input[6], Vec3d_t* offset, Vec3d_t* scale)
{
	uint8_t	cnt = 0;
	double eps = 0.000000001;
	double step = 100.0;
	double lambda = 0.001;
	double beta[6]; // the result, offset and scale of x, y, z
	double residual[6]; // residual
	double diff[6][3]; // difference between xi and xi_offset (e.t. xi- xi_offset)

	double Jacobian[6][6]; // Jacobian matrix
	double JT_J[6][6]; // J^T * J matrix
	double JT_R[6]; // J^T * R matrix

	// initialization
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1;

	while(cnt<100 && step>eps) // monitor iteration times and step length
	{
		// initialization
		cnt++;
		step = 0;

		for(uint8_t i=0; i<6; i++)
		{
			JT_R[i] = 0;

			for(uint8_t j=0; j<6; j++)
			{
				JT_J[i][j] = 0;
			}

			// calculate difference between xi and xi_offset
			diff[i][0] = input[i].x - beta[0];
			diff[i][1] = input[i].y - beta[1];
			diff[i][2] = input[i].z - beta[2];

			// calculate residual
			residual[i] = 1.0;
			for(uint8_t j=0; j<3; j++)
			{
				residual[i] = residual[i]-diff[i][j]*diff[i][j]*beta[3+j]*beta[3+j];
			}
		}

		// calculate Jacobian matrix
		for(uint8_t i=0; i<6; i++)
		{
			for(uint8_t j=0; j<3; j++)
			{
				Jacobian[i][j] = 2*beta[3+j]*beta[3+j]*diff[i][j];
				Jacobian[i][j+3] = -2*beta[3+j]*diff[i][j]*diff[i][j];
			}
		}

		// calculate J^T * J matrix and J^T * R matrix
		for(uint8_t i=0; i<6; i++)
		{
			JT_R[i] = 0;
			for(uint8_t j=0; j<6; j++)
			{
				// calculate J^T * J matrix
				JT_J[i][j] = 0;
				for(uint8_t k=0; k<6; k++)
				{
					JT_J[i][j] += Jacobian[k][i] * Jacobian[k][j];
				}

				// calculate J^T * R matrix
				JT_R[i] += Jacobian[j][i] * residual[j];
			}
		}

		// calculate J^T * J + lambda * I matrix
		for(int i=0; i<6; i++)
		{
			JT_J[i][i] += lambda;
		}

		// solve J^T*J * delta = J^T*R by GaussElimination

		// 1. make J^T*J to upper triangular matrix
		for(uint8_t i=0; i<6; i++) // i : row 
		{
			for(uint8_t j=i+1; j<6; j++) // j : row belows i, make ith column to 0
			{
				double quotient = JT_J[j][i] / JT_J[i][i];
				if(quotient)
				{
					// update JT_J
					JT_J[j][i] = 0;
					for(uint8_t k=i+1; k<6; k++)
					{
						JT_J[j][k] -= JT_J[i][k] * quotient;
					}

					// update JT_R
					JT_R[j] -= JT_R[i] * quotient;
				}
			}
		}

		// 2. make J^T*J to diagonal matrix
		for(int8_t i=5; i>=0; i--)
		{
			double quotient = JT_J[i][i];
			JT_J[i][i] = 1.0;

			// update ith row
			for(uint8_t j=i+1; j<6; j++)
			{
				JT_J[i][j] /= quotient;
			}
			JT_R[i] /= quotient;
			
			// update jth row
			for(uint8_t j=0; j<i; j++)
			{
				/*
						JT_J 	JT_R
						a    	b
					i: 	1    	c
				*/
				JT_R[j] -= JT_J[j][i]*JT_R[i];
				JT_J[j][i] = 0;
			}
		}

		// calculate step
		double step_cur = 0;
		for(uint8_t i=0; i<6; i++)
		{
			if(i < 3)
			{
				step_cur += JT_R[i]*JT_R[i];
			}
			else
			{
				step_cur += JT_R[i]*JT_R[i]/(beta[i]*beta[i]);
			}
			beta[i] -= JT_R[i];
		}

		// update lambda
		// lambda -> 0   : GaussNewton ( bigger step )
		// lambda -> inf : GradientDescent ( smaller step )
		if(step_cur > step)
		{
			lambda *= 2;
		}
		else
		{
			lambda /= 3;
		}
		
		step = step_cur;

	}
	
	// update result
	offset->x = beta[0];
	offset->y = beta[1];
	offset->z = beta[2];
	scale->x = beta[3];
	scale->y = beta[4];
	scale->z = beta[5];
}
