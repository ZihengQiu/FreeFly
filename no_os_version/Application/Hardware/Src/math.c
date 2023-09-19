#include "stm32f4xx.h"                  // Device header
#include "math.h"

void GaussNewton_official(Vec3d_t input[6], Vec3d_t offset, Vec3d_t scale, double length)
{
	uint8_t times = 0;
	double eps = 0.00000001;
	double change = 100.0;
	double data[3];
	double beta[6];
	double delta[6];
	double JtR[6];
	double JtJ[6][6];

	// Initialize
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1 / length;
	
	while(change > eps)
	{
		// Reset JtR and JtJ
		for(uint8_t i = 0; i < 6; i++)
		{
			JtR[i] = 0;
			for(uint8_t j = 0; j < 6; j++)
			{
				JtJ[i][j] = 0;
			}
		}

		// Calculate JtR and JtJ
		for(uint8_t i = 0; i < 6; i++)
		{
			data[0] = input[i].x;
			data[1] = input[i].y;
			data[2] = input[i].z;

			double dx, b, residual = 1.0, jacobian[6];
			for(uint8_t j = 0; j<3; j++)
			{
				b = beta[3+j];
				dx = data[j] - beta[j];
				residual -= b*b*dx*dx;
				jacobian[j] = 2*b*b*dx;
				jacobian[3+j] = -2*b*dx*dx;
			}

			for(uint8_t j = 0; j < 6; j++)
			{
				JtR[j] += jacobian[j] * residual;
				for(uint8_t k = 0; k < 6; k++)
				{
					JtJ[j][k] += jacobian[j] * jacobian[k];
				}
			}
		}

		// Gauss Elimination
		double mu;
		for(uint8_t i=0; i<6; i++)
		{
			for(uint8_t j=i+1; j<6; j++)
			{
				mu = JtJ[j][i] / JtJ[i][i];
				if(mu != 0x0f)
				{
					JtR[j] -= mu * JtR[i];
					for(uint8_t k=j; k<6; k++)
					{
						JtJ[k][j] -= mu * JtJ[k][i];
					}
				}
			}
		}

		for(uint8_t i=5; i>=0; i--)
		{
			JtR[i] /= JtJ[i][i];
			JtJ[i][i] = 1;

			for(uint8_t j=0; j<i; j++)
			{
				mu = JtJ[i][j];
				JtR[j] -= mu * JtR[i];
				JtJ[i][j] = 0;
			}
		}

		for(uint8_t i=0; i<6; i++)
		{
			delta[i] = JtR[i];
		}

		change = delta[0]*delta[0] +
				 delta[1]*delta[1] + 
				 delta[2]*delta[2] +
				 delta[3]*delta[3] / (beta[3]*beta[3]) +
				 delta[4]*delta[4] / (beta[4]*beta[4]) +
				 delta[5]*delta[5] / (beta[5]*beta[5]);
		
		for(uint8_t i=0; i<6; i++)
		{
			beta[i] += delta[i];
		}

		if(times++ > 100)
		{
			break;
		}
	}

	offset.x = beta[0];
	offset.y = beta[1];
	offset.z = beta[2];
	scale.x = beta[3];
	scale.y = beta[4];
	scale.z = beta[5];
	
}

void GaussNewton(Vec3d_t *input[6], Vec3d_t offset, Vec3d_t scale)
{
	uint8_t	cnt = 0;
	double eps = 0.0001;
	double step = 100.0;
	double beta[6]; // the result, offset and scale of x, y, z
	double residual[6]; // residual
	double diff[6][3]; // difference between xi and xi_offset (e.t. xi- xi_offset)

	double Hessian[6][6]; // Hessian matrix
	double Jacobian[6][6]; // Jacobian matrix
	double JT_J[6][6]; // J^T * J matrix
	double JT_R[6]; // J^T * R matrix

	// initialization
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1;

	// while(cnt<100 || step>eps) // monitor iteration times and step length
	{
		// initialization
		cnt++;
		step = 0;
		for(uint8_t i=0; i<6; i++)
		{
			// calculate difference between xi and xi_offset
			diff[i][0] = input[i]->x - beta[0];
			diff[i][1] = input[i]->y - beta[1];
			diff[i][2] = input[i]->z - beta[2];

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
			}
			for(uint8_t j=3; j<6; j++)
			{
				Jacobian[i][j] = -2*beta[3+j]*diff[i][j]*diff[i][j];
			}
		}

		// calculate J^T * J matrix and J^T * R matrix
		for(uint8_t i=0; i<6; i++)
		{
			for(uint8_t j=0; j<6; j++)
			{
				// calculate J^T * R matrix
				for(uint8_t k=0; k<6; k++)
				{
					JT_J[i][j] = Jacobian[i][k] * Jacobian[k][j];
				}

				// calculate J^T * R matrix
				JT_R[i] = Jacobian[j][i] * residual[j];
			}
		}

		// Gauss Elimination : J^T*J * delta = J^T*R

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
		for(uint8_t i=5; i>=0; i--)
		{
			double quotient = JT_J[i][i];
			for(uint8_t j=i+1; j<6; j++)
			{
				JT_J[i][j] /= quotient;
			}
			JT_R[i] /= quotient;
		}

		// update beta and step
		for(uint8_t i=0; i<6; i++)
		{
			step += (beta[i] - JT_R[i]);
			beta[i] = JT_R[i];
		}

	}
}