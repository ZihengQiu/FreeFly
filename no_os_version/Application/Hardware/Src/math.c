#include "stm32f4xx.h"                  // Device header
#include "math.h"

void GaussNewton(Vec3d input[6], Vec3d offset, Vec3d scale, double length)
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
