#ifndef __MATH_H
#define __MATH_H

typedef struct
{
	double x, y, z;
}Vec3d_t;

void GaussNewton(Vec3d_t input[6], Vec3d_t* offset, Vec3d_t* scale);

#endif
