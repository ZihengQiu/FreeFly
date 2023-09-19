#ifndef __MATH_H
#define __MATH_H

typedef struct
{
	double x, y, z;
}Vec3d;


void GaussNewton(Vec3d input[6], Vec3d offset, Vec3d scale, double length);

#endif
