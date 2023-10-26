#ifndef __CONTROL_H
#define __CONTROL_H

typedef struct
{
	float kp, ki, kd;
	float err, err_last;
	float target, current;
	float p_out, i_out, d_out, out;
}pid_t;

#endif
