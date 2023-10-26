#include "control.h"

void update_pid(pid_t *pid)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->p_out;

	pid->i_out += pid->ki * pid->err;

	pid->d_out = pid->kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;

	pid->out = pid->p_out + pid->i_out + pid->d_out;
	
}