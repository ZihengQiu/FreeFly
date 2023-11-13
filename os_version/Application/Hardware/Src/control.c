#include "control.h"
#include "motor.h"
#include "mathkit.h"
#include "os_cpu.h"
#include "receiver.h"
#include <sys/_stdint.h>

#define ANGLE_MAX 20

#define pid_i_enabled 1
#define pid_d_enabled 1

 // 0: angle pid, 1: velocity pid
pid_t pid_roll[2]  = {{1, 1, 1}, {1 ,1 ,1}}, 
	  pid_pitch[2] = {{1, 1, 1}, {1 ,1 ,1}},
	  pid_yaw[2]   = {{1, 1, 1}, {1 ,1 ,1}};

void IntegralOutputLimit(pid_t *pid)
{
	if(pid->i_out > pid->i_out_limit)
	{
		pid->i_out = pid->i_out_limit;
	}
	else if(pid->i_out < -pid->i_out_limit)
	{
		pid->i_out = -pid->i_out_limit;
	}
}

void OutputLimit(pid_t *pid)
{
	if(pid->out > pid->out_limit)
	{
		pid->out = pid->out_limit;
	}
	else if(pid->out < -pid->out_limit)
	{
		pid->out = -pid->out_limit;
	}
}

void UpdateAnglePID(pid_t *pid, float gyro)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->err;

#if pid_i_enabled == 1
	if(pid->err < pid->err_limit)
	{
		pid->i_out += pid->ki * pid->err;
		// IntegralOutputLimit(pid);
	}
	else
	{
		pid->i_out = 0;
	}
#endif

#if pid_d_enabled == 1
		pid->d_out = pid->kd * gyro;
#endif

	pid->out = pid->p_out + pid->i_out + pid->d_out;
	// OutputLimit(pid);
}

void UpdateVelocityPID(pid_t *pid)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->err;

#if pid_i_enabled == 1
	if(pid->err < pid->err_limit)
	{
		pid->i_out += pid->ki * pid->err;
		// IntegralOutputLimit(pid);
	}
	else
	{
		pid->i_out = 0;
	}
#endif

#if pid_d_enabled == 1
		pid->d_out = pid->kd * (pid->err - pid->err_last);
#endif
	
	pid->err_last = pid->err;

	pid->out = pid->p_out + pid->i_out + pid->d_out;
	// OutputLimit(pid);
}

void UpdatePID(pid_t *pid_inner, pid_t *pid_outer, float gyro)
{
	UpdateAnglePID(pid_inner, gyro);
	pid_outer->target = pid_inner->out;
	UpdateVelocityPID(pid_outer);
}

void MotorControl(vec3d_t angle_cur, vec3d_t gyro)
{
	//map ppm_val to angle
	uint32_t throttle = ppm_val[THR];
	vec3d_t angle_target;
	angle_target.x = ((float)ppm_val[AIL]-PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MIN_VAL)*ANGLE_MAX;
	angle_target.y = ((float)ppm_val[ELE]-PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MIN_VAL)*ANGLE_MAX;
	angle_target.z = ((float)ppm_val[RUD]-PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MIN_VAL)*ANGLE_MAX;

	// update angle in pid_t
	pid_roll[0].target = angle_target.x;
	pid_roll[0].current = angle_cur.x;
	pid_pitch[0].target = angle_target.y;
	pid_pitch[0].current = angle_cur.y;
	pid_yaw[0].target = angle_target.z;
	pid_yaw[0].current = angle_cur.z;

	UpdatePID(&pid_roll[0], &pid_roll[1], gyro.x);
	UpdatePID(&pid_pitch[0], &pid_pitch[1], gyro.y);
	UpdatePID(&pid_yaw[0], &pid_yaw[1], gyro.z);

	// update motor speed
	motor_compare[0] = throttle - pid_pitch[1].out + pid_roll[1].out - pid_yaw[1].out;
	motor_compare[1] = throttle - pid_pitch[1].out - pid_roll[1].out + pid_yaw[1].out;
	motor_compare[2] = throttle + pid_pitch[1].out + pid_roll[1].out + pid_yaw[1].out;
	motor_compare[3] = throttle + pid_pitch[1].out - pid_roll[1].out - pid_yaw[1].out;

	// update motor speed
	MotorSetSpeed();
}