#include "control.h"
#include "mathkit.h"

pid_t pid_roll, pid_pitch, pid_yaw;

void UpdateAnglePID(pid_t *pid, float gyro)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->p_out;
	pid->i_out += pid->ki * pid->err;
	pid->d_out = pid->kd * gyro;

	pid->out = pid->p_out + pid->i_out + pid->d_out;
}

void UpdateVelocityPID(pid_t *pid)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->p_out;

	pid->i_out += pid->ki * pid->err;

	pid->d_out = pid->kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;

	pid->out = pid->p_out + pid->i_out + pid->d_out;
}

void UpdatePID(pid_t *pid, float gyro)
{
	UpdateAnglePID(pid, gyro);
	UpdateVelocityPID(pid);
}

void MotorControl(float motor[4], vec3d_t angle_cur, vec3d_t angle_target, vec3d_t gyro, float throttle)
{
	// TODO : map ppm_val to angle

	// update angle in pid_t
	pid_pitch.target = angle_target.x;
	pid_pitch.current = angle_cur.x;
	pid_roll.target = angle_target.y;
	pid_roll.current = angle_cur.y;
	pid_yaw.target = angle_target.z;
	pid_yaw.current = angle_cur.z;

	UpdatePID(&pid_pitch, gyro.x);
	UpdatePID(&pid_roll, gyro.y);
	UpdatePID(&pid_yaw, gyro.z);

	// update motor speed
	motor[0] = throttle + pid_pitch.out + pid_roll.out + pid_yaw.out;
	motor[1] = throttle + pid_pitch.out - pid_roll.out - pid_yaw.out;
	motor[2] = throttle - pid_pitch.out + pid_roll.out - pid_yaw.out;
	motor[3] = throttle - pid_pitch.out - pid_roll.out + pid_yaw.out;

	// TODO : limit motor speed

	// update motor speed
	
}