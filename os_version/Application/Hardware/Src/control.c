#include "control.h"
#include "motor.h"
#include "mathkit.h"
#include "receiver.h"
#include <sys/_stdint.h>

#define ANGLE_MAX 20

pid_t pid_roll[2], pid_pitch[2], pid_yaw[2]; // 0: angle pid, 1: velocity pid

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

void UpdatePID(pid_t *pid_inner, pid_t *pid_outer, float gyro)
{
	UpdateAnglePID(pid_inner, gyro);
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