#include "control.h"
#include "motor.h"
#include "mathkit.h"
#include "os_cpu.h"
#include "receiver.h"

#include <stdlib.h>

#define ANGLE_MAX 	20
#define ANGULAR_MAX 30

BOOLEAN pid_i_enabled = 1, pid_d_enabled = 1;

 // 0: angle(external) pid, 1: velocity(inner) pid
pid_t pid_roll[2] = {
		{
			.kp = 1.3, .ki = 0.001, .kd = -0.05,
			.err_limit = 300, .i_out_limit = 200, .out_limit = 500
		},
		{
			.kp = 1.5, .ki = 0.016, .kd = 40,
			.err_limit = 1000, .i_out_limit = 100, .out_limit = 200
		}
	}, 
	pid_pitch[2] = {
		{
			.kp = 1.1, .ki = 0.001, .kd = -0.05,
			.err_limit = 300, .i_out_limit = 200, .out_limit = 500
		},
		{
			.kp = 1.5, .ki = 0.016, .kd = 40,
			.err_limit = 1000, .i_out_limit = 100, .out_limit = 200
		}
	},
	pid_yaw[2] = {
		{
			.kp = 1, .ki = 0, .kd = 0,
			.err_limit = 500, .i_out_limit = 500, .out_limit = 500
		},
		{
			.kp = 5, .ki = 0.02, .kd = 0,
			.err_limit = 500, .i_out_limit = 500, .out_limit = 500
		}
	};

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

	if (pid_i_enabled == 1 && ppm_val[THR] > 1500) // ensure the stable takeoff
	{
		if(abs(pid->err) < pid->err_limit) // prevent integral windup
		{
			pid->i_out += pid->ki * pid->err;
			IntegralOutputLimit(pid);
		}
	}
	else
	{
		pid->i_out = 0;
	}

	if(pid_d_enabled == 1)
	{
		pid->d_out = pid->kd * gyro; // derivative of angle(error) here is angular velocity
	}

	pid->out = pid->p_out + pid->i_out + pid->d_out;
	OutputLimit(pid);
}

void UpdateVelocityPID(pid_t *pid)
{
	pid->err = pid->target - pid->current;

	pid->p_out = pid->kp * pid->err;

	if (pid_i_enabled == 1 && ppm_val[THR] > 1500) // ensure the stable takeoff
	{
		if(abs(pid->err) < pid->err_limit) // prevent integral windup
		{
			pid->i_out += pid->ki * pid->err;
			IntegralOutputLimit(pid);
		}
	}
	else
	{
		pid->i_out = 0;
	}
	  
	if(pid_d_enabled == 1)
	{
		pid->d_out = pid->kd * (pid->err - pid->err_last);
	}
	
	pid->err_last = pid->err;

	pid->out = pid->p_out + pid->i_out + pid->d_out;
	OutputLimit(pid);
}

void UpdatePID(pid_t *pid_outer, pid_t *pid_inner, float gyro)
{
	UpdateAnglePID(pid_outer, gyro);
	pid_inner->target = pid_outer->out;
	UpdateVelocityPID(pid_inner);
}

void MotorControl(vec3d_t angle_cur, vec3d_t gyro)
{
	// map ppm_val to target angle(for roll and pitch) and velocity(for yaw)
	pid_roll[0].target = ((float)ppm_val[AIL]-PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MID_VAL)*ANGLE_MAX;
	pid_roll[0].current = angle_cur.x;
	pid_roll[1].current = gyro.x;

	pid_pitch[0].target = ((float)ppm_val[ELE]-PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MID_VAL)*ANGLE_MAX;
	pid_pitch[0].current = angle_cur.y;
	pid_pitch[1].current = gyro.y;
	
	pid_yaw[1].target = -1*((float)ppm_val[RUD] - PPM_MID_VAL)/(PPM_MAX_VAL-PPM_MID_VAL)*ANGULAR_MAX;
	pid_yaw[1].current = gyro.z;

	// update pid
	UpdatePID(&pid_roll[0], &pid_roll[1], gyro.x);
	UpdatePID(&pid_pitch[0], &pid_pitch[1], gyro.y);
	UpdateVelocityPID(&pid_yaw[1]);

	// compute compare value for each motor by throttle and pid output
	uint32_t throttle = ppm_val[THR];
	motor_compare[0] = throttle - pid_pitch[1].out + pid_roll[1].out - pid_yaw[1].out;
	motor_compare[1] = throttle - pid_pitch[1].out - pid_roll[1].out + pid_yaw[1].out;
	motor_compare[2] = throttle + pid_pitch[1].out + pid_roll[1].out + pid_yaw[1].out;
	motor_compare[3] = throttle + pid_pitch[1].out - pid_roll[1].out - pid_yaw[1].out;

	// update motor speed by compare value
	MotorSetSpeed();
}