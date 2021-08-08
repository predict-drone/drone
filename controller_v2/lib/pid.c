#include <stdio.h>
#include <stdint.h>
#include <malloc.h>

#include "pid.h"
#include "timer.h"
#include "main.h"


void pid_create(pid_instance_t* pid, double kp, double ki, double kd, double kr,
		double kf)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->kr = kr;
	pid->kf = kf;
	pid->ki_accum = 0.0;
	pid->kd_last = 0.0;
	pid->kr_accum = 0.0;
	pid->kf_last = 0.0;
	pid->last_time = micros();
}

double pid_fire(pid_instance_t pid, double input, double feedback)
{
	uint64_t current_time = micros();
	double elapsed_time = (current_time - pid.last_time) / 1000000.0;
	pid.last_time = current_time;


	double error;
	if (pid.kf != 0) {
		double cf = pid.kf * (feedback - pid.kf_last) / elapsed_time;
		error = input - feedback * cf;
	} else {
		 error = input - feedback;
	}

	pid.kf_last = feedback;


	pid.ki_accum += error * elapsed_time;
	double ci = pid.ki * pid.ki_accum;
	double cd = pid.kd * (error - pid.kd_last) / elapsed_time;
	pid.kd_last = error;

	double pid_out = (pid.kp * error + ci + cd);

	if (pid.kr != 0) {
		pid.kr_accum += pid_out * elapsed_time;
		double cr = pid.kr * pid.kr_accum;
		pid_out = cr;
	}

	return pid_out;
}
