#include <stdio.h>
#include <stdint.h>
#include <malloc.h>

#include "pid.h"
#include "timer.h"
#include "main.h"


void pid_create(pid_instance_t* pid, fixedpt kp, fixedpt ki, fixedpt kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->ki_accum = 0.0;
	pid->kd_last = 0.0;
	pid->last_time = micros();
}

fixedpt pid_fire(pid_instance_t pid, fixedpt input, fixedpt feedback)
{
	uint64_t current_time = micros();
	fixedpt elapsed_time = (fixedpt) fixedpt_xdiv(
			fixedpt_fromint(current_time - pid.last_time),
			fixedpt_fromint(1000000));
	pid.last_time = current_time;


	fixedpt error = input - feedback;

	pid.ki_accum += fixedpt_mul(error, elapsed_time);
	fixedpt ci = fixedpt_mul(pid.ki, pid.ki_accum);
	fixedpt cd = fixedpt_div(
			fixedpt_mul(pid.kd, (error - pid.kd_last)),
			elapsed_time);
	pid.kd_last = error;

	fixedpt pid_out = fixedpt_mul(pid.kp, error) + ci + cd;

	return pid_out;
}
