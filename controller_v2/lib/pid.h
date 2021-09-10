#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct pid_instance_t {
	double kp;
	double ki;
	double kd;
	double kr;
	double kf;
	double ki_accum;
	double kd_last;
	double kr_accum;
	double kf_last;
	uint64_t last_time;
} pid_instance_t;

void pid_create(pid_instance_t* pid, double kp, double ki, double kd, double kr,
		double kf);

double pid_fire(pid_instance_t pid, double input, double feedback);

#endif // __PID_H
