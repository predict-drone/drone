#ifndef __PID_H
#define __PID_H

#include <stdint.h>

#include "lib/fixedptc.h"

typedef struct pid_instance_t {
	fixedpt kp;
	fixedpt ki;
	fixedpt kd;
	fixedpt ki_accum;
	fixedpt kd_last;
	uint64_t last_time;
} pid_instance_t;

void pid_create(pid_instance_t* pid, fixedpt kp, fixedpt ki, fixedpt kd);

fixedpt pid_fire(pid_instance_t pid, fixedpt input, fixedpt feedback);

#endif // __PID_H
