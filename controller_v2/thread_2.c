#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "lib/angles.h"
#include "lib/timer.h"
#include "lib/pid.h"
#include "lib/transmitter.h"
#include "lib/motor.h"

#define BASE_THROTTLE 1750

/** Data structures --------------------------------------------------------- */
typedef struct pid_out_t {
	fixedpt pitch;
	fixedpt roll;
	fixedpt yaw;
} pid_out_t;

/** Static variables -------------------------------------------------------- */
static pid_out_t pid_pos;
static pid_out_t pid_vel;

/** Public functions -------------------------------------------------------- */
void* thread_2_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 2\n");
	pthread_mutex_unlock(&print_mtx);

	while (!angles_is_init());

	pthread_mutex_lock(&pid_mtx);
	pid_create(&pitch_pid, fixedpt_rconst(0.8), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&roll_pid, fixedpt_rconst(0.8), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));

	pid_create(&pitch_vel_pid, fixedpt_rconst(0.8), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&roll_vel_pid, fixedpt_rconst(0.8), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&yaw_vel_pid, fixedpt_rconst(0.8), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pthread_mutex_unlock(&pid_mtx);

	uint64_t next_time = micros();
	while (true) {
		if (next_time < micros()) {
			angles_t angles = get_angles();

			pthread_mutex_lock(&pid_mtx);
			pid_pos.pitch = pid_fire(pitch_pid, 0, angles.pitch);
			pid_pos.roll = pid_fire(roll_pid, 0, angles.roll);
			pid_vel.pitch = pid_fire(pitch_vel_pid, pid_pos.pitch,
					angles.pitch_speed);
			pid_vel.roll = pid_fire(roll_vel_pid, pid_pos.roll,
					angles.roll_speed);
			pid_vel.yaw = pid_fire(yaw_vel_pid, 0, angles.yaw_speed);
			pthread_mutex_unlock(&pid_mtx);

			transmitter_read();
			transmitter tm = get_transmitter_values();

			//fixedpt throttle = fixedpt_fromint(tm.throttle);
			//pthread_mutex_lock(&print_mtx);
			//printf("Throttle: %d %ld\n", tm.throttle, throttle>>16);
			//pthread_mutex_unlock(&print_mtx);
			fixedpt throttle = fixedpt_fromint(1800);

			motor_t motor;
			if (throttle > fixedpt_rconst(1500)) {

				motor.m0 = fixedpt_toint(throttle - pid_vel.pitch - pid_vel.roll);
				motor.m1 = fixedpt_toint(throttle + pid_vel.pitch - pid_vel.roll);
				motor.m2 = fixedpt_toint(throttle + pid_vel.pitch + pid_vel.roll);
				motor.m3 = fixedpt_toint(throttle - pid_vel.pitch + pid_vel.roll);

				if (motor.m0 < BASE_THROTTLE)
					motor.m0 = BASE_THROTTLE;
				else if (motor.m1 < BASE_THROTTLE)
					motor.m1 = BASE_THROTTLE;
				else if (motor.m2 < BASE_THROTTLE)
					motor.m2 = BASE_THROTTLE;
				else if (motor.m3 < BASE_THROTTLE)
					motor.m3 = BASE_THROTTLE;
			} else {
				memset(&motor, 0, sizeof(motor));
			}

			motor_run(motor);

			pthread_mutex_lock(&print_mtx);
			//printf("Motor: %d %d %d, %d\n", motor.m0, motor.m1, motor.m2, motor.m3);
			//printf("Motor: %.2f %.2f %.1f, %.1f, %.1f, %.1f %lld\n",
			//		fixedpt_tofloat(angles.pitch_speed),
			//		fixedpt_tofloat(angles.roll_speed),
			//		fixedpt_tofloat(pid_pos.pitch), fixedpt_tofloat(pid_pos.roll),
			//		fixedpt_tofloat(pid_vel.pitch), fixedpt_tofloat(pid_vel.roll),
			//		te-ti);
			pthread_mutex_unlock(&print_mtx);
			next_time += 5000;
		}
	}
}
