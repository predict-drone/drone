#include <stdio.h>
#include <stdbool.h>

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
	pid_create(&pitch_pid, fixedpt_rconst(0.4), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&roll_pid, fixedpt_rconst(0.4), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));

	pid_create(&pitch_vel_pid, fixedpt_rconst(0.4), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&roll_vel_pid, fixedpt_rconst(0.4), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pid_create(&yaw_vel_pid, fixedpt_rconst(0.4), fixedpt_rconst(0.0),
			fixedpt_rconst(0.0));
	pthread_mutex_unlock(&pid_mtx);

	uint64_t next_time = micros();
	while (true) {
		if (next_time < micros()) {
			uint64_t ti = micros();
			fixedpt motor_0, motor_1, motor_2, motor_3;
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
			fixedpt throttle = fixedpt_fromint(1700);
			if (throttle > fixedpt_rconst(1500)) {

				motor_0 = throttle - pid_vel.pitch - pid_vel.roll;
				motor_1 = throttle + pid_vel.pitch - pid_vel.roll;
				motor_2 = throttle + pid_vel.pitch + pid_vel.roll;
				motor_3 = throttle - pid_vel.pitch + pid_vel.roll;

				if (motor_0 < BASE_THROTTLE)
					motor_0 = BASE_THROTTLE;
				else if (motor_1 < BASE_THROTTLE)
					motor_1 = BASE_THROTTLE;
				else if (motor_2 < BASE_THROTTLE)
					motor_2 = BASE_THROTTLE;
				else if (motor_3 < BASE_THROTTLE)
					motor_3 = BASE_THROTTLE;
			} else {
				motor_0 = 0;
				motor_1 = 0;
				motor_2 = 0;
				motor_3 = 0;
			}

			motor_run(0, fixedpt_toint(motor_0));
			motor_run(1, fixedpt_toint(motor_1));
			motor_run(2, fixedpt_toint(motor_2));
			motor_run(3, fixedpt_toint(motor_3));

			uint64_t te = micros();

			pthread_mutex_lock(&print_mtx);
			printf("Motor: %.2f %.2f %.1f, %.1f, %.1f, %.1f %lld\n", 
					fixedpt_tofloat(angles.pitch_speed),
					fixedpt_tofloat(angles.roll_speed),
					fixedpt_tofloat(pid_pos.pitch), fixedpt_tofloat(pid_pos.roll),
					fixedpt_tofloat(pid_vel.pitch), fixedpt_tofloat(pid_vel.roll),
					te-ti);
			pthread_mutex_unlock(&print_mtx);
			next_time += 5000;
		}
	}
}
