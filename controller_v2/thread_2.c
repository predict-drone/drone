#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/angles.h"
#include "lib/timer.h"
#include "lib/pid.h"
#include "lib/transmitter.h"
#include "lib/motor.h"

#define CHANGE 0.1
//#define BASE_THROTTLE 1770
#define BASE_THROTTLE 1750
#define THROTTLE_SCALE 10

/** Data structures --------------------------------------------------------- */
typedef struct pid_out_t {
	double pitch;
	double roll;
	double yaw;
} pid_out_t;

/** Static variables -------------------------------------------------------- */
static pid_out_t pid_angle;
static pid_out_t pid_vel;

static volatile bool p_up_flag, p_down_flag;
static volatile bool d_up_flag, d_down_flag;

/** Public functions -------------------------------------------------------- */
void* thread_2_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 2\n");
	pthread_mutex_unlock(&print_mtx);

	while (!angles_is_init());

	pthread_mutex_lock(&pid_mtx);
	pid_create(&pitch_pid, 0.4, 0.0, 0.2, 0.0, 0.0);
	pid_create(&roll_pid, 0.4, 0.0, 0.2, 0.0, 0.0);
	//pid_create(&yaw_pid, 0, 0.0, 0.0, 0.0, 0.0);

	pid_create(&pitch_vel_pid, 0.4, 0.0, 0.2, 0.0, 0.0);
	pid_create(&roll_vel_pid, 0.4, 0.0, 0.2, 0.0, 0.0);
	pid_create(&yaw_vel_pid, 0.4, 0.0, 0.2, 0.0, 0.0);
	pthread_mutex_unlock(&pid_mtx);

	uint64_t next_time = micros();
	while (true) {
		if (next_time < micros()) {
			double motor_0 = 0, motor_1 = 0, motor_2 = 0, motor_3 = 0;
			angles_t angles_vel = get_angle_vel();
			angles_t angles = get_angles();

			pthread_mutex_lock(&pid_mtx);
			pid_angle.pitch = pid_fire(pitch_pid, 0, angles.pitch);
			pid_angle.roll = pid_fire(roll_pid, 0, angles.roll);
			//pid_angle.yaw = pid_fire(yaw_pid, 0, angles.yaw);
			pid_vel.pitch = pid_fire(pitch_vel_pid, pid_angle.pitch, angles_vel.pitch);
			pid_vel.roll = pid_fire(roll_vel_pid, pid_angle.roll, angles_vel.roll);
			pid_vel.yaw = pid_fire(yaw_vel_pid, 0, angles_vel.yaw);
			pthread_mutex_unlock(&pid_mtx);

			transmitter_read();
			transmitter tm = get_transmitter_values();

			int throttle = tm.throttle;
			if (tm.throttle > 1500) {

				motor_0 = tm.throttle - pid_vel.pitch - pid_vel.roll - pid_vel.yaw;
				motor_1 = tm.throttle + pid_vel.pitch - pid_vel.roll + pid_vel.yaw;
				motor_2 = tm.throttle + pid_vel.pitch + pid_vel.roll - pid_vel.yaw;
				motor_3 = tm.throttle + pid_vel.roll - pid_vel.pitch + pid_vel.yaw;

				if (motor_0 < BASE_THROTTLE)
					motor_0 = BASE_THROTTLE;
				else if (motor_1 < BASE_THROTTLE)
					motor_1 = BASE_THROTTLE;
				else if (motor_2 < BASE_THROTTLE)
					motor_2 = BASE_THROTTLE;
				else if (motor_3 < BASE_THROTTLE)
					motor_3 = BASE_THROTTLE;
			}

			if (tm.throttle < 1200) {
				motor_0 = 0;
				motor_1 = 0;
				motor_2 = 0;
				motor_3 = 0;
			}

			motor_run(0, (int) motor_0);
			motor_run(1, (int) motor_1);
			motor_run(2, (int) motor_2);
			motor_run(3, (int) motor_3);

			//pthread_mutex_lock(&print_mtx);
			//printf("Motor: %.3f, %.3f, %.3f, %.3f\n", motor_0, motor_1, motor_2, motor_3);
			//pthread_mutex_unlock(&print_mtx);

			//printf("pitch_a: %.2f pitch_pid: %.2f pitch_v: %.2f pitch_pidv: %.2f m1: %.2f m2: %d\n", angles.pitch, pid_angle.pitch, angles_vel.pitch, pid_vel.pitch, pitch_vel_pid.kp, tm.throttle);

			next_time += 50000;
		}
	}
}
