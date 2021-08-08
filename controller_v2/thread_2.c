#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/angles.h"
#include "lib/timer.h"
#include "lib/pid.h"
#include "lib/transmitter.h"
#include "lib/motor.h"

#define CHANGE 0.2
#define BASE_THROTTLE 1720
#define THROTTLE_SCALE 10

/** Static variables -------------------------------------------------------- */
static pid_instance_t pitch_pid;
static pid_instance_t roll_pid;
static pid_instance_t yaw_pid;
static volatile bool controller_flag;

static volatile bool p_up_flag, p_down_flag;
static volatile bool d_up_flag, d_down_flag;

/** Function prototypes ----------------------------------------------------- */
static void controller_flag_cb(void);

/** Function definitions ---------------------------------------------------- */
static void controller_flag_cb(void)
{
	controller_flag = true;
}

/** Public functions -------------------------------------------------------- */
void* thread_2_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 2\n");
	pthread_mutex_unlock(&print_mtx);

	timer_init();

	app_timer_t* controller_timer = create_timer(controller_flag_cb, 500, 50);

	while (!angles_is_init());

	pid_create(&pitch_pid, 0.0, 0.0, 0.0, 0.0, 0.0);
	pid_create(&roll_pid, 0.0, 0.0, 0.0, 0.0, 0.0);
	//pid_create(&yaw_pid, 1.0, 0.0, 0.0, 0.0, 0.0);

	while (true) {
		if (controller_flag) {
			controller_flag = false;
			double motor_0=0, motor_1 = 0, motor_2 = 0, motor_3 = 0;
			angles_t angles = get_angles();

			double pitch_out = pid_fire(pitch_pid, 0, angles.pitch);
			double roll_out = pid_fire(roll_pid, 0, angles.roll);
			double yaw_out = pid_fire(yaw_pid, 0, angles.yaw);

			transmitter_read();
			transmitter tm = get_transmitter_values();

			if (tm.throttle > 1800) {
				motor_0 = BASE_THROTTLE - pitch_out - roll_out;
				motor_1 = BASE_THROTTLE + pitch_out - roll_out;
				motor_2 = BASE_THROTTLE + pitch_out + roll_out;
				motor_3 = BASE_THROTTLE + roll_out - pitch_out;
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
			//printf("Motor: %.3f, %.3f, %.3f, %.3f\n", motor_0, motor_1, motor_2,
					//motor_3);
			//pthread_mutex_unlock(&print_mtx);
			//
			//
			printf("%d %d %.1f %.1f \n", tm.pitch, tm.roll, pitch_pid.kp, pitch_pid.kd);

			if (tm.pitch > 1900) p_up_flag = true;
			else if (tm.pitch < 1100) p_down_flag = true;
			else if (tm.pitch > 1300 && tm.pitch < 1700) {
				if (p_up_flag) {
					pitch_pid.kp += CHANGE;
					roll_pid.kp += CHANGE;
					p_up_flag = false;
				} else if (p_down_flag) {
					pitch_pid.kp -= CHANGE;
					roll_pid.kp -= CHANGE;
					p_down_flag = false;
				}
			}

			if (tm.roll > 1900) d_up_flag = true;
			else if (tm.roll < 1100) d_down_flag = true;
			else if (tm.roll > 1300 && tm.roll < 1700) {
				if (d_up_flag) {
					pitch_pid.kd += CHANGE;
					roll_pid.kd += CHANGE;
					d_up_flag = false;
				} else if (d_down_flag) {
					pitch_pid.kd -= CHANGE;
					roll_pid.kd -= CHANGE;
					d_down_flag = false;
				}
			}
		}


	}
}
