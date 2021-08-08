#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

#include "lib/timer.h"
#include "lib/angles.h"

static volatile bool print_flag;
static volatile bool read_flag;


static void print_flag_cb(void);
static void read_flag_cb(void);


static void print_flag_cb(void)
{
	print_flag = true;
}

static void read_flag_cb(void)
{
	read_flag = true;
}

static void* thread1_main(void* attr)
{
	printf("Angles example\n");
	angles_init();
	timer_init();

	app_timer_t* print_timer = create_timer(print_flag_cb, 500, 1000);
	app_timer_t* read_timer = create_timer(read_flag_cb, 100, 200);

	while (true) {

		if (print_flag) {
			print_flag = false;
			angles_t angles = get_angles2();
			printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f\n", angles.pitch,
					angles.roll, angles.yaw);
		}

		if (read_flag) {
			calculate_angles2();
		}
	}

	return NULL;
}

int main(void)
{
	// Init threads
	pthread_t thread1;
	pthread_create(&thread1, NULL, thread1_main, NULL);

	while (true);

	return 0;
}
