#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/angles.h"
#include "lib/timer.h"

/** Static variables -------------------------------------------------------- */
static volatile bool read_angles_flag;

/** Function prototypes ----------------------------------------------------- */
static void read_angles_flag_cb(void);

/** Function definitions ---------------------------------------------------- */
static void read_angles_flag_cb(void)
{
	read_angles_flag = true;
}

/** Public functions -------------------------------------------------------- */
void* thread_1_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 1\n");
	pthread_mutex_unlock(&print_mtx);

	angles_init();
	timer_init();
	app_timer_t* angles_timer = create_timer(read_angles_flag_cb, 500, 5);

	while (true) {
		if (read_angles_flag) {
			read_angles_flag = false;
			calculate_angles();
		}
	}
}
