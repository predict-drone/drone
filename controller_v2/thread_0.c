#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/timer.h"

/** Static variables -------------------------------------------------------- */
static volatile bool print_flag;

/** Function prototypes ----------------------------------------------------- */
static void print_flag_cb(void);

/** Function definitions ---------------------------------------------------- */
static void print_flag_cb(void)
{
	print_flag = true;
}

/** Public functions -------------------------------------------------------- */
void* thread_0_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 0\n");
	pthread_mutex_unlock(&print_mtx);

	timer_init();

	app_timer_t* print_timer = create_timer(print_flag_cb, 500, 60000);

	while (1) {
		if (print_flag) {
			print_flag = false;
			pthread_mutex_lock(&print_mtx);
			printf("Heartbeat\n");
			pthread_mutex_unlock(&print_mtx);
		}
	}
}
