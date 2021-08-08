#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

#include "lib/timer.h"
#include "lib/transmitter.h"

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

static void* thread2_main(void* attr)
{
	printf("Transmitter example\n");

	timer_init();

	app_timer_t* print_timer = create_timer(print_flag_cb, 500, 1000);
	app_timer_t* read_timer = create_timer(read_flag_cb, 100, 5);

	while (true) {

		if (print_flag) {
			print_flag = false;
			transmitter transmitter = get_transmitter_values();
			printf("\n throttle\t:%d", transmitter.throttle);
			printf("\n roll\t\t:%d", transmitter.roll);
			printf("\n pitch\t\t:%d", transmitter.pitch);
			printf("\n yaw\t\t:%d", transmitter.yaw);
			printf("\n Channel-5\t\t:%d", transmitter.channel_5);
			printf("\n Channel-6\t\t:%d", transmitter.channel_6);
		}

		if (read_flag) {
			transmitter_read();
		}
	}

	return NULL;
}


static void* thread1_main(void* args)
{
	while (1);
}

int main(void)
{
	// Init threads
	pthread_t thread1, thread2;
	pthread_create(&thread1, NULL, thread1_main, NULL);
	pthread_create(&thread2, NULL, thread2_main, NULL);

	while (true);

	return 0;
}
