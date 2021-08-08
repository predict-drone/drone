#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>
#include <math.h>

#include "lib/timer.h"

#define LOOPS 40


static int counter;
static pthread_mutex_t counter_mtx = PTHREAD_MUTEX_INITIALIZER;
static int sec_counter;
static bool print_flag;


static void computing_intensive_task(void);
static void* thread1_main(void *args);


static void computing_intensive_task(void)
{
	double aux = 3000.34;
	for(int i = 0; i < LOOPS; i++) {
		for(int j = 0; j < LOOPS; j++) {
			double aux2 = aux * 74835.3;
			double aux3 = aux2 - 3489.003;
			aux = fabs(aux3/(aux-1.54) - aux2);
		}
	}
}

static void* thread1_main(void *args)
{
	while (1) {
		computing_intensive_task();
		pthread_mutex_lock(&counter_mtx);
		counter++;
		pthread_mutex_unlock(&counter_mtx);
	}
}

static void print_flag_cb(void)
{
	print_flag = true;
	sec_counter++;
}

int main(void)
{
	printf("Starting multhread example\n");

	timer_init();

	pthread_t thread1, thread2, thread3;
	pthread_create(&thread1, NULL, thread1_main, NULL);
	pthread_create(&thread2, NULL, thread1_main, NULL);
	pthread_create(&thread3, NULL, thread1_main, NULL);

	app_timer_t* timer = create_timer(print_flag_cb, 500, 1000);


	while (1) {
		if (print_flag) {
			print_flag = false;
			pthread_mutex_lock(&counter_mtx);
			printf("Task executed %d times in %d seconds\n", counter, sec_counter);
			pthread_mutex_unlock(&counter_mtx);
		}
		
	}

	return 0;
}
