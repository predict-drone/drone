#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "lib/timer.h"

/** Public functions -------------------------------------------------------- */
void* thread_0_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 0\n");
	pthread_mutex_unlock(&print_mtx);

	while (true);
}
