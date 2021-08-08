#include <stdio.h>

#include "main.h"
#include "lib/uart.h"
#include "lib/timer.h"

static uint8_t buffer[512];

void* thread_3_main(void* args)
{
	pthread_mutex_lock(&print_mtx);
	printf("Thread 3\n");
	pthread_mutex_unlock(&print_mtx);

	while (1) {

		unsigned char c;
		int result = uart2_read(&c);

		if (result) {
			pthread_mutex_lock(&print_mtx);
			printf("DATA (%d): %c, %d\n", result, c, c);
			pthread_mutex_unlock(&print_mtx);
		}

		delay(1000);
	}

}
