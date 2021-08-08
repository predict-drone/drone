#include <stdio.h>
#include <stdint.h>

#include "lib/timer.h"

static int counter;


static void timer_callback(void);


static void timer_callback(void)
{
	counter++;
}


int main(void)
{
	printf("Timer driver example\n");

	timer_init();
	app_timer_t* timer = create_timer(timer_callback, 500, 1000);

	while (counter < 20) {
		printf("Time passed: %d\n", counter);
		delay(500);
	}

	printf("\n");
	printf("End!\n");

	return 0;
}
