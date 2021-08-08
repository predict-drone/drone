#include <stdio.h>
#include <stdbool.h>

#include "lib/timer.h"
#include "lib/leds.h"

static int seconds_counter;
static volatile bool print_flag;

static void fast_leds_cb(void);
static void slow_leds_cb(void);
static void counter_cb(void);


static void fast_leds_cb(void)
{
	led_toggle(0);
	led_toggle(1);
}

static void slow_leds_cb(void)
{
	led_toggle(3);
	led_toggle(4);
}

static void counter_cb(void)
{
	seconds_counter++;
	led_toggle(7);
}

static void print_flag_cb(void)
{
	print_flag = true;
}


int main(void)
{
	printf("Leds with timers example\n");

	timer_init();
	led_off_all();

	app_timer_t* t1 = create_timer(fast_leds_cb, 500, 250);
	app_timer_t* t2 = create_timer(slow_leds_cb, 500, 5000);
	app_timer_t* t3 = create_timer(counter_cb, 500, 1000);
	app_timer_t* t4 = create_timer(print_flag_cb, 500, 500);

	while (1) {
		if (print_flag) {
			print_flag = false;
			printf("Seconds since init: %d\n", seconds_counter);
		}
	}

	return 0;
}
