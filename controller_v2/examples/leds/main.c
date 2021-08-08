#include <stdio.h>

#include "leds.h"

#define LOOP_DELAY 2000

int main(void)
{
	printf("Hello world!\n");
	int i, j;
	led_on_all();

	for (;;) {
		led_toggle(6);
		for (i=LOOP_DELAY; i!=0; --i)
			for (j=LOOP_DELAY; j!=0; --j)
				led_on(7);


		for (i=LOOP_DELAY; i!=0; --i)
			for (j=LOOP_DELAY; j!=0; --j)
				led_off(7);

	}

	return 0;
}
