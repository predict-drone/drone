/*
    This is a minimal C program executed on the FPGA version of Patmos.
    An embedded Hello World program: a blinking LED.

    Additional to the blinking LED we write to the UART '0' and '1' (if available).

    Author: Martin Schoeberl
    Copyright: DTU, BSD License
*/

#include "include/bootable.h"
#include <machine/spm.h>

#define LOOP_DELAY 2000

int main() {

	volatile _SPM int *led_ptr  = (volatile _SPM int *) PATMOS_IO_LED;
	volatile _SPM int *timer  = (volatile _SPM int *) (PATMOS_IO_TIMER+12);
	int i, j;
	int next = 0;
	for (;;) {
		UART_DATA = '1';
		next = *timer + 10000000;
		while (*timer-next<0)
			*led_ptr = 1;	


		UART_DATA = '0';
		next = *timer + 10000000;
		while (*timer-next<0)
			*led_ptr = 0;

	}
}
