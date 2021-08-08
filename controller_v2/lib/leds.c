#include <machine/patmos.h>
#include <machine/spm.h>

#include "leds.h"

#define LED_COUNT        8

//volatile _SPM int *led_ptr  = (volatile _SPM int *) (PATMOS_IO_LED);
#define LED_REG (*(volatile _SPM int *) (PATMOS_IO_LED))

void led_on(int led)
{
	if (led < 0 || led >= LED_COUNT) {
		return;
	}
	LED_REG |= 1 << led;
}

void led_off(int led)
{
	if (led < 0 || led >= LED_COUNT) {
		return;
	}
	LED_REG &= 0xff ^ (1 << led);
}

void led_toggle(int led)
{
	if (led < 0 || led >= LED_COUNT) {
		return;
	}
	LED_REG ^= 1 << led;
}

void led_on_all(void)
{
	LED_REG = 0xff;
}

void led_off_all(void)
{
	LED_REG = 0x00;
}
