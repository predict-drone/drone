#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

typedef void (*timer_cb_t)(void);

typedef struct app_timer_t {
	int        id;
	timer_cb_t timer_cb;
	uint64_t   time;	
	uint32_t   period;
} app_timer_t;


void timer_init(void);

app_timer_t* create_timer(timer_cb_t timer_cb, uint32_t time_ms,
		uint32_t period_ms);

void delete_timer(app_timer_t* timer);

uint64_t micros(void);

void delay(uint32_t time_ms);

#endif // __TIMER_H
