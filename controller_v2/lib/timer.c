#include <stddef.h>
#include <machine/exceptions.h>
#include <machine/rtc.h>

#include "timer.h"
#include "util.h"

#define __PATMOS_TIMER_BASE       0xF0020000
#define __PATMOS_TIMER_HICLK      (__PATMOS_TIMER_BASE + 0x00)
#define __PATMOS_TIMER_LOCLK      (__PATMOS_TIMER_BASE + 0x04)
#define __PATMOS_TIMER_HIUSEC     (__PATMOS_TIMER_BASE + 0x08)
#define __PATMOS_TIMER_LOUSEC     (__PATMOS_TIMER_BASE + 0x0c)

#define TIMER_LIST_SIZE 15

static app_timer_t timer_list[4][TIMER_LIST_SIZE];

static void us_interrupt_cb(void) __attribute__((naked));
static void set_next_timer(void);
static void execute_timer(void);


static void us_interrupt_cb(void)
{
	exc_prologue();
	execute_timer();
	exc_epilogue();
}

static void set_next_timer(void)
{
	int core_id = get_core_id();
	int earliest_index = -1;
	int64_t earliest_time = 0;

	for (int i = 0; i < TIMER_LIST_SIZE; i++) {
		if (timer_list[core_id][i].timer_cb != NULL) {
			if (earliest_index == -1) {
				earliest_index = i;
				earliest_time = timer_list[core_id][i].time;
			} else if (earliest_time <= 0
					&& timer_list[core_id][i].time < timer_list[core_id][earliest_index].time) {
				earliest_index = i;
				earliest_time = timer_list[core_id][i].time;
			}
		}
	}
	
	if (earliest_index != -1) {
		arm_usec_timer(earliest_time);	
	}
}

static void execute_timer(void)
{
	int core_id = get_core_id();
	uint64_t current_time = micros();
	for (int i = 0; i < TIMER_LIST_SIZE; i++) {
		if (timer_list[core_id][i].timer_cb != NULL) {
			if (timer_list[core_id][i].time <= current_time) {
				timer_list[core_id][i].timer_cb();
				if (timer_list[core_id][i].period) {
					timer_list[core_id][i].time += timer_list[core_id][i].period*1000;
				} else {
					timer_list[core_id][i].timer_cb = NULL;
				}
			}
		}
	}
	set_next_timer();
}


void timer_init(void)
{
	exc_register(17, us_interrupt_cb);
	intr_unmask_all();
	intr_clear_all_pending();
	intr_enable();
}

app_timer_t* create_timer(timer_cb_t timer_cb, uint32_t time_ms,
		uint32_t period_ms)
{
	int core_id = get_core_id();
	app_timer_t* result_timer = NULL;
	uint64_t current_time = micros();

	for (int i = 0; i < TIMER_LIST_SIZE; i++) {
		if (timer_list[core_id][i].timer_cb == NULL) {
			timer_list[core_id][i].id = i;
			timer_list[core_id][i].timer_cb = timer_cb;
			timer_list[core_id][i].time = current_time + time_ms*1000;
			timer_list[core_id][i].period = period_ms;
			result_timer = &(timer_list[core_id][i]);
			set_next_timer();
			break;
		}
	}
	return result_timer;
}

void delete_timer(app_timer_t* timer)
{
	int core_id = get_core_id();
	if (timer_list[core_id][timer->id].timer_cb != NULL) {
		timer_list[core_id][timer->id].timer_cb = NULL;
		set_next_timer();
	}
}

uint64_t micros(void)
{
	return (uint64_t) get_cpu_usecs();
}

void delay(uint32_t time_ms)
{
	uint64_t time_us = micros() + time_ms * 1000;
	while (micros() < time_us);
}
