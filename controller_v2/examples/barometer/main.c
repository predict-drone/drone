#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

#include "lib/timer.h"
#include "sensors/baro.h"
#include "lib/i2c_master.h"


#define MS5611_ADDR 0x77	//Barometer v2 variables
#define BARO_REG                   0xA0   // R



// Shared variables
uint16_t C[7];
int loop_counter, loop_timer,start=0;
uint8_t barometer_counter, temperature_counter;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff, baro_data;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
static pthread_mutex_t data_mtx = PTHREAD_MUTEX_INITIALIZER;

// Thread 0 vairables
static volatile bool print_flag;

// Thread 1 vairables
static volatile bool read_flag;


static void print_flag_cb(void);
static void read_flag_cb(void);


static void print_flag_cb(void)
{
	print_flag = true;
}

static void read_flag_cb(void)
{
	read_flag = true;
}

static void* thread1_main(void* attr)
{
	printf("Barometer example\n");
	
	barometer_init();
	
	uint64_t time = micros();
	int period = 250000;
	time += period;

	while (true) {
		if (time < micros()) {
			pthread_mutex_lock(&data_mtx);
			baro_data = check_barometer();
			pthread_mutex_unlock(&data_mtx);
			time += period;
		}
	}
}


int main(void)
{	

	timer_init();
	
	

	// Init threads
	pthread_t thread1;
	pthread_create(&thread1, NULL, thread1_main, NULL);

	app_timer_t* timer = create_timer(print_flag_cb, 500, 2000);


	while (true) {
		if (print_flag) {
			print_flag = false;

			pthread_mutex_lock(&data_mtx);
			printf("Barometer: %f\n", baro_data);
			pthread_mutex_unlock(&data_mtx);
		}
	}

	return 0;
}
