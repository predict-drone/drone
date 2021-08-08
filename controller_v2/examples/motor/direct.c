#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>
#include <machine/patmos.h>
#include <machine/spm.h>
#include "lib/transmitter.h"
#include "lib/motor.h"


static int counter = 1;
static pthread_mutex_t counter_mtx = PTHREAD_MUTEX_INITIALIZER;
transmitter tm;
static int counter;



static void* thread1_main(void *args)
{
  while(1);
}



static void* thread2_main(void *args)
{	
		
	while(1){
	
		transmitter_read(); 		
		tm = get_transmitter_values();		//Reading values from the remote controller
		
		pthread_mutex_lock(&counter_mtx);
		printf("\n Throttle = %d", tm.throttle);
		pthread_mutex_unlock(&counter_mtx); 
		
		motor_run(0, tm.throttle);
		motor_run(1, tm.throttle);
		motor_run(2, tm.throttle);
		motor_run(3, tm.throttle);
	}
}


int main() {

	pthread_t thread1, thread2;
	pthread_create(&thread1, NULL, thread1_main, NULL);
	pthread_create(&thread2, NULL, thread2_main, NULL);

	while(true){
		pthread_mutex_lock(&counter_mtx);
		printf("\nCounter:%d", counter);
		pthread_mutex_unlock(&counter_mtx);
		
	}
	return 0;
}
