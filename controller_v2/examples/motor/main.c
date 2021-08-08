
#include <stdio.h>
#include <stdint.h>
#include <machine/patmos.h>
#include <machine/spm.h>

#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

static int counter;
static pthread_mutex_t counter_mtx = PTHREAD_MUTEX_INITIALIZER;
static int sec_counter;
static bool print_flag;


//motors
volatile _SPM int* led_ptr = (volatile _SPM int*) PATMOS_IO_LED;
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
volatile _SPM int* m = (volatile _SPM int*) PATMOS_IO_ACT+0x10;
#define m1 0
#define m2 1
#define m3 2
#define m4 3

static int counter;
static int i, j = 1;


//This function increases the pwm output from 1 to 2000, then drops back to 1. All 4 motors receive the same value.
static void actuator(void)
{
		pthread_mutex_lock(&counter_mtx);
		counter=j;
		pthread_mutex_unlock(&counter_mtx);
		j = j+100;
		if (j >= 2000) j = 1;
		*(MOTOR + m1) = j;
		*(MOTOR + m2) = j;
		*(MOTOR + m3) = j;
		*(MOTOR + m4) = j;
		

}



static void* thread1_main(void *args)
{
	while(1);
}


static void* thread2_main(void *args)
{
	while(1)
	{
		actuator();
	}
}

int main() 
{
	pthread_t thread1, thread2;
	pthread_create(&thread1, NULL, thread1_main, NULL);
	pthread_create(&thread2, NULL, thread2_main, NULL);	//Only the 3rd thread can control the motors. The 1st is the main function itself.

	
	
	while(1)
	{
		pthread_mutex_lock(&counter_mtx);
		printf("\nCounter:%d",counter);
		pthread_mutex_unlock(&counter_mtx);	
		
	}		
	return 0;
}
