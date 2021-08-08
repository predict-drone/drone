
#include <stdio.h>
#include <stdint.h>
#include <machine/patmos.h>
#include <machine/spm.h>
#include "lib/transmitter.h"

#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>

static int counter = 1;
static pthread_mutex_t counter_mtx = PTHREAD_MUTEX_INITIALIZER;
transmitter tm;

//motors
volatile _SPM int* led_ptr = (volatile _SPM int*) PATMOS_IO_LED;
volatile _SPM int* m = (volatile _SPM int*) PATMOS_IO_ACT+0x10;
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3

static int counter;
static int i=0,j=1,k=0;
static bool startmotor = false, stopmotor = false, final = false, m1_cal = false, m2_cal = false, m3_cal = false, m4_cal = false, calibrated = false;

/*static void motor_calibration(void)
{	

	pthread_mutex_lock(&counter_mtx);
	counter = j;  
	printf("\nMotor is calibrating %d", j);  
	pthread_mutex_unlock(&counter_mtx);
	
	
	i = 40000;
	while(i>0 && !m1_cal)
	{
		j = j + 100;
		if (j >= 2000) j = 1;
		*(MOTOR + m1) = j;
	}
	m1_cal = true;
	*(MOTOR + m1) = 0;
	
	i = 40000;
	while(i>0 && !m2_cal)
	{
		j = j + 100;
		if (j >= 2000) j = 1;
		*(MOTOR + m2) = j;
	}
	m2_cal = true;
	*(MOTOR + m2) = 0;
	
	i = 40000;
	while(i>0 && !m3_cal) 
	{
		j = j + 100;
		if (j >= 2000) j = 1;
		*(MOTOR + m3) = j;
	}
	m3_cal = true;
	*(MOTOR + m3) = 0;
	
	i = 40000;
	while(i>0 && !m4_cal)
	{
		j = j + 100;
		if (j >= 2000) j = 1;
		*(MOTOR + m4) = j;
	}
	m4_cal = true;
	*(MOTOR + m4) = 0;
	
	if (m1_cal && m2_cal && m3_cal && m4_cal) calibrated = true;

}
*/

static void actuator_start(void)
{
	pthread_mutex_lock(&counter_mtx);
	counter = j;  
	printf("\nMotor is starting %d", j);  
	pthread_mutex_unlock(&counter_mtx);

	j = j + 100;
	if (j >= 2000) j = 1;
	*(MOTOR + m1) = j;
	*(MOTOR + m2) = j;
	*(MOTOR + m3) = j;
	*(MOTOR + m4) = j;
}


static void motor_start(int number)
{
	pthread_mutex_lock(&counter_mtx);
	counter = j;  
	printf("\nMotor is starting %d", j);  
	pthread_mutex_unlock(&counter_mtx);

	j = j + 100;
	if (j >= 2000) j = 1;
	*(MOTOR + number) = j;
	*(MOTOR + number) = j;
	*(MOTOR + number) = j;
	*(MOTOR + number) = j;
}



static void actuator_stop(void)
{
	pthread_mutex_lock(&counter_mtx);
	counter = j;
	printf("\n Stopping motor: %d", j);
	pthread_mutex_unlock(&counter_mtx);
	
	j = j - 50;
	if (j <= 0) j = 0;
	*(MOTOR + m1) = j;
	*(MOTOR + m2) = j;
	*(MOTOR + m3) = j;
	*(MOTOR + m4) = j;
	
	if (tm.throttle <= 1100) final = true;
}


static void* thread1_main(void *args)
{
  while(1);
}



static void* thread2_main(void *args)
{	
	
	while(1){
	
		transmitter_read(); 		
		tm = get_transmitter_values();
		
		pthread_mutex_lock(&counter_mtx);
		printf("/n Throttle = %d", tm.throttle);
		pthread_mutex_unlock(&counter_mtx); 
		
		if (tm.throttle < 1200 && tm.throttle > 800){
			startmotor = false;
			stopmotor = true;
			}
		if (tm.throttle < 2200 && tm.throttle > 1800){
			startmotor = true;
			stopmotor = false;
			}
			

		

		
		if (startmotor)
	  		actuator_start();
		//if (startmotor && !stopmotor)
			//actuator_run();
		if (stopmotor)
			actuator_stop();

		/*if (final) break;

		while (i > 0)
		{
			i--;
		}
		
		if (i == 0)
			i = 200000;
		*/
	}
}


int main() {

	pthread_t thread1, thread2;
	pthread_create(&thread1, NULL, thread1_main, NULL);
	pthread_create(&thread2, NULL, thread2_main, NULL);

	while(true){
		*led_ptr = 0xff;
		pthread_mutex_lock(&counter_mtx);
		printf("\nCounter:%d", counter);
		pthread_mutex_unlock(&counter_mtx);
		
		
		/*while (i > 0)
		{
			i--;
		}
		if (i == 0)
			i = 200000;
		*/
	}
	return 0;
}
