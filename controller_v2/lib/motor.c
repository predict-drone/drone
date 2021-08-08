#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>
#include <machine/patmos.h>
#include <machine/spm.h>
#include "transmitter.h"
#include "motor.h"



//This function controls all the motors with the same power
void motor_run_all(int throttle)
{
	throttle = throttle * 2 - 2000;			//Here we scale the throttle signal from [1000-2000] to [0-2000]
	if (throttle < 700) throttle = 700;
	if (throttle > 2000) throttle = 2000;
	
	
	*(MOTOR + m1) = throttle;
	*(MOTOR + m2) = throttle;
	*(MOTOR + m3) = throttle;
	*(MOTOR + m4) = throttle;
	
}

//This function controls a selected motor and sets the others to minimum power (zero speed)
void motor_run(int motor_number, int throttle)
{	
	if (motor_number > 3) motor_number = 3;
	if (motor_number < 0) motor_number = 0;
	throttle = throttle * 2 - 2000;
	if (throttle < 700) throttle = 700;
	//else if (throttle < 1000) throttle = 1000;
	
	*(MOTOR + motor_number) = throttle;
	
}
