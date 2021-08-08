#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <pthread_mutex.h>
#include <machine/patmos.h>
#include <machine/spm.h>
#include "transmitter.h"


//motors
#define MOTOR ( ( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10 )
#define m1 0
#define m2 1
#define m3 2
#define m4 3
#define minpower 700


void motor_run_all(int throttle);
void motor_run(int motor_number, int throttle);
