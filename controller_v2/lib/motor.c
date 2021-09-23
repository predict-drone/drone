#include <machine/patmos.h>
#include <machine/spm.h>

#include "motor.h"


#define MOTOR (( volatile _IODEV unsigned * )  PATMOS_IO_ACT+0x10)


void motor_run(motor_t throttle)
{	
	int* throttle_array = (int*)(&throttle);
	for (int i = 0; i < 4; i++) {
		int t = throttle_array[i]*2 - 2000;
		if (t < 700) {
			t = 700;
		}
		*(MOTOR + i) = t;
	}
}
