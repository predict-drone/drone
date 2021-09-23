#ifndef __MOTOR_H
#define __MOTOR_H

typedef struct motor_t {
	int m0;
	int m1;
	int m2;
	int m3;
} motor_t;

void motor_run(motor_t throttle);

#endif // __MOTOR_H
