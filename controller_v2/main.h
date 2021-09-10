#ifndef __MAIN_H
#define __MAIN_H

#include <pthread_mutex.h>
#include <machine/patmos.h>
#include <machine/spm.h>

#include "lib/pid.h"


/** Shared variables and mutexes -------------------------------------------- */
extern pthread_mutex_t print_mtx;
extern pthread_mutex_t pid_mtx;
extern pid_instance_t pitch_pid;
extern pid_instance_t roll_pid;
extern pid_instance_t yaw_pid;
extern pid_instance_t pitch_vel_pid;
extern pid_instance_t roll_vel_pid;
extern pid_instance_t yaw_vel_pid;


/** Thread main functions --------------------------------------------------- */
void* thread_0_main(void* args);
void* thread_1_main(void* args);
void* thread_2_main(void* args);
void* thread_3_main(void* args);

#endif // __MAIN_H
