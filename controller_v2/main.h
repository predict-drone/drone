#ifndef __MAIN_H
#define __MAIN_H

#include <pthread_mutex.h>
#include <machine/patmos.h>
#include <machine/spm.h>


/** Shared variables and mutexes -------------------------------------------- */
extern pthread_mutex_t print_mtx;


/** Thread main functions --------------------------------------------------- */
void* thread_0_main(void* args);
void* thread_1_main(void* args);
void* thread_2_main(void* args);
void* thread_3_main(void* args);

#endif // __MAIN_H
