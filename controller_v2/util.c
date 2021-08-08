#include "machine/patmos.h"
#include "util.h"


#define COREID ((volatile _IODEV unsigned *) __PATMOS_CPUINFO_COREID)


int get_core_id(void)
{
	return *COREID;
}
