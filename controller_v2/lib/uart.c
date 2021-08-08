#include <machine/patmos.h>

#include "uart.h"

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)


int uart2_write(unsigned char data)
{
    if ((*UART2 & 0x00000001) != 0)
    {
        *UART2 = (unsigned int)data;
        return 1;
    }
    else
    {
        data = 0;
        return 0;
    }
}

int uart2_read(unsigned char *data)
{
    if ((*UART2 & 0x00000002) != 0)
    {
        *data = (unsigned char)(*(UART2 + 1) & 0x000000FF);
        return 1;
    }
    else
    {
        *data = 0;
        return 0;
    }
}
