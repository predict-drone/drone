#include <machine/patmos.h>

#include "uart.h"

#define UART2 ((volatile _IODEV unsigned *)PATMOS_IO_UART2)
#define UART3 ((volatile _IODEV unsigned *)PATMOS_IO_UART3)


int uart2_write(unsigned char data)
{
    if ((*UART2 & 0x00000001) != 0)
    {
        *UART2 = (unsigned int)data;
        return 1;
    }
    else
    {
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
        return 0;
    }
}

int uart3_write(unsigned char data)
{
    if ((*UART3 & 0x00000001) != 0)
    {
        *UART3 = (unsigned int)data;
        return 1;
    }
    else
    {
        return 0;
    }
}

int uart3_read(unsigned char *data)
{
    if ((*UART3 & 0x00000002) != 0)
    {
        *data = (unsigned char)(*(UART3 + 1) & 0x000000FF);
        return 1;
    }
    else
    {
        return 0;
    }
}
