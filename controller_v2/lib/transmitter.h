#ifndef __TRANSMITTER_H
#define __TRANSMITTER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define RECEIVER ( ( volatile _IODEV unsigned * ) PATMOS_IO_ACT )

typedef struct transmitter {
	__int32_t throttle;
	__int32_t roll;
	__int32_t pitch;
	__int32_t yaw;
	__int32_t channel_5;
	__int32_t channel_6;
} transmitter;


int receiver_read( unsigned int receiver_id );

int convert_receiver_channel( unsigned int channel, bool reverse );

void transmitter_read( void );

transmitter get_transmitter_values( void );

#endif // __TRANSMITTER_H
