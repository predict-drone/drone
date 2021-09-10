#include "communication.h"
#include <stdio.h>
#include "uart.h"
#include "main.h"

/** Static variables -------------------------------------------------------- */
static comm_rx_cb_t rx_cb;
static uint8_t buffer[256];
static uint8_t buffer_index;

/** Public functions -------------------------------------------------------- */
void comm_init(comm_rx_cb_t rx_callback)
{
	rx_cb = rx_callback;
}

void comm_process(void)
{
	uint8_t c;
	if (uart3_read(&c)) {
		pthread_mutex_lock(&print_mtx);
		printf("READ3: %c %d\n", c, c);
		pthread_mutex_unlock(&print_mtx);
		buffer[buffer_index] = c;
		if (buffer_index == buffer[0]) {
			rx_cb(buffer[0], &(buffer[1]));
			buffer_index = 0;
		} else {
			buffer_index++;
		}
	}
}

void comm_send_msg(uint8_t msg_len, uint8_t* msg)
{
	while(!uart3_write(msg_len));
	for (int i = 0; i < msg_len; i++) {
		while(!uart3_write(msg[i]));
	}
}
