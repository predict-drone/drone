#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include <stdint.h>

enum comm_op_t {
	COMM_OP_GET_PID_VALUES = 0x01,
	COMM_OP_SET_PID_VALUES,
	COMM_OP_SET_BASE_THROTTLE,
	COMM_OP_LOG_PID,
	COMM_OP_LOG_POSITION,
	COMM_OP_LOG_MOTOR,
};

enum comm_op_rsp_t {
	COMM_OP_RSP_GET_PID_VALUES = 0x81,
	COMM_OP_RSP_SET_PID_VALUES,
	COMM_OP_RSP_ANGLE_VALUES,
	COMM_OP_RSP_SET_BASE_THROTTLE,
	COMM_OP_RSP_LOG_PID,
	COMM_OP_RSP_LOG_POSITION,
	COMM_OP_RSP_LOG_MOTOR,
};

enum comm_pid_id_t {
	COMM_PID_PITCH = 0x01,
	COMM_PID_ROLL,
	COMM_PID_YAW,
	COMM_PID_PITCH_VEL,
	COMM_PID_ROLL_VEL,
	COMM_PID_YAW_VEL,
};

typedef struct comm_pid_values_t {
	uint8_t opcode;
	uint8_t pid_id;
	uint32_t kp;
	uint32_t ki;
	uint32_t kd;
} __attribute__((packed)) comm_pid_values_t;

typedef struct comm_data_t {
	uint8_t opcode;
	int32_t x;
	int32_t y;
	int32_t z;
} __attribute__((packed)) comm_data_t;


typedef void (*comm_rx_cb_t)(uint8_t msg_len, uint8_t* msg);

void comm_init(comm_rx_cb_t rx_callback);

void comm_process(void);

void comm_send_msg(uint8_t msg_len, uint8_t* msg);

#endif // __COMMUNICATION_H
